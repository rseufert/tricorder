#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_infrared.h>
#include <gui/gui.h>
#include <input/input.h>
#include <storage/storage.h>
#include <datetime/datetime.h>
#include <notification/notification_messages.h>
#include <infrared.h>
#include <infrared_transmit.h>
#include <lib/subghz/devices/cc1101_configs.h>
#include <lib/subghz/receiver.h>
#include <lib/subghz/subghz_worker.h>
#include <lib/subghz/subghz_protocol_registry.h>
#include <lib/subghz/protocols/base.h>
#include <nfc/nfc.h>
#include <nfc/nfc_device.h>
#include <nfc/nfc_scanner.h>
#include <nfc/nfc_poller.h>
#include <nfc/protocols/iso14443_3a/iso14443_3a_poller.h>
#include <nfc/protocols/iso14443_3b/iso14443_3b_poller.h>
#include <nfc/protocols/iso15693_3/iso15693_3_poller.h>
#include <nfc/protocols/felica/felica_poller.h>
#include <nfc/protocols/st25tb/st25tb_poller.h>
#include <lfrfid/lfrfid_worker.h>
#include <lfrfid/protocols/lfrfid_protocols.h>
#include <toolbox/protocols/protocol_dict.h>
#include <ibutton/ibutton_worker.h>
#include <ibutton/ibutton_protocols.h>
#include <ibutton/ibutton_key.h>

// Screen layout: mode tabs down the left, readout panel on the right
#define TAB_W    22
#define TAB_ROWS 4
#define GX       26 // graph left edge
#define GW       102 // graph width
#define GY       13 // graph top
#define GH       40 // graph height
#define GB       (GY + GH) // graph baseline
#define LINE_LEN 26

#define RF_POINTS  GW
#define RF_CHUNK   12 // points measured per loop pass
#define RF_FLOOR   (-95) // dBm at the bottom of the graph
#define RF_SPAN    65 // dB from bottom to top
#define RF_CONTACT (-65) // dBm that counts as a contact

#define IR_MAX        512
#define IR_GAP_US     15000 // a silence this long splits two frames
#define IR_TIMEOUT_US 150000
#define IR_MIN_EDGES  8
#define IR_SAVE_PATH  EXT_PATH("infrared/Tricorder.ir")

#define TAG_MAX     3
#define TAG_UID_MAX 10

#define PIN_FULL_MV 2500

#define BOOT_TICKS     30
#define SETTINGS_PATH  APP_DATA_PATH("settings.bin")
#define SETTINGS_MAGIC 0x31435254 // "TRC1"

typedef enum {
    ModeRf,
    ModeIr,
    ModeEmf,
    ModeTag,
    ModeKey,
    ModePin,
    ModePwr,
    ModeAll,
    ModeCount,
} Mode;

static const char* const mode_names[ModeCount] =
    {"RF", "IR", "EMF", "TAG", "KEY", "PIN", "PWR", "ALL"};

typedef struct {
    uint32_t lo;
    uint32_t hi;
    const char* name;
} Band;

static const Band bands[] = {
    {300000000, 348000000, "300-348"},
    {387000000, 464000000, "387-464"},
    {779000000, 928000000, "779-928"},
};
#define BAND_COUNT COUNT_OF(bands)

typedef struct {
    const char* name;
    const uint8_t* regs;
} Modulation;

static const Modulation modulations[] = {
    {"AM650", subghz_device_cc1101_preset_ook_650khz_async_regs},
    {"AM270", subghz_device_cc1101_preset_ook_270khz_async_regs},
    {"FM238", subghz_device_cc1101_preset_2fsk_dev2_38khz_async_regs},
    {"FM476", subghz_device_cc1101_preset_2fsk_dev47_6khz_async_regs},
};
#define MOD_COUNT COUNT_OF(modulations)

static const uint16_t ir_zooms[] = {50, 100, 200, 500, 1000}; // microseconds per pixel
#define IR_ZOOM_COUNT COUNT_OF(ir_zooms)

typedef struct {
    const char* name;
    const GpioPin* pin;
    FuriHalAdcChannel channel;
} ProbePin;

static const ProbePin probe_pins[] = {
    {"A7 (2)", &gpio_ext_pa7, FuriHalAdcChannel12},
    {"A6 (3)", &gpio_ext_pa6, FuriHalAdcChannel11},
    {"A4 (4)", &gpio_ext_pa4, FuriHalAdcChannel9},
    {"C3 (7)", &gpio_ext_pc3, FuriHalAdcChannel4},
    {"C1 (15)", &gpio_ext_pc1, FuriHalAdcChannel2},
    {"C0 (16)", &gpio_ext_pc0, FuriHalAdcChannel1},
};
#define PIN_COUNT COUNT_OF(probe_pins)

typedef enum {
    KeyRfid,
    KeyIbutton,
    KeyKindCount,
} KeyKind;

typedef enum {
    TagScan, // scanner looking for any tag
    TagPoll, // reading the UID of the tag just found
    TagRest, // short pause before scanning again
} TagState;

typedef enum {
    SweepRf,
    SweepIr,
    SweepEmf,
    SweepTag,
    SweepCount,
} SweepPhase;

typedef enum {
    MenuRfView,
    MenuRfMod,
    MenuIrSend,
    MenuIrSave,
    MenuSound,
} MenuItem;
#define MENU_MAX 3

typedef struct {
    uint16_t freq; // 0 = rest
    uint8_t passes;
} Chirp;

static const Chirp chirp_open[] = {{1568, 1}, {2093, 1}, {2637, 1}, {3136, 2}, {0, 0}};
static const Chirp chirp_mode[] = {{2093, 1}, {2794, 1}, {0, 0}};
static const Chirp chirp_contact[] = {{2637, 1}, {3520, 1}, {2637, 1}, {3520, 1}, {0, 0}};
static const Chirp chirp_lock[] = {{1760, 1}, {2349, 1}, {3136, 1}, {0, 0}};

typedef struct {
    uint32_t magic;
    uint8_t sound;
    uint8_t mode;
    uint8_t band;
    uint8_t waterfall;
    uint8_t mod;
    uint8_t ir_zoom;
    uint8_t pin;
    uint8_t key_kind;
} Settings;

typedef struct {
    FuriMutex* mutex;
    FuriMessageQueue* queue;
    ViewPort* view_port;
    NotificationApp* notifications;
    bool running;
    Mode mode;
    uint32_t frame;
    uint8_t boot;

    // Sub-GHz
    uint8_t band;
    bool track;
    bool waterfall;
    uint8_t mod;
    int8_t rssi[RF_POINTS];
    int8_t peak[RF_POINTS];
    uint8_t fall[GH][RF_POINTS]; // waterfall rows, newest first, 0..3
    uint8_t scan_i;
    uint8_t max_i;
    uint32_t track_freq;
    bool retune;
    int8_t track_rssi;
    int8_t hist[RF_POINTS];
    SubGhzEnvironment* sg_env;
    SubGhzReceiver* sg_receiver;
    SubGhzWorker* sg_worker;
    char rf_ident[2][LINE_LEN];
    bool rf_ident_new;

    // Infrared
    FuriStreamBuffer* ir_stream;
    volatile bool ir_timeout;
    bool ir_live;
    uint32_t ir_pending[IR_MAX];
    size_t ir_pending_n;
    uint32_t ir_edges;
    InfraredDecoderHandler* ir_decoder;
    uint32_t ir_timings[IR_MAX];
    size_t ir_n;
    uint32_t ir_total;
    bool ir_decoded;
    InfraredMessage ir_msg;
    uint32_t ir_count;
    uint8_t ir_zoom;
    uint32_t ir_off;
    bool ir_new;

    // Reader fields
    bool nfc_held;
    bool nfc_field;
    bool lf_field;
    uint32_t lf_freq;
    uint8_t emf_hist[GW]; // bit 0 = NFC, bit 1 = LF

    // NFC tags
    Nfc* nfc;
    NfcScanner* scanner;
    NfcPoller* tag_poller;
    NfcDevice* tag_device;
    TagState tag_state;
    bool tag_event;
    volatile bool tag_poll_done;
    NfcProtocol tag_pending[TAG_MAX];
    size_t tag_pending_n;
    NfcProtocol tag_protocols[TAG_MAX];
    size_t tag_n;
    uint8_t tag_uid[TAG_UID_MAX];
    size_t tag_uid_len;
    bool tag_present;
    uint32_t tag_scan_start;
    uint32_t tag_deadline;

    // 125 kHz RFID and iButton keys
    uint8_t key_kind;
    ProtocolDict* key_dict;
    LFRFIDWorker* key_rfid;
    iButtonProtocols* key_ib_protocols;
    iButtonWorker* key_ib_worker;
    iButtonKey* key_ib_key;
    volatile bool key_event;
    volatile int32_t key_protocol;
    bool key_reading;
    uint32_t key_restart_at;
    uint32_t key_count;
    char key_name[LINE_LEN];
    char key_lines[3][LINE_LEN];

    // GPIO probe
    FuriHalAdcHandle* adc;
    uint8_t pin;
    uint16_t pin_mv;
    bool pin_fresh;
    uint16_t pin_hist[GW];

    // Power
    uint8_t pwr_pct;
    int16_t pwr_mv;
    int16_t pwr_ma;
    int16_t pwr_temp10;
    bool pwr_charging;
    int16_t pwr_hist[GW];

    // Full sweep
    SweepPhase sw_phase;
    bool sw_active;
    uint16_t sw_ticks;
    uint16_t sw_pass;
    uint8_t sw_saved_band;
    int8_t sw_rf_best;
    uint32_t sw_rf_freq;
    uint32_t sw_ir_count;
    uint8_t sw_emf;
    char sw_text[SweepCount][LINE_LEN];
    bool sw_hit[SweepCount];

    // Menu
    bool menu_open;
    uint8_t menu_sel;

    // Sound
    bool sound;
    bool speaker_held;
    bool tone_on;
    const Chirp* chirp;
    uint8_t chirp_left;
    uint8_t contact; // 0 = none, else signal strength 1..GH
    uint8_t toast;
    char toast_text[LINE_LEN];
} App;

// ---------------------------------------------------------------- helpers

// Split text on line breaks into fixed-size lines; returns how many were filled
static size_t set_lines(char (*dst)[LINE_LEN], size_t max, const char* text) {
    size_t n = 0;
    while(*text && n < max) {
        size_t len = 0;
        while(text[len] && text[len] != '\n' && text[len] != '\r')
            len++;
        size_t copy = MIN(len, (size_t)LINE_LEN - 1);
        if(copy) {
            memcpy(dst[n], text, copy);
            dst[n][copy] = 0;
            n++;
        }
        text += len;
        while(*text == '\n' || *text == '\r')
            text++;
    }
    for(size_t i = n; i < max; i++)
        dst[i][0] = 0;
    return n;
}

static void toast(App* app, const char* text) {
    strlcpy(app->toast_text, text, sizeof(app->toast_text));
    app->toast = 30;
}

static bool tick_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

// ---------------------------------------------------------------- sound

static void sound_tone(App* app, float freq) {
    if(!app->sound || furi_hal_rtc_is_flag_set(FuriHalRtcFlagStealthMode)) return;
    if(!app->speaker_held) {
        if(!furi_hal_speaker_acquire(10)) return;
        app->speaker_held = true;
    }
    furi_hal_speaker_start(freq, 0.3f);
    app->tone_on = true;
}

static void sound_quiet(App* app) {
    if(app->tone_on) {
        furi_hal_speaker_stop();
        app->tone_on = false;
    }
}

static void sound_release(App* app) {
    sound_quiet(app);
    if(app->speaker_held) {
        furi_hal_speaker_release();
        app->speaker_held = false;
    }
}

static void sound_chirp(App* app, const Chirp* chirp) {
    app->chirp = chirp;
    app->chirp_left = 0;
}

// Called once per loop pass: plays the queued chirp, else warbles on a contact
static void sound_update(App* app) {
    if(app->chirp) {
        if(app->chirp_left == 0) {
            if(app->chirp->passes == 0) {
                app->chirp = NULL;
                sound_quiet(app);
                return;
            }
            if(app->chirp->freq) {
                sound_tone(app, app->chirp->freq);
            } else {
                sound_quiet(app);
            }
            app->chirp_left = app->chirp->passes;
        }
        if(--app->chirp_left == 0) app->chirp++;
        return;
    }
    if(app->contact) {
        float freq = 900.0f + app->contact * 45.0f;
        if(app->frame & 1) freq *= 1.26f;
        sound_tone(app, freq);
    } else {
        sound_quiet(app);
    }
}

// ---------------------------------------------------------------- Sub-GHz

static uint32_t rf_point_freq(uint8_t band, uint8_t i) {
    const Band* b = &bands[band];
    return b->lo + (uint32_t)(((uint64_t)(b->hi - b->lo) * i) / (RF_POINTS - 1));
}

static int rf_level(int dbm) {
    int v = (dbm - RF_FLOOR) * GH / RF_SPAN;
    return CLAMP(v, GH, 0);
}

static uint8_t rf_fall_level(int dbm) {
    if(dbm >= -64) return 3;
    if(dbm >= -70) return 2;
    if(dbm >= -75) return 1;
    return 0;
}

static int8_t rf_read(void) {
    float rssi = furi_hal_subghz_get_rssi();
    return (int8_t)CLAMP(rssi, 0.0f, -127.0f);
}

static void rf_clear(App* app) {
    for(size_t i = 0; i < RF_POINTS; i++) {
        app->rssi[i] = RF_FLOOR;
        app->peak[i] = RF_FLOOR;
        app->hist[i] = RF_FLOOR;
    }
    memset(app->fall, 0, sizeof(app->fall));
    app->scan_i = 0;
    app->max_i = 0;
}

static void rf_radio_start(App* app) {
    furi_hal_subghz_reset();
    furi_hal_subghz_idle();
    furi_hal_subghz_load_custom_preset(modulations[0].regs);
    rf_clear(app);
    app->track = false;
}

static void rf_radio_stop(void) {
    furi_hal_subghz_idle();
    furi_hal_subghz_sleep();
}

// Runs in the Sub-GHz worker thread when a decoder recognises a transmission
static void rf_ident_callback(
    SubGhzReceiver* receiver,
    SubGhzProtocolDecoderBase* decoder_base,
    void* context) {
    App* app = context;
    FuriString* text = furi_string_alloc();
    subghz_protocol_decoder_base_get_string(decoder_base, text);
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    set_lines(app->rf_ident, 2, furi_string_get_cstr(text));
    app->rf_ident_new = true;
    furi_mutex_release(app->mutex);
    furi_string_free(text);
    subghz_receiver_reset(receiver);
}

static void rf_listen_stop(App* app) {
    if(subghz_worker_is_running(app->sg_worker)) {
        subghz_worker_stop(app->sg_worker);
        furi_hal_subghz_stop_async_rx();
    }
}

static void rf_start(App* app) {
    rf_radio_start(app);
    app->sg_env = subghz_environment_alloc();
    subghz_environment_set_protocol_registry(app->sg_env, (void*)&subghz_protocol_registry);
    app->sg_receiver = subghz_receiver_alloc_init(app->sg_env);
    subghz_receiver_set_filter(app->sg_receiver, SubGhzProtocolFlag_Decodable);
    subghz_receiver_set_rx_callback(app->sg_receiver, rf_ident_callback, app);
    app->sg_worker = subghz_worker_alloc();
    subghz_worker_set_overrun_callback(
        app->sg_worker, (SubGhzWorkerOverrunCallback)subghz_receiver_reset);
    subghz_worker_set_pair_callback(
        app->sg_worker, (SubGhzWorkerPairCallback)subghz_receiver_decode);
    subghz_worker_set_context(app->sg_worker, app->sg_receiver);
}

static void rf_stop(App* app) {
    rf_listen_stop(app);
    rf_radio_stop();
    subghz_worker_free(app->sg_worker);
    subghz_receiver_free(app->sg_receiver);
    subghz_environment_free(app->sg_env);
    app->sg_worker = NULL;
    app->sg_receiver = NULL;
    app->sg_env = NULL;
    app->track = false;
}

static void rf_step_scan(App* app) {
    int8_t vals[RF_CHUNK];
    uint8_t start = app->scan_i;
    uint8_t n = MIN(RF_CHUNK, RF_POINTS - start);
    for(uint8_t k = 0; k < n; k++) {
        furi_hal_subghz_idle();
        furi_hal_subghz_set_frequency_and_path(rf_point_freq(app->band, start + k));
        furi_hal_subghz_rx();
        furi_delay_ms(2);
        vals[k] = rf_read();
    }
    furi_hal_subghz_idle();

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    for(uint8_t k = 0; k < n; k++) {
        uint8_t i = start + k;
        app->rssi[i] = vals[k];
        if(vals[k] > app->peak[i]) app->peak[i] = vals[k];
    }
    app->scan_i = start + n;
    if(app->scan_i >= RF_POINTS) {
        // sweep finished: find the strongest point, let peaks decay, add a waterfall row
        app->scan_i = 0;
        uint8_t best = 0;
        memmove(app->fall[1], app->fall[0], (GH - 1) * RF_POINTS);
        for(uint8_t i = 0; i < RF_POINTS; i++) {
            if(app->rssi[i] > app->rssi[best]) best = i;
            if(app->peak[i] > app->rssi[i]) app->peak[i] -= 2;
            app->fall[0][i] = rf_fall_level(app->rssi[i]);
        }
        app->max_i = best;
        app->contact = app->rssi[best] > RF_CONTACT ? MAX(rf_level(app->rssi[best]), 1) : 0;
    }
    furi_mutex_release(app->mutex);
}

static void rf_step_track(App* app) {
    if(app->retune) {
        // (re)start listening on the chosen frequency with the chosen modulation
        rf_listen_stop(app);
        furi_hal_subghz_idle();
        furi_hal_subghz_load_custom_preset(modulations[app->mod].regs);
        furi_hal_subghz_set_frequency_and_path(app->track_freq);
        subghz_receiver_reset(app->sg_receiver);
        furi_hal_subghz_start_async_rx(subghz_worker_rx_callback, app->sg_worker);
        subghz_worker_start(app->sg_worker);
        app->retune = false;
        furi_delay_ms(2);
    }
    int8_t best = -127;
    for(uint8_t k = 0; k < 8; k++) {
        int8_t v = rf_read();
        if(v > best) best = v;
        furi_delay_ms(4);
    }
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    memmove(app->hist, app->hist + 1, RF_POINTS - 1);
    app->hist[RF_POINTS - 1] = best;
    app->track_rssi = best;
    app->contact = best > RF_CONTACT ? MAX(rf_level(best), 1) : 0;
    bool ident = app->rf_ident_new;
    app->rf_ident_new = false;
    furi_mutex_release(app->mutex);

    if(ident) {
        sound_chirp(app, chirp_contact);
        notification_message(app->notifications, &sequence_blink_green_10);
    }
}

static void rf_enter_track(App* app) {
    // lock on to the strongest point of the last sweep, rounded to 10 kHz
    uint32_t freq = rf_point_freq(app->band, app->max_i);
    app->track_freq = (freq + 5000) / 10000 * 10000;
    for(size_t i = 0; i < RF_POINTS; i++)
        app->hist[i] = RF_FLOOR;
    app->track_rssi = RF_FLOOR;
    app->rf_ident[0][0] = 0;
    app->rf_ident[1][0] = 0;
    app->track = true;
    app->retune = true;
    app->contact = 0;
}

// Call without the mutex held: stopping the worker waits for its thread
static void rf_leave_track(App* app) {
    rf_listen_stop(app);
    furi_hal_subghz_idle();
    furi_hal_subghz_load_custom_preset(modulations[0].regs);
    app->track = false;
    app->scan_i = 0;
    app->contact = 0;
}

static void rf_tune(App* app, int32_t delta) {
    const Band* b = &bands[app->band];
    int64_t freq = (int64_t)app->track_freq + delta;
    app->track_freq = (uint32_t)CLAMP(freq, (int64_t)b->hi, (int64_t)b->lo);
    app->rf_ident[0][0] = 0;
    app->rf_ident[1][0] = 0;
    app->retune = true;
}

// ---------------------------------------------------------------- infrared

// Edges come straight from the receiver interrupt and are queued for the main loop
static void ir_capture_isr(void* context, bool level, uint32_t duration) {
    App* app = context;
    uint32_t packed = (duration & 0x7FFFFFFF) | (level ? 0x80000000 : 0);
    furi_stream_buffer_send(app->ir_stream, &packed, sizeof(packed), 0);
}

static void ir_timeout_isr(void* context) {
    App* app = context;
    app->ir_timeout = true;
}

// Turn the pending edges into the displayed frame. Call with the mutex held.
static void ir_commit(App* app) {
    size_t count = app->ir_pending_n;
    app->ir_pending_n = 0;
    if(count < IR_MIN_EDGES) return; // repeat codes and stray flashes

    memcpy(app->ir_timings, app->ir_pending, count * sizeof(uint32_t));
    app->ir_n = count;
    app->ir_total = 0;
    app->ir_decoded = false;
    infrared_reset_decoder(app->ir_decoder);
    bool level = true;
    for(size_t i = 0; i < count; i++) {
        app->ir_total += app->ir_timings[i];
        if(!app->ir_decoded) {
            const InfraredMessage* msg =
                infrared_decode(app->ir_decoder, level, app->ir_timings[i]);
            if(msg) {
                app->ir_msg = *msg;
                app->ir_decoded = true;
            }
        }
        level = !level;
    }
    if(!app->ir_decoded) {
        const InfraredMessage* msg = infrared_check_decoder_ready(app->ir_decoder);
        if(msg) {
            app->ir_msg = *msg;
            app->ir_decoded = true;
        }
    }
    app->ir_count++;
    app->ir_off = 0;
    app->ir_new = true;
}

static void ir_step(App* app) {
    // read the flag before draining: every edge of a timed-out frame is already queued
    bool timeout = app->ir_timeout;
    if(timeout) app->ir_timeout = false;

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    uint32_t packed;
    while(furi_stream_buffer_receive(app->ir_stream, &packed, sizeof(packed), 0) ==
          sizeof(packed)) {
        bool level = packed & 0x80000000;
        uint32_t duration = packed & 0x7FFFFFFF;
        app->ir_edges++;
        if(!level) {
            if(app->ir_pending_n == 0) continue;
            if(duration > IR_GAP_US) {
                ir_commit(app); // a long silence ends the frame
                continue;
            }
        }
        if(app->ir_pending_n < IR_MAX) app->ir_pending[app->ir_pending_n++] = duration;
    }
    if(timeout) ir_commit(app);
    bool fresh = app->ir_new;
    bool decoded = app->ir_decoded;
    app->ir_new = false;
    furi_mutex_release(app->mutex);

    if(fresh) {
        sound_chirp(app, decoded ? chirp_contact : chirp_mode);
        notification_message(app->notifications, &sequence_blink_blue_10);
    }
}

static void ir_rx_begin(App* app) {
    app->ir_pending_n = 0;
    app->ir_timeout = false;
    app->ir_live = !furi_hal_infrared_is_busy();
    if(!app->ir_live) return;
    furi_stream_buffer_reset(app->ir_stream);
    furi_hal_infrared_async_rx_set_capture_isr_callback(ir_capture_isr, app);
    furi_hal_infrared_async_rx_set_timeout_isr_callback(ir_timeout_isr, app);
    furi_hal_infrared_async_rx_start();
    furi_hal_infrared_async_rx_set_timeout(IR_TIMEOUT_US);
}

static void ir_rx_end(App* app) {
    if(!app->ir_live) return;
    furi_hal_infrared_async_rx_set_timeout_isr_callback(NULL, NULL);
    furi_hal_infrared_async_rx_set_capture_isr_callback(NULL, NULL);
    furi_hal_infrared_async_rx_stop();
    app->ir_live = false;
}

static void ir_start(App* app) {
    app->ir_decoder = infrared_alloc_decoder();
    app->ir_stream = furi_stream_buffer_alloc(sizeof(uint32_t) * 512, sizeof(uint32_t));
    ir_rx_begin(app);
}

static void ir_stop(App* app) {
    ir_rx_end(app);
    furi_stream_buffer_free(app->ir_stream);
    infrared_free_decoder(app->ir_decoder);
    app->ir_stream = NULL;
    app->ir_decoder = NULL;
}

static void ir_scroll(App* app, int dir) {
    uint32_t upp = ir_zooms[app->ir_zoom];
    uint32_t view = GW * upp;
    uint32_t max_off = app->ir_total > view ? app->ir_total - view : 0;
    int64_t off = (int64_t)app->ir_off + (int64_t)dir * 32 * upp;
    app->ir_off = (uint32_t)CLAMP(off, (int64_t)max_off, 0);
}

// Play the captured signal back out of the IR LEDs
static void ir_send(App* app) {
    if(app->ir_n == 0) {
        toast(app, "NOTHING TO SEND");
        return;
    }
    if(!app->ir_live) {
        toast(app, "IR SENSOR BUSY");
        return;
    }
    // the receiver and the transmitter share one peripheral: stop listening first
    ir_rx_end(app);
    if(app->ir_decoded) {
        InfraredMessage msg = app->ir_msg;
        msg.repeat = false;
        infrared_send(&msg, 1);
    } else {
        infrared_send_raw(app->ir_timings, app->ir_n, true);
    }
    ir_rx_begin(app);
    notification_message(app->notifications, &sequence_blink_magenta_10);
    sound_chirp(app, chirp_lock);
    toast(app, "SIGNAL SENT");
}

// Append the captured signal to a file the Infrared app can open
static void ir_save(App* app) {
    if(app->ir_n == 0) {
        toast(app, "NOTHING TO SAVE");
        return;
    }
    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_simply_mkdir(storage, EXT_PATH("infrared"));
    File* file = storage_file_alloc(storage);
    bool ok = false;
    if(storage_file_open(file, IR_SAVE_PATH, FSAM_WRITE, FSOM_OPEN_APPEND)) {
        FuriString* text = furi_string_alloc();
        if(storage_file_size(file) == 0) {
            furi_string_cat_str(text, "Filetype: IR signals file\nVersion: 1\n");
        }
        DateTime dt;
        furi_hal_rtc_get_datetime(&dt);
        furi_string_cat_printf(
            text,
            "#\nname: Scan_%02u%02u_%02u%02u%02u\n",
            dt.month,
            dt.day,
            dt.hour,
            dt.minute,
            dt.second);
        if(app->ir_decoded) {
            uint32_t a = app->ir_msg.address;
            uint32_t c = app->ir_msg.command;
            furi_string_cat_printf(
                text,
                "type: parsed\nprotocol: %s\n"
                "address: %02lX %02lX %02lX %02lX\n"
                "command: %02lX %02lX %02lX %02lX\n",
                infrared_get_protocol_name(app->ir_msg.protocol),
                a & 0xFF,
                (a >> 8) & 0xFF,
                (a >> 16) & 0xFF,
                (a >> 24) & 0xFF,
                c & 0xFF,
                (c >> 8) & 0xFF,
                (c >> 16) & 0xFF,
                (c >> 24) & 0xFF);
        } else {
            furi_string_cat_str(text, "type: raw\nfrequency: 38000\nduty_cycle: 0.330000\ndata:");
            for(size_t i = 0; i < app->ir_n; i++) {
                furi_string_cat_printf(text, " %lu", app->ir_timings[i]);
            }
            furi_string_cat_str(text, "\n");
        }
        size_t len = furi_string_size(text);
        ok = storage_file_write(file, furi_string_get_cstr(text), len) == len;
        furi_string_free(text);
    }
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
    toast(app, ok ? "SAVED TO SD CARD" : "SAVE FAILED");
}

// ---------------------------------------------------------------- reader fields

static void emf_start(App* app) {
    memset(app->emf_hist, 0, sizeof(app->emf_hist));
    app->nfc_field = false;
    app->lf_field = false;
    // The NFC chip is only reachable while we hold it. Leave it in low-power mode:
    // waking it claims a timer that the LF field counter also needs.
    app->nfc_held = furi_hal_nfc_acquire() == FuriHalNfcErrorNone;
    if(app->nfc_held) furi_hal_nfc_field_detect_start();
    furi_hal_rfid_field_detect_start();
}

static void emf_stop(App* app) {
    furi_hal_rfid_field_detect_stop();
    if(app->nfc_held) {
        furi_hal_nfc_field_detect_stop();
        furi_hal_nfc_release();
        app->nfc_held = false;
    }
}

static void emf_step(App* app) {
    uint32_t freq = 0;
    bool lf = furi_hal_rfid_field_is_present(&freq);
    bool nfc = app->nfc_held && furi_hal_nfc_field_is_present();

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    bool was = app->nfc_field || app->lf_field;
    app->lf_field = lf;
    app->nfc_field = nfc;
    if(lf) app->lf_freq = freq;
    memmove(app->emf_hist, app->emf_hist + 1, GW - 1);
    app->emf_hist[GW - 1] = (nfc ? 1 : 0) | (lf ? 2 : 0);
    app->contact = nfc ? 30 : lf ? 14 : 0;
    furi_mutex_release(app->mutex);

    if((nfc || lf) && !was) notification_message(app->notifications, &sequence_blink_cyan_10);
}

// ---------------------------------------------------------------- NFC tags

static void tag_scanner_callback(NfcScannerEvent event, void* context) {
    App* app = context;
    if(event.type != NfcScannerEventTypeDetected) return;
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    if(!app->tag_event) {
        app->tag_pending_n = MIN(event.data.protocol_num, (size_t)TAG_MAX);
        memcpy(app->tag_pending, event.data.protocols, app->tag_pending_n * sizeof(NfcProtocol));
        app->tag_event = true;
    }
    furi_mutex_release(app->mutex);
}

// Runs in the NFC worker thread once the tag has been activated
static NfcCommand tag_poller_callback(NfcGenericEvent event, void* context) {
    App* app = context;
    bool ready = false;
    switch(event.protocol) {
    case NfcProtocolIso14443_3a:
        ready = ((Iso14443_3aPollerEvent*)event.event_data)->type ==
                Iso14443_3aPollerEventTypeReady;
        break;
    case NfcProtocolIso14443_3b:
        ready = ((Iso14443_3bPollerEvent*)event.event_data)->type ==
                Iso14443_3bPollerEventTypeReady;
        break;
    case NfcProtocolIso15693_3:
        ready = ((Iso15693_3PollerEvent*)event.event_data)->type == Iso15693_3PollerEventTypeReady;
        break;
    case NfcProtocolFelica:
        ready = ((FelicaPollerEvent*)event.event_data)->type != FelicaPollerEventTypeError;
        break;
    case NfcProtocolSt25tb:
        ready = ((St25tbPollerEvent*)event.event_data)->type == St25tbPollerEventTypeReady;
        break;
    default:
        break;
    }
    if(ready) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        nfc_device_set_data(app->tag_device, event.protocol, nfc_poller_get_data(app->tag_poller));
        size_t len = 0;
        const uint8_t* uid = nfc_device_get_uid(app->tag_device, &len);
        len = MIN(len, (size_t)TAG_UID_MAX);
        if(uid) memcpy(app->tag_uid, uid, len);
        app->tag_uid_len = uid ? len : 0;
        furi_mutex_release(app->mutex);
    }
    app->tag_poll_done = true;
    return NfcCommandStop;
}

static bool tag_can_poll(NfcProtocol protocol) {
    return protocol == NfcProtocolIso14443_3a || protocol == NfcProtocolIso14443_3b ||
           protocol == NfcProtocolIso15693_3 || protocol == NfcProtocolFelica ||
           protocol == NfcProtocolSt25tb;
}

static void tag_scan_begin(App* app) {
    nfc_scanner_start(app->scanner, tag_scanner_callback, app);
    app->tag_state = TagScan;
    app->tag_scan_start = furi_get_tick();
}

static void tag_start(App* app) {
    app->nfc = nfc_alloc();
    app->scanner = nfc_scanner_alloc(app->nfc);
    app->tag_device = nfc_device_alloc();
    app->tag_n = 0;
    app->tag_uid_len = 0;
    app->tag_present = false;
    app->tag_event = false;
    tag_scan_begin(app);
}

static void tag_stop(App* app) {
    if(app->tag_state == TagScan) {
        nfc_scanner_stop(app->scanner);
    } else if(app->tag_state == TagPoll) {
        nfc_poller_stop(app->tag_poller);
        nfc_poller_free(app->tag_poller);
        app->tag_poller = NULL;
    }
    app->tag_state = TagRest;
    nfc_scanner_free(app->scanner);
    nfc_device_free(app->tag_device);
    nfc_free(app->nfc);
    app->scanner = NULL;
    app->tag_device = NULL;
    app->nfc = NULL;
}

static void tag_step(App* app) {
    uint32_t now = furi_get_tick();
    switch(app->tag_state) {
    case TagScan: {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        bool event = app->tag_event;
        furi_mutex_release(app->mutex);
        if(!event) {
            // nothing answered for a while: the tag has gone
            if(app->tag_present && now - app->tag_scan_start > 1500) app->tag_present = false;
            break;
        }
        // the scanner must be stopped from outside its own callback
        nfc_scanner_stop(app->scanner);
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        bool fresh = !app->tag_present;
        memcpy(app->tag_protocols, app->tag_pending, sizeof(app->tag_protocols));
        app->tag_n = app->tag_pending_n;
        app->tag_present = true;
        app->tag_event = false;
        if(fresh) app->tag_uid_len = 0;
        furi_mutex_release(app->mutex);
        if(fresh) {
            sound_chirp(app, chirp_contact);
            notification_message(app->notifications, &sequence_single_vibro);
            notification_message(app->notifications, &sequence_blink_green_10);
        }
        // the UID lives in the base protocol underneath whatever was detected
        NfcProtocol base = app->tag_protocols[0];
        for(NfcProtocol parent = nfc_protocol_get_parent(base); parent != NfcProtocolInvalid;
            parent = nfc_protocol_get_parent(base)) {
            base = parent;
        }
        if(app->tag_n && tag_can_poll(base)) {
            app->tag_poll_done = false;
            app->tag_poller = nfc_poller_alloc(app->nfc, base);
            nfc_poller_start(app->tag_poller, tag_poller_callback, app);
            app->tag_state = TagPoll;
            app->tag_deadline = now + 600;
        } else {
            app->tag_state = TagRest;
            app->tag_deadline = now + 700;
        }
        break;
    }
    case TagPoll:
        if(app->tag_poll_done || tick_reached(now, app->tag_deadline)) {
            nfc_poller_stop(app->tag_poller);
            nfc_poller_free(app->tag_poller);
            app->tag_poller = NULL;
            app->tag_state = TagRest;
            app->tag_deadline = now + 700;
        }
        break;
    case TagRest:
        if(tick_reached(now, app->tag_deadline)) tag_scan_begin(app);
        break;
    }
}

// ---------------------------------------------------------------- RFID and iButton keys

static void key_rfid_callback(LFRFIDWorkerReadResult result, ProtocolId protocol, void* context) {
    App* app = context;
    if(result == LFRFIDWorkerReadDone) {
        app->key_protocol = protocol;
        app->key_event = true;
    }
}

static void key_ibutton_callback(void* context) {
    App* app = context;
    app->key_event = true;
}

static void key_read_begin(App* app) {
    app->key_event = false;
    if(app->key_kind == KeyRfid) {
        lfrfid_worker_read_start(app->key_rfid, LFRFIDWorkerReadTypeAuto, key_rfid_callback, app);
    } else {
        ibutton_key_reset(app->key_ib_key);
        ibutton_worker_read_start(app->key_ib_worker, app->key_ib_key);
    }
    app->key_reading = true;
}

static void key_start(App* app) {
    app->key_name[0] = 0;
    set_lines(app->key_lines, 3, "");
    if(app->key_kind == KeyRfid) {
        app->key_dict = protocol_dict_alloc(lfrfid_protocols, LFRFIDProtocolMax);
        app->key_rfid = lfrfid_worker_alloc(app->key_dict);
        lfrfid_worker_start_thread(app->key_rfid);
    } else {
        app->key_ib_protocols = ibutton_protocols_alloc();
        app->key_ib_key =
            ibutton_key_alloc(ibutton_protocols_get_max_data_size(app->key_ib_protocols));
        app->key_ib_worker = ibutton_worker_alloc(app->key_ib_protocols);
        ibutton_worker_start_thread(app->key_ib_worker);
        ibutton_worker_read_set_callback(app->key_ib_worker, key_ibutton_callback, app);
    }
    key_read_begin(app);
}

static void key_stop(App* app) {
    if(app->key_kind == KeyRfid) {
        lfrfid_worker_stop(app->key_rfid);
        lfrfid_worker_stop_thread(app->key_rfid);
        lfrfid_worker_free(app->key_rfid);
        protocol_dict_free(app->key_dict);
        app->key_rfid = NULL;
        app->key_dict = NULL;
    } else {
        ibutton_worker_stop(app->key_ib_worker);
        ibutton_worker_stop_thread(app->key_ib_worker);
        ibutton_worker_free(app->key_ib_worker);
        ibutton_key_free(app->key_ib_key);
        ibutton_protocols_free(app->key_ib_protocols);
        app->key_ib_worker = NULL;
        app->key_ib_key = NULL;
        app->key_ib_protocols = NULL;
    }
    app->key_reading = false;
}

static void key_step(App* app) {
    uint32_t now = furi_get_tick();
    if(!app->key_reading) {
        if(tick_reached(now, app->key_restart_at)) key_read_begin(app);
        return;
    }
    if(!app->key_event) return;

    // a key was read: stop the worker, describe the key, then rest before reading again
    FuriString* text = furi_string_alloc();
    const char* name;
    if(app->key_kind == KeyRfid) {
        lfrfid_worker_stop(app->key_rfid);
        name = protocol_dict_get_name(app->key_dict, app->key_protocol);
        protocol_dict_render_brief_data(app->key_dict, text, app->key_protocol);
    } else {
        ibutton_worker_stop(app->key_ib_worker);
        name = ibutton_protocols_get_name(
            app->key_ib_protocols, ibutton_key_get_protocol_id(app->key_ib_key));
        ibutton_protocols_render_brief_data(app->key_ib_protocols, app->key_ib_key, text);
    }
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    strlcpy(app->key_name, name ? name : "Unknown", sizeof(app->key_name));
    set_lines(app->key_lines, 3, furi_string_get_cstr(text));
    app->key_count++;
    furi_mutex_release(app->mutex);
    furi_string_free(text);

    app->key_reading = false;
    app->key_restart_at = now + 1500;
    sound_chirp(app, chirp_contact);
    notification_message(app->notifications, &sequence_single_vibro);
    notification_message(app->notifications, &sequence_blink_green_10);
}

// ---------------------------------------------------------------- GPIO probe

static void pin_select(App* app) {
    furi_hal_gpio_init(probe_pins[app->pin].pin, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
    app->pin_fresh = true;
}

static void pin_start(App* app) {
    app->adc = furi_hal_adc_acquire();
    furi_hal_adc_configure_ex(
        app->adc,
        FuriHalAdcScale2500,
        FuriHalAdcClockSync64,
        FuriHalAdcOversample64,
        FuriHalAdcSamplingtime247_5);
    pin_select(app);
}

static void pin_stop(App* app) {
    furi_hal_adc_release(app->adc);
    app->adc = NULL;
}

static void pin_step(App* app) {
    uint16_t raw = furi_hal_adc_read(app->adc, probe_pins[app->pin].channel);
    float mv = furi_hal_adc_convert_to_voltage(app->adc, raw);
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    app->pin_mv = (uint16_t)CLAMP(mv, 9999.0f, 0.0f);
    if(app->pin_fresh) {
        // start the trace flat at the first reading rather than at zero
        for(size_t i = 0; i < GW; i++)
            app->pin_hist[i] = app->pin_mv;
        app->pin_fresh = false;
    }
    memmove(app->pin_hist, app->pin_hist + 1, sizeof(app->pin_hist) - sizeof(uint16_t));
    app->pin_hist[GW - 1] = app->pin_mv;
    furi_mutex_release(app->mutex);
}

// ---------------------------------------------------------------- power

static void pwr_read(App* app) {
    float volts = furi_hal_power_get_battery_voltage(FuriHalPowerICFuelGauge);
    float amps = furi_hal_power_get_battery_current(FuriHalPowerICFuelGauge);
    float temp = furi_hal_power_get_battery_temperature(FuriHalPowerICFuelGauge);
    uint8_t pct = furi_hal_power_get_pct();
    bool charging = furi_hal_power_is_charging();

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    app->pwr_mv = (int16_t)(volts * 1000.0f);
    app->pwr_ma = (int16_t)(amps * 1000.0f);
    app->pwr_temp10 = (int16_t)(temp * 10.0f);
    app->pwr_pct = pct;
    app->pwr_charging = charging;
    memmove(app->pwr_hist, app->pwr_hist + 1, sizeof(app->pwr_hist) - sizeof(int16_t));
    app->pwr_hist[GW - 1] = app->pwr_ma;
    furi_mutex_release(app->mutex);
}

static void pwr_start(App* app) {
    memset(app->pwr_hist, 0, sizeof(app->pwr_hist));
    pwr_read(app);
}

static void pwr_step(App* app) {
    // the fuel gauge is on a slow bus: four readings a second is plenty
    if(app->frame % 5 == 0) pwr_read(app);
}

// ---------------------------------------------------------------- full sweep

static const char* const sweep_idle[SweepCount] = {"RF  --", "IR  --", "EMF --", "TAG --"};

static void sweep_phase_begin(App* app) {
    app->sw_ticks = 0;
    switch(app->sw_phase) {
    case SweepRf:
        rf_radio_start(app);
        app->band = 0;
        app->sw_rf_best = -127;
        break;
    case SweepIr:
        ir_start(app);
        app->sw_ir_count = app->ir_count;
        break;
    case SweepEmf:
        emf_start(app);
        app->sw_emf = 0;
        break;
    case SweepTag:
        tag_start(app);
        break;
    default:
        break;
    }
    app->sw_active = true;
}

static void sweep_phase_end(App* app) {
    if(!app->sw_active) return;
    switch(app->sw_phase) {
    case SweepRf:
        rf_radio_stop();
        break;
    case SweepIr:
        ir_stop(app);
        break;
    case SweepEmf:
        emf_stop(app);
        break;
    case SweepTag:
        tag_stop(app);
        break;
    default:
        break;
    }
    app->sw_active = false;
    app->contact = 0;
}

static void sweep_report(App* app, bool hit, const char* text) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    bool fresh = hit && !app->sw_hit[app->sw_phase];
    strlcpy(app->sw_text[app->sw_phase], text, LINE_LEN);
    app->sw_hit[app->sw_phase] = hit;
    furi_mutex_release(app->mutex);
    if(fresh) sound_chirp(app, chirp_contact);

    sweep_phase_end(app);
    app->sw_phase = (app->sw_phase + 1) % SweepCount;
    if(app->sw_phase == SweepRf) app->sw_pass++;
    sweep_phase_begin(app);
}

static void sweep_start(App* app) {
    app->sw_saved_band = app->band;
    app->sw_phase = SweepRf;
    app->sw_pass = 1;
    for(size_t i = 0; i < SweepCount; i++) {
        strlcpy(app->sw_text[i], sweep_idle[i], LINE_LEN);
        app->sw_hit[i] = false;
    }
    sweep_phase_begin(app);
}

static void sweep_stop(App* app) {
    sweep_phase_end(app);
    app->band = app->sw_saved_band;
}

static void sweep_step(App* app) {
    char text[LINE_LEN];
    app->sw_ticks++;
    switch(app->sw_phase) {
    case SweepRf:
        rf_step_scan(app);
        if(app->scan_i != 0) break;
        // one band done: keep the strongest signal across all three
        if(app->rssi[app->max_i] > app->sw_rf_best) {
            app->sw_rf_best = app->rssi[app->max_i];
            app->sw_rf_freq = rf_point_freq(app->band, app->max_i);
        }
        if(app->band + 1 < (int)BAND_COUNT) {
            app->band++;
            break;
        }
        if(app->sw_rf_best > RF_CONTACT) {
            snprintf(
                text,
                sizeof(text),
                "RF  %lu.%02lu %ddBm",
                app->sw_rf_freq / 1000000,
                (app->sw_rf_freq % 1000000) / 10000,
                app->sw_rf_best);
        } else {
            snprintf(text, sizeof(text), "RF  quiet (%ddBm)", app->sw_rf_best);
        }
        sweep_report(app, app->sw_rf_best > RF_CONTACT, text);
        break;
    case SweepIr:
        ir_step(app);
        if(app->ir_count != app->sw_ir_count) {
            if(app->ir_decoded) {
                snprintf(
                    text,
                    sizeof(text),
                    "IR  %s C:%lX",
                    infrared_get_protocol_name(app->ir_msg.protocol),
                    app->ir_msg.command);
            } else {
                snprintf(text, sizeof(text), "IR  unknown signal");
            }
            sweep_report(app, true, text);
        } else if(app->sw_ticks >= 30) {
            sweep_report(app, false, "IR  none");
        }
        break;
    case SweepEmf:
        emf_step(app);
        if(app->nfc_field) app->sw_emf |= 1;
        if(app->lf_field) app->sw_emf |= 2;
        if(app->sw_ticks >= 12) {
            static const char* const names[] = {
                "EMF no reader field",
                "EMF NFC reader",
                "EMF LF reader",
                "EMF NFC + LF reader",
            };
            sweep_report(app, app->sw_emf != 0, names[app->sw_emf]);
        }
        break;
    case SweepTag:
        tag_step(app);
        if(app->tag_n && app->tag_state == TagRest) {
            snprintf(
                text, sizeof(text), "TAG %s", nfc_device_get_protocol_name(app->tag_protocols[0]));
            sweep_report(app, true, text);
        } else if(app->sw_ticks >= 30 && app->tag_state == TagScan) {
            sweep_report(app, false, "TAG none");
        }
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------- modes

static void mode_start(App* app) {
    app->contact = 0;
    switch(app->mode) {
    case ModeRf:
        rf_start(app);
        break;
    case ModeIr:
        ir_start(app);
        break;
    case ModeEmf:
        emf_start(app);
        break;
    case ModeTag:
        tag_start(app);
        break;
    case ModeKey:
        key_start(app);
        break;
    case ModePin:
        pin_start(app);
        break;
    case ModePwr:
        pwr_start(app);
        break;
    case ModeAll:
        sweep_start(app);
        break;
    default:
        break;
    }
}

static void mode_stop(App* app) {
    switch(app->mode) {
    case ModeRf:
        rf_stop(app);
        break;
    case ModeIr:
        ir_stop(app);
        break;
    case ModeEmf:
        emf_stop(app);
        break;
    case ModeTag:
        tag_stop(app);
        break;
    case ModeKey:
        key_stop(app);
        break;
    case ModePin:
        pin_stop(app);
        break;
    case ModeAll:
        sweep_stop(app);
        break;
    default:
        break;
    }
    app->contact = 0;
}

static void mode_step(App* app) {
    switch(app->mode) {
    case ModeRf:
        if(app->track) {
            rf_step_track(app);
        } else {
            rf_step_scan(app);
        }
        break;
    case ModeIr:
        ir_step(app);
        break;
    case ModeEmf:
        emf_step(app);
        break;
    case ModeTag:
        tag_step(app);
        break;
    case ModeKey:
        key_step(app);
        break;
    case ModePin:
        pin_step(app);
        break;
    case ModePwr:
        pwr_step(app);
        break;
    case ModeAll:
        sweep_step(app);
        break;
    default:
        break;
    }
}

// The Sub-GHz sweep paces itself; every other sensor ticks at 20 Hz
static bool mode_self_paced(App* app) {
    if(app->boot) return false;
    return app->mode == ModeRf || (app->mode == ModeAll && app->sw_phase == SweepRf);
}

static void mode_switch(App* app, int dir) {
    mode_stop(app);
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    app->mode = (Mode)((app->mode + ModeCount + dir) % ModeCount);
    furi_mutex_release(app->mutex);
    mode_start(app);
    sound_chirp(app, chirp_mode);
}

// ---------------------------------------------------------------- menu

static size_t menu_items(App* app, MenuItem* items) {
    size_t n = 0;
    if(app->mode == ModeRf) {
        items[n++] = MenuRfView;
        items[n++] = MenuRfMod;
    } else if(app->mode == ModeIr) {
        items[n++] = MenuIrSend;
        items[n++] = MenuIrSave;
    }
    items[n++] = MenuSound;
    return n;
}

static void menu_label(App* app, MenuItem item, char* buf, size_t size) {
    switch(item) {
    case MenuRfView:
        snprintf(buf, size, "View: %s", app->waterfall ? "Waterfall" : "Spectrum");
        break;
    case MenuRfMod:
        snprintf(buf, size, "Track as: %s", modulations[app->mod].name);
        break;
    case MenuIrSend:
        snprintf(buf, size, "Send signal");
        break;
    case MenuIrSave:
        snprintf(buf, size, "Save to SD card");
        break;
    case MenuSound:
        snprintf(buf, size, "Sound: %s", app->sound ? "On" : "Off");
        break;
    }
}

// Returns true if the menu should close
static bool menu_select(App* app, MenuItem item) {
    switch(item) {
    case MenuRfView:
        app->waterfall = !app->waterfall;
        return false;
    case MenuRfMod:
        app->mod = (app->mod + 1) % MOD_COUNT;
        if(app->track) app->retune = true;
        return false;
    case MenuIrSend:
        ir_send(app);
        return true;
    case MenuIrSave:
        ir_save(app);
        return true;
    case MenuSound:
        app->sound = !app->sound;
        if(app->sound) {
            sound_chirp(app, chirp_mode);
        } else {
            app->chirp = NULL;
            sound_release(app);
        }
        return false;
    }
    return true;
}

// ---------------------------------------------------------------- drawing

static void draw_frame(Canvas* canvas, App* app, const char* title, const char* status) {
    canvas_set_font(canvas, FontSecondary);
    int top = CLAMP((int)app->mode - 1, ModeCount - TAB_ROWS, 0);
    for(int row = 0; row < TAB_ROWS; row++) {
        int i = top + row;
        int y = row * 16;
        if(i == (int)app->mode) {
            canvas_draw_rbox(canvas, 0, y, TAB_W + 2, 15, 3);
            canvas_set_color(canvas, ColorWhite);
            canvas_draw_str_aligned(
                canvas, TAB_W / 2, y + 4, AlignCenter, AlignTop, mode_names[i]);
            canvas_set_color(canvas, ColorBlack);
        } else {
            canvas_draw_rframe(canvas, 0, y, TAB_W, 15, 3);
            canvas_draw_str_aligned(
                canvas, TAB_W / 2, y + 4, AlignCenter, AlignTop, mode_names[i]);
        }
    }

    canvas_draw_str(canvas, GX, 8, title);
    // three little blocks that chase while the sensor is live
    for(int i = 0; i < 3; i++) {
        int x = 119 + i * 3;
        if((int)(app->frame / 2 % 3) == i) {
            canvas_draw_box(canvas, x, 2, 2, 6);
        } else {
            canvas_draw_box(canvas, x, 6, 2, 2);
        }
    }
    canvas_draw_str_aligned(canvas, 116, 8, AlignRight, AlignBottom, status);
    canvas_draw_box(canvas, GX, 10, GW, 2);
}

static void draw_footer(Canvas* canvas, App* app, const char* text) {
    if(app->toast) text = app->toast_text;
    canvas_draw_str(canvas, GX, 63, text);
}

// Text on a white patch so it stays readable over a graph
static void draw_label(Canvas* canvas, int x, int y, const char* text) {
    if(!text[0]) return;
    int w = canvas_string_width(canvas, text);
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, x, y - 8, w + 2, 9);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_str(canvas, x + 1, y - 1, text);
}

static void draw_rf(Canvas* canvas, App* app) {
    char title[24];
    char foot[32];
    if(!app->track) {
        snprintf(title, sizeof(title), "RF %s", bands[app->band].name);
        draw_frame(canvas, app, title, app->waterfall ? "FALL" : "SCAN");
        if(app->waterfall) {
            for(int r = 0; r < GH; r++) {
                const uint8_t* row = app->fall[r];
                int y = GY + 1 + r;
                for(int x = 0; x < RF_POINTS; x++) {
                    uint8_t lvl = row[x];
                    if(lvl == 3 || (lvl == 2 && ((x + r) & 1)) ||
                       (lvl == 1 && ((x + 2 * r) & 3) == 0)) {
                        canvas_draw_dot(canvas, GX + x, y);
                    }
                }
            }
        } else {
            for(int i = 0; i < RF_POINTS; i++) {
                int h = rf_level(app->rssi[i]);
                if(h) canvas_draw_line(canvas, GX + i, GB, GX + i, GB - h);
                int p = rf_level(app->peak[i]);
                if(p > h) canvas_draw_dot(canvas, GX + i, GB - p);
            }
            // sweep cursor along the top, marker over the strongest point
            canvas_draw_line(canvas, GX + app->scan_i, GY, GX + app->scan_i, GY + 1);
            int mx = GX + app->max_i;
            int my = GB - rf_level(app->rssi[app->max_i]) - 3;
            if(my < GY + 2) my = GY + 2;
            canvas_draw_line(canvas, mx - 2, my - 2, mx + 2, my - 2);
            canvas_draw_line(canvas, mx - 1, my - 1, mx + 1, my - 1);
            canvas_draw_dot(canvas, mx, my);
        }

        uint32_t f = rf_point_freq(app->band, app->max_i);
        snprintf(
            foot,
            sizeof(foot),
            "PEAK %lu.%02lu  %ddBm",
            f / 1000000,
            (f % 1000000) / 10000,
            app->rssi[app->max_i]);
        draw_footer(canvas, app, foot);
    } else {
        uint32_t f = app->track_freq;
        snprintf(title, sizeof(title), "RF %lu.%02lu", f / 1000000, (f % 1000000) / 10000);
        draw_frame(canvas, app, title, modulations[app->mod].name);
        int prev = GB - rf_level(app->hist[0]);
        for(int i = 1; i < RF_POINTS; i++) {
            int y = GB - rf_level(app->hist[i]);
            canvas_draw_line(canvas, GX + i - 1, prev, GX + i, y);
            prev = y;
        }
        // dotted line at the contact threshold
        int ty = GB - rf_level(RF_CONTACT);
        for(int x = GX; x < GX + GW; x += 4)
            canvas_draw_dot(canvas, x, ty);
        // what the decoders made of the last transmission
        draw_label(canvas, GX, GY + 10, app->rf_ident[0]);
        draw_label(canvas, GX, GY + 19, app->rf_ident[1]);

        snprintf(foot, sizeof(foot), "%ddBm", app->track_rssi);
        draw_footer(canvas, app, foot);
        if(!app->toast) {
            int w = rf_level(app->track_rssi) * 56 / GH;
            canvas_draw_frame(canvas, 70, 57, 58, 6);
            canvas_draw_box(canvas, 71, 58, w, 4);
        }
    }
    canvas_draw_line(canvas, GX, GB + 1, GX + GW - 1, GB + 1);
}

static void draw_ir(Canvas* canvas, App* app) {
    char title[24];
    char status[16];
    char foot[40];
    snprintf(title, sizeof(title), "IR #%lu", app->ir_count);
    uint32_t upp = ir_zooms[app->ir_zoom];
    snprintf(status, sizeof(status), "%luus", upp);
    draw_frame(canvas, app, title, status);

    if(app->ir_n == 0) {
        canvas_draw_str_aligned(
            canvas, GX + GW / 2, 28, AlignCenter, AlignBottom, "AWAITING SIGNAL");
        canvas_draw_str_aligned(
            canvas, GX + GW / 2, 40, AlignCenter, AlignBottom, "Aim a remote at me");
        snprintf(
            foot,
            sizeof(foot),
            app->ir_live ? "%lu flashes seen" : "IR SENSOR BUSY",
            app->ir_edges / 2);
        draw_footer(canvas, app, foot);
        return;
    }

    const int y_hi = GY + 6;
    const int y_lo = GB - 6;
    int64_t t = 0;
    bool level = true;
    int last_x = -1;
    for(size_t i = 0; i < app->ir_n; i++) {
        int64_t start = t - app->ir_off;
        t += app->ir_timings[i];
        int64_t end = t - app->ir_off;
        int64_t xs = start / (int64_t)upp;
        int64_t xe = end / (int64_t)upp;
        if(end >= 0 && xs < GW) {
            if(start >= 0) canvas_draw_line(canvas, GX + xs, y_hi, GX + xs, y_lo);
            int x0 = MAX(xs, 0);
            int x1 = MIN(xe, GW - 1);
            int y = level ? y_hi : y_lo;
            canvas_draw_line(canvas, GX + x0, y, GX + x1, y);
            last_x = x1;
        }
        level = !level;
        if(xs >= GW) break;
    }
    // idle line after the last edge
    if(last_x >= 0 && last_x < GW - 1 && t - app->ir_off < (int64_t)GW * upp) {
        canvas_draw_line(canvas, GX + last_x, y_hi, GX + last_x, y_lo);
        canvas_draw_line(canvas, GX + last_x, y_lo, GX + GW - 1, y_lo);
    }
    // scroll position along the bottom of the graph
    uint32_t view = GW * upp;
    if(app->ir_total > view) {
        int bar_w = MAX((int)((uint64_t)GW * view / app->ir_total), 4);
        int bar_x = (int)((uint64_t)(GW - bar_w) * app->ir_off / (app->ir_total - view));
        canvas_draw_box(canvas, GX + bar_x, GB, bar_w, 2);
    }

    if(app->ir_decoded) {
        snprintf(
            foot,
            sizeof(foot),
            "%s A:%lX C:%lX",
            infrared_get_protocol_name(app->ir_msg.protocol),
            app->ir_msg.address,
            app->ir_msg.command);
    } else {
        snprintf(foot, sizeof(foot), "UNKNOWN  %u edges", (unsigned)app->ir_n);
    }
    draw_footer(canvas, app, foot);
}

static void draw_emf_row(Canvas* canvas, int y, const char* label, bool on, const char* value) {
    canvas_draw_str(canvas, GX, y + 8, label);
    if(on) {
        canvas_draw_rbox(canvas, 86, y, 42, 10, 2);
        canvas_set_color(canvas, ColorWhite);
        canvas_draw_str_aligned(canvas, 107, y + 2, AlignCenter, AlignTop, value);
        canvas_set_color(canvas, ColorBlack);
    } else {
        canvas_draw_rframe(canvas, 86, y, 42, 10, 2);
        canvas_draw_str_aligned(canvas, 107, y + 2, AlignCenter, AlignTop, "- -");
    }
}

static void draw_emf(Canvas* canvas, App* app) {
    bool any = app->nfc_field || app->lf_field;
    draw_frame(canvas, app, "EMF FIELDS", any ? "CONTACT" : "SCAN");

    char lf[16];
    snprintf(lf, sizeof(lf), "%lu.%lukHz", app->lf_freq / 1000, (app->lf_freq % 1000) / 100);
    draw_emf_row(canvas, GY + 1, "NFC 13.56M", app->nfc_field, "FIELD");
    draw_emf_row(canvas, GY + 13, "RFID LF", app->lf_field, lf);

    // presence history: NFC on the upper track, LF on the lower
    for(int i = 0; i < GW; i++) {
        if(app->emf_hist[i] & 1) canvas_draw_line(canvas, GX + i, 41, GX + i, 45);
        if(app->emf_hist[i] & 2) canvas_draw_line(canvas, GX + i, 48, GX + i, 52);
        if((i & 3) == 0) {
            canvas_draw_dot(canvas, GX + i, 46);
            canvas_draw_dot(canvas, GX + i, 53);
        }
    }
    draw_footer(canvas, app, any ? "READER FIELD NEARBY" : "NO READER FIELD");
}

static void draw_scanning(Canvas* canvas, App* app) {
    int cx = GX + GW / 2;
    int cy = GY + GH / 2;
    int r = 4 + (app->frame / 2 % 4) * 5;
    canvas_draw_disc(canvas, cx, cy, 2);
    canvas_draw_circle(canvas, cx, cy, r);
}

static void draw_tag(Canvas* canvas, App* app) {
    draw_frame(
        canvas,
        app,
        "NFC TAG",
        app->tag_present ? "CONTACT" :
        app->tag_n       ? "LAST" :
                           "SCAN");
    if(app->tag_n == 0) {
        draw_scanning(canvas, app);
        draw_footer(canvas, app, "Hold a tag to my back");
        return;
    }
    for(size_t i = 0; i < app->tag_n; i++) {
        int y = GY + 9 + i * 10;
        canvas_draw_box(canvas, GX, y - 6, 3, 5);
        canvas_draw_str(canvas, GX + 6, y, nfc_device_get_protocol_name(app->tag_protocols[i]));
    }
    char foot[40] = "UID not read";
    if(app->tag_uid_len) {
        size_t pos = snprintf(foot, sizeof(foot), "UID ");
        for(size_t i = 0; i < app->tag_uid_len && pos + 3 < sizeof(foot); i++) {
            pos += snprintf(foot + pos, sizeof(foot) - pos, "%02X", app->tag_uid[i]);
        }
    }
    draw_footer(canvas, app, foot);
}

static void draw_key(Canvas* canvas, App* app) {
    bool rfid = app->key_kind == KeyRfid;
    draw_frame(
        canvas, app, rfid ? "KEY RFID 125k" : "KEY iButton", app->key_name[0] ? "READ" : "SCAN");
    if(!app->key_name[0]) {
        draw_scanning(canvas, app);
        draw_footer(canvas, app, rfid ? "Hold a fob to my back" : "Touch key to the pins");
        return;
    }
    canvas_draw_box(canvas, GX, GY + 3, 3, 5);
    canvas_draw_str(canvas, GX + 6, GY + 9, app->key_name);
    for(int i = 0; i < 3; i++) {
        canvas_draw_str(canvas, GX, GY + 19 + i * 10, app->key_lines[i]);
    }
    char foot[32];
    snprintf(foot, sizeof(foot), "%lu READ", app->key_count);
    draw_footer(canvas, app, foot);
}

static void draw_pin(Canvas* canvas, App* app) {
    char title[24];
    char status[16];
    char foot[40];
    snprintf(title, sizeof(title), "PIN %s", probe_pins[app->pin].name);
    snprintf(status, sizeof(status), "%u.%02uV", app->pin_mv / 1000, (app->pin_mv % 1000) / 10);
    draw_frame(canvas, app, title, status);

    uint16_t lo = 0xFFFF;
    uint16_t hi = 0;
    int prev = 0;
    for(int i = 0; i < GW; i++) {
        uint16_t mv = app->pin_hist[i];
        if(mv < lo) lo = mv;
        if(mv > hi) hi = mv;
        int y = GB - MIN((int)mv, PIN_FULL_MV) * GH / PIN_FULL_MV;
        if(i) canvas_draw_line(canvas, GX + i - 1, prev, GX + i, y);
        prev = y;
    }
    // dotted grid lines at each half volt
    for(int mv = 500; mv <= PIN_FULL_MV; mv += 500) {
        int y = GB - mv * GH / PIN_FULL_MV;
        for(int x = GX; x < GX + GW; x += 6)
            canvas_draw_dot(canvas, x, y);
    }
    canvas_draw_line(canvas, GX, GB + 1, GX + GW - 1, GB + 1);

    if(app->pin_mv >= PIN_FULL_MV - 20) {
        snprintf(foot, sizeof(foot), "OVER RANGE (2.5V MAX)");
    } else {
        snprintf(
            foot,
            sizeof(foot),
            "MIN %u.%02u  MAX %u.%02u",
            lo / 1000,
            (lo % 1000) / 10,
            hi / 1000,
            (hi % 1000) / 10);
    }
    draw_footer(canvas, app, foot);
}

static void draw_pwr(Canvas* canvas, App* app) {
    char status[16];
    char line[40];
    snprintf(status, sizeof(status), "%u%%", app->pwr_pct);
    draw_frame(canvas, app, "POWER CELL", status);

    snprintf(line, sizeof(line), "CELL %d.%02dV", app->pwr_mv / 1000, (app->pwr_mv % 1000) / 10);
    canvas_draw_str(canvas, GX, GY + 9, line);
    snprintf(line, sizeof(line), "%d.%dC", app->pwr_temp10 / 10, abs(app->pwr_temp10 % 10));
    canvas_draw_str_aligned(canvas, 127, GY + 9, AlignRight, AlignBottom, line);
    snprintf(
        line, sizeof(line), "%s %dmA", app->pwr_ma >= 0 ? "CHARGE" : "DRAW", abs(app->pwr_ma));
    canvas_draw_str(canvas, GX, GY + 18, line);

    // current history, scaled to the largest value on screen
    const int top = GY + 22;
    int peak = 50;
    for(int i = 0; i < GW; i++) {
        int v = abs(app->pwr_hist[i]);
        if(v > peak) peak = v;
    }
    for(int i = 0; i < GW; i++) {
        int h = abs(app->pwr_hist[i]) * (GB - top) / peak;
        if(h) canvas_draw_line(canvas, GX + i, GB, GX + i, GB - h);
    }
    canvas_draw_line(canvas, GX, GB + 1, GX + GW - 1, GB + 1);

    const char* state = "POWER NOMINAL";
    if(app->pwr_charging) {
        state = app->pwr_pct >= 100 ? "FULLY CHARGED" : "CHARGING";
    } else if(app->pwr_pct < 15) {
        state = "POWER LOW";
    }
    draw_footer(canvas, app, state);
}

static void draw_all(Canvas* canvas, App* app) {
    char status[16];
    snprintf(status, sizeof(status), "#%u", app->sw_pass);
    draw_frame(canvas, app, "FULL SWEEP", status);
    for(int i = 0; i < SweepCount; i++) {
        int y = GY + 9 + i * 10;
        if(i == (int)app->sw_phase) {
            // the sensor being read right now blinks an arrow
            if(app->frame / 3 % 2) canvas_draw_str(canvas, GX, y, ">");
        } else if(app->sw_hit[i]) {
            canvas_draw_box(canvas, GX, y - 6, 3, 5);
        }
        canvas_draw_str(canvas, GX + 6, y, app->sw_text[i]);
    }
    char foot[32];
    switch(app->sw_phase) {
    case SweepRf:
        snprintf(foot, sizeof(foot), "SCANNING RF %s", bands[app->band].name);
        break;
    case SweepIr:
        snprintf(foot, sizeof(foot), "LISTENING FOR IR");
        break;
    case SweepEmf:
        snprintf(foot, sizeof(foot), "SENSING FIELDS");
        break;
    default:
        snprintf(foot, sizeof(foot), "POLLING FOR TAGS");
        break;
    }
    draw_footer(canvas, app, foot);
}

static void draw_menu(Canvas* canvas, App* app) {
    MenuItem items[MENU_MAX];
    size_t n = menu_items(app, items);
    const int x = GX + 3;
    const int w = GW - 6;
    const int h = n * 11 + 3;
    const int y = GY + 2;
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_box(canvas, x - 1, y - 1, w + 2, h + 2);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_rframe(canvas, x, y, w, h, 3);
    char label[32];
    for(size_t i = 0; i < n; i++) {
        int iy = y + 2 + i * 11;
        menu_label(app, items[i], label, sizeof(label));
        if(i == app->menu_sel) {
            canvas_draw_rbox(canvas, x + 2, iy, w - 4, 10, 2);
            canvas_set_color(canvas, ColorWhite);
            canvas_draw_str(canvas, x + 5, iy + 8, label);
            canvas_set_color(canvas, ColorBlack);
        } else {
            canvas_draw_str(canvas, x + 5, iy + 8, label);
        }
    }
}

static void draw_boot(Canvas* canvas, App* app) {
    int p = BOOT_TICKS - app->boot;
    // the frame sweeps in: a bar across the top, then down the left side
    int bar_w = MIN(p * 12, 128);
    canvas_draw_rbox(canvas, 0, 0, bar_w, 9, 3);
    int side_h = CLAMP((p - 4) * 6, 64, 0);
    if(side_h > 6) canvas_draw_rbox(canvas, 0, 0, 14, side_h, 3);
    if(p > 4) canvas_draw_box(canvas, 0, 4, 14, 6);

    canvas_set_font(canvas, FontPrimary);
    if(p > 6) {
        static const char name[] = "TRICORDER";
        char shown[sizeof(name)];
        size_t n = MIN((size_t)(p - 6), sizeof(name) - 1);
        memcpy(shown, name, n);
        shown[n] = 0;
        canvas_draw_str(canvas, 24, 26, shown);
    }
    canvas_set_font(canvas, FontSecondary);
    // each sensor reports in
    int online = CLAMP((p - 12) / 2, ModeCount - 1, 0);
    for(int i = 0; i < online; i++) {
        int x = 24 + (i % 4) * 26;
        int y = 38 + (i / 4) * 10;
        canvas_draw_box(canvas, x, y - 6, 3, 5);
        canvas_draw_str(canvas, x + 5, y, mode_names[i]);
    }
    if(p > 26) canvas_draw_str(canvas, 24, 62, "SENSORS ONLINE");
}

static void draw_callback(Canvas* canvas, void* context) {
    App* app = context;
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    canvas_clear(canvas);
    if(app->boot) {
        draw_boot(canvas, app);
        furi_mutex_release(app->mutex);
        return;
    }
    switch(app->mode) {
    case ModeRf:
        draw_rf(canvas, app);
        break;
    case ModeIr:
        draw_ir(canvas, app);
        break;
    case ModeEmf:
        draw_emf(canvas, app);
        break;
    case ModeTag:
        draw_tag(canvas, app);
        break;
    case ModeKey:
        draw_key(canvas, app);
        break;
    case ModePin:
        draw_pin(canvas, app);
        break;
    case ModePwr:
        draw_pwr(canvas, app);
        break;
    case ModeAll:
        draw_all(canvas, app);
        break;
    default:
        break;
    }
    if(app->menu_open) draw_menu(canvas, app);
    furi_mutex_release(app->mutex);
}

// ---------------------------------------------------------------- input

static void input_callback(InputEvent* event, void* context) {
    FuriMessageQueue* queue = context;
    furi_message_queue_put(queue, event, 0);
}

static void handle_ok(App* app) {
    if(app->mode == ModeRf) {
        if(app->track) {
            rf_leave_track(app);
        } else {
            furi_mutex_acquire(app->mutex, FuriWaitForever);
            rf_enter_track(app);
            furi_mutex_release(app->mutex);
            sound_chirp(app, chirp_lock);
        }
        return;
    }
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    switch(app->mode) {
    case ModeIr:
        app->ir_zoom = (app->ir_zoom + 1) % IR_ZOOM_COUNT;
        ir_scroll(app, 0);
        break;
    case ModeEmf:
        memset(app->emf_hist, 0, sizeof(app->emf_hist));
        break;
    case ModeTag:
        app->tag_n = 0;
        app->tag_uid_len = 0;
        app->tag_present = false;
        break;
    case ModeKey:
        app->key_name[0] = 0;
        break;
    case ModePin:
        app->pin_fresh = true;
        break;
    default:
        break;
    }
    furi_mutex_release(app->mutex);
}

static void handle_side(App* app, int dir, bool repeat) {
    if(app->mode == ModeKey) {
        if(repeat) return;
        // the RFID and iButton readers share a pin, so only one runs at a time
        key_stop(app);
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->key_kind = (app->key_kind + KeyKindCount + dir) % KeyKindCount;
        furi_mutex_release(app->mutex);
        key_start(app);
        return;
    }
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    switch(app->mode) {
    case ModeRf:
        if(app->track) {
            rf_tune(app, dir * (repeat ? 100000 : 10000));
        } else if(!repeat) {
            app->band = (app->band + BAND_COUNT + dir) % BAND_COUNT;
            rf_clear(app);
            app->contact = 0;
        }
        break;
    case ModeIr:
        ir_scroll(app, dir);
        break;
    case ModePin:
        if(!repeat) {
            app->pin = (app->pin + PIN_COUNT + dir) % PIN_COUNT;
            pin_select(app);
        }
        break;
    default:
        break;
    }
    furi_mutex_release(app->mutex);
}

static void handle_menu_input(App* app, const InputEvent* event) {
    if(event->type != InputTypeShort && event->type != InputTypeRepeat) return;
    MenuItem items[MENU_MAX];
    size_t n = menu_items(app, items);
    switch(event->key) {
    case InputKeyUp:
        app->menu_sel = (app->menu_sel + n - 1) % n;
        break;
    case InputKeyDown:
        app->menu_sel = (app->menu_sel + 1) % n;
        break;
    case InputKeyOk:
        if(event->type == InputTypeShort && menu_select(app, items[app->menu_sel])) {
            app->menu_open = false;
        }
        break;
    case InputKeyBack:
        app->menu_open = false;
        break;
    default:
        break;
    }
}

static void handle_input(App* app, const InputEvent* event) {
    if(app->boot) {
        // any key skips the start-up sequence
        if(event->type == InputTypeShort) app->boot = 1;
        return;
    }
    if(app->menu_open) {
        handle_menu_input(app, event);
        return;
    }
    if(event->type == InputTypeLong && event->key == InputKeyOk) {
        app->menu_open = true;
        app->menu_sel = 0;
        return;
    }
    if(event->type != InputTypeShort && event->type != InputTypeRepeat) return;
    bool repeat = event->type == InputTypeRepeat;

    switch(event->key) {
    case InputKeyBack:
        if(repeat) break;
        if(app->mode == ModeRf && app->track) {
            rf_leave_track(app);
        } else {
            app->running = false;
        }
        break;
    case InputKeyUp:
        if(!repeat) mode_switch(app, -1);
        break;
    case InputKeyDown:
        if(!repeat) mode_switch(app, 1);
        break;
    case InputKeyLeft:
        handle_side(app, -1, repeat);
        break;
    case InputKeyRight:
        handle_side(app, 1, repeat);
        break;
    case InputKeyOk:
        if(!repeat) handle_ok(app);
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------- settings

static void settings_load(App* app) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);
    Settings s;
    if(storage_file_open(file, SETTINGS_PATH, FSAM_READ, FSOM_OPEN_EXISTING) &&
       storage_file_read(file, &s, sizeof(s)) == sizeof(s) && s.magic == SETTINGS_MAGIC) {
        app->sound = s.sound != 0;
        app->mode = s.mode < ModeCount ? s.mode : ModeRf;
        app->band = s.band < BAND_COUNT ? s.band : 1;
        app->waterfall = s.waterfall != 0;
        app->mod = s.mod < MOD_COUNT ? s.mod : 0;
        app->ir_zoom = s.ir_zoom < IR_ZOOM_COUNT ? s.ir_zoom : 2;
        app->pin = s.pin < PIN_COUNT ? s.pin : 0;
        app->key_kind = s.key_kind < KeyKindCount ? s.key_kind : KeyRfid;
    }
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

static void settings_save(App* app) {
    Settings s = {
        .magic = SETTINGS_MAGIC,
        .sound = app->sound,
        .mode = app->mode,
        .band = app->band,
        .waterfall = app->waterfall,
        .mod = app->mod,
        .ir_zoom = app->ir_zoom,
        .pin = app->pin,
        .key_kind = app->key_kind,
    };
    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);
    if(storage_file_open(file, SETTINGS_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        storage_file_write(file, &s, sizeof(s));
    }
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
}

// ---------------------------------------------------------------- main

int32_t tricorder_app(void* p) {
    UNUSED(p);
    App* app = malloc(sizeof(App));
    memset(app, 0, sizeof(App));
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    app->running = true;
    app->sound = true;
    app->band = 1;
    app->ir_zoom = 2;
    settings_load(app);
    app->boot = BOOT_TICKS;

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app->queue);
    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, app->view_port, GuiLayerFullscreen);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);
    notification_message(app->notifications, &sequence_display_backlight_enforce_on);

    sound_chirp(app, chirp_open);

    InputEvent event;
    while(app->running) {
        uint32_t wait = mode_self_paced(app) ? 0 : 50;
        while(furi_message_queue_get(app->queue, &event, wait) == FuriStatusOk) {
            handle_input(app, &event);
            wait = 0;
            if(!app->running) break;
        }
        if(!app->running) break;

        if(app->boot) {
            // sensors come up once the start-up sequence has played
            if(--app->boot == 0) mode_start(app);
        } else {
            mode_step(app);
        }
        app->frame++;
        if(app->toast) app->toast--;
        sound_update(app);
        view_port_update(app->view_port);
    }

    if(!app->boot) mode_stop(app);
    sound_release(app);
    settings_save(app);
    notification_message(app->notifications, &sequence_display_backlight_enforce_auto);
    furi_record_close(RECORD_NOTIFICATION);
    gui_remove_view_port(gui, app->view_port);
    furi_record_close(RECORD_GUI);
    view_port_free(app->view_port);
    furi_message_queue_free(app->queue);
    furi_mutex_free(app->mutex);
    free(app);
    return 0;
}
