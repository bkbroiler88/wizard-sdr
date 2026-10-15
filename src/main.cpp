// ============================================================================
//  Signalhound CYD -- Phase 1 skeleton
//
//  Classic CYD 2.8" (ESP32-2432S028R, ESP32-WROOM-32, ILI9341 320x240, XPT2046).
//  Standalone 2.4 GHz spectrum watcher. The ESP-SDR debug-path RX driver port
//  lands in Phase 2 -- this skeleton proves the dual-core shape, boots the UI,
//  renders a live spectrum from SYNTHETIC data so the display + touch stack is
//  validated independently of the (still-to-port) register-level RF code.
//
//  Core-0: RF capture task -- today, a synthetic FFT generator at ~60 Hz that
//          mirrors what a real spectrum frame will look like. Pushes
//          FftFrame structs through a FreeRTOS queue to Core-1.
//  Core-1: UI task -- TFT redraw, XPT2046 touch, screen state machine.
//          Nine screens scaffolded; HOME + LIVE wired, others are placeholder
//          "coming soon" cards that return to HOME.
//
//  Design invariants (per the spec):
//    - Capture never waits on UI (queue is lock-free from producer side).
//    - UI never blocks on capture (queue read is non-blocking with timeout 0).
//    - Dirty-rect redraw only, no full-screen repaints per frame
//      (per the project-wide no-flicker rule).
//
//  Patterns lifted directly from cyd-bulb-controller/src/main.cpp:
//    - Dedicated-VSPI XPT2046 touch (25/32/39/33) at 1 MHz
//    - 3-point L calibration with NVS persistence (kCalNamespace="shdr")
//    - Averaged touchRead() over 3 raw samples for noise rejection
//
// ============================================================================

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Preferences.h>
#include "esp_sdr.h"
#include "esp_wifi.h"            // for esp_wifi_set_channel in radio arbiter
// esp_phy_init.h is guarded in Arduino's build; declare the two funcs we
// need manually. Enum: 1=WIFI, 2=BT, 4=IEEE802154 (per esp_phy_init.h).
extern "C" {
    void esp_phy_disable(int modem);
    void esp_phy_enable(int modem);
    // Public libphy helper: tell PHY we are WiFi-only (no BT coexistence).
    void phy_set_wifi_mode_only(bool wifi_only);
    // OOK / Morse keyer entry points (src/tx_ook.cpp).
    void txOokInit(void);
    void txOokStop(void);
    bool txOokIsActive(void);
    void txOokMorse(const char* text, int wpm, uint8_t chan, uint8_t bo);
    void txOokPattern(const char* bits, int bit_ms, uint8_t chan, uint8_t bo);
}
constexpr int kPhyModemWifi = 1;

// ----------------------------------------------------------------------------
// Wireless Wizard palette (Electric Blue preset — matches cyd-bulb-controller
// src/wizard_theme.h). Black ground, concentric blue-glow buttons, no icons;
// labels only. The spectrum trace keeps its scanner-green color because that's
// data, not chrome.
// ----------------------------------------------------------------------------
namespace color {
inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
// Wizard theme (Electric Blue preset)
const uint16_t BG         = 0x0000;   // pure black
const uint16_t DIM        = 0x0333;   // outer glow ring / disabled text
const uint16_t MID        = 0x03BF;   // mid glow ring / secondary accent
const uint16_t BRIGHT     = 0x07FF;   // bright border / primary text
const uint16_t SHADOW     = 0x0086;   // offset drop-shadow

// Fixed accents (unchanged across presets, same convention as bulb-controller)
const uint16_t ALERT_RED  = 0xF800;
const uint16_t OK_GREEN   = 0x07E0;
const uint16_t WARN_AMBER = 0xFD20;
const uint16_t MUTED      = 0x4208;

// Spectrum-plot specific (data, not chrome)
const uint16_t TRACE      = rgb(170, 230, 120);
}

// ----------------------------------------------------------------------------
// Screen IDs (nine per the spec; Phase 1 wires 1 and 2, stubs the rest)
// ----------------------------------------------------------------------------
enum class Screen : uint8_t {
    HOME = 0,
    LIVE,
    TRIGGER,
    RECORDING,
    CAPTURES,
    DECODING,
    FINDING,
    TUNE,         // tabbed: TUNE | GAIN (see g_tuneTab)
    TRANSMIT,     // hub: picks CW vs 802.11 vs Morse vs Hop sub-screen
    TX_CW,        // sub-screen: CW tone + SWEEP (cert_test path)
    TX_80211,     // sub-screen: 802.11 beacon TX
    TX_MORSE,     // sub-screen: Morse CW keyer (ham-ready)
    TX_HOP,       // sub-screen: FHSS reference emitter
    SETTINGS,
    _COUNT
};

// Set true the first time a cert_test (CW/SWEEP) START fires in this boot.
// Checked when leaving TX_CW -- if dirty, ESP.restart() to release the phy
// buffers cert_test's rftest_init permanently allocates (no deinit API).
static volatile bool g_certTestDirty = false;

// Where to land after a boot. 0=HOME, 1=TRANSMIT hub, 2=TX_CW, 3=TX_80211.
// Written to NVS whenever a sub-screen is entered so a reboot (manual or
// automatic cert_test-cleanup) returns the user to where they were.
static uint8_t g_bootDest = 0;

enum class TuneTab : uint8_t { TUNE = 0, GAIN = 1 };
static TuneTab g_tuneTab = TuneTab::TUNE;

enum class LiveTab : uint8_t { SPEC = 0, HOP = 1 };
static LiveTab g_liveTab = LiveTab::SPEC;

// Hunt mode within the HOP tab. ADV_CYCLE rotates through the three BLE
// advertising channels; ADV_LOCK_* parks on one. CONN/WIFI/ZB are reserved
// placeholders (dim chips in the UI, not yet wired to tune profiles).
enum class HopMode : uint8_t {
    ADV_CYCLE   = 0,
    ADV_LOCK_37 = 1,
    ADV_LOCK_38 = 2,
    ADV_LOCK_39 = 3,
    // Reserved for future hunts (chips are dim):
    CONN        = 4,
    WIFI        = 5,
    ZB          = 6,
};
static HopMode g_hopMode = HopMode::ADV_CYCLE;

// Tune save/restore around HOP tab visits: HOP takes over g_wifiCh +
// g_wantSpan to drive the hunt's own cycle; we restore the user's prior
// settings when they leave the tab. (SpanMode's underlying values match
// clockIdx: S16=0, S80=1, S40=2 -- 1 == SpanMode::S80.)
static uint8_t  g_hopSaveCh      = 7;
static uint8_t  g_hopSaveSpan    = 1;    // = (uint8_t)SpanMode::S80
static bool     g_hopTuneSaved   = false;
static uint32_t g_hopCycleNextMs = 0;
static int      g_hopCycleIdx    = 0;

// kAdvTunes[] and kConnTunes[] are defined after SpanMode (line ~237);
// forward-declared here so hop helpers above can reference them.
struct AdvTune  { uint8_t wifiCh; uint8_t span; uint16_t targetMhz; int bleCh; };
struct ConnTune { uint8_t wifiCh; uint8_t span; };
extern const AdvTune  kAdvTunes[3];
extern const ConnTune kConnTunes[2];
constexpr uint32_t kHopCycleDwellMs = 300;

static const char* const kScreenName[] = {
    "HOME", "LIVE", "TRIGGER", "RECORD", "CAPTURES",
    "DECODING", "FINDING", "TUNE", "TRANSMIT",
    "TX_CW", "TX_80211", "TX_MORSE", "TX_HOP", "SETTINGS",
};

// ----------------------------------------------------------------------------
// Capture -> UI message: one FFT frame
//
// Phase 1: 128 bins over the 2.4 GHz ISM band (2402..2482 MHz). Values are
// quantized 0..255 to match what the on-chip FFT delivers once the ESP-SDR
// driver lands (compact spectra, see docs/spectrum.md in esp-sdr).
// ----------------------------------------------------------------------------
// Runtime FFT size: 128 default, selectable to 256 via Tune > GAIN tab.
// Arrays are sized at compile time to kFftMaxBins; only the first g_appliedFftN
// entries are used at runtime.
constexpr int kFftMaxBins = 256;
constexpr int kFftMinBins = 128;
// Shim for the pre-runtime-N code paths that still want a conservative upper
// bound (frame struct, waterfall, etc.).
constexpr int kFftBins    = kFftMaxBins;

struct FftFrame {
    uint32_t seq;
    uint8_t  bins[kFftMaxBins];
    int16_t  peakDbm;
    uint8_t  peakBin;
    uint16_t fftN;        // actual bin count populated in this frame
};

static QueueHandle_t g_fftQueue = nullptr;

// ----------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------
TFT_eSPI tft;

constexpr int kTouchCs   = 33;
constexpr int kTouchClk  = 25;
constexpr int kTouchMosi = 32;
constexpr int kTouchMiso = 39;
constexpr int kTouchIrq  = 36;
SPIClass touchSpi(VSPI);

// SD lives on the same VSPI peripheral as touch, but on the CYD's physical
// SD-slot pads (18/19/23/5). The pin mux is re-pointed per operation:
// touch runs with VSPI mux = touch pads; SD write briefly re-muxes to SD
// pads, then restores. All SD + touch access is from the UI task on Core 1
// so no cross-task race.
constexpr int kSdCs   = 5;
constexpr int kSdSck  = 18;
constexpr int kSdMiso = 19;
constexpr int kSdMosi = 23;
SPIClass sdSpi(VSPI);
// Idle noise on this specific CYD panel reads rz 300-510 with no finger.
// Threshold must be comfortably above that; real resistive touch typically
// clears 1000-3500.
constexpr uint16_t kTouchPressureThreshold = 800;

constexpr char kCalNamespace[] = "shdr";
constexpr char kCfgNamespace[] = "shcfg";    // config (freq/span/FFT/gain/trig)

// Brightness: PWM on BL (GPIO 21) via LEDC channel 0. 0..255 duty.
constexpr int  kBlPin        = 21;
constexpr int  kBlChannel    = 0;
constexpr int  kBlFreqHz     = 25000;   // 25 kHz, above human hearing
                                        // (5 kHz made the CYD caps/inductor sing)
constexpr int  kBlResBits    = 8;
static uint8_t g_brightness  = 255;

static void setBrightness(uint8_t b) {
    g_brightness = b;
    ledcWrite(kBlChannel, b);
}
constexpr int  kCalMargin      = 30;

struct TouchCalibration {
    bool     swapXY;
    uint16_t rawAtX0, rawAtX1;
    uint16_t rawAtY0, rawAtY1;
};
TouchCalibration touchCal;

// Current state (UI thread owns these)
static Screen   g_screen         = Screen::HOME;
static Screen   g_screenLast     = Screen::_COUNT;   // forces first-paint
static Screen   g_prevScreen     = Screen::HOME;     // where Back returns to
static FftFrame g_latestFrame    = {};
static bool     g_haveFrame      = false;
static uint32_t g_lastFrameSeen  = 0;

// Live-screen tuning. UI writes the "desired" fields; the RF task is the
// only consumer of the driver and applies pending retunes between captures.
// SpanMode values ARE clockIdx values (16 MS/s=0, 80 MS/s=1, 40 MS/s=2) so
// we can pass straight through to espSdrAcquireBurst.
enum class SpanMode : uint8_t { S16 = 0, S80 = 1, S40 = 2 };

// HopMode's adv-channel tune profiles. Each profile puts the target MHz
// safely inside the span (clear of the edge-guard rejection zone).
const AdvTune kAdvTunes[3] = {
    { 1,  (uint8_t)SpanMode::S40, 2402, 37 },
    { 4,  (uint8_t)SpanMode::S40, 2426, 38 },
    { 13, (uint8_t)SpanMode::S40, 2480, 39 },
};

// CONN hunt: BLE data chs are the even BT chs 2..22 + 26..76 (37 channels).
// Two 40 MHz tune positions cover the full data-ch space with a bit of
// overlap in the middle:
//   ch 3  S40: center 2422, range 2402-2442 → BT 0..40 (data chs 2-40)
//   ch 11 S40: center 2462, range 2442-2482 → BT 40..78 (data chs 40-76)
// ConnTune struct is forward-declared above (SpanMode block).
const ConnTune kConnTunes[2] = {
    { 3,  (uint8_t)SpanMode::S40 },
    { 11, (uint8_t)SpanMode::S40 },
};

static volatile uint8_t  g_wifiCh      = 6;        // UI → RF: desired channel
static volatile uint8_t  g_wantSpan    = (uint8_t)SpanMode::S16;
static uint8_t           g_appliedCh   = 6;
static SpanMode          g_appliedSpan = SpanMode::S16;
static uint32_t          g_freqMHz     = 2437;
static uint8_t           g_bwMHz       = 20;
// Gain -- UI writes, RF task applies between captures. 0xffu == "no change".
static volatile bool     g_wantAgc     = true;
static volatile uint8_t  g_wantGain    = 0xffu;
static bool              g_appliedAgc  = true;
static uint8_t           g_appliedGain = 0;        // set from gainMax at first apply
static bool              g_agc         = true;     // legacy mirror for the LIVE pill text
// Set true while the vendored cert_test is driving continuous TX from the
// serial CLI (or Transmit screen). rfTask MUST skip every radio-touching
// call while this is set, or acquireBurst will yank the chip back to RX
// and kill the CW emission after a few scan cycles.
static volatile bool g_cwActive = false;

// Forward decls for vendored cert_test (librftest.a from esp-phy-lib @ IDF v5.5).
// The full extern "C" block is further down near the serial CLI; these forward
// decls are so the Transmit-screen touch handler (which runs on Core-1 UI)
// can call them too.
extern "C" {
    void esp_phy_rftest_init(void);
    void esp_phy_rftest_config(unsigned char);
    void esp_phy_wifi_tx_tone(int start, int chan, int backoff);
    // BT CW tone via Bluetooth radio (vendored libbttestmode.a). Channel 0..78
    // maps to 2402..2480 MHz (1 MHz grid, finer than WiFi's 5 MHz grid).
    void esp_phy_bt_tx_tone(int start, int chan, int backoff);
    // Fast BT PLL retune while a CW tone is already up (libphy internal).
    // Equivalent of chip_v7_set_chan_nomac but on the BT radio path.
    void bt_chan_change(int bt_chan);
    // esp_phy_wifi_tx signature in our vendored librftest.a differs from the
    // esp-idf v5.5 open-source example (which has 6 args, channel first).
    // Keep the 4-arg form that linked and ran without crashing even if the
    // channel semantics are unclear.
    void esp_phy_wifi_tx(unsigned int rate, int backoff, int pkt_num, int pkt_length_choice);
    void esp_phy_tx_contin_en(bool enable);
    void esp_phy_cbw40m_en(bool enable);
    void esp_phy_test_start_stop(unsigned char value);
    // libphy internals -- explicit channel setter that packet TX inherits.
    // esp_phy_wifi_tx has no channel arg; phy_change_channel steers the PLL.
    void phy_change_channel(int chan);
    // Lower-level ESP32 (chip_v7) hardware PLL retune. Signature is
    // (primary_chan, secondary_chan). Pass 0 for secondary (20 MHz mode).
    void chip_v7_set_chan_nomac(int primary, int secondary);
    // Candidate "actual TX frequency write" -- name is literal. Unknown
    // arg list; trying with one int (channel) and will revise if link fails.
    void freq_write_wifi_chan(int chan);
}

// User-selectable TX power backoff (0.25 dB per unit). 0 = max power (~+19 dBm);
// 20 = -5 dB; 40 = -10 dB; 80 = -20 dB. See phy_cert_test.h.
static volatile uint8_t g_txBackoff = 0;

// Emission mode:
//   0 = TONE     (WiFi CW on selected channel, 5 MHz grid)
//   1 = PKT20    (continuous 11n packet TX; wide but no channel control)
//   2 = SWEEP    (WiFi CW tone swept ±span ch around center, 5 MHz granular)
//   3 = BT       (BT CW on selected channel, 1 MHz grid via WiFi-ch mapping)
//   4 = BT_SWEEP (BT CW rapid retune across the full 2.4 GHz ISM band or a
//                 user-chosen span or BLE adv channels)
static volatile uint8_t g_cwMode = 0;
// BT_SWEEP sub-pattern (only relevant when g_cwMode == 4):
//   0 = FULL     (sweep BT ch 0..78 = 2402..2480 MHz, entire ISM band)
//   1 = SPAN     (±span MHz around the selected center BT channel)
//   2 = BLE_ADV  (interleave BLE adv channels: BT 0 / 24 / 78)
static volatile uint8_t g_btSweepPat = 0;
// Direct BT channel 0..78 (= 2402 + N MHz). Used by BT (mode 3) and BT_SWEEP
// SPAN (mode 4 pattern 1). Default 10 = 2412 MHz (same spot as WiFi ch 1).
static volatile uint8_t g_btChan = 10;
// SWEEP mode -- matches cyd-bulb-controller nrf_jam.cpp CW handler:
// init carrier once, then retune the PLL at microsecond rate via a
// direct low-level channel setter. Yield to the scheduler every
// kYieldEvery steps.
//
// Dwell is now USER-controlled via a slider on the Transmit screen,
// range 130us (PLL settle min) .. 2000us. Overrides the earlier
// auto-adaptive dwell. Default 500us (bulb's WLAN default).
static constexpr uint32_t kSweepMinDwellUs = 130;  // PLL settle floor
static constexpr uint32_t kSweepMaxDwellUs = 2000;
static constexpr int      kYieldEvery      = 50;    // yield every 50 steps
static volatile uint16_t  g_sweepDwellUs   = 500;
// User-selectable sweep span (±N WiFi channels). Default ±2 = ~20 MHz wide.
static volatile uint8_t   g_cwSweepSpan    = 2;
static volatile uint32_t g_cwSweepLast = 0;
static volatile int8_t   g_cwSweepPos  = 0;
// Pending request from UI to change backoff while CW is live. Carried out
// by cwPoll (never by the touch handler) to keep the UI task snappy.
static volatile bool    g_cwReqPaChange = false;
static volatile uint8_t g_cwReqNewBo    = 0;

// Deferred CW request flags. Touch handler sets these; a dedicated Core-0
// task (cwTask) runs the heavy phy init so UI (Core-1) is never blocked.
static volatile bool    g_cwReqStart = false;
static volatile bool    g_cwReqStop  = false;
static volatile uint8_t g_cwReqCh    = 11;
static volatile uint8_t g_cwReqBo    = 0;

// Live telemetry (cheap, readable from any core). Hits increments on each
// hop cycle (or ~every 200 ms when not hopping) so UI can show "alive".
static volatile uint32_t g_cwHits        = 0;
// Teardown-timeout plumbing mirrored from the bulb's jam engine: stop
// request handling bails after 800 ms even if the phy call hangs.
constexpr uint32_t kCwStopTimeoutMs = 800;

// Forward decl so setup() can hand this to xTaskCreatePinnedToCore before
// the actual definition further down.
void cwTask(void*);

// ---------------------------------------------------------------------------
// Radio arbiter -- single owner of the phy's TX state. Any code that wants
// to put the radio into a specific TX mode goes through radioSwitchMode().
// The arbiter does: current.exit() -> settle -> target.enter() -> settle.
// Settle budget per transition is ~150 ms (user-selected "balanced").
//
// Scope: cert_test TX modes only (CW, PKT20). RX scan + 802.11 TX stay on
// their current paths for now; they're still gated by g_cwActive so the
// arbiter has exclusive phy access while in a TX mode.
// ---------------------------------------------------------------------------
enum RadioMode : uint8_t {
    RADIO_IDLE     = 0,
    RADIO_TX_CW    = 1,
    RADIO_TX_PKT20 = 2,
    RADIO_TX_BT_CW = 3,     // Bluetooth CW tone (1 MHz grid, 2402-2480)
};
static volatile uint8_t g_radioMode = RADIO_IDLE;
static volatile uint8_t g_radioChan = 0;
static volatile uint8_t g_radioBo   = 0;
static SemaphoreHandle_t g_radioMtx = nullptr;

static const char* radioModeName(uint8_t m) {
    switch (m) {
        case RADIO_IDLE:     return "IDLE";
        case RADIO_TX_CW:    return "CW";
        case RADIO_TX_PKT20: return "PKT20";
        case RADIO_TX_BT_CW: return "BT_CW";
        default:             return "?";
    }
}
// Convert WiFi channel 1..13 to the matching BT channel index so the same
// channel grid in the UI drives both radio paths. WiFi ch N center = 2412
// + 5*(N-1) MHz. BT ch N center = 2402 + N MHz. So BT = 5*N + 5.
static inline int wifiChToBtCh(int wifiCh) {
    int bt = 5 * wifiCh + 5;
    if (bt < 0)  bt = 0;
    if (bt > 78) bt = 78;
    return bt;
}

// Clean phy transition. Returns true on success. Serialized via g_radioMtx
// so concurrent callers queue instead of racing.
static bool radioSwitchMode(uint8_t target, uint8_t chan, uint8_t bo) {
    if (!g_radioMtx) return false;
    xSemaphoreTake(g_radioMtx, portMAX_DELAY);

    uint8_t from = g_radioMode;
    // No-op if already in the exact state requested.
    if (from == target && g_radioChan == chan && g_radioBo == bo) {
        xSemaphoreGive(g_radioMtx);
        return true;
    }
    Serial.printf("[radio] %s(ch=%u) -> %s(ch=%u bo=%u)\n",
                  radioModeName(from), g_radioChan,
                  radioModeName(target), chan, bo);

    // FAST PATH for same-mode hops. CW and PKT20 both use the tone-retune
    // trick: a brief wifi_tx_tone(1, ch, bo) call moves the PLL to the
    // requested channel. PKT20 then re-arms its continuous packet TX on
    // the newly-tuned frequency.
    if (from == target && from != RADIO_IDLE && bo == g_radioBo) {
        if (from == RADIO_TX_BT_CW) {
            esp_phy_bt_tx_tone(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(3));
            esp_phy_bt_tx_tone(1, wifiChToBtCh(chan), bo);
        } else if (from == RADIO_TX_CW) {
            Serial.println("[radio]   FAST retune (CW tone)");
            esp_phy_wifi_tx_tone(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(3));
            esp_phy_wifi_tx_tone(1, chan, bo);
        } else {
            // PKT20 fast hop:
            //   1) cmdstop packet TX
            //   2) retune via brief tone-on at target channel
            //   3) stop retune tone
            //   4) re-arm continuous packet TX (PLL stays on new channel)
            Serial.println("[radio]   FAST retune (PKT20)");
            esp_phy_test_start_stop(0);                  // cmdstop current TX
            vTaskDelay(pdMS_TO_TICKS(20));
            esp_phy_wifi_tx_tone(1, chan, bo);           // retune PLL
            vTaskDelay(pdMS_TO_TICKS(5));
            esp_phy_wifi_tx_tone(0, 0, 0);               // release PLL
            vTaskDelay(pdMS_TO_TICKS(5));
            esp_phy_test_start_stop(1);                  // BUSY again
            esp_phy_wifi_tx(0x17, bo, 1, 2);             // re-arm template
        }
        g_radioChan = chan;
        xSemaphoreGive(g_radioMtx);
        Serial.printf("[radio] -> %s ch=%u (%u MHz) [fast]\n",
                      radioModeName(target), chan, 2407 + 5 * chan);
        return true;
    }

    // ---- Step 1: EXIT current mode ----
    // Real STOP signal is esp_phy_test_start_stop(0) (the "cmdstop" command
    // in Espressif's RF test firmware). That lets the internal wifi_tx loop
    // detect cancel and exit cleanly; pkt20Task then self-deletes.
    switch (from) {
        case RADIO_TX_CW:
            esp_phy_wifi_tx_tone(0, 0, 0);
            break;
        case RADIO_TX_BT_CW:
            esp_phy_bt_tx_tone(0, 0, 0);
            break;
        case RADIO_TX_PKT20:
            Serial.println("[radio]   cmdstop (test_start_stop(0))");
            esp_phy_test_start_stop(0);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_phy_tx_contin_en(0);
            break;
        default:
            break;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    // ---- Step 2: ENTER target mode ----
    if (target == RADIO_IDLE) {
        if (from != RADIO_IDLE) {
            enableCore0WDT();
            // Clean phy teardown + re-enable after cert_test. esp_phy_disable
            // (from esp_phy_init.h) is Espressif's documented way to shut
            // down the PHY/RF module. Pairs with esp_phy_enable to restore.
            // Follow with wifi stop/start to re-init the WiFi driver state.
            // Arbiter does NOT auto-reboot here anymore. The UI layer owns
            // that decision: leaving the TX_CW sub-screen with g_certTestDirty
            // true triggers rebootForRadioRelease(). Within the sub-screen
            // the user is free to start/stop/retune without reboots -- CW
            // tone's own stop (wifi_tx_tone(0,0,0)) works instantly, and we
            // don't try to coexist with 802.11 TX from this state.
            Serial.println("[radio]   (IDLE from cert_test; dirty flag set)");
        }
        g_radioMode = RADIO_IDLE;
        g_radioChan = 0;
        g_radioBo   = 0;
        xSemaphoreGive(g_radioMtx);
        Serial.println("[radio] -> IDLE");
        return true;
    }
    if (from == RADIO_IDLE) {
        disableCore0WDT();
        // Lazy cert_test init on first use (setup() hung if we called this
        // at boot). Idempotent subsequent calls return quickly.
        static bool s_rftestInited = false;
        if (!s_rftestInited) {
            Serial.println("[radio]   first-use rftest_init + config");
            esp_phy_rftest_init();
            esp_phy_rftest_config(0);
            s_rftestInited = true;
        }
    }
    switch (target) {
        case RADIO_TX_CW:
            esp_phy_test_start_stop(3);
            esp_phy_wifi_tx_tone(1, chan, bo);
            break;
        case RADIO_TX_BT_CW: {
            // In BT modes we address BT channels directly (0..78 = 1 MHz grid).
            // The arbiter's `chan` arg carries the direct BT channel when
            // mode is BT (3) or BT_SWEEP (4); fall back to WiFi mapping if
            // somebody passed a WiFi channel for backward-compat.
            int btCh = (chan >= 0 && chan <= 78) ? chan : wifiChToBtCh(chan);
            Serial.printf("[radio]   bt_tx_tone(1, %d, %d) = %d MHz\n",
                          btCh, bo, 2402 + btCh);
            esp_phy_test_start_stop(3);
            esp_phy_bt_tx_tone(1, btCh, bo);
            break;
        }
        case RADIO_TX_PKT20:
            // (1) Retune PLL via a brief wifi_tx_tone -- that call verifiably
            //     moves the frequency to the requested channel.
            // (2) Stop the tone (so hardware is free for packet TX).
            // (3) Force HT20 (even if cbw40m_en(false) might not persist,
            //     call it so we at least set the intent).
            // (4) BUSY state + continuous mode + wifi_tx template -- infinite
            //     back-to-back packets at whatever freq the PLL is on now.
            Serial.printf("[radio]   PKT20: retune via tone(1,%d), then start\n", chan);
            esp_phy_wifi_tx_tone(1, chan, bo);
            vTaskDelay(pdMS_TO_TICKS(5));
            esp_phy_wifi_tx_tone(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(5));
            esp_phy_cbw40m_en(false);
            esp_phy_test_start_stop(1);
            esp_phy_tx_contin_en(true);
            esp_phy_wifi_tx(0x17, bo, 1, 2);
            break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    g_radioMode = target;
    g_radioChan = chan;
    g_radioBo   = bo;
    xSemaphoreGive(g_radioMtx);
    Serial.printf("[radio] -> %s ch=%u (%u MHz) bo=%u\n",
                  radioModeName(target), chan, 2407 + 5 * chan, bo);
    return true;
}

// Hop-mode CW: step through user-selected channels in g_cwHopMask with
// g_cwHopDwellMs per step. Default mask = all 13 Wi-Fi channels.
static volatile bool     g_cwHop         = false;
static volatile uint16_t g_cwHopDwellMs  = 100;
static volatile uint8_t  g_cwHopCurCh    = 1;    // written by hopper, read by UI
static volatile uint16_t g_cwHopMask     = 0x1FFF;   // bits 0..12 = ch 1..13

// Hop preset selector. Tapping the HOP label cycles through:
//   0 OFF       -- single channel from the grid
//   1 CUSTOM    -- user-tapped channel mask (what we had before)
//   2 WIFI_161  -- 1 / 6 / 11 (non-overlapping)
//   3 WIFI_ALL  -- 1..13 full band
//   4 BLE_ADV   -- WiFi ch 1, 4, 13 (closest WiFi centers to BLE adv 2402/2426/2480)
//   5 VIDEO     -- WiFi ch 1, 5, 9, 12 (closest to 2414/2432/2450/2468 analog AV)
enum CwHopPreset : uint8_t {
    HOP_OFF = 0, HOP_CUSTOM, HOP_WIFI_161, HOP_WIFI_ALL, HOP_BLE_ADV, HOP_VIDEO,
    HOP_PRESET_COUNT
};
static volatile uint8_t g_cwHopPreset = HOP_OFF;
// Bit N set = include WiFi channel (N+1) in the hop set.
static const uint16_t kHopMask_WIFI_161 = (1u<<0) | (1u<<5) | (1u<<10);
static const uint16_t kHopMask_WIFI_ALL = 0x1FFF;
static const uint16_t kHopMask_BLE_ADV  = (1u<<0) | (1u<<3) | (1u<<12);
static const uint16_t kHopMask_VIDEO    = (1u<<0) | (1u<<4) | (1u<<8) | (1u<<11);
static const char* kHopPresetLbl[HOP_PRESET_COUNT] = {
    "HOP  OFF",
    "HOP  Custom",
    "HOP  WiFi 1/6/11",
    "HOP  WiFi All (1-13)",
    "HOP  BLE Adv (1/4/13)",
    "HOP  Video (1/5/9/12)",
};
// Forward-declare g_txHop (the 802.11 hop flag; its storage lives later).
// Preset selection drives BOTH g_cwHop (CW hopper) and g_txHop (802.11
// beacon hopper) so the UI's single HOP button controls both TX paths.
extern bool g_txHop;
static void applyHopPreset(uint8_t preset) {
    g_cwHopPreset = preset;
    bool hopping = (preset != HOP_OFF);
    g_txHop = hopping;
    switch (preset) {
        case HOP_OFF:       break;
        case HOP_CUSTOM:    break;    // keep current mask
        case HOP_WIFI_161:  g_cwHopMask = kHopMask_WIFI_161; break;
        case HOP_WIFI_ALL:  g_cwHopMask = kHopMask_WIFI_ALL; break;
        case HOP_BLE_ADV:   g_cwHopMask = kHopMask_BLE_ADV;  break;
        case HOP_VIDEO:     g_cwHopMask = kHopMask_VIDEO;    break;
    }
}

static inline bool chanInMask(uint8_t ch) {
    return ch >= 1 && ch <= 13 && (g_cwHopMask & (1u << (ch - 1))) != 0;
}
static inline uint8_t nextMaskedChan(uint8_t cur) {
    if (g_cwHopMask == 0) return 11;             // fallback if user cleared mask
    uint8_t c = cur;
    for (int i = 0; i < 13; i++) {
        c = (c % 13) + 1;
        if (chanInMask(c)) return c;
    }
    return cur;
}

// Monotonic counter bumped by rfTask every time a retune actually lands.
// UI task reads this to reset peak-hold / per-bin history on band change.
static volatile uint32_t g_retuneSeq   = 0;

// Dwell / max-hold aggregation: do N captures per UI frame and send the
// per-bin max as the frame. UI stays at 60 Hz, on-air coverage scales x N.
// Values: 1 (default, instantaneous), 4, 16, 64.
static volatile uint8_t  g_wantPasses    = 1;
static uint8_t           g_appliedPasses = 1;

// Averaging strength (per-bin EMA on Core-0). Values 1, 2, 4, 8, 16 map to
// alpha = 1/N; lower alpha = heavier smoothing, longer latency. Default 4
// matches the previous hard-coded kBinAlpha=0.25f.
static volatile uint8_t  g_wantAvg       = 4;
static uint8_t           g_appliedAvg    = 4;

// TX: UI toggles g_txRequest to start. RF task starts, sets g_txRunning=true,
// and loops until UI clears g_txRunning OR safety timeout trips.
enum class TxMode : uint8_t { BEACON = 0, DEAUTH = 1, PROBE = 2, CARRIER = 3 };
static volatile bool     g_txRequest      = false;// UI → RF: begin TX now
static volatile bool     g_txRunning      = false;// RF → UI (and UI STOP)
static volatile uint32_t g_wantToneMs     = 0;    // >0 → fire ROM CW tone
static bool              g_toneSel        = false;// MODE tab: 802.11 vs TONE
static volatile uint32_t g_lastTxCount    = 0;    // frames sent in last burst
static volatile uint32_t g_lastTxFinishMs = 0;    // millis() when burst ended
static uint16_t          g_txDurMs        = 500;  // total burst duration
static uint16_t          g_txIntervalUs   = 1000; // pacing between frames
static uint8_t           g_txMode         = (uint8_t)TxMode::CARRIER;
static uint8_t           g_txChannel      = 6;    // single-channel pick
bool                     g_txHop          = false;// hop over all 13 channels (non-static so applyHopPreset can set it)
static uint16_t          g_txHopDwellMs   = 100;  // ms per channel while hopping

// Beacon frame. SSID "SH-TX-TEST". DS channel byte patched at TX time.
static const uint8_t kTxBeacon[] = {
    0x80, 0x00, 0x00, 0x00,
    0xff,0xff,0xff,0xff,0xff,0xff,
    0xDE,0xAD,0xBE,0xEF,0x00,0x01,
    0xDE,0xAD,0xBE,0xEF,0x00,0x01,
    0x00, 0x00,
    0,0,0,0,0,0,0,0,
    0x64, 0x00,
    0x31, 0x04,
    0x00, 0x0A, 'S','H','-','T','X','-','T','E','S','T',
    0x01, 0x08, 0x82,0x84,0x8B,0x96,0x24,0x30,0x48,0x6C,
    0x03, 0x01, 0x06,
};

// Broadcast Deauth frame. Reason 7 "Class 3 frame from unassociated STA".
// Standard pentest shape; still 802.11-shaped so it carries like any beacon.
static const uint8_t kTxDeauth[] = {
    0xC0, 0x00, 0x00, 0x00,                   // FC: deauth
    0xff,0xff,0xff,0xff,0xff,0xff,            // DA (broadcast)
    0xDE,0xAD,0xBE,0xEF,0x00,0x02,            // SA
    0xDE,0xAD,0xBE,0xEF,0x00,0x02,            // BSSID
    0x00, 0x00,                               // Seq
    0x07, 0x00,                               // Reason code 7
};

// Probe Request, wildcard SSID. 802.11 TX type 0 subtype 4.
static const uint8_t kTxProbe[] = {
    0x40, 0x00, 0x00, 0x00,                   // FC: probe req
    0xff,0xff,0xff,0xff,0xff,0xff,            // DA (broadcast)
    0xDE,0xAD,0xBE,0xEF,0x00,0x03,            // SA
    0xff,0xff,0xff,0xff,0xff,0xff,            // BSSID (bcast for probe)
    0x00, 0x00,                               // Seq
    0x00, 0x00,                               // SSID wildcard
    0x01, 0x08, 0x82,0x84,0x8B,0x96,0x24,0x30,0x48,0x6C,
};

// Trigger state. UI owns these except g_trigFireCount / g_trigLastFireMs,
// which the UI task bumps when a frame crosses the threshold.
enum class TrigBand : uint8_t { ALL = 0, LEFT = 1, RIGHT = 2 };
// Sub-tabs within the TRIGGER screen: CFG (threshold / band), REC (record
// on/off + buffer status), CAPS (scrolling list of captured events).
enum class TrigTab : uint8_t { CFG = 0, REC = 1, CAPS = 2 };
static TrigTab g_trigTab = TrigTab::CFG;
static int     g_capScroll = 0;         // top row index in CAPS list
static bool     g_trigEnabled    = false;
static uint8_t  g_trigThreshold  = 160;      // code 0..255
static TrigBand g_trigBand       = TrigBand::ALL;
static uint32_t g_trigFireCount  = 0;
static uint32_t g_trigLastFireMs = 0;
// Edge-trigger state: fire on rising edge above threshold, re-arm only when
// peak drops back below (threshold - kTrigHysteresis). Without this a
// sustained Wi-Fi burst fires at 60 Hz.
static bool     g_trigArmed      = true;
constexpr uint8_t kTrigHysteresis = 10;
// Visual flash state: when non-zero, LIVE paints a bright border + fire marker.
static uint8_t  g_trigFlashFrames = 0;

// -----------------------------------------------------------------------------
// Capture buffer: a RAM ring of trigger-fire events. UI reads via CAPS tab.
// -----------------------------------------------------------------------------
struct CaptureEvent {
    uint32_t tsMs;             // millis() at fire
    uint8_t  peakBin;          // bin index that triggered
    uint8_t  peakLevel;        // 0..255 at fire
    uint8_t  band;             // TrigBand value
    uint8_t  wifiCh;           // tune state at fire
    uint8_t  spanMode;         // SpanMode value
    uint8_t  fftN;             // 128 or 256 (bins at fire)
    uint8_t  _pad[1];
    uint8_t  snap[64];         // max-pooled bins at fire (for sparkline)
};

constexpr int kCapCapacity = 64;
static CaptureEvent g_capBuf[kCapCapacity] = {};
static int          g_capHead      = 0;          // next write slot (ring)
static int          g_capCount     = 0;          // valid events (<= capacity)
static bool         g_recordOn     = false;      // toggle: log fires?
static bool         g_recordRing   = true;       // true=overwrite, false=stop-when-full
static bool         g_recordFull   = false;      // one-shot buffer stopped

// Hop Trail: compact per-channel activity log. Classifier reads this to
// label active hoppers (BLE conn / BLE adv / WiFi) without demod. RAM only.
struct HopEvent {
    uint32_t tsMs;         // millis() at frame
    uint8_t  binIdx;       // dominant bin
    uint8_t  level;        // 0..255
    uint8_t  wifiCh;       // tune state (so we can map bin -> BT ch at render)
    uint8_t  spanMode;
};  // 8 bytes

constexpr int kHopRingSize = 512;             // ~5-10 s of hop history
static HopEvent g_hopRing[kHopRingSize] = {};
static int      g_hopHead  = 0;
static int      g_hopCount = 0;

// Scatter strip: scrolling ring (waterfall-style) of per-slot dominant BT ch.
// 100 cols x 50 ms = 5 s window. Each slot stores (btCh + 1), 0 = empty.
constexpr int kScatterCols   = 100;
constexpr int kScatterColMs  = 50;
static uint8_t  g_scatterRing[kScatterCols]      = {0};
static uint8_t  g_scatterLastDrawn[kScatterCols] = {0};
static int      g_scatterHead      = 0;
static uint32_t g_scatterLastShift = 0;

// Hop detection thresholds (both apply):
//   g_hopMinLevel: absolute floor (0..255). A peak below this is noise.
//   g_hopDelta:    required margin above the per-bin running noise floor.
// Defaults tuned so a 10-30 count signal spike at any bin above its own
// local floor registers, while uniform noise jitter does not.
static volatile uint8_t g_hopMinLevel = 30;
static volatile uint8_t g_hopDelta    = 18;

// Per-bin running noise floor. EWMA with asymmetric time constants: climbs
// slowly when samples are above the floor (so real signals don't inflate
// it), descends quickly when samples drop. Values stored in Q4 (shifted
// left by 4) so a 1-unit sample moves the floor by 1/16 per update.
static uint16_t g_binFloor[kFftMaxBins] = {0};
static bool     g_binFloorPrimed = false;

// Reject bins within kEdgeGuard of either FFT edge. These bins carry DC
// offset, aliasing, and windowing artifacts that fake "signal" there --
// the main cause of phantom BLE adv hits at BT 0 and BT 78.
constexpr int kEdgeGuard = 4;

// Hoprun stats: cumulative counters bumped inside hopLog while a timed
// `hoprun N` is active. Snapshot at run start (zeroed), diff printed at end.
struct HopRunStats {
    uint32_t total;
    uint32_t adv0, adv24, adv78;
    uint32_t connCnt;
    uint32_t connDtSum;
    uint32_t fastHop;
    uint32_t perCh[79];
};
static volatile bool          g_hoprunActive = false;
static volatile HopRunStats   g_hoprun = {};

// Usage-bar + classifier dirty caches, used by drawLiveHopTabTick.
static uint8_t g_hopUsageLastH[79]  = {0};
static char    g_hopClassMainCache[3][40] = {};
static char    g_hopClassSubCache[3][24]  = {};
static int     g_hopLastRateShown   = -1;
static bool    g_hopTabPrimed       = false;

// SD mirror -- when mounted, every logged event is appended to
// /signalhound/captures.bin. On boot, the last 64 events are loaded back into
// the RAM ring so the user sees history after a reboot.
static bool     g_sdMounted = false;
static uint32_t g_sdCapTotal = 0;    // total events written to SD this session + loaded
static const char* kCapDir  = "/signalhound";
static const char* kCapFile = "/signalhound/captures.bin";
static const uint32_t kSdCapMagic = 0x41434853;   // 'SHCA' LE
static const uint32_t kSdCapVer   = 1;

// SD helpers --------------------------------------------------------------
// VSPI is shared between touch (pads 25/32/39/33) and the SD card slot
// (pads 18/19/23/5). SPIClass::begin() early-returns once _spi is set, so
// after the first SD mount we re-point VSPI's GPIO matrix routing via the
// hal functions, not SPIClass. Touch invariant: pins stay on touch at rest.
// All switching is from the UI task on Core 1, so no cross-task race.
static spi_t* g_vspiBus = nullptr;
static int    g_curSck  = -1, g_curMiso = -1, g_curMosi = -1;

static void vspiMux(int sck, int miso, int mosi) {
    if (!g_vspiBus) return;
    if (g_curSck  >= 0 && g_curSck  != sck)  spiDetachSCK(g_vspiBus,  g_curSck);
    if (g_curMiso >= 0 && g_curMiso != miso) spiDetachMISO(g_vspiBus, g_curMiso);
    if (g_curMosi >= 0 && g_curMosi != mosi) spiDetachMOSI(g_vspiBus, g_curMosi);
    spiAttachSCK(g_vspiBus,  sck);
    spiAttachMISO(g_vspiBus, miso);
    spiAttachMOSI(g_vspiBus, mosi);
    g_curSck = sck; g_curMiso = miso; g_curMosi = mosi;
}
static void vspiToSd()    { vspiMux(kSdSck,    kSdMiso,    kSdMosi); }
static void vspiToTouch() { vspiMux(kTouchClk, kTouchMiso, kTouchMosi); }

static void tryMountSd() {
    // SPIClass::begin() will no-op if we try to re-point touch's instance,
    // so skip SPIClass and drive mux via hal. Attach SD pins first so
    // SD.begin's internal transfers land on the right pins; our later
    // vspiToTouch() flips them back.
    sdSpi.begin(kSdSck, kSdMiso, kSdMosi, kSdCs);     // sets up sdSpi._spi
    g_vspiBus = spiStartBus(VSPI, SPI_CLOCK_DIV16, SPI_MODE0, SPI_MSBFIRST);
    g_curSck = kSdSck; g_curMiso = kSdMiso; g_curMosi = kSdMosi;

    bool ok = SD.begin(kSdCs, sdSpi, 4000000);
    if (!ok) {
        Serial.println("[sd] mount failed");
        vspiToTouch();
        return;
    }
    uint8_t t = SD.cardType();
    if (t == CARD_NONE) {
        Serial.println("[sd] card type NONE");
        vspiToTouch();
        return;
    }
    uint64_t sz = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("[sd] mounted, type=%u size=%lu MB\n",
                  (unsigned)t, (unsigned long)sz);
    if (!SD.exists(kCapDir)) {
        if (!SD.mkdir(kCapDir)) Serial.println("[sd] mkdir failed");
    }
    g_sdMounted = true;
    // Count existing events in file for the status readout.
    if (SD.exists(kCapFile)) {
        File f = SD.open(kCapFile, FILE_READ);
        if (f) {
            size_t sz2 = f.size();
            if (sz2 >= 8) g_sdCapTotal = (sz2 - 8) / sizeof(CaptureEvent);
            f.close();
        }
    }
    vspiToTouch();
}

static void sdAppendEvent(const CaptureEvent* ev) {
    if (!g_sdMounted) return;
    vspiToSd();
    File f;
    if (SD.exists(kCapFile)) {
        f = SD.open(kCapFile, FILE_APPEND);
    } else {
        // FILE_WRITE creates+truncates; only use for fresh file + header.
        f = SD.open(kCapFile, FILE_WRITE);
        if (f) {
            uint8_t hdr[8];
            memcpy(hdr,     &kSdCapMagic, 4);
            memcpy(hdr + 4, &kSdCapVer,   4);
            f.write(hdr, 8);
        }
    }
    if (!f) { Serial.println("[sd] open fail"); vspiToTouch(); return; }
    size_t wrote = f.write((const uint8_t*)ev, sizeof(CaptureEvent));
    f.close();
    if (wrote == sizeof(CaptureEvent)) g_sdCapTotal++;
    else Serial.printf("[sd] short write %u\n", (unsigned)wrote);
    vspiToTouch();
}

static void sdLoadLastEvents(int n) {
    if (!g_sdMounted) return;
    vspiToSd();
    if (!SD.exists(kCapFile)) { vspiToTouch(); return; }
    File f = SD.open(kCapFile, FILE_READ);
    if (!f) { vspiToTouch(); return; }
    size_t fsz = f.size();
    if (fsz <= 8) { f.close(); vspiToTouch(); return; }
    size_t  nEvents = (fsz - 8) / sizeof(CaptureEvent);
    int     want = (nEvents < (size_t)n) ? (int)nEvents : n;
    size_t  offset = 8 + (nEvents - want) * sizeof(CaptureEvent);
    f.seek(offset);
    for (int i = 0; i < want; i++) {
        CaptureEvent ev;
        if (f.read((uint8_t*)&ev, sizeof(ev)) != sizeof(ev)) break;
        g_capBuf[g_capHead] = ev;
        g_capHead = (g_capHead + 1) % kCapCapacity;
        if (g_capCount < kCapCapacity) g_capCount++;
    }
    f.close();
    Serial.printf("[sd] loaded %d of %u events from captures.bin\n",
                  want, (unsigned)nEvents);
    vspiToTouch();
}

// Hop Trail helpers --------------------------------------------------------
// Map an FFT bin + tune state back to a BT channel (0..78 = 2402..2480 MHz).
// Returns -1 when the bin's frequency is outside the ISM band (e.g. WiFi
// ch 13 at 40 MHz span extends past 2482 MHz; those out-of-band bins must
// NOT be bucketed into BT 78 or they'd pile up as phantom activity).
static int hopBinToBtCh(uint8_t binIdx, uint8_t wifiCh, uint8_t spanMode, uint8_t fftN) {
    int bwMHz;
    switch ((SpanMode)spanMode) {
        case SpanMode::S40: bwMHz = 40; break;
        case SpanMode::S80: bwMHz = 80; break;
        default:            bwMHz = 16; break;
    }
    if (fftN == 0) fftN = 128;
    int centerMhz = 2407 + wifiCh * 5;
    int binMhz    = centerMhz - bwMHz / 2 + (binIdx * bwMHz) / fftN;
    int bt        = binMhz - 2402;
    if (bt < 0 || bt > 78) return -1;
    return bt;
}

// Append a HopEvent if the dominant bin shifted. Debounced to kill the
// flicker between a true hop and bin-adjacency noise, and gated by a soft
// level floor so we don't log pure spectrum noise.
static void hopLog(const FftFrame& f) {
    uint32_t now = millis();
    // Scatter ring shift happens on every call -- quiet slots must still age
    // off the strip, not freeze on screen.
    if (g_scatterLastShift == 0) g_scatterLastShift = now;
    while ((now - g_scatterLastShift) >= (uint32_t)kScatterColMs) {
        g_scatterLastShift += kScatterColMs;
        g_scatterHead = (g_scatterHead + 1) % kScatterCols;
        g_scatterRing[g_scatterHead] = 0;
    }

    int N = f.fftN ? f.fftN : 128;

    // Prime noise floor on first frame with the raw bins.
    if (!g_binFloorPrimed) {
        for (int i = 0; i < N; i++) g_binFloor[i] = (uint16_t)f.bins[i] << 4;
        g_binFloorPrimed = true;
    } else {
        // Asymmetric EWMA: climb slow (1/32), descend fast (1/8).
        for (int i = 0; i < N; i++) {
            uint16_t cur = g_binFloor[i];
            uint16_t sample = (uint16_t)f.bins[i] << 4;
            if (sample > cur) g_binFloor[i] = cur + ((sample - cur) >> 5);
            else              g_binFloor[i] = cur - ((cur - sample) >> 3);
        }
    }

    // Multi-peak detection: pick the top-3 bins above threshold, each at
    // least kPeakSep bins from any other. Edge-guard rejects bin 0..3 and
    // N-5..N-1 (DC + aliasing artifacts fake signal there). Per-bin floor
    // + delta kills uniform noise jitter.
    constexpr int kMaxPeaks = 3;
    constexpr int kPeakSep  = 6;   // ~3.75 MHz at 80 MHz span, 128 bins
    int     pBin[kMaxPeaks];
    uint8_t pLvl[kMaxPeaks];
    int     nPeaks = 0;
    for (int i = kEdgeGuard; i < N - kEdgeGuard; i++) {
        uint8_t v = f.bins[i];
        if (v < g_hopMinLevel) continue;
        // Must stand out above its own local floor by at least g_hopDelta.
        uint8_t floor = (uint8_t)(g_binFloor[i] >> 4);
        if (v < floor || (v - floor) < g_hopDelta) continue;
        bool merged = false;
        for (int k = 0; k < nPeaks; k++) {
            int d = pBin[k] - i; if (d < 0) d = -d;
            if (d < kPeakSep) {
                if (v > pLvl[k]) { pBin[k] = i; pLvl[k] = v; }
                merged = true;
                break;
            }
        }
        if (merged) continue;
        if (nPeaks < kMaxPeaks) {
            pBin[nPeaks] = i; pLvl[nPeaks] = v; nPeaks++;
        } else {
            int wkIdx = 0;
            for (int k = 1; k < kMaxPeaks; k++)
                if (pLvl[k] < pLvl[wkIdx]) wkIdx = k;
            if (v > pLvl[wkIdx]) { pBin[wkIdx] = i; pLvl[wkIdx] = v; }
        }
    }
    if (nPeaks == 0) return;

    // Log each peak with per-peak debounce. Scan back a few slots since
    // multi-peak frames write multiple events in a row with distinct bins.
    int bestBin = pBin[0]; uint8_t bestLvl = pLvl[0];
    for (int k = 1; k < nPeaks; k++) {
        if (pLvl[k] > bestLvl) { bestLvl = pLvl[k]; bestBin = pBin[k]; }
    }
    for (int k = 0; k < nPeaks; k++) {
        int peakBin = pBin[k];
        bool skip = false;
        int scanBack = (g_hopCount < 8) ? g_hopCount : 8;
        for (int s = 0; s < scanBack; s++) {
            int lidx = (g_hopHead - 1 - s + kHopRingSize) % kHopRingSize;
            const HopEvent& last = g_hopRing[lidx];
            if ((now - last.tsMs) >= 30) break;
            int dBin = (int)last.binIdx - peakBin; if (dBin < 0) dBin = -dBin;
            if (dBin <= 1) { skip = true; break; }
        }
        if (skip) continue;
        // Peek at the next-older ring entry for conn/fast-hop stats BEFORE
        // we overwrite the slot.
        int    prevBin = -1;
        uint32_t prevTs = 0;
        if (g_hopCount > 0) {
            int pidx = (g_hopHead - 1 + kHopRingSize) % kHopRingSize;
            prevBin = g_hopRing[pidx].binIdx;
            prevTs  = g_hopRing[pidx].tsMs;
        }
        HopEvent& ev = g_hopRing[g_hopHead];
        ev.tsMs     = now;
        ev.binIdx   = (uint8_t)peakBin;
        ev.level    = pLvl[k];
        ev.wifiCh   = g_wifiCh;
        ev.spanMode = g_wantSpan;
        g_hopHead = (g_hopHead + 1) % kHopRingSize;
        if (g_hopCount < kHopRingSize) g_hopCount++;
        // Hoprun lifetime counters (if a timed run is active).
        if (g_hoprunActive) {
            int bt = hopBinToBtCh((uint8_t)peakBin, g_wifiCh, g_wantSpan, N);
            if (bt >= 0) {       // out-of-band bins ignored
                g_hoprun.total++;
                if      (bt == 0)  g_hoprun.adv0++;
                else if (bt == 24) g_hoprun.adv24++;
                else if (bt == 78) g_hoprun.adv78++;
                g_hoprun.perCh[bt]++;
                if (prevBin >= 0) {
                    uint32_t dt = (now > prevTs) ? (now - prevTs) : 0;
                    int d = prevBin - peakBin; if (d < 0) d = -d;
                    if (dt >= 5 && dt <= 500 && d > 4) {
                        g_hoprun.connCnt++;
                        g_hoprun.connDtSum += dt;
                    }
                    if (dt >= 1 && dt < 2 && d > 4) g_hoprun.fastHop++;
                }
            }
        }
    }
    // Scatter: stamp the strongest peak only (one dominant hop per 50ms slot).
    int bt = hopBinToBtCh((uint8_t)bestBin, g_wifiCh, g_wantSpan, N);
    if (bt >= 0) g_scatterRing[g_scatterHead] = (uint8_t)(bt + 1);
}

static void sdDeleteCapFile() {
    if (!g_sdMounted) return;
    vspiToSd();
    if (SD.exists(kCapFile)) SD.remove(kCapFile);
    g_sdCapTotal = 0;
    vspiToTouch();
}

// Config persistence: set g_cfgDirty from any UI touch that changes a saved
// field; uiTask flushes it to NVS a short time later (debounced). Helper
// bodies live further down (after all the g_want* globals are declared).
static bool     g_cfgDirty       = false;
static uint32_t g_cfgLastSaveMs  = 0;
static void loadConfig();
static void saveConfig();

static uint8_t spanBwMHz(SpanMode s) {
    switch (s) {
        case SpanMode::S16: return 16;
        case SpanMode::S40: return 40;
        case SpanMode::S80: return 80;
    }
    return 16;
}

// --- Named presets (center + span pairs tuned via the nearest WiFi channel).
//     Order drives the Tune-screen row left-to-right.
struct TunePreset {
    const char* label;    // short name
    const char* sub;      // freq hint
    uint8_t     wifiCh;
    SpanMode    span;
};
static const TunePreset kPresets[] = {
    { "BLE 37", "2402", 1,  SpanMode::S40 },   // ch1 ± 20 covers 2392-2432
    { "BLE 38", "2426", 4,  SpanMode::S16 },   // ch4 ±  8 covers 2419-2435
    { "BLE 39", "2480", 13, SpanMode::S40 },   // ch13± 20 covers 2452-2492
    { "MWave",  "2450", 9,  SpanMode::S16 },   // ch9 ±  8 covers 2444-2460
    { "Lo 40M", "2420", 3,  SpanMode::S40 },   // ch3 ± 20 covers 2402-2442
    { "Hi 40M", "2460", 11, SpanMode::S40 },   // ch11± 20 covers 2442-2482
};
constexpr int kNumPresets = sizeof(kPresets) / sizeof(kPresets[0]);

// ----------------------------------------------------------------------------
// Touch (direct VSPI, mirrors cyd-bulb-controller/main.cpp:230-261)
// ----------------------------------------------------------------------------
void touchInit() {
    pinMode(kTouchIrq, INPUT);
    pinMode(kTouchCs, OUTPUT);
    digitalWrite(kTouchCs, HIGH);
    touchSpi.begin(kTouchClk, kTouchMiso, kTouchMosi, kTouchCs);
}

bool touchReadRaw(uint16_t& rx, uint16_t& ry, uint16_t& rz) {
    touchSpi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(kTouchCs, LOW);
    touchSpi.transfer(0xB0);
    uint16_t z1 = touchSpi.transfer16(0xC0) >> 3;
    uint16_t z2 = touchSpi.transfer16(0x90) >> 3;
    uint16_t x  = touchSpi.transfer16(0xD0) >> 3;
    uint16_t y  = touchSpi.transfer16(0x00) >> 3;
    digitalWrite(kTouchCs, HIGH);
    touchSpi.endTransaction();
    rz = 0xFFF + z1 - z2;
    rx = x;
    ry = y;
    bool pressed = rz > kTouchPressureThreshold && rz <= 0xFFF;
    // Diagnostic: throttled print of raw z so we can see what the panel is
    // actually reporting. If z stays near 0xFFF when untouched, pressure
    // sensing is working; the threshold just needs tuning.
    static unsigned long lastZprint = 0;
    unsigned long now = millis();
    if (now - lastZprint > 300) {
        Serial.printf("[tc] z1=%u z2=%u rz=%u pressed=%d\n", z1, z2, rz, pressed ? 1 : 0);
        lastZprint = now;
    }
    return pressed;
}

bool touchRead(uint16_t& rx, uint16_t& ry) {
    constexpr int kSamples = 3;
    uint32_t sumX = 0, sumY = 0;
    int hits = 0;
    for (int i = 0; i < kSamples; i++) {
        uint16_t x, y, z;
        if (touchReadRaw(x, y, z)) {
            sumX += x; sumY += y; hits++;
        }
    }
    if (hits == 0) return false;
    rx = sumX / hits;
    ry = sumY / hits;
    return true;
}

bool loadTouchCalibration() {
    Preferences prefs;
    if (!prefs.begin(kCalNamespace, true)) return false;
    size_t got = prefs.getBytes("cal", &touchCal, sizeof(touchCal));
    prefs.end();
    return got == sizeof(touchCal);
}
void saveTouchCalibration() {
    Preferences prefs;
    prefs.begin(kCalNamespace, false);
    prefs.putBytes("cal", &touchCal, sizeof(touchCal));
    prefs.end();
}

static void captureCalibrationPoint(int tx, int ty, uint16_t& outRawX, uint16_t& outRawY) {
    tft.fillCircle(tx, ty, 6, color::WARN_AMBER);
    tft.drawCircle(tx, ty, 14, color::BRIGHT);
    uint16_t rx, ry;
    while (!touchRead(rx, ry)) { delay(15); }
    uint32_t sumX = 0, sumY = 0; int n = 0;
    unsigned long start = millis();
    while (millis() - start < 600) {
        if (touchRead(rx, ry)) { sumX += rx; sumY += ry; n++; }
        delay(15);
    }
    outRawX = n ? sumX / n : rx;
    outRawY = n ? sumY / n : ry;
    while (touchRead(rx, ry)) { delay(15); }
    tft.fillCircle(tx, ty, 14, color::BG);
}

void runTouchCalibration() {
    const int x0 = kCalMargin,                y0 = kCalMargin;
    const int x1 = tft.width()  - kCalMargin, y1 = tft.height() - kCalMargin;

    tft.fillScreen(color::BG);
    tft.setTextColor(color::BRIGHT, color::BG);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("TOUCH CALIBRATION", tft.width() / 2, 50, 4);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString("Tap each target firmly", tft.width() / 2, 80, 2);
    delay(1000);

    uint16_t rxTL, ryTL, rxTR, ryTR, rxBL, ryBL;
    captureCalibrationPoint(x0, y0, rxTL, ryTL);
    captureCalibrationPoint(x1, y0, rxTR, ryTR);
    captureCalibrationPoint(x0, y1, rxBL, ryBL);

    int dFromRawX = abs((int)rxTR - (int)rxTL);
    int dFromRawY = abs((int)ryTR - (int)ryTL);
    touchCal.swapXY = dFromRawY > dFromRawX;

    if (!touchCal.swapXY) {
        touchCal.rawAtX0 = rxTL; touchCal.rawAtX1 = rxTR;
        touchCal.rawAtY0 = ryTL; touchCal.rawAtY1 = ryBL;
    } else {
        touchCal.rawAtX0 = ryTL; touchCal.rawAtX1 = ryTR;
        touchCal.rawAtY0 = rxTL; touchCal.rawAtY1 = rxBL;
    }
    saveTouchCalibration();

    tft.fillScreen(color::BG);
    tft.setTextColor(color::OK_GREEN, color::BG);
    tft.drawString("CALIBRATION SAVED", tft.width() / 2, tft.height() / 2, 4);
    delay(700);
}

void applyTouchCalibration(uint16_t rawX, uint16_t rawY, int& outX, int& outY) {
    const int x0 = kCalMargin,                y0 = kCalMargin;
    const int x1 = tft.width()  - kCalMargin, y1 = tft.height() - kCalMargin;
    uint16_t axisX = touchCal.swapXY ? rawY : rawX;
    uint16_t axisY = touchCal.swapXY ? rawX : rawY;
    outX = constrain((int)map(axisX, touchCal.rawAtX0, touchCal.rawAtX1, x0, x1), 0, tft.width() - 1);
    outY = constrain((int)map(axisY, touchCal.rawAtY0, touchCal.rawAtY1, y0, y1), 0, tft.height() - 1);
}

bool getTouch(int& x, int& y) {
    uint16_t rx, ry;
    if (!touchRead(rx, ry)) return false;
    applyTouchCalibration(rx, ry, x, y);
    return true;
}

// ----------------------------------------------------------------------------
// Core-0 RF task
//
// Phase 2.2: real FFT producer.
//   - If ESP-SDR is ready: capture 256 I/Q words @ 16 MS/s, unpack to float
//     complex, run 128-point FFT, compute per-bin log-power → 0..255 code,
//     fftshift so DC is in the center, push an FftFrame to the UI queue.
//   - Fallback: if a capture fails, emit a synthetic frame so LIVE never
//     stalls visually.
// Target rate ~60 Hz (16 ms period). Capture is ~24 µs, FFT is ~40 µs on
// classic ESP32 at 240 MHz, both well under budget.
// ----------------------------------------------------------------------------

#include <math.h>

// Twiddles computed once for the MAX FFT size. Smaller FFTs reuse them with
// stride = kFftMaxBins / len, since W_{M*N}^{M*k} == W_N^k.
static float g_twRe[kFftMaxBins / 2];
static float g_twIm[kFftMaxBins / 2];
static bool  g_twReady = false;

static void initTwiddles() {
    for (int k = 0; k < kFftMaxBins / 2; k++) {
        float a = -2.0f * (float)M_PI * (float)k / (float)kFftMaxBins;
        g_twRe[k] = cosf(a);
        g_twIm[k] = sinf(a);
    }
    g_twReady = true;
}

// In-place radix-2 decimation-in-time FFT, runtime-sized (N must be a power
// of 2 and <= kFftMaxBins). Twiddles are the kFftMaxBins table used with
// stride (kFftMaxBins / len).
static void fftN(float* re, float* im, int N) {
    // Bit-reversal permutation
    for (int i = 1, j = 0; i < N; i++) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    // Butterflies
    for (int len = 2; len <= N; len <<= 1) {
        int half   = len >> 1;
        int stride = kFftMaxBins / len;       // stride into max-N twiddle table
        for (int i = 0; i < N; i += len) {
            for (int j = 0; j < half; j++) {
                int tw = j * stride;
                float wr = g_twRe[tw], wi = g_twIm[tw];
                int a = i + j, b = i + j + half;
                float yr = re[b] * wr - im[b] * wi;
                float yi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - yr; im[b] = im[a] - yi;
                re[a] = re[a] + yr; im[a] = im[a] + yi;
            }
        }
    }
}

// Unpack ESP-SDR packed word: metadata in upper bits, I in bits [2..11],
// Q in bits [12..21]. Values are 10-bit signed.
static inline void unpackIq(uint32_t w, float& I, float& Q) {
    int16_t i = (int16_t)((w >> 2)  & 0x3ff);
    int16_t q = (int16_t)((w >> 12) & 0x3ff);
    if (i & 0x200) i |= 0xfc00;
    if (q & 0x200) q |= 0xfc00;
    I = (float)i;
    Q = (float)q;
}

// Global noise floor tracker: an EMA toward the current-frame minimum log2
// power across all bins. Robust: noise is noise, one floor fits all bins.
// Replaces earlier per-bin min-hold which stuck high on bins that happened
// to catch a signal on frame 1.
static float g_noiseFloor = 0.0f;
static constexpr float kFloorEmaAlpha = 0.05f;    // responsive, ~20-frame tc
static constexpr float kFloorMargin   = 2.5f;     // log2 above min = baseline;
                                                  // higher pins noise to code 0
static constexpr float kCodePerLog2   = 30.0f;    // 30 codes / log2; 255 code
                                                  // = ~8.5 log2 headroom

// Current / wanted FFT size (power of 2 in [kFftMinBins, kFftMaxBins]).
// UI writes g_wantFftN; RF task applies it at the top of each loop.
static volatile uint16_t g_wantFftN    = kFftMinBins;
static uint16_t          g_appliedFftN = kFftMinBins;

static void fillSyntheticFrame(FftFrame& f, uint32_t seq) {
    f.seq  = seq;
    f.fftN = g_appliedFftN;
    const int N = g_appliedFftN;
    const float centers[] = { 8.0f * (float)N / 128.0f,
                              64.0f * (float)N / 128.0f,
                              120.0f * (float)N / 128.0f };
    const float sigma     = 10.0f * (float)N / 128.0f;
    uint8_t pk = 40, pkBin = 0;
    for (int i = 0; i < N; i++) {
        float v = 0.0f;
        for (float c : centers) {
            float d = (float)i - c;
            v = fmaxf(v, expf(-(d * d) / (2.0f * sigma * sigma)));
        }
        int val = 40 + (int)(v * 180.0f) + (int)random(-8, 9);
        if (val < 0) val = 0; if (val > 255) val = 255;
        f.bins[i] = (uint8_t)val;
        if (val > pk) { pk = (uint8_t)val; pkBin = (uint8_t)i; }
    }
    f.peakBin = pkBin;
    f.peakDbm = -100 + (int16_t)((pk * 70) / 255);
}

// Exponential moving average per bin.
static float g_binAvg[kFftMaxBins] = {};
static bool  g_binAvgPrimed        = false;

// After fftshift, bin N/2 corresponds to DC (LO carrier leakage).
static inline void notchDc(uint8_t* bins, int N) {
    int c = N / 2;
    bins[c] = (uint8_t)(((int)bins[c - 1] + (int)bins[c + 1]) / 2);
}

// Big working buffers kept off-stack. Sized for the max FFT.
static uint32_t s_burst[kFftMaxBins];
static float    s_re[kFftMaxBins];
static float    s_im[kFftMaxBins];

static void rfTask(void*) {
    initTwiddles();
    uint32_t* const burst = s_burst;
    float*    const re    = s_re;
    float*    const im    = s_im;
    FftFrame f = {};
    uint32_t seq = 0;
    unsigned captureFails = 0;

    for (;;) {
        f.seq = seq++;
        bool haveReal = false;

        // While CW TX is active, do NOT touch the radio -- any RX setup
        // (retune, AGC, or acquireBurst) will stomp the TX path and kill
        // the on-air tone. Just sleep and let the UI continue to tick.
        if (g_cwActive) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Honor pending retune from the UI task. One-way request: UI
        //     writes g_wifiCh/g_wantSpan; we're the only consumer of the
        //     driver, so the retune runs here, on Core-0, between captures.
        SpanMode wantSpan = (SpanMode)g_wantSpan;
        uint8_t  want     = g_wifiCh;
        if ((want != g_appliedCh || wantSpan != g_appliedSpan) && espSdrReady()) {
            if (espSdrSetChannel(want)) {
                g_appliedCh   = want;
                g_appliedSpan = wantSpan;
                g_freqMHz     = espSdrFreqMHz();
                g_bwMHz       = spanBwMHz(wantSpan);
                // Invalidate smoothing so the new band/span doesn't inherit
                // the old band's floor/peak history.
                g_binAvgPrimed = false;
                g_retuneSeq++;
            }
        }
        // --- Gain / AGC pending changes
        if (g_wantAgc != g_appliedAgc && espSdrReady()) {
            if (espSdrSetAgc(g_wantAgc)) g_appliedAgc = g_wantAgc;
            g_agc = g_appliedAgc;
        }
        if (!g_appliedAgc && g_wantGain != 0xffu && g_wantGain != g_appliedGain
                          && espSdrReady()) {
            if (espSdrSetGain(g_wantGain)) g_appliedGain = g_wantGain;
        }
        // --- Pending FFT size change: validate + reset band-dependent state.
        if (g_wantFftN != g_appliedFftN) {
            uint16_t nN = g_wantFftN;
            if (nN != 128 && nN != 256) nN = 128;    // clamp to supported set
            g_appliedFftN  = nN;
            g_binAvgPrimed = false;
            g_retuneSeq++;
        }
        // --- Pending passes (max-hold dwell) change
        if (g_wantPasses != g_appliedPasses) {
            uint8_t p = g_wantPasses;
            if (p != 1 && p != 4 && p != 16 && p != 64) p = 1;
            g_appliedPasses = p;
        }
        if (g_wantAvg != g_appliedAvg) {
            uint8_t a = g_wantAvg;
            if (a != 1 && a != 2 && a != 4 && a != 8 && a != 16) a = 4;
            g_appliedAvg = a;
            g_binAvgPrimed = false;    // reset so new alpha isn't biased
        }
        const float binAlpha = 1.0f / (float)g_appliedAvg;

        // CW tone is now driven directly from the UI task via esp_phy_wifi_tx_tone
        // (vendored librftest.a). rfTask no longer runs it -- the g_cwActive gate
        // above makes rfTask yield while CW holds.

        // --- TX start request from UI task (toggle model: UI sets request,
        //     RF loops with g_txRunning as the live cancel flag).
        if (g_txRequest && !g_txRunning && espSdrReady()) {
            g_txRequest = false;
            g_txRunning = true;
            const uint8_t  ch    = (g_txChannel >= 1 && g_txChannel <= 14)
                                     ? g_txChannel : g_appliedCh;
            // Hard safety cap -- RF task will exit after this even if STOP
            // never comes (CLI disconnected, UI frozen, etc). User-selected
            // Dur chip, min 100ms, max 60s.
            uint32_t capMs = g_txDurMs;
            if (capMs < 100)   capMs = 100;
            if (capMs > 60000) capMs = 60000;
            uint32_t pacing = g_txIntervalUs;
            // Only beacon frame remains (other modes dropped; CARRIER=pace0).
            static uint8_t txFrame[sizeof(kTxBeacon)];
            memcpy(txFrame, kTxBeacon, sizeof(kTxBeacon));
            uint32_t sent = 0;
            if (g_txHop) {
                uint16_t dwellMs = g_txHopDwellMs > 10 ? g_txHopDwellMs : 10;
                uint32_t t0 = millis();
                uint8_t  hopCh = (ch >= 1 && ch <= 13) ? ch : 1;
                while (g_txRunning && (millis() - t0) < capMs) {
                    txFrame[sizeof(kTxBeacon) - 1] = hopCh;
                    uint32_t slice = dwellMs;
                    uint32_t left  = capMs - (millis() - t0);
                    if (slice > left) slice = left;
                    sent += espSdrTxBurst(txFrame, sizeof(kTxBeacon),
                                          slice, pacing, hopCh, &g_txRunning);
                    hopCh = (hopCh % 13) + 1;
                }
            } else {
                txFrame[sizeof(kTxBeacon) - 1] = ch;
                sent = espSdrTxBurst(txFrame, sizeof(kTxBeacon),
                                     capMs, pacing, ch, &g_txRunning);
            }
            g_lastTxCount    = sent;
            g_lastTxFinishMs = millis();
            g_txRunning      = false;
            g_binAvgPrimed   = false;
        }
        const uint8_t  clockIdx = (uint8_t)g_appliedSpan;
        const int      N        = g_appliedFftN;
        const int      passes   = g_appliedPasses;
        f.fftN = (uint16_t)N;

        constexpr bool kAcquireEnabled = true;
        if (kAcquireEnabled && espSdrReady()) {
            uint32_t capUs = 0;
            // Max-hold across N captures per frame. First pass seeds bins;
            // subsequent passes raise bins where that pass's code was higher.
            uint8_t pk = 0, pkBin = 0;
            bool    anyPassOk = false;
            for (int pass = 0; pass < passes; pass++) {
                if (!espSdrAcquireBurst(burst, (size_t)N, clockIdx, &capUs)) {
                    continue;
                }
                for (int i = 0; i < N; i++) {
                    unpackIq(burst[i], re[i], im[i]);
                }
                // DC removal
                float sumI = 0, sumQ = 0;
                for (int i = 0; i < N; i++) { sumI += re[i]; sumQ += im[i]; }
                float meanI = sumI / N, meanQ = sumQ / N;
                for (int i = 0; i < N; i++) { re[i] -= meanI; im[i] -= meanQ; }
                // Hann window
                for (int i = 0; i < N; i++) {
                    float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (N - 1)));
                    re[i] *= w; im[i] *= w;
                }
                fftN(re, im, N);

                // Per-bin EMA + log2 power + global noise floor (same as before,
                // but updated only on the first pass so smoothing doesn't drift
                // with max-hold spikes).
                float lpVals[kFftMaxBins];
                float lpMin =  1e9f;
                float lpMax = -1e9f;
                for (int k = 0; k < N; k++) {
                    float p = re[k] * re[k] + im[k] * im[k];
                    if (pass == 0) {
                        if (!g_binAvgPrimed) g_binAvg[k] = p;
                        else                 g_binAvg[k] += binAlpha * (p - g_binAvg[k]);
                    }
                    float lp = (g_binAvg[k] > 1.0f) ? log2f(g_binAvg[k]) : 0.0f;
                    // Overlay this pass's instantaneous power on top of EMA
                    // so a burst that doesn't survive EMA still shows up.
                    float lpPass = (p > 1.0f) ? log2f(p) : 0.0f;
                    if (lpPass > lp) lp = lpPass;
                    lpVals[k] = lp;
                    if (lp < lpMin) lpMin = lp;
                    if (lp > lpMax) lpMax = lp;
                }
                if (pass == 0) {
                    if (!g_binAvgPrimed) g_noiseFloor = lpMin;
                    else                 g_noiseFloor += kFloorEmaAlpha * (lpMin - g_noiseFloor);
                    g_binAvgPrimed = true;
                }
                const float baseline = g_noiseFloor + kFloorMargin;

                // Encode and max-hold into f.bins (with fftshift).
                for (int k = 0; k < N; k++) {
                    float d = lpVals[k] - baseline;
                    if (d < 0.0f) d = 0.0f;
                    int code = (int)(d * kCodePerLog2);
                    if (code > 255) code = 255;
                    int shifted = (k + N / 2) % N;
                    if (pass == 0 || (uint8_t)code > f.bins[shifted]) {
                        f.bins[shifted] = (uint8_t)code;
                    }
                    if (f.bins[shifted] > pk) {
                        pk    = f.bins[shifted];
                        pkBin = (uint8_t)shifted;
                    }
                }
                anyPassOk = true;
            }

            if (anyPassOk) {
                notchDc(f.bins, N);

                if ((seq % 60) == 0) {
                    int bMin = 255, bMax = 0, bSum = 0;
                    for (int i = 0; i < N; i++) {
                        if (f.bins[i] < bMin) bMin = f.bins[i];
                        if (f.bins[i] > bMax) bMax = f.bins[i];
                        bSum += f.bins[i];
                    }
                    Serial.printf("[rf] N=%d passes=%d floor=%.1f  "
                                  "bins=[min=%d avg=%d max=%d peak_bin=%d]\n",
                                  N, passes, g_noiseFloor,
                                  bMin, bSum / N, bMax, pkBin);
                }

                f.peakBin = pkBin;
                f.peakDbm = -100 + (int16_t)((pk * 70) / 255);
                haveReal = true;
                captureFails = 0;
            } else {
                captureFails++;
                if (captureFails == 1 || (captureFails % 60) == 0) {
                    Serial.printf("[rf] acquire failed, falling back to synthetic (%u)\n",
                                  captureFails);
                }
            }
        }
        if (!haveReal) fillSyntheticFrame(f, seq);

        xQueueOverwrite(g_fftQueue, &f);
        vTaskDelay(pdMS_TO_TICKS(16));
    }
}

// ----------------------------------------------------------------------------
// Drawing helpers (Wireless Wizard theme patterns)
// ----------------------------------------------------------------------------

// Fits a label inside (x,y,w,h) at the given font. Shrinks or wraps if needed.
// Simplified version of bulb-controller's drawFittedLabel -- no multi-line wrap
// here (our labels are short); drops to a smaller font if too wide.
static void drawFittedLabel(int x, int y, int w, int h, const char* label, uint8_t font = 2) {
    int maxPx = w - 8;
    uint8_t f = font;
    if (tft.textWidth(label, f) > maxPx && f > 1) f = 1;
    tft.setTextDatum(MC_DATUM);
    tft.drawString(label, x + w / 2, y + h / 2, f);
}

// The glowing rounded-rect button: three concentric border rings outside the
// body + a dark drop-shadow offset down-right. Interior is transparent (fills
// black so no previous content leaks through). Lifted directly from
// cyd-bulb-controller/src/main.cpp:drawHomeButton.
static void drawGlowButton(int x, int y, int w, int h,
                           const char* label, bool enabled = true, uint8_t font = 2) {
    if (!enabled) {
        tft.fillRoundRect(x, y, w, h, 8, color::BG);
        tft.drawRoundRect(x, y, w, h, 8, color::DIM);
        tft.setTextColor(color::DIM, color::BG);
        drawFittedLabel(x, y, w, h, label, font);
        return;
    }
    tft.drawRoundRect(x + 2, y + 2, w, h, 8, color::SHADOW);
    tft.fillRoundRect(x, y, w, h, 8, color::BG);
    tft.drawRoundRect(x - 2, y - 2, w + 4, h + 4, 10, color::DIM);
    tft.drawRoundRect(x - 1, y - 1, w + 2, h + 2, 9,  color::MID);
    tft.drawRoundRect(x, y, w, h, 8, color::BRIGHT);
    tft.setTextColor(color::BRIGHT, color::BG);
    drawFittedLabel(x, y, w, h, label, font);
}

// Two-pass glow title text: dim copy offset down-right, bright copy on top.
static void drawGlowTitle(const char* text, int x, int y, uint8_t font = 4) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString(text, x + 1, y + 1, font);
    tft.setTextColor(color::BRIGHT, color::BG);
    tft.drawString(text, x, y, font);
}

// Top-left corner "Home" button for every non-HOME screen (replaces the full
// back strip -- the wizard theme uses corner buttons, not strips).
constexpr int kCornerBtnW = 70;
constexpr int kCornerBtnH = 26;
constexpr int kCornerBtnX = 6;
constexpr int kCornerBtnY = 6;

static void drawCornerHome() {
    drawGlowButton(kCornerBtnX, kCornerBtnY, kCornerBtnW, kCornerBtnH, "Home");
}

// Same corner slot, different label -- used on screens entered from a non-Home
// screen (currently Tune, reachable from both HOME and LIVE).
static void drawCornerBack() {
    drawGlowButton(kCornerBtnX, kCornerBtnY, kCornerBtnW, kCornerBtnH, "Back");
}

static bool inCornerHome(int x, int y) {
    return x >= kCornerBtnX - 4 && x < kCornerBtnX + kCornerBtnW + 4
        && y >= kCornerBtnY - 4 && y < kCornerBtnY + kCornerBtnH + 4;
}

// Tune chip sits right of Home on the LIVE top strip (same size as Home).
constexpr int kLiveTuneX = kCornerBtnX + kCornerBtnW + 8;
static void drawLiveTune() {
    drawGlowButton(kLiveTuneX, kCornerBtnY, kCornerBtnW, kCornerBtnH, "Tune");
}
static bool inLiveTune(int x, int y) {
    return x >= kLiveTuneX - 4 && x < kLiveTuneX + kCornerBtnW + 4
        && y >= kCornerBtnY - 4 && y < kCornerBtnY + kCornerBtnH + 4;
}

// HOP toggle chip: top-right corner (opposite the Home chip), so the readings
// line between Tune and the chip has room without overlap. Tap flips SPEC/HOP;
// label shows the destination.
constexpr int kLiveHopW = 44;
constexpr int kLiveHopX = 320 - 6 - kLiveHopW;    // 270
static void drawLiveHopChip() {
    bool hopActive = (g_liveTab == LiveTab::HOP);
    drawGlowButton(kLiveHopX, kCornerBtnY, kLiveHopW, kCornerBtnH,
                   hopActive ? "SPEC" : "HOP", hopActive, 2);
}
static bool inLiveHopChip(int x, int y) {
    return x >= kLiveHopX - 4 && x < kLiveHopX + kLiveHopW + 4
        && y >= kCornerBtnY - 4 && y < kCornerBtnY + kCornerBtnH + 4;
}

// ----------------------------------------------------------------------------
// HOME screen -- Wireless Wizard glow buttons, text labels only (no icons).
// ----------------------------------------------------------------------------
struct HomeTile {
    Screen       screen;
    const char*  label;
    TuneTab      initialTab;    // only honored when screen == TUNE
};
static const HomeTile kTiles[6] = {
    { Screen::LIVE,      "Live",     TuneTab::TUNE },
    { Screen::TUNE,      "Tune",     TuneTab::TUNE },
    { Screen::TUNE,      "Gain",     TuneTab::GAIN },
    { Screen::TRIGGER,   "Trigger",  TuneTab::TUNE },
    { Screen::TRANSMIT,  "Transmit", TuneTab::TUNE },
    { Screen::SETTINGS,  "Settings", TuneTab::TUNE },
};

// 3-column by 2-row grid under the title.
constexpr int kHomeTitleY = 24;   // vertical center of title text
constexpr int kHomeGridTop = 54;
constexpr int kHomeGridLeftPad = 10;
constexpr int kHomeGridGap = 10;

static void tileRect(int idx, int& x, int& y, int& w, int& h) {
    const int cols = 3, rows = 2;
    int availW = tft.width()  - kHomeGridLeftPad * 2;
    int availH = tft.height() - kHomeGridTop - 10;
    w = (availW - (cols - 1) * kHomeGridGap) / cols;
    h = (availH - (rows - 1) * kHomeGridGap) / rows;
    int col = idx % cols, row = idx / cols;
    x = kHomeGridLeftPad + col * (w + kHomeGridGap);
    y = kHomeGridTop + row * (h + kHomeGridGap);
}

static void drawHome() {
    tft.fillScreen(color::BG);
    drawGlowTitle("Signalhound", tft.width() / 2, kHomeTitleY, 4);
    for (int i = 0; i < 6; i++) {
        int x, y, w, h; tileRect(i, x, y, w, h);
        drawGlowButton(x, y, w, h, kTiles[i].label, true, 2);
    }
}

static bool homeTouch(int tx, int ty) {
    for (int i = 0; i < 6; i++) {
        int x, y, w, h; tileRect(i, x, y, w, h);
        if (tx >= x && tx < x + w && ty >= y && ty < y + h) {
            g_screen  = kTiles[i].screen;
            if (g_screen == Screen::TUNE) g_tuneTab = kTiles[i].initialTab;
            return true;
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
// LIVE screen
//
// Layout (320x240 landscape, under 22px top bar, above 22px back strip):
//   plot area:        y 24..130 (108 px tall)   -- spectrum trace
//   waterfall:        y 134..216 (82 px tall)   -- 36 rows x 2px
//   readings row:     drawn into the top bar area
// Dirty-rect redraws plot + waterfall only; headers are static until nav.
// ----------------------------------------------------------------------------
// Vertical layout (320x240 landscape):
//   0..32    chrome       -- corner Home button + title + readings line
//   40..147  plot         -- 108px tall, 128 bins = 2.5px per bin
//   152..223 waterfall    -- 72px tall, 36 rows * 2px
//   224..239 pill strip   -- 16px tall, 4 status chips
constexpr int kPlotTop = 40;
constexpr int kPlotHeight = 108;
constexpr int kWfTop = kPlotTop + kPlotHeight + 4;    // 152
constexpr int kWfRows = 36;
constexpr int kWfRowH = 2;
constexpr int kWfHeight = kWfRows * kWfRowH;          // 72
constexpr int kPillStripTop = kWfTop + kWfHeight + 1; // 225
constexpr int kPillStripH   = 15;

static uint8_t g_wfRing[kWfRows][kFftBins] = {};
static int     g_wfHead = 0;
static int16_t g_prevY[kFftBins];       // for incremental trace draw
static bool    g_livePrimed = false;

// Peak-hold: per-bin highest code seen since the last retune, with a slow
// linear decay so stale peaks fade. Reset when g_retuneSeq advances.
static uint8_t  g_peakHold[kFftBins] = {};
static int16_t  g_prevHoldY[kFftBins];     // last-drawn hold-marker y per bin
static uint32_t g_seenRetuneSeq = 0xffffffff;    // forces first-frame reset
// UI-side smoothed trace. Max-hold aggregation on Core-0 leaves noise bins
// jittering; display gets a mild EMA so the trace reads cleanly. Waterfall
// + peak-hold still consume the raw max-held bins.
static uint8_t  g_binSmooth[kFftBins] = {};
static bool     g_binSmoothPrimed = false;
// Hold decays 1 code every N frames (~60Hz → 60/N codes/sec).
// N=2 → 30 c/s, i.e. 255→0 in ~8.5s. Feels right for a lingering peak marker.
constexpr uint8_t kPeakHoldDecayEvery = 2;
static uint32_t g_peakHoldTick = 0;

static uint16_t wfColor(uint8_t v) {
    // 6-stop cool->hot ramp matching the mockup
    static const uint8_t s[6][3] = {
        {  6,  9, 26}, { 27, 42, 90}, { 45, 79,145},
        { 79,139,182}, {212,180, 61}, {227, 74, 42}
    };
    float t = v / 255.0f;
    float x = t * 5.0f;
    int   i = (int)x; if (i > 4) i = 4;
    float f = x - i;
    uint8_t r = s[i][0] + (uint8_t)((s[i+1][0] - s[i][0]) * f);
    uint8_t g = s[i][1] + (uint8_t)((s[i+1][1] - s[i][1]) * f);
    uint8_t b = s[i][2] + (uint8_t)((s[i+1][2] - s[i][2]) * f);
    return color::rgb(r, g, b);
}

static void drawLivePills();      // forward decl -- defined after drawLiveFrame

static void drawLiveFrame(uint16_t ms) {
    const int w = tft.width();
    const int plotLeft = 6, plotRight = w - 6;
    const int plotW = plotRight - plotLeft;

    // ---------- Readings row below the title (updated each redraw) ----------
    char hdr[48];
    // Slot between Tune (ends at x=154) and HOP chip (starts at 270) is only
    // ~104 px wide; the BW is already shown in the pill strip below, so keep
    // this line to {freq | peak} and skip BW to avoid clipping into Tune.
    if (g_appliedSpan == SpanMode::S80) {
        snprintf(hdr, sizeof(hdr), "ISM %s %d",
                 g_agc ? "AGC" : "MAN", (int)g_latestFrame.peakDbm);
    } else {
        snprintf(hdr, sizeof(hdr), "%u %s %d",
                 (unsigned)g_freqMHz,
                 g_agc ? "AGC" : "MAN", (int)g_latestFrame.peakDbm);
    }
    // Readings sit in the gap between Tune (left) and HOP chip (right).
    const int kReadingsLeft  = kLiveTuneX + kCornerBtnW + 6;    // 160
    const int kReadingsRight = kLiveHopX - 6;                   // 264
    tft.fillRect(kReadingsLeft, 10, kReadingsRight - kReadingsLeft, 20, color::BG);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString(hdr, kReadingsRight, 20, 2);

    // ---------- Peak-hold update (per bin, with slow decay) ---------------
    // Reset if the RF task retuned (new band) or if the live screen just
    // entered the first frame.
    uint32_t currentSeq = g_retuneSeq;
    if (currentSeq != g_seenRetuneSeq) {
        memset(g_peakHold, 0, sizeof(g_peakHold));
        // Also clear the waterfall ring -- a size change leaves stale tails
        // in rows whose previous N was larger than the new N.
        memset(g_wfRing, 0, sizeof(g_wfRing));
        g_binSmoothPrimed = false;
        g_seenRetuneSeq = currentSeq;
    }
    const int N = g_latestFrame.fftN ? g_latestFrame.fftN : kFftMinBins;
    bool decayNow = ((g_peakHoldTick++ % kPeakHoldDecayEvery) == 0);
    uint8_t sceneMax = 0;
    // Update per-bin smoothed trace (plot) alongside peak-hold.
    // alpha ~ 0.33 (new/3 + prev*2/3). Cheap, no float.
    for (int i = 0; i < N; i++) {
        uint8_t v = g_latestFrame.bins[i];
        if (v > g_peakHold[i]) g_peakHold[i] = v;
        else if (decayNow && g_peakHold[i] > v) g_peakHold[i]--;
        if (g_peakHold[i] > sceneMax) sceneMax = g_peakHold[i];
        if (!g_binSmoothPrimed) {
            g_binSmooth[i] = v;
        } else {
            g_binSmooth[i] = (uint8_t)(((int)g_binSmooth[i] * 2 + (int)v) / 3);
        }
    }
    g_binSmoothPrimed = true;

    // ---------- Autoscale the plot y-axis -----------------------------------
    // Static 0..255 maps real peaks (code ~90) to about a third of the plot
    // height, making the trace look flat. Scale to the recent peak-hold max
    // instead, so peaks always reach near the top. Minimum floor so pure
    // noise doesn't blow up into a full-screen rain of pixels.
    constexpr uint8_t kScaleMin = 48;
    uint32_t scaleDen = sceneMax;
    if (scaleDen < kScaleMin) scaleDen = kScaleMin;
    // 15% headroom so the top of the plot is a hair above the current peak.
    scaleDen = (scaleDen * 115u) / 100u;
    if (scaleDen > 255) scaleDen = 255;
    auto codeToY = [&](uint8_t code) {
        uint32_t c = code;
        if (c > scaleDen) c = scaleDen;
        return (int)(kPlotTop + kPlotHeight - 1
                     - (c * (uint32_t)(kPlotHeight - 1)) / scaleDen);
    };

    // ---------- Trigger: edge-triggered + optional band gate ----------
    // Band gate: scan only the half of bins that matches g_trigBand so a
    // tuned-in capture doesn't trip on the other side's noise.
    int bandLo = 0, bandHi = N;
    if (g_trigBand == TrigBand::LEFT)  bandHi = N / 2;
    if (g_trigBand == TrigBand::RIGHT) bandLo = N / 2;
    uint8_t framePeak = 0;
    for (int i = bandLo; i < bandHi; i++) {
        if (g_latestFrame.bins[i] > framePeak) framePeak = g_latestFrame.bins[i];
    }
    if (g_trigEnabled) {
        if (g_trigArmed && framePeak >= g_trigThreshold) {
            g_trigFireCount++;
            g_trigLastFireMs  = millis();
            g_trigFlashFrames = 6;        // ~100 ms at 60Hz
            g_trigArmed       = false;
            // Record event to the ring buffer if recording is armed.
            if (g_recordOn && !g_recordFull) {
                CaptureEvent* ev = &g_capBuf[g_capHead];
                ev->tsMs      = g_trigLastFireMs;
                ev->peakBin   = g_latestFrame.peakBin;
                ev->peakLevel = framePeak;
                ev->band      = (uint8_t)g_trigBand;
                ev->wifiCh    = g_wifiCh;
                ev->spanMode  = g_wantSpan;
                ev->fftN      = (uint8_t)(N > 255 ? 255 : N);
                // Max-pool N bins down to 64 so a sparkline fits regardless of N.
                int stride = N / 64; if (stride < 1) stride = 1;
                for (int s = 0; s < 64; s++) {
                    int src = s * stride;
                    uint8_t m = 0;
                    for (int k = 0; k < stride && (src + k) < N; k++) {
                        if (g_latestFrame.bins[src + k] > m) m = g_latestFrame.bins[src + k];
                    }
                    ev->snap[s] = m;
                }
                g_capHead = (g_capHead + 1) % kCapCapacity;
                if (g_capCount < kCapCapacity) g_capCount++;
                else if (!g_recordRing)        g_recordFull = true;
                if (!g_recordRing && g_capCount >= kCapCapacity) g_recordFull = true;
                // Mirror to SD if the card is up. SD append is append-only;
                // RING vs ONE-SHOT only governs the RAM ring.
                if (g_sdMounted) sdAppendEvent(ev);
            }
        } else if (!g_trigArmed
                   && framePeak < (int)g_trigThreshold - (int)kTrigHysteresis) {
            g_trigArmed = true;
        }
    }

    // ---------- Spectrum trace: filled columns from baseline up ----------
    // "Flat floor along the bottom, spikes jumping up" look. Clear the whole
    // column, fill a dim-green band from the bottom up to y, cap the peak bin
    // in amber. Noise bins settle as a 1-2px band at the baseline.
    const int plotBot = kPlotTop + kPlotHeight - 1;
    for (int i = 0; i < N; i++) {
        int x = plotLeft + (i * plotW) / (N - 1);
        // Plot uses the smoothed trace to kill max-hold noise jitter;
        // waterfall + peak-hold still consume raw max-held bins.
        uint8_t v  = g_binSmooth[i];
        uint8_t vh = g_peakHold[i];
        int y  = codeToY(v);
        int yh = codeToY(vh);
        if (g_livePrimed) {
            tft.drawFastVLine(x, kPlotTop, kPlotHeight, color::BG);
        }
        if (y < plotBot) {
            tft.drawFastVLine(x, y + 1, plotBot - y, color::DIM);
        }
        // Peak-hold marker: a dim amber pixel at the all-time high for this
        // bin. Drawn BEFORE the trace peak so a live peak that reaches it
        // overwrites with bright amber on top.
        if (vh > v + 2 && yh < plotBot) {
            tft.drawPixel(x, yh, color::WARN_AMBER);
        }
        if (i == g_latestFrame.peakBin) {
            // Current-frame peak bin: bright amber 3-pixel column.
            tft.drawPixel(x, y - 1, color::WARN_AMBER);
            tft.drawPixel(x, y,     color::WARN_AMBER);
            tft.drawPixel(x, y + 1, color::WARN_AMBER);
        } else {
            tft.drawPixel(x, y, color::TRACE);
        }
        g_prevY[i]     = y;
        g_prevHoldY[i] = yh;
    }
    g_livePrimed = true;

    // ---------- Trigger threshold line + fire flash ----------
    if (g_trigEnabled) {
        int ty = codeToY(g_trigThreshold);
        // Dashed line across only the gated band -- visually makes the gate
        // obvious without a separate overlay.
        int lx1 = plotLeft + (bandLo * plotW) / N;
        int lx2 = plotLeft + ((bandHi - 1) * plotW) / N;
        for (int xd = lx1; xd < lx2; xd += 6) {
            int dashW = (xd + 3 > lx2) ? (lx2 - xd) : 3;
            if (dashW > 0) tft.drawFastHLine(xd, ty, dashW, color::WARN_AMBER);
        }
    }
    if (g_trigFlashFrames > 0) {
        tft.drawRect(5, kPlotTop - 1, tft.width() - 10, kPlotHeight + 2,
                     color::WARN_AMBER);
        g_trigFlashFrames--;
        if (g_trigFlashFrames == 0) {
            // Repaint the plot's existing border back to normal.
            tft.drawRect(5, kPlotTop - 1, tft.width() - 10, kPlotHeight + 2,
                         color::BG);
        }
    }

    // ---------- Pill-strip refresh when any pill content changed ----------
    static uint32_t s_lastPillSig = 0;
    uint32_t sig = ((uint32_t)g_appliedFftN << 16)
                 | ((uint32_t)(g_agc ? 1 : 0) << 15)
                 | ((uint32_t)(g_trigEnabled ? 1 : 0) << 14)
                 | ((uint32_t)g_appliedAvg << 24)
                 | ((uint32_t)(g_trigFireCount & 0x3fff));
    if (sig != s_lastPillSig) {
        drawLivePills();
        s_lastPillSig = sig;
    }

    // ---------- Waterfall: shift ring, paint newest row ----------
    memcpy(g_wfRing[g_wfHead], g_latestFrame.bins, (size_t)N);
    g_wfHead = (g_wfHead + 1) % kWfRows;
    for (int r = 0; r < kWfRows; r++) {
        int ring = (g_wfHead + r) % kWfRows;
        int y = kWfTop + r * kWfRowH;
        for (int i = 0; i < N; i++) {
            int x = plotLeft + (i * plotW) / (N - 1);
            uint16_t c = wfColor(g_wfRing[ring][i]);
            tft.drawFastVLine(x, y, kWfRowH, c);
        }
    }
    (void)ms;
}

// --- Bottom status pill strip: 4 chips rendered in miniature glow-button
//     style. Current state: AGC mode / averaging / peak hold / FFT size.
static void drawLivePills() {
    const int y = kPillStripTop;
    const int h = kPillStripH;
    const int n = 4;
    const int gap = 4;
    const int w = (tft.width() - 2 * gap - (n - 1) * gap) / n;
    struct Pill { const char* text; uint16_t color; };
    char agcBuf[8];
    snprintf(agcBuf, sizeof(agcBuf), "%s", g_agc ? "AGC" : "MAN");
    char fftBuf[12];
    snprintf(fftBuf, sizeof(fftBuf), "FFT %u", (unsigned)g_appliedFftN);
    char avgBuf[12];
    snprintf(avgBuf, sizeof(avgBuf), "AVG x%u", (unsigned)g_appliedAvg);
    char trigBuf[16];
    if (g_trigEnabled) {
        snprintf(trigBuf, sizeof(trigBuf), "TRIG %lu", (unsigned long)g_trigFireCount);
    } else {
        snprintf(trigBuf, sizeof(trigBuf), "%s", "PK HOLD");
    }
    uint16_t trigColor = g_trigEnabled ? color::OK_GREEN : color::WARN_AMBER;
    Pill pills[4] = {
        { agcBuf,    g_agc ? color::OK_GREEN : color::WARN_AMBER },
        { avgBuf,    color::MID },
        { trigBuf,   trigColor },
        { fftBuf,    color::DIM },
    };
    // Clear strip first (in case we re-draw after a state change)
    tft.fillRect(0, y - 1, tft.width(), h + 1, color::BG);
    int x = gap;
    for (int i = 0; i < n; i++) {
        tft.drawRoundRect(x, y, w, h, 3, pills[i].color);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(pills[i].color, color::BG);
        tft.drawString(pills[i].text, x + w / 2, y + h / 2, 1);
        x += w + gap;
    }
}

// Forward decls -- HOP tab lives further down the file.
static void drawLiveHopTab();
static void drawLiveHopTabTick();

static void drawLiveSpecTab() {
    // plot axes
    tft.drawFastHLine(6, kPlotTop + kPlotHeight, tft.width() - 12, color::DIM);
    tft.drawFastVLine(6, kPlotTop, kPlotHeight, color::DIM);
    // waterfall border
    tft.drawRect(5, kWfTop - 1, tft.width() - 10, kWfHeight + 2, color::DIM);
    // bottom status pills
    drawLivePills();

    g_livePrimed = false;
    // Reset peak-hold on LIVE entry so it starts fresh (don't carry stale
    // marks across a navigate-away-and-back).
    memset(g_peakHold, 0, sizeof(g_peakHold));
}

static void drawLive() {
    tft.fillScreen(color::BG);
    drawCornerHome();
    drawLiveTune();
    drawLiveHopChip();
    // Title removed: the readings strip (freq/BW/mode/peak) drawn by
    // drawLiveFrame identifies the SPEC view. HOP tab paints its own chrome.
    if (g_liveTab == LiveTab::HOP) {
        drawLiveHopTab();
    } else {
        drawLiveSpecTab();
    }
}

// ----------------------------------------------------------------------------
// HOP tab -- real-time hop classifier. Three sections:
//   usage bar (y 40..90):     per-BT-channel activity over last 2 s
//   time scatter (y 94..170): recent hops plotted (time x channel)
//   classifier readout:       3 lines summarizing active hoppers
// Dirty-rect approach: redraw body at ~5 Hz via drawLiveHopTab(), while
// hopLog() keeps the ring warm at full frame rate.
// ----------------------------------------------------------------------------
// Mode chip row -- added for Step B hunt-mode selector.
constexpr int kHopModeY     = 36,  kHopModeH     = 22;
constexpr int kHopUsageY    = 64,  kHopUsageH    = 40;
constexpr int kHopScatterY  = 108, kHopScatterH  = 60;
constexpr int kHopClassY    = 174, kHopClassH    = 48;
constexpr int kHopMargin    = 6;

// Classification counts over last 2 s. Written by classifier, read by renderer.
struct HopClass {
    uint8_t  usagePerBtCh[79];   // event count per BT channel (last 2 s), saturated 0..255
    uint8_t  adv0, adv24, adv78;  // event counts on the three BLE adv chs
    uint16_t wifiBand6, wifiBand11;   // event counts in BT ch bands 25..35 and 50..60
    uint16_t connCount;          // candidate BLE conn hops (dt in 5..500 ms, bin shift large)
    uint32_t connIntvEstMs;      // median dt for conn candidates
    uint16_t fastHop;            // dt < 2 ms count (BT candidate)
    uint16_t totalRecent;        // events in last 2 s
    uint8_t  distinctChs;        // count of BT ch with usage > 0
    uint8_t  topCh;              // most-used BT ch
    uint16_t topChCount;         // its hit count
    uint16_t nonTopCount;        // events NOT on topCh +/-1 (hop diversity)
};

static void classifyHops(HopClass& c) {
    memset(&c, 0, sizeof(c));
    if (g_hopCount == 0) return;
    uint32_t now = millis();
    // Walk ring from newest backwards; stop when older than 2 s.
    int    head  = g_hopHead;
    int    count = g_hopCount;
    uint16_t connDtAcc[64]; uint8_t connDtN = 0;
    const HopEvent* prev = nullptr;
    for (int i = 0; i < count; i++) {
        int    idx = (head - 1 - i + kHopRingSize) % kHopRingSize;
        const HopEvent& ev = g_hopRing[idx];
        uint32_t age = now - ev.tsMs;
        if (age > 2000) break;
        int bt = hopBinToBtCh(ev.binIdx, ev.wifiCh, ev.spanMode, 128);
        if (bt < 0) continue;   // out-of-band -- ignore
        c.totalRecent++;
        if (c.usagePerBtCh[bt] < 255) c.usagePerBtCh[bt]++;
        if      (bt == 0)   c.adv0++;
        else if (bt == 24)  c.adv24++;
        else if (bt == 78)  c.adv78++;
        if (bt >= 25 && bt <= 35) c.wifiBand6++;
        if (bt >= 50 && bt <= 60) c.wifiBand11++;
        // Dt + hop classification against the NEXT-older event (prev iter in time)
        if (prev) {
            uint32_t dt = (prev->tsMs > ev.tsMs) ? (prev->tsMs - ev.tsMs) : 0;
            int d = (int)prev->binIdx - ev.binIdx; if (d < 0) d = -d;
            // dt == 0 means same-ts multi-peak co-log, not a real hop
            if (dt >= 1 && dt < 2 && d > 4) c.fastHop++;
            if (dt >= 5 && dt <= 500 && d > 4) {
                c.connCount++;
                if (connDtN < 64) connDtAcc[connDtN++] = (uint16_t)dt;
            }
        }
        prev = &ev;
    }
    if (connDtN > 0) {
        // Pick median -- simple insertion sort of a small buffer.
        for (int i = 1; i < connDtN; i++) {
            uint16_t v = connDtAcc[i]; int j = i - 1;
            while (j >= 0 && connDtAcc[j] > v) { connDtAcc[j + 1] = connDtAcc[j]; j--; }
            connDtAcc[j + 1] = v;
        }
        c.connIntvEstMs = connDtAcc[connDtN / 2];
    }
    // Diversity metrics: a true BLE conn visits many data channels; WiFi
    // ch 6/7/11 bursts cluster tightly around one center. We need both.
    int top = 0, topCnt = 0;
    for (int bt = 0; bt < 79; bt++) {
        if (c.usagePerBtCh[bt] > 0) c.distinctChs++;
        if (c.usagePerBtCh[bt] > topCnt) { topCnt = c.usagePerBtCh[bt]; top = bt; }
    }
    c.topCh = (uint8_t)top;
    c.topChCount = (uint16_t)topCnt;
    // Events not within +/-2 bins of topCh (= off-dominant-carrier)
    int offTop = 0;
    for (int bt = 0; bt < 79; bt++) {
        if (bt >= top - 2 && bt <= top + 2) continue;
        offTop += c.usagePerBtCh[bt];
    }
    c.nonTopCount = (uint16_t)offTop;
}

// On-target: whether a given BT ch is one of the mode's hunt channels.
// Off-target events are suppressed in the usage bar + scatter so each
// mode's view only paints what it's actually hunting.
static bool hopIsOnTarget(int bt) {
    switch (g_hopMode) {
        case HopMode::ADV_CYCLE:
        case HopMode::ADV_LOCK_37:
        case HopMode::ADV_LOCK_38:
        case HopMode::ADV_LOCK_39: {
            int d0  = bt;         if (d0  < 0) d0  = -d0;
            int d24 = bt - 24;    if (d24 < 0) d24 = -d24;
            int d78 = bt - 78;    if (d78 < 0) d78 = -d78;
            return (d0 <= 3 || d24 <= 3 || d78 <= 3);
        }
        case HopMode::CONN: {
            // BLE data chs = even BT except 0 (adv 37), 24 (adv 38), 78 (adv 39)
            if (bt == 0 || bt == 24 || bt == 78) return false;
            return (bt & 1) == 0;
        }
        default: return false;    // placeholder modes paint nothing
    }
}
static uint16_t hopColForBtCh(int /*bt*/) {
    return (g_hopMode == HopMode::CONN) ? color::BRIGHT     // cyan for conn hops
                                        : color::WARN_AMBER; // amber for adv
}

// --- Usage bar: per-column dirty redraw --------------------------------
static void hopDrawUsageBar(const HopClass& cls) {
    const int baseY = kHopUsageY + kHopUsageH - 2;
    const int W = 320 - 2 * kHopMargin;
    for (int bt = 0; bt < 79; bt++) {
        // ADV mode: zero out off-target channels so we never paint them.
        // Dirty check still fires so any previously-painted bar gets
        // cleared when the mode starts.
        int usage = hopIsOnTarget(bt) ? cls.usagePerBtCh[bt] : 0;
        int newH = (usage > 15 ? 15 : usage) * (kHopUsageH - 8) / 15;
        if (newH < 0) newH = 0;
        if (newH == (int)g_hopUsageLastH[bt]) continue;
        int oldH = g_hopUsageLastH[bt];
        g_hopUsageLastH[bt] = (uint8_t)newH;
        int x0 = kHopMargin + (bt * W) / 79;
        int x1 = kHopMargin + ((bt + 1) * W) / 79;
        int w = x1 - x0 - 1; if (w < 2) w = x1 - x0;
        int clearH = (oldH > newH ? oldH : newH);
        if (clearH > 0) {
            tft.fillRect(x0, baseY - clearH, w, clearH, color::BG);
        }
        if (newH > 0) {
            tft.fillRect(x0, baseY - newH, w, newH, hopColForBtCh(bt));
        }
    }
}

// --- Scatter strip: scrolling ring, per-column dirty redraw ------------
static void hopDrawScatter() {
    const int sxL = kHopMargin + 1;
    const int sxR = 320 - kHopMargin - 1;
    const int sxW = sxR - sxL;
    const int syT = kHopScatterY + 7;
    const int syB = kHopScatterY + kHopScatterH - 3;
    const int syH = syB - syT;
    for (int c = 0; c < kScatterCols; c++) {
        // Column c (left to right) reads from ring offset after head.
        int ringIdx = (g_scatterHead + 1 + c) % kScatterCols;
        uint8_t v = g_scatterRing[ringIdx];
        if (v == g_scatterLastDrawn[c]) continue;
        g_scatterLastDrawn[c] = v;
        int x0 = sxL + (c * sxW) / kScatterCols;
        int x1 = sxL + ((c + 1) * sxW) / kScatterCols;
        int w = x1 - x0; if (w < 1) w = 1;
        // Clear the column's full inner height (fast single rect)
        tft.fillRect(x0, syT, w, syH, color::BG);
        if (v > 0) {
            int bt = v - 1;
            if (hopIsOnTarget(bt)) {
                int y = syB - (bt * (syH - 2)) / 79;
                tft.fillRect(x0, y, w, 2, hopColForBtCh(bt));
            }
            // Off-target: cleared above, no dot drawn.
        }
    }
}

// --- Classifier readout: 3 lines, text-cache dirty redraw -------------
// Classifier text is mode-aware. In ADV modes we only honestly know
// per-adv-channel hit counts; "BLE conn" / "WiFi" classification from
// SURVEY mode is removed because the narrow-span capture can't support
// those claims. Placeholder modes show a "not yet" readout.
static void hopFormatLine_ADV(int row, const HopClass& cls,
                              char* main, size_t mainSz,
                              char* sub,  size_t subSz,
                              uint16_t& markerCol) {
    static const int    kBle[3]   = { 37, 38, 39 };
    static const uint16_t kMhz[3] = { 2402, 2426, 2480 };
    markerCol = color::WARN_AMBER;
    unsigned hits = 0;
    switch (row) {
        case 0: hits = cls.adv0;  break;
        case 1: hits = cls.adv24; break;
        case 2: hits = cls.adv78; break;
    }
    // Visual cue: the ADV channel the device is currently tuned to or
    // locked on gets its row marked. In CYCLE mode the current slot
    // determines that; in LOCK mode it's fixed.
    int tuned = -1;
    if      (g_hopMode == HopMode::ADV_LOCK_37) tuned = 0;
    else if (g_hopMode == HopMode::ADV_LOCK_38) tuned = 1;
    else if (g_hopMode == HopMode::ADV_LOCK_39) tuned = 2;
    else if (g_hopMode == HopMode::ADV_CYCLE)   tuned = g_hopCycleIdx;
    const char* mark = (row == tuned) ? "*" : " ";
    snprintf(main, mainSz, "%sBLE %d:", mark, kBle[row]);
    snprintf(sub,  subSz,  "%u (%u MHz)", hits, (unsigned)kMhz[row]);
}

// CONN hunt: BLE connections hop across the 37 data channels (even BT chs
// 2..22 + 26..76). We measure how many distinct data chs got hits in the
// last 2 s, how many total data-ch events, and the median conn-candidate
// interval. Real BLE conn fingerprint = broad channel coverage + regular
// inter-hop interval. Thin coverage or no interval = probably not a conn.
static void hopFormatLine_CONN(int row, const HopClass& cls,
                               char* main, size_t mainSz,
                               char* sub,  size_t subSz,
                               uint16_t& markerCol) {
    markerCol = color::BRIGHT;
    int dataHits = 0;
    int dataChs  = 0;
    for (int bt = 2; bt < 78; bt += 2) {
        if (bt == 24) continue;    // adv 38, not data
        if (cls.usagePerBtCh[bt] > 0) {
            dataHits += cls.usagePerBtCh[bt];
            dataChs++;
        }
    }
    if (row == 0) {
        snprintf(main, mainSz, "data hits:");
        snprintf(sub,  subSz,  "%d  in %d ch", dataHits, dataChs);
    } else if (row == 1) {
        if (cls.connCount >= 5 && dataChs >= 4 && cls.connIntvEstMs >= 5) {
            snprintf(main, mainSz, "interval:");
            snprintf(sub,  subSz,  "~%u ms", (unsigned)cls.connIntvEstMs);
        } else {
            snprintf(main, mainSz, "interval:");
            snprintf(sub,  subSz,  "n/a");
        }
    } else {
        snprintf(main, mainSz, "coverage:");
        snprintf(sub,  subSz,  "%d / 37 chs", dataChs);
    }
}

static void hopFormatLine_NotImpl(int row, const char* name,
                                  char* main, size_t mainSz,
                                  char* sub,  size_t subSz,
                                  uint16_t& markerCol) {
    markerCol = color::DIM;
    if (row == 0) {
        snprintf(main, mainSz, "%s hunt:", name);
        snprintf(sub,  subSz,  "not yet");
    } else if (row == 1) {
        snprintf(main, mainSz, "coming later");
        sub[0] = 0;
    } else {
        snprintf(main, mainSz, "tap CYC/37/38/39");
        sub[0] = 0;
    }
}

static void hopFormatLine(int row, const HopClass& cls,
                          char* main, size_t mainSz,
                          char* sub,  size_t subSz,
                          uint16_t& markerCol) {
    switch (g_hopMode) {
        case HopMode::ADV_CYCLE:
        case HopMode::ADV_LOCK_37:
        case HopMode::ADV_LOCK_38:
        case HopMode::ADV_LOCK_39:
            hopFormatLine_ADV(row, cls, main, mainSz, sub, subSz, markerCol);
            break;
        case HopMode::CONN:
            hopFormatLine_CONN(row, cls, main, mainSz, sub, subSz, markerCol);
            break;
        case HopMode::WIFI:
            hopFormatLine_NotImpl(row, "WIFI", main, mainSz, sub, subSz, markerCol);
            break;
        case HopMode::ZB:
            hopFormatLine_NotImpl(row, "ZB",   main, mainSz, sub, subSz, markerCol);
            break;
    }
}

static void hopDrawClassifier(const HopClass& cls) {
    for (int row = 0; row < 3; row++) {
        char main[40]; char sub[24]; uint16_t markerCol = color::BRIGHT;
        hopFormatLine(row, cls, main, sizeof(main), sub, sizeof(sub), markerCol);
        int lineY = kHopClassY + 8 + row * 16;
        if (strcmp(main, g_hopClassMainCache[row]) != 0) {
            strncpy(g_hopClassMainCache[row], main, sizeof(g_hopClassMainCache[0]) - 1);
            g_hopClassMainCache[row][sizeof(g_hopClassMainCache[0]) - 1] = 0;
            tft.fillRect(kHopMargin, lineY - 8, 174 - kHopMargin, 16, color::BG);
            tft.fillCircle(kHopMargin + 4, lineY, 3, markerCol);
            tft.setTextDatum(ML_DATUM);
            tft.setTextColor(color::BRIGHT, color::BG);
            tft.drawString(main, kHopMargin + 14, lineY, 2);
        }
        if (strcmp(sub, g_hopClassSubCache[row]) != 0) {
            strncpy(g_hopClassSubCache[row], sub, sizeof(g_hopClassSubCache[0]) - 1);
            g_hopClassSubCache[row][sizeof(g_hopClassSubCache[0]) - 1] = 0;
            tft.fillRect(180, lineY - 8, 320 - 180 - 4, 16, color::BG);
            tft.setTextDatum(ML_DATUM);
            tft.setTextColor(color::MID, color::BG);
            tft.drawString(sub, 180, lineY, 2);
        }
    }
}

// Returns the inclusive BT-ch range the current span can see.
static void hopVisibleBtRange(int& lo, int& hi) {
    int bwMHz;
    switch (g_wantSpan) {
        case (uint8_t)SpanMode::S40: bwMHz = 40; break;
        case (uint8_t)SpanMode::S80: bwMHz = 80; break;
        default:                     bwMHz = 16; break;
    }
    int centerMhz = 2407 + g_wifiCh * 5;
    lo = centerMhz - bwMHz / 2 - 2402;
    hi = centerMhz + bwMHz / 2 - 2402;
    if (lo < 0)  lo = 0;
    if (hi > 78) hi = 78;
}

// Called once from drawLive() on entering the HOP tab: paints the fixed
// chrome (section labels + section borders) and primes dirty caches so the
// next tick fills every cell. No whole-body fillRect at any point.
// ---- Mode chip row ------------------------------------------------------
// Seven chips: CYC | 37 | 38 | 39   | CONN | WIFI | ZB
// Indices 0..3 are ADV sub-modes (active), 4..6 are placeholders (dim).
constexpr int kHopChipW   = 38;
constexpr int kHopChipGap = 5;
constexpr int kHopChipSep = 10;   // extra gap between the ADV group and the rest
static void hopModeChipRect(int idx, int& x, int& y, int& w, int& h) {
    y = kHopModeY; h = kHopModeH; w = kHopChipW;
    if (idx < 4) {
        x = kHopMargin + idx * (kHopChipW + kHopChipGap);
    } else {
        x = kHopMargin + 4 * (kHopChipW + kHopChipGap) + kHopChipSep
          + (idx - 4) * (kHopChipW + kHopChipGap);
    }
}
static const char* const kHopChipLbl[7] = { "CYC", "37", "38", "39", "CONN", "WIFI", "ZB" };

static void drawHopModeChips() {
    for (int i = 0; i < 7; i++) {
        int x, y, w, h; hopModeChipRect(i, x, y, w, h);
        // Enabled hunts: 0..3 (ADV sub-modes) + 4 (CONN). 5=WIFI, 6=ZB stubs.
        bool enabled = (i <= 4);
        bool on      = ((int)g_hopMode == i);
        if (enabled) {
            drawGlowButton(x, y, w, h, kHopChipLbl[i], on, 2);
        } else if (on) {
            // Selected placeholder: amber ring + amber text to signal
            // "mode selected but not implemented yet."
            tft.drawRoundRect(x, y, w, h, 4, color::WARN_AMBER);
            tft.setTextDatum(MC_DATUM);
            tft.setTextColor(color::WARN_AMBER, color::BG);
            tft.drawString(kHopChipLbl[i], x + w / 2, y + h / 2, 1);
        } else {
            // Idle placeholder: dim ring + dim text.
            tft.drawRoundRect(x, y, w, h, 4, color::DIM);
            tft.setTextDatum(MC_DATUM);
            tft.setTextColor(color::DIM, color::BG);
            tft.drawString(kHopChipLbl[i], x + w / 2, y + h / 2, 1);
        }
    }
}
static bool inHopModeChip(int tx, int ty, int& outIdx) {
    if (ty < kHopModeY - 4 || ty >= kHopModeY + kHopModeH + 4) return false;
    for (int i = 0; i < 7; i++) {
        int x, y, w, h; hopModeChipRect(i, x, y, w, h);
        if (tx >= x - 2 && tx < x + w + 2) { outIdx = i; return true; }
    }
    return false;
}

// ---- Mode state transitions ---------------------------------------------
static void hopApplyMode(HopMode m) {
    g_hopMode = m;
    switch (m) {
        case HopMode::ADV_LOCK_37:
            g_wifiCh = kAdvTunes[0].wifiCh; g_wantSpan = kAdvTunes[0].span;
            break;
        case HopMode::ADV_LOCK_38:
            g_wifiCh = kAdvTunes[1].wifiCh; g_wantSpan = kAdvTunes[1].span;
            break;
        case HopMode::ADV_LOCK_39:
            g_wifiCh = kAdvTunes[2].wifiCh; g_wantSpan = kAdvTunes[2].span;
            break;
        case HopMode::ADV_CYCLE:
            g_hopCycleIdx    = 0;
            g_wifiCh         = kAdvTunes[0].wifiCh;
            g_wantSpan       = kAdvTunes[0].span;
            g_hopCycleNextMs = millis() + kHopCycleDwellMs;
            break;
        case HopMode::CONN:
            g_hopCycleIdx    = 0;
            g_wifiCh         = kConnTunes[0].wifiCh;
            g_wantSpan       = kConnTunes[0].span;
            g_hopCycleNextMs = millis() + kHopCycleDwellMs;
            break;
        default: break;  // WIFI/ZB stubs
    }
    // Invalidate per-bin floor so the previous tune's noise profile doesn't
    // mask fresh events on the new center freq.
    g_binFloorPrimed = false;
}

static void hopEnterTab() {
    if (!g_hopTuneSaved) {
        g_hopSaveCh    = g_wifiCh;
        g_hopSaveSpan  = g_wantSpan;
        g_hopTuneSaved = true;
    }
    hopApplyMode(g_hopMode);
}
static void hopExitTab() {
    if (g_hopTuneSaved) {
        g_wifiCh       = g_hopSaveCh;
        g_wantSpan     = g_hopSaveSpan;
        g_hopTuneSaved = false;
    }
}
// Called each frame-drain tick when HOP tab is up. Advances the per-mode
// cycle by retuning every kHopCycleDwellMs milliseconds. ADV_CYCLE rotates
// through 3 adv tunes; CONN rotates between 2 data-ch tunes.
static void hopCycleTick() {
    uint32_t now = millis();
    if ((int32_t)(now - g_hopCycleNextMs) < 0) return;
    if (g_hopMode == HopMode::ADV_CYCLE) {
        g_hopCycleIdx    = (g_hopCycleIdx + 1) % 3;
        g_wifiCh         = kAdvTunes[g_hopCycleIdx].wifiCh;
        g_wantSpan       = kAdvTunes[g_hopCycleIdx].span;
    } else if (g_hopMode == HopMode::CONN) {
        g_hopCycleIdx    = (g_hopCycleIdx + 1) % 2;
        g_wifiCh         = kConnTunes[g_hopCycleIdx].wifiCh;
        g_wantSpan       = kConnTunes[g_hopCycleIdx].span;
    } else {
        return;    // non-cycling modes
    }
    g_hopCycleNextMs = now + kHopCycleDwellMs;
    g_binFloorPrimed = false;
}

static void drawLiveHopTab() {
    // Mode chip row (static chrome for this tab)
    tft.fillRect(0, kHopModeY - 2, 320, kHopModeH + 4, color::BG);
    drawHopModeChips();

    // Section 1 header + baseline (fixed, drawn once per enter)
    tft.fillRect(0, kHopUsageY - 2, 320, kHopUsageH + 4, color::BG);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString("usage 2s", kHopMargin, kHopUsageY, 1);
    const int baseY = kHopUsageY + kHopUsageH - 2;
    tft.drawFastHLine(kHopMargin, baseY + 1, 320 - 2 * kHopMargin, color::DIM);
    // Visible-band bracket: dim outline on the usage bar showing which BT
    // channels the current tune actually covers (16/40/80 MHz span).
    int vLo, vHi; hopVisibleBtRange(vLo, vHi);
    const int W = 320 - 2 * kHopMargin;
    int bx0 = kHopMargin + (vLo * W) / 79;
    int bx1 = kHopMargin + ((vHi + 1) * W) / 79;
    tft.drawRect(bx0, kHopUsageY + 10, bx1 - bx0, kHopUsageH - 12, color::DIM);

    // Section 2 header + scatter border
    tft.fillRect(0, kHopScatterY, 320, kHopScatterH, color::BG);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString("time -> now  5s", kHopMargin, kHopScatterY + 3, 1);
    tft.drawRect(kHopMargin, kHopScatterY + 6, 320 - 2 * kHopMargin,
                 kHopScatterH - 8, color::DIM);

    // Section 3 divider
    tft.fillRect(0, kHopClassY - 2, 320, kHopClassH + 2, color::BG);
    tft.drawFastHLine(kHopMargin, kHopClassY - 2, 320 - 2 * kHopMargin, color::DIM);

    // Reset dirty caches so the next paint fills every live cell from BG.
    memset(g_hopUsageLastH,    0, sizeof(g_hopUsageLastH));
    memset(g_scatterLastDrawn, 0, sizeof(g_scatterLastDrawn));
    memset(g_hopClassMainCache, 0, sizeof(g_hopClassMainCache));
    memset(g_hopClassSubCache,  0, sizeof(g_hopClassSubCache));
    g_hopLastRateShown = -1;
    g_hopTabPrimed = true;
    // First-paint pass: push current state onto screen through the dirty path.
    HopClass cls; classifyHops(cls);
    hopDrawUsageBar(cls);
    hopDrawScatter();
    hopDrawClassifier(cls);
}

// Incremental update, called from the frame-drain tick when HOP tab is up.
static void drawLiveHopTabTick() {
    if (!g_hopTabPrimed) return;
    HopClass cls; classifyHops(cls);
    hopDrawUsageBar(cls);
    hopDrawScatter();
    hopDrawClassifier(cls);
    // Hops/sec diagnostic: totalRecent is a 2s count, divide for /s.
    int rate = cls.totalRecent / 2;
    if (rate != g_hopLastRateShown) {
        g_hopLastRateShown = rate;
        char buf[20];
        snprintf(buf, sizeof(buf), "%d/s  ch 0..78", rate);
        tft.fillRect(220, kHopUsageY - 6, 94, 12, color::BG);
        tft.setTextDatum(MR_DATUM);
        tft.setTextColor(color::DIM, color::BG);
        tft.drawString(buf, 314, kHopUsageY, 1);
    }
}

// ----------------------------------------------------------------------------
// Placeholder screens (Phase 1): all screens other than HOME and LIVE
// ----------------------------------------------------------------------------
static void drawPlaceholder(Screen s) {
    tft.fillScreen(color::BG);
    drawCornerHome();
    drawGlowTitle(kScreenName[(int)s], tft.width() / 2 + 20, 20, 4);

    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString("Coming in Phase 2", tft.width() / 2, tft.height() / 2 - 10, 4);
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString("Tap Home to return",
                   tft.width() / 2, tft.height() / 2 + 20, 2);
}

// ----------------------------------------------------------------------------
// TUNE screen: pick a WiFi channel (1..13) and the RF task retunes.
// Layout (320x240):
//   y 36..64  WIDE strip (full width)
//   y 72..102 Presets row -- 6 tiles across
//   y 128..230 WiFi channel grid -- 5 cols x 3 rows (1-5, 6-10, 11-13)
constexpr int kTuneLeft   = 10;
constexpr int kTuneRight  = 310;
constexpr int kTuneGridW  = kTuneRight - kTuneLeft;   // 300

// WIDE strip
constexpr int kWideY = 36, kWideH = 28;

// Presets row
constexpr int kPresetY   = 72;
constexpr int kPresetH   = 30;
constexpr int kPresetGap = 4;
constexpr int kPresetW   = (kTuneGridW - (kNumPresets - 1) * kPresetGap) / kNumPresets;

// WiFi channel grid
constexpr int kTuneCols    = 5;
constexpr int kTuneRows    = 3;
constexpr int kTuneGridTop = 128;
constexpr int kTuneGridBot = 232;
constexpr int kTuneGap     = 6;

static void wideTileRect(int& x, int& y, int& w, int& h) {
    x = kTuneLeft; y = kWideY; w = kTuneGridW; h = kWideH;
}
static void presetTileRect(int idx, int& x, int& y, int& w, int& h) {
    w = kPresetW; h = kPresetH;
    x = kTuneLeft + idx * (kPresetW + kPresetGap);
    y = kPresetY;
}
static void tuneTileRect(int ch, int& x, int& y, int& w, int& h) {
    int i = ch - 1;
    int col = i % kTuneCols;
    int row = i / kTuneCols;
    int gridH = kTuneGridBot - kTuneGridTop;
    w = (kTuneGridW - (kTuneCols - 1) * kTuneGap) / kTuneCols;
    h = (gridH       - (kTuneRows - 1) * kTuneGap) / kTuneRows;
    x = kTuneLeft    + col * (w + kTuneGap);
    y = kTuneGridTop + row * (h + kTuneGap);
}

static bool presetMatchesApplied(const TunePreset& p) {
    return (p.wifiCh == g_appliedCh) && (p.span == g_appliedSpan);
}
static bool wifiChActive(int ch) {
    return (g_appliedSpan == SpanMode::S16) && ((uint8_t)ch == g_appliedCh);
}

static void drawTuneTabContent() {
    // WIDE strip (80 MS/s full-band)
    {
        int x, y, w, h; wideTileRect(x, y, w, h);
        bool active = (g_appliedSpan == SpanMode::S80);
        drawGlowButton(x, y, w, h,
                       active ? "WIDE  2402-2482" : "WIDE  (full band)",
                       active, 2);
    }

    // Presets row -- each tile shows label + sub freq
    for (int i = 0; i < kNumPresets; i++) {
        int x, y, w, h; presetTileRect(i, x, y, w, h);
        bool active = presetMatchesApplied(kPresets[i]);
        drawGlowButton(x, y, w, h, "", active, 2);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(active ? color::BRIGHT : color::MID, color::BG);
        tft.drawString(kPresets[i].label, x + w / 2, y + 9, 1);
        tft.setTextColor(active ? color::MID : color::DIM, color::BG);
        tft.drawString(kPresets[i].sub,   x + w / 2, y + 21, 1);
    }

    // Faint divider + section label between presets and channel grid
    tft.drawFastHLine(kTuneLeft, 110, kTuneGridW, color::DIM);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString("WiFi CHANNELS", tft.width() / 2, 118, 1);

    // WiFi channel grid 1..13
    for (int ch = 1; ch <= 13; ch++) {
        int x, y, w, h; tuneTileRect(ch, x, y, w, h);
        char label[8];
        snprintf(label, sizeof(label), "%d", ch);
        drawGlowButton(x, y, w, h, label, wifiChActive(ch), 4);
    }
}

static bool tuneTabTouch(int tx, int ty) {
    // WIDE strip
    {
        int x, y, w, h; wideTileRect(x, y, w, h);
        if (tx >= x && tx < x + w && ty >= y && ty < y + h) {
            g_wifiCh   = 7;                               // ch 7 (2442) centers ISM
            g_wantSpan = (uint8_t)SpanMode::S80;
            return true;
        }
    }
    // Presets row
    for (int i = 0; i < kNumPresets; i++) {
        int x, y, w, h; presetTileRect(i, x, y, w, h);
        if (tx >= x && tx < x + w && ty >= y && ty < y + h) {
            g_wifiCh   = kPresets[i].wifiCh;
            g_wantSpan = (uint8_t)kPresets[i].span;
            return true;
        }
    }
    // WiFi grid
    for (int ch = 1; ch <= 13; ch++) {
        int x, y, w, h; tuneTileRect(ch, x, y, w, h);
        if (tx >= x && tx < x + w && ty >= y && ty < y + h) {
            g_wifiCh   = (uint8_t)ch;
            g_wantSpan = (uint8_t)SpanMode::S16;
            return true;
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
// GAIN tab of the Tune screen: AGC on/off toggle + manual gain adjuster.
// Shares the top row with the tab bar (drawn by drawTune dispatcher).
// ----------------------------------------------------------------------------
constexpr int kGainMargin  = 10;
constexpr int kAgcY        = 42, kAgcH   = 36;
constexpr int kAdjY        = 84, kAdjH   = 36;        // -10 -1 VAL +1 +10 row
constexpr int kBarY        = 126, kBarH  = 8;
constexpr int kBarPadX     = 20;
constexpr int kDwellRowY   = 140, kDwellRowH = 22;
constexpr int kFftRowY2    = 166, kFftRowH2  = 22;
constexpr int kAvgRowY     = 192, kAvgRowH   = 22;

// Five tiles in the ±1/±10 adjuster row: -10, -1, (readout), +1, +10.
// The readout is wider than the step buttons.
static void gainAdjRects(int& xM10, int& xM1, int& xV, int& xP1, int& xP10,
                         int& wStep, int& wVal, int& h) {
    const int gap = 6;
    const int left = kGainMargin, right = 320 - kGainMargin;
    const int totalW = right - left;
    wStep = 48;
    wVal  = totalW - 4 * (wStep + gap);
    h     = kAdjH;
    xM10 = left;
    xM1  = xM10 + wStep + gap;
    xV   = xM1  + wStep + gap;
    xP1  = xV   + wVal  + gap;
    xP10 = xP1  + wStep + gap;
}

static void drawGainTabContent() {
    // --- AGC toggle (full-width glow button) ---
    {
        const int x = kGainMargin, w = 320 - 2 * kGainMargin;
        drawGlowButton(x, kAgcY, w, kAgcH, "", true, 2);
        char label[32];
        snprintf(label, sizeof(label),
                 g_appliedAgc ? "AGC  ON    (auto)" : "AGC  OFF   (manual)");
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(g_appliedAgc ? color::OK_GREEN : color::WARN_AMBER,
                         color::BG);
        tft.drawString(label, x + w / 2, kAgcY + kAgcH / 2, 4);
    }

    // --- Manual gain adjuster (dimmed when AGC is on) ---
    const bool manActive = !g_appliedAgc;
    int xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow;
    gainAdjRects(xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow);

    drawGlowButton(xM10, kAdjY, wStep, hRow, "-10", manActive, 2);
    drawGlowButton(xM1,  kAdjY, wStep, hRow, "-1",  manActive, 2);
    drawGlowButton(xP1,  kAdjY, wStep, hRow, "+1",  manActive, 2);
    drawGlowButton(xP10, kAdjY, wStep, hRow, "+10", manActive, 2);

    // Readout tile (not a button) -- font 2 since the row is slimmer now
    tft.drawRect(xV, kAdjY, wVal, hRow, manActive ? color::MID : color::DIM);
    char val[16];
    snprintf(val, sizeof(val), "%u / %u", (unsigned)g_appliedGain,
             (unsigned)(espSdrGainMax() ? espSdrGainMax() : 72));
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(manActive ? color::BRIGHT : color::DIM, color::BG);
    tft.drawString(val, xV + wVal / 2, kAdjY + hRow / 2, 2);

    // --- Bar visualization: scaled 0..gain_max across the whole content width
    const uint8_t gmax = espSdrGainMax() ? espSdrGainMax() : 72;
    const int barX = kGainMargin + kBarPadX;
    const int barW = 320 - 2 * (kGainMargin + kBarPadX);
    tft.drawRect(barX, kBarY, barW, kBarH, manActive ? color::MID : color::DIM);
    const int filled = (int)((g_appliedGain * (uint32_t)(barW - 2)) / (uint32_t)gmax);
    if (manActive && filled > 0) {
        tft.fillRect(barX + 1, kBarY + 1, filled, kBarH - 2, color::BRIGHT);
    }

    // --- Dwell (max-hold passes per frame) selector row ---
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString("Dwell", kGainMargin, kDwellRowY + kDwellRowH / 2, 2);
    const uint8_t dwVals[4] = { 1, 4, 16, 64 };
    const int dwBaseX = 70, dwW = 56, dwGap = 6;
    for (int i = 0; i < 4; i++) {
        int tx = dwBaseX + i * (dwW + dwGap);
        char lbl[8]; snprintf(lbl, sizeof(lbl), "x%u", (unsigned)dwVals[i]);
        drawGlowButton(tx, kDwellRowY, dwW, kDwellRowH, lbl,
                       g_appliedPasses == dwVals[i], 2);
    }

    // --- FFT bins selector row ---
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString("FFT", kGainMargin, kFftRowY2 + kFftRowH2 / 2, 2);
    const uint16_t sizes[2] = { 128, 256 };
    const int fftBaseX = 70, fftW = 118, fftGap = 6;
    for (int i = 0; i < 2; i++) {
        int tx = fftBaseX + i * (fftW + fftGap);
        char lbl[8]; snprintf(lbl, sizeof(lbl), "%u", (unsigned)sizes[i]);
        drawGlowButton(tx, kFftRowY2, fftW, kFftRowH2, lbl,
                       g_appliedFftN == sizes[i], 2);
    }

    // --- Averaging strength selector row (x1 raw .. x16 heavy) ---
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString("Avg", kGainMargin, kAvgRowY + kAvgRowH / 2, 2);
    const uint8_t avgVals[5] = { 1, 2, 4, 8, 16 };
    const int avBaseX = 70, avW = 44, avGap = 6;
    for (int i = 0; i < 5; i++) {
        int tx = avBaseX + i * (avW + avGap);
        char lbl[8]; snprintf(lbl, sizeof(lbl), "x%u", (unsigned)avgVals[i]);
        drawGlowButton(tx, kAvgRowY, avW, kAvgRowH, lbl,
                       g_appliedAvg == avgVals[i], 2);
    }
}

static bool gainTabTouch(int tx, int ty) {
    const int wAgc = 320 - 2 * kGainMargin;
    if (tx >= kGainMargin && tx < kGainMargin + wAgc
        && ty >= kAgcY && ty < kAgcY + kAgcH) {
        g_wantAgc = !g_appliedAgc;
        return true;
    }
    // Dwell row (max-hold passes)
    if (ty >= kDwellRowY && ty < kDwellRowY + kDwellRowH) {
        const int dwBaseX = 70, dwW = 56, dwGap = 6;
        const uint8_t dwVals[4] = { 1, 4, 16, 64 };
        for (int i = 0; i < 4; i++) {
            int bx = dwBaseX + i * (dwW + dwGap);
            if (tx >= bx && tx < bx + dwW) {
                g_wantPasses = dwVals[i];
                return true;
            }
        }
    }
    // FFT bins row
    if (ty >= kFftRowY2 && ty < kFftRowY2 + kFftRowH2) {
        const int fftBaseX = 70, fftW = 118, fftGap = 6;
        const uint16_t sizes[2] = { 128, 256 };
        for (int i = 0; i < 2; i++) {
            int bx = fftBaseX + i * (fftW + fftGap);
            if (tx >= bx && tx < bx + fftW) {
                g_wantFftN = sizes[i];
                return true;
            }
        }
    }
    // Avg row
    if (ty >= kAvgRowY && ty < kAvgRowY + kAvgRowH) {
        const int avBaseX = 70, avW = 44, avGap = 6;
        const uint8_t avgVals[5] = { 1, 2, 4, 8, 16 };
        for (int i = 0; i < 5; i++) {
            int bx = avBaseX + i * (avW + avGap);
            if (tx >= bx && tx < bx + avW) {
                g_wantAvg = avgVals[i];
                return true;
            }
        }
    }
    if (g_appliedAgc) return false;      // adjuster disabled while AGC on
    int xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow;
    gainAdjRects(xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow);
    if (ty < kAdjY || ty >= kAdjY + hRow) return false;
    int delta = 0;
    if      (tx >= xM10 && tx < xM10 + wStep) delta = -10;
    else if (tx >= xM1  && tx < xM1  + wStep) delta = -1;
    else if (tx >= xP1  && tx < xP1  + wStep) delta =  1;
    else if (tx >= xP10 && tx < xP10 + wStep) delta = 10;
    else return false;
    const uint8_t gmax = espSdrGainMax() ? espSdrGainMax() : 72;
    int next = (int)g_appliedGain + delta;
    if (next < 0)       next = 0;
    if (next > gmax)    next = gmax;
    g_wantGain = (uint8_t)next;
    return true;
}

// ----------------------------------------------------------------------------
// Tune screen dispatcher: Back button + tab bar + active tab's content.
// Tab bar sits in the top row next to Back (no separate title).
// ----------------------------------------------------------------------------
constexpr int kTuneTabLeft = kCornerBtnX + kCornerBtnW + 8;    // 84
constexpr int kTuneTabGap  = 4;
constexpr int kTuneTabW    = (320 - kTuneTabLeft - 10 - kTuneTabGap) / 2;  // 112
constexpr int kTuneTabY    = kCornerBtnY;
constexpr int kTuneTabH    = kCornerBtnH;

static void drawTuneTabBar() {
    const int x1 = kTuneTabLeft;
    const int x2 = kTuneTabLeft + kTuneTabW + kTuneTabGap;
    drawGlowButton(x1, kTuneTabY, kTuneTabW, kTuneTabH,
                   "TUNE", g_tuneTab == TuneTab::TUNE, 2);
    drawGlowButton(x2, kTuneTabY, kTuneTabW, kTuneTabH,
                   "GAIN", g_tuneTab == TuneTab::GAIN, 2);
}

static bool inTuneTabBar(int tx, int ty, TuneTab& which) {
    if (ty < kTuneTabY - 4 || ty >= kTuneTabY + kTuneTabH + 4) return false;
    const int x1 = kTuneTabLeft;
    const int x2 = kTuneTabLeft + kTuneTabW + kTuneTabGap;
    if (tx >= x1 - 4 && tx < x1 + kTuneTabW + 4) { which = TuneTab::TUNE; return true; }
    if (tx >= x2 - 4 && tx < x2 + kTuneTabW + 4) { which = TuneTab::GAIN; return true; }
    return false;
}

static void drawTune() {
    tft.fillScreen(color::BG);
    drawCornerBack();
    drawTuneTabBar();
    if (g_tuneTab == TuneTab::TUNE) drawTuneTabContent();
    else                            drawGainTabContent();
}

static bool tuneTouch(int tx, int ty) {
    TuneTab which;
    if (inTuneTabBar(tx, ty, which)) {
        if (which != g_tuneTab) { g_tuneTab = which; return true; }
        return false;
    }
    return (g_tuneTab == TuneTab::TUNE) ? tuneTabTouch(tx, ty)
                                        : gainTabTouch(tx, ty);
}

// ----------------------------------------------------------------------------
// TRIGGER screen -- enable toggle + threshold adjuster + fire-count readout.
// ----------------------------------------------------------------------------
constexpr int kTrigMargin   = 10;
constexpr int kTrigEnableY  = 44, kTrigEnableH = 44;
constexpr int kTrigAdjY     = 102, kTrigAdjH   = 42;
constexpr int kTrigBarY     = 158, kTrigBarH   = 10;
constexpr int kTrigStatY    = 190;

static void trigAdjRects(int& xM10, int& xM1, int& xV, int& xP1, int& xP10,
                         int& wStep, int& wVal, int& h) {
    const int gap = 6;
    const int left = kTrigMargin, right = 320 - kTrigMargin;
    const int totalW = right - left;
    wStep = 48;
    wVal  = totalW - 4 * (wStep + gap);
    h     = kTrigAdjH;
    xM10 = left;
    xM1  = xM10 + wStep + gap;
    xV   = xM1  + wStep + gap;
    xP1  = xV   + wVal  + gap;
    xP10 = xP1  + wStep + gap;
}

// -----------------------------------------------------------------------------
// TRIGGER tab bar -- CFG | REC | CAPS, right of the corner Home button.
// -----------------------------------------------------------------------------
constexpr int kTrigTabH    = 26;
constexpr int kTrigTabY    = 6;
constexpr int kTrigTabW    = 60;
constexpr int kTrigTabGap  = 6;
constexpr int kTrigTabX0   = kCornerBtnX + kCornerBtnW + 10;   // right of Home

static void trigTabRect(int idx, int& x, int& y, int& w, int& h) {
    x = kTrigTabX0 + idx * (kTrigTabW + kTrigTabGap);
    y = kTrigTabY; w = kTrigTabW; h = kTrigTabH;
}
static void drawTrigTabBar() {
    static const char* lbl[3] = { "CFG", "REC", "CAPS" };
    for (int i = 0; i < 3; i++) {
        int x, y, w, h; trigTabRect(i, x, y, w, h);
        drawGlowButton(x, y, w, h, lbl[i], (int)g_trigTab == i, 2);
    }
}
static bool inTrigTabBar(int tx, int ty, TrigTab& which) {
    if (ty < kTrigTabY - 4 || ty >= kTrigTabY + kTrigTabH + 4) return false;
    for (int i = 0; i < 3; i++) {
        int x, y, w, h; trigTabRect(i, x, y, w, h);
        if (tx >= x - 4 && tx < x + w + 4) {
            which = (TrigTab)i;
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------------
// CFG tab -- the original threshold / band / fires UI (unchanged behavior).
// -----------------------------------------------------------------------------
static void drawTrigCfgTab() {
    // Enable toggle (full-width)
    const int x = kTrigMargin, w = 320 - 2 * kTrigMargin;
    drawGlowButton(x, kTrigEnableY, w, kTrigEnableH, "", true, 2);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(g_trigEnabled ? color::OK_GREEN : color::DIM, color::BG);
    tft.drawString(g_trigEnabled ? "TRIGGER  ON" : "TRIGGER  OFF",
                   x + w / 2, kTrigEnableY + kTrigEnableH / 2, 4);

    // Threshold adjuster
    int xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow;
    trigAdjRects(xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow);
    drawGlowButton(xM10, kTrigAdjY, wStep, hRow, "-10", true, 2);
    drawGlowButton(xM1,  kTrigAdjY, wStep, hRow, "-1",  true, 2);
    drawGlowButton(xP1,  kTrigAdjY, wStep, hRow, "+1",  true, 2);
    drawGlowButton(xP10, kTrigAdjY, wStep, hRow, "+10", true, 2);
    tft.drawRect(xV, kTrigAdjY, wVal, hRow, color::MID);
    char val[16];
    snprintf(val, sizeof(val), "%u", (unsigned)g_trigThreshold);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::BRIGHT, color::BG);
    tft.drawString(val, xV + wVal / 2, kTrigAdjY + hRow / 2, 4);

    // Position bar 0..255
    const int barX = kTrigMargin + 20;
    const int barW = 320 - 2 * (kTrigMargin + 20);
    tft.drawRect(barX, kTrigBarY, barW, kTrigBarH, color::MID);
    int filled = (g_trigThreshold * (barW - 2)) / 255;
    if (filled > 0) tft.fillRect(barX + 1, kTrigBarY + 1, filled, kTrigBarH - 2,
                                 color::WARN_AMBER);

    // Status readout: fire count + last fired (age in seconds)
    char status[48];
    if (g_trigFireCount == 0) {
        snprintf(status, sizeof(status), "fires: 0");
    } else {
        uint32_t ageMs = millis() - g_trigLastFireMs;
        snprintf(status, sizeof(status), "fires: %lu   last: %lus ago",
                 (unsigned long)g_trigFireCount,
                 (unsigned long)(ageMs / 1000));
    }
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString(status, tft.width() / 2, kTrigStatY, 2);

    // Band selector row (ALL / LEFT / RIGHT)
    const int bandRowY = 208, bandRowH = 24;
    const int bandLeft = kTrigMargin, bandRight = 320 - kTrigMargin;
    const int bandGap  = 6;
    const int bandW    = (bandRight - bandLeft - 2 * bandGap) / 3;
    const char* labels[3] = { "ALL", "LEFT", "RIGHT" };
    const TrigBand vals[3] = { TrigBand::ALL, TrigBand::LEFT, TrigBand::RIGHT };
    for (int i = 0; i < 3; i++) {
        int bx = bandLeft + i * (bandW + bandGap);
        drawGlowButton(bx, bandRowY, bandW, bandRowH,
                       labels[i], g_trigBand == vals[i], 2);
    }
}

static bool trigCfgTouch(int tx, int ty) {
    // Enable toggle
    const int w = 320 - 2 * kTrigMargin;
    if (tx >= kTrigMargin && tx < kTrigMargin + w
        && ty >= kTrigEnableY && ty < kTrigEnableY + kTrigEnableH) {
        g_trigEnabled = !g_trigEnabled;
        if (g_trigEnabled) {
            // Starting fresh: reset counter so stats reflect this session,
            // and arm so the first crossing fires.
            g_trigFireCount  = 0;
            g_trigLastFireMs = 0;
            g_trigArmed      = true;
        }
        return true;
    }
    // Band selector row
    {
        const int bandRowY = 208, bandRowH = 24;
        const int bandLeft = kTrigMargin, bandRight = 320 - kTrigMargin;
        const int bandGap  = 6;
        const int bandW    = (bandRight - bandLeft - 2 * bandGap) / 3;
        const TrigBand vals[3] = { TrigBand::ALL, TrigBand::LEFT, TrigBand::RIGHT };
        if (ty >= bandRowY && ty < bandRowY + bandRowH) {
            for (int i = 0; i < 3; i++) {
                int bx = bandLeft + i * (bandW + bandGap);
                if (tx >= bx && tx < bx + bandW) {
                    g_trigBand  = vals[i];
                    g_trigArmed = true;    // re-arm when band changes
                    return true;
                }
            }
        }
    }
    // Threshold adjuster
    int xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow;
    trigAdjRects(xM10, xM1, xV, xP1, xP10, wStep, wVal, hRow);
    if (ty < kTrigAdjY || ty >= kTrigAdjY + hRow) return false;
    int delta = 0;
    if      (tx >= xM10 && tx < xM10 + wStep) delta = -10;
    else if (tx >= xM1  && tx < xM1  + wStep) delta = -1;
    else if (tx >= xP1  && tx < xP1  + wStep) delta =  1;
    else if (tx >= xP10 && tx < xP10 + wStep) delta = 10;
    else return false;
    int next = (int)g_trigThreshold + delta;
    if (next < 0)   next = 0;
    if (next > 255) next = 255;
    g_trigThreshold = (uint8_t)next;
    return true;
}

// -----------------------------------------------------------------------------
// REC tab -- Record toggle, Ring/One-Shot mode, buffer bar, Clear.
// -----------------------------------------------------------------------------
constexpr int kRecRecY   = 44, kRecRecH   = 44;    // record toggle
constexpr int kRecModeY  = 100, kRecModeH = 28;    // ring/one-shot
constexpr int kRecBarY   = 142, kRecBarH  = 18;
constexpr int kRecStatY  = 172;
constexpr int kRecClearY = 192, kRecClearH = 36;

static void drawTrigRecTab() {
    // Record toggle
    const int x = kTrigMargin, w = 320 - 2 * kTrigMargin;
    drawGlowButton(x, kRecRecY, w, kRecRecH, "", true, 2);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(g_recordOn ? color::OK_GREEN : color::DIM, color::BG);
    tft.drawString(g_recordOn ? (g_recordFull ? "RECORD  FULL" : "RECORD  ON")
                              : "RECORD  OFF",
                   x + w / 2, kRecRecY + kRecRecH / 2, 4);

    // Mode row: RING | ONE-SHOT
    const int halfW = (w - 6) / 2;
    drawGlowButton(x,           kRecModeY, halfW, kRecModeH, "RING",      g_recordRing,  2);
    drawGlowButton(x + halfW+6, kRecModeY, halfW, kRecModeH, "ONE-SHOT", !g_recordRing, 2);

    // Buffer bar
    const int barX = x + 20, barW = w - 40;
    tft.drawRect(barX, kRecBarY, barW, kRecBarH, color::MID);
    int filled = (g_capCount * (barW - 2)) / kCapCapacity;
    if (filled > 0) {
        tft.fillRect(barX + 1, kRecBarY + 1, filled, kRecBarH - 2,
                     g_recordFull ? color::WARN_AMBER : color::OK_GREEN);
    }
    char stat[48];
    snprintf(stat, sizeof(stat), "RAM %d / %d", g_capCount, kCapCapacity);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString(stat, 320 / 2, kRecStatY, 2);

    // SD status line (right below the RAM line)
    char sdLine[48];
    if (g_sdMounted) {
        snprintf(sdLine, sizeof(sdLine), "SD %lu total", (unsigned long)g_sdCapTotal);
        tft.setTextColor(color::OK_GREEN, color::BG);
    } else {
        snprintf(sdLine, sizeof(sdLine), "SD not present");
        tft.setTextColor(color::DIM, color::BG);
    }
    tft.drawString(sdLine, 320 / 2, kRecStatY + 14, 1);

    // Clear button
    drawGlowButton(x, kRecClearY, w, kRecClearH,
                   g_sdMounted ? "CLEAR RAM + SD" : "CLEAR BUFFER", true, 2);
}

static bool trigRecTouch(int tx, int ty) {
    const int x = kTrigMargin, w = 320 - 2 * kTrigMargin;
    if (tx < x || tx >= x + w) return false;
    // Record toggle
    if (ty >= kRecRecY && ty < kRecRecY + kRecRecH) {
        g_recordOn = !g_recordOn;
        if (g_recordOn && g_recordFull && !g_recordRing) {
            // Resuming after one-shot full means user wants fresh ring; keep buffer.
            // Actually: don't resume logging until they clear or switch to ring.
        }
        return true;
    }
    // Mode row
    const int halfW = (w - 6) / 2;
    if (ty >= kRecModeY && ty < kRecModeY + kRecModeH) {
        if (tx < x + halfW)             g_recordRing = true;
        else if (tx >= x + halfW + 6)   g_recordRing = false;
        if (g_recordRing) g_recordFull = false;
        return true;
    }
    // Clear button
    if (ty >= kRecClearY && ty < kRecClearY + kRecClearH) {
        g_capCount = 0;
        g_capHead  = 0;
        g_recordFull = false;
        if (g_sdMounted) sdDeleteCapFile();
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
// CAPS tab -- scrolling list of CaptureEvents, newest first, with sparkline.
// -----------------------------------------------------------------------------
constexpr int kCapRowH      = 36;
constexpr int kCapListTop   = 40;
constexpr int kCapListBot   = 232;
constexpr int kCapScrollW   = 28;              // right-edge scroll strip

static int capRowsVisible() {
    return (kCapListBot - kCapListTop) / kCapRowH;
}

// Returns the event at logical "newest-first index i" (0 = newest).
static const CaptureEvent* capAt(int i) {
    if (i < 0 || i >= g_capCount) return nullptr;
    // Latest written is at (g_capHead - 1) mod capacity.
    int idx = g_capHead - 1 - i;
    while (idx < 0) idx += kCapCapacity;
    idx %= kCapCapacity;
    return &g_capBuf[idx];
}

// Convert bin -> MHz given WiFi channel + span used at fire time.
static uint16_t capFreqMhz(const CaptureEvent* ev) {
    int centerKHz = 2407000 + (int)ev->wifiCh * 5000;
    int bwMHz;
    switch ((SpanMode)ev->spanMode) {
        case SpanMode::S40: bwMHz = 40; break;
        case SpanMode::S80: bwMHz = 80; break;
        default:            bwMHz = 16; break;
    }
    // bin 0 = center - bw/2, bin fftN-1 = center + bw/2
    int binMhz = (int)ev->peakBin * bwMHz / (ev->fftN ? ev->fftN : 128);
    int freqKHz = centerKHz - bwMHz * 500 + binMhz * 1000;
    return (uint16_t)((freqKHz + 500) / 1000);
}

static void drawTrigCapsTab() {
    const int left   = kTrigMargin;
    const int rowsN  = capRowsVisible();
    const int listRt = 320 - kTrigMargin - kCapScrollW;
    const int listW  = listRt - left;

    if (g_capCount == 0) {
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(color::MID, color::BG);
        tft.drawString("No captures yet",
                       320 / 2, (kCapListTop + kCapListBot) / 2 - 8, 4);
        tft.setTextColor(color::DIM, color::BG);
        tft.drawString("Enable TRIGGER on CFG + RECORD on REC",
                       320 / 2, (kCapListTop + kCapListBot) / 2 + 20, 2);
    } else {
        for (int r = 0; r < rowsN; r++) {
            int idx = g_capScroll + r;
            const CaptureEvent* ev = capAt(idx);
            int ry = kCapListTop + r * kCapRowH;
            if (!ev) continue;
            // Row frame
            tft.drawRect(left, ry, listW, kCapRowH - 2, color::MID);
            // Freq label (large)
            uint16_t mhz = capFreqMhz(ev);
            char fbuf[20];
            snprintf(fbuf, sizeof(fbuf), "%u MHz", (unsigned)mhz);
            tft.setTextDatum(ML_DATUM);
            tft.setTextColor(color::OK_GREEN, color::BG);
            tft.drawString(fbuf, left + 6, ry + 11, 2);
            // Age + level (smaller, under freq)
            uint32_t ageMs = millis() - ev->tsMs;
            char ageBuf[32];
            if (ageMs < 60000) snprintf(ageBuf, sizeof(ageBuf), "%lus  lv %u",
                                        (unsigned long)(ageMs / 1000), (unsigned)ev->peakLevel);
            else               snprintf(ageBuf, sizeof(ageBuf), "%lum  lv %u",
                                        (unsigned long)(ageMs / 60000), (unsigned)ev->peakLevel);
            tft.setTextColor(color::DIM, color::BG);
            tft.drawString(ageBuf, left + 6, ry + 25, 1);
            // Sparkline on right side of row
            const int spX = left + listW / 2 + 6;
            const int spY = ry + 4;
            const int spW = listW - (listW / 2 + 6) - 6;
            const int spH = kCapRowH - 10;
            tft.drawRect(spX, spY, spW, spH, color::DIM);
            // Draw 64 bins into spW pixels (compressed if spW<64, stretched if >)
            for (int s = 0; s < 64; s++) {
                int xS = spX + 1 + (s * (spW - 2)) / 64;
                int xE = spX + 1 + ((s + 1) * (spW - 2)) / 64;
                if (xE <= xS) xE = xS + 1;
                int h = (ev->snap[s] * (spH - 2)) / 255;
                if (h > 0) {
                    tft.fillRect(xS, spY + spH - 1 - h, xE - xS, h, color::OK_GREEN);
                }
            }
            // Peak marker on sparkline
            int pMarkX = spX + 1 + (ev->peakBin * (spW - 2))
                       / (ev->fftN ? ev->fftN : 128);
            if (pMarkX >= spX && pMarkX < spX + spW) {
                tft.drawFastVLine(pMarkX, spY, spH, color::WARN_AMBER);
            }
        }
    }

    // Scroll strip (up/down buttons)
    const int sbX = 320 - kTrigMargin - kCapScrollW;
    const int upY = kCapListTop;
    const int dnY = kCapListBot - kCapScrollW;
    drawGlowButton(sbX, upY, kCapScrollW, kCapScrollW, "+", true, 4);
    drawGlowButton(sbX, dnY, kCapScrollW, kCapScrollW, "-", true, 4);
    // Scroll position indicator (small bar in middle)
    const int midTop = upY + kCapScrollW + 4;
    const int midBot = dnY - 4;
    const int midH   = midBot - midTop;
    tft.drawRect(sbX + 8, midTop, kCapScrollW - 16, midH, color::DIM);
    if (g_capCount > rowsN) {
        int thumbH = (rowsN * midH) / g_capCount;
        if (thumbH < 8) thumbH = 8;
        int thumbMax = midH - thumbH;
        int thumbY   = midTop + (g_capScroll * thumbMax) / (g_capCount - rowsN);
        tft.fillRect(sbX + 10, thumbY, kCapScrollW - 20, thumbH, color::OK_GREEN);
    }
}

static bool trigCapsTouch(int tx, int ty) {
    const int sbX = 320 - kTrigMargin - kCapScrollW;
    if (tx >= sbX && tx < sbX + kCapScrollW) {
        int rowsN = capRowsVisible();
        int maxScroll = g_capCount - rowsN; if (maxScroll < 0) maxScroll = 0;
        if (ty >= kCapListTop && ty < kCapListTop + kCapScrollW) {
            // up = older (newest-first list) -> scroll up = index +1? no: up moves toward NEWEST (idx 0)
            g_capScroll--;
            if (g_capScroll < 0) g_capScroll = 0;
            return true;
        }
        if (ty >= kCapListBot - kCapScrollW && ty < kCapListBot) {
            g_capScroll++;
            if (g_capScroll > maxScroll) g_capScroll = maxScroll;
            return true;
        }
    }
    return false;
}

// -----------------------------------------------------------------------------
// Dispatcher.
// -----------------------------------------------------------------------------
static void drawTriggerScreen() {
    tft.fillScreen(color::BG);
    drawCornerHome();
    drawTrigTabBar();
    switch (g_trigTab) {
        case TrigTab::CFG:  drawTrigCfgTab();  break;
        case TrigTab::REC:  drawTrigRecTab();  break;
        case TrigTab::CAPS: drawTrigCapsTab(); break;
    }
}

static bool triggerTouch(int tx, int ty) {
    TrigTab which;
    if (inTrigTabBar(tx, ty, which)) {
        if (which != g_trigTab) {
            g_trigTab   = which;
            g_capScroll = 0;        // reset scroll when entering CAPS
            return true;
        }
        return false;
    }
    switch (g_trigTab) {
        case TrigTab::CFG:  return trigCfgTouch(tx, ty);
        case TrigTab::REC:  return trigRecTouch(tx, ty);
        case TrigTab::CAPS: return trigCapsTouch(tx, ty);
    }
    return false;
}

// ----------------------------------------------------------------------------
// TRANSMIT hub -- tabbed MODE | TUNE, like the main Tune screen.
// MODE tab:  mode picker + status + FIRE
// TUNE tab:  TX channel grid + duration row + pacing row
// ----------------------------------------------------------------------------
enum class TxTab : uint8_t { MODE = 0, TUNE = 1 };
static TxTab g_txTab = TxTab::MODE;

constexpr int kTxMargin = 10;

// Shared tab bar (right of the Back button)
static void drawTxTabBar() {
    const int x1 = kTuneTabLeft;
    const int x2 = kTuneTabLeft + kTuneTabW + kTuneTabGap;
    drawGlowButton(x1, kTuneTabY, kTuneTabW, kTuneTabH,
                   "MODE", g_txTab == TxTab::MODE, 2);
    drawGlowButton(x2, kTuneTabY, kTuneTabW, kTuneTabH,
                   "TUNE", g_txTab == TxTab::TUNE, 2);
}
static bool inTxTabBar(int tx, int ty, TxTab& which) {
    if (ty < kTuneTabY - 4 || ty >= kTuneTabY + kTuneTabH + 4) return false;
    const int x1 = kTuneTabLeft;
    const int x2 = kTuneTabLeft + kTuneTabW + kTuneTabGap;
    if (tx >= x1 - 4 && tx < x1 + kTuneTabW + 4) { which = TxTab::MODE; return true; }
    if (tx >= x2 - 4 && tx < x2 + kTuneTabW + 4) { which = TxTab::TUNE; return true; }
    return false;
}

// --- MODE tab -------------------------------------------------------------
constexpr int kTxKindY = 42,  kTxKindH = 22;
constexpr int kTxFireY = 70,  kTxFireH = 72;
constexpr int kTxStatY = 150;
constexpr int kTxWarnY = 212;

static void drawTxModeTab() {
    // On the sub-screens the kind is fixed (TX_CW -> tone, TX_80211 -> 802.11)
    // so we show a single read-only label instead of the KIND selector.
    const int gap = 6;
    const int kw = (320 - 2 * kTxMargin - gap) / 2;
    if (g_screen == Screen::TX_CW) {
        drawGlowButton(kTxMargin, kTxKindY, 320 - 2 * kTxMargin, kTxKindH,
                       "CW / SWEEP", true, 2);
    } else if (g_screen == Screen::TX_80211) {
        drawGlowButton(kTxMargin, kTxKindY, 320 - 2 * kTxMargin, kTxKindH,
                       "802.11 BEACON", true, 2);
    } else {
        // Legacy TRANSMIT (hub) rendering fallback -- shouldn't be hit.
        drawGlowButton(kTxMargin,                 kTxKindY, kw, kTxKindH,
                       "802.11", !g_toneSel, 2);
        drawGlowButton(kTxMargin + kw + gap,      kTxKindY, kw, kTxKindH,
                       "TONE  (CW experiment)", g_toneSel, 2);
    }

    // Big START/STOP toggle button
    const int fx = kTxMargin, fw = 320 - 2 * kTxMargin;
    drawGlowButton(fx, kTxFireY, fw, kTxFireH, "", true, 2);
    tft.setTextDatum(MC_DATUM);
    const bool live = g_txRunning;
    tft.setTextColor(live ? color::WARN_AMBER : color::OK_GREEN, color::BG);
    tft.drawString(live ? "STOP"
                        : (g_toneSel ? "START TONE" : "START"),
                   fx + fw / 2, kTxFireY + kTxFireH / 2, 4);

    // Config readout -- what the next FIRE will actually do
    char info[64];
    if (g_txHop) {
        snprintf(info, sizeof(info), "HOP 1..13   %ums  %ums/ch",
                 (unsigned)g_txDurMs, (unsigned)g_txHopDwellMs);
    } else {
        snprintf(info, sizeof(info), "ch%u   %ums  pace %uus",
                 (unsigned)g_txChannel,
                 (unsigned)g_txDurMs, (unsigned)g_txIntervalUs);
    }
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString(info, tft.width() / 2, kTxStatY, 2);

    // Last-burst stats
    char stat[64];
    uint32_t lastCnt = g_lastTxCount;
    uint32_t finMs   = g_lastTxFinishMs;
    if (lastCnt == 0 && finMs == 0) {
        snprintf(stat, sizeof(stat), "idle");
    } else {
        uint32_t age = (millis() - finMs) / 1000;
        snprintf(stat, sizeof(stat), "sent %lu  (%lus ago)",
                 (unsigned long)lastCnt, (unsigned long)age);
    }
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString(stat, tft.width() / 2, kTxStatY + 22, 2);

    tft.setTextColor(color::WARN_AMBER, color::BG);
    tft.drawString("Raw 802.11 TX. Short bursts, own network only.",
                   tft.width() / 2, kTxWarnY, 1);
}

static bool txModeTouch(int tx, int ty) {
    // Kind selector row -- disabled when inside a sub-screen (TX_CW or
    // TX_80211). Those screens lock the kind so a stray tap can't flip us
    // into the other path without going through the Hub reboot flow.
    if (g_screen != Screen::TX_CW && g_screen != Screen::TX_80211 &&
        ty >= kTxKindY && ty < kTxKindY + kTxKindH) {
        const int gap = 6;
        const int kw = (320 - 2 * kTxMargin - gap) / 2;
        if (tx >= kTxMargin && tx < kTxMargin + kw) {
            g_toneSel = false; return true;
        }
        if (tx >= kTxMargin + kw + gap && tx < kTxMargin + 2 * kw + gap) {
            g_toneSel = true; return true;
        }
    }
    // START / STOP toggle
    if (ty >= kTxFireY && ty < kTxFireY + kTxFireH
        && tx >= kTxMargin && tx < 320 - kTxMargin) {
        if (g_toneSel) {
            // Post a start/stop request -- loop() does the heavy phy init
            // so this touch handler returns immediately (no WDT trip).
            if (g_txRunning || g_cwActive) {
                g_cwReqStop = true;
                g_txRunning = false;    // paint UI immediately; cwPoll clears g_cwActive
            } else {
                // Pick starting channel: in hop mode (any preset except OFF),
                // use lowest-set bit of mask; else g_txChannel.
                uint8_t startCh = 11;
                if (g_cwHopPreset != HOP_OFF) {
                    for (int b = 0; b < 13; b++) {
                        if (g_cwHopMask & (1u << b)) { startCh = (uint8_t)(b + 1); break; }
                    }
                    g_cwHop = true;
                } else {
                    g_cwHop  = false;
                    startCh  = (g_txChannel >= 1 && g_txChannel <= 13)
                               ? g_txChannel : 11;
                }
                g_cwReqCh    = startCh;
                g_cwReqBo    = g_txBackoff;
                g_cwReqStart = true;
                g_txRunning  = true;                   // paint UI immediately
            }
        } else {
            if (g_txRunning) {
                g_txRunning = false;         // STOP → tells rfTask to exit
            } else if (!g_txRequest) {
                g_txRequest = true;          // START → rfTask will begin
            }
        }
        return true;
    }
    return false;
}

// --- TUNE tab: HOP toggle + channel grid + Dur/Pace/Dwell inline rows ----
constexpr int kTxTuneHopY     = 44,  kTxTuneHopH    = 24;
constexpr int kTxTuneChTop    = 74;        // channel grid y
constexpr int kTxTuneChGapY   = 4;
constexpr int kTxTuneChRowH   = 22;        // 2 rows → chTop..chTop+2*22+gap
// BT direct-channel selector (replaces HOP+grid when in BT modes)
constexpr int kTxBtFreqY      = 44, kTxBtFreqH   = 26;
constexpr int kTxBtSliderY    = 76, kTxBtSliderH = 44;
constexpr int kTxBtBtnW       = 40;
constexpr int kTxBtMinusX     = kTxMargin;
constexpr int kTxBtPlusX      = 320 - kTxMargin - kTxBtBtnW;
constexpr int kTxTuneDurY2    = 128, kTxTuneDurH2   = 22;
constexpr int kTxTunePaceY2   = 158, kTxTunePaceH2  = 22;
constexpr int kTxTuneDwellY   = 188, kTxTuneDwellH  = 22;
// New: sweep-dwell slider at the very bottom of the TUNE tab (only shown
// in SWEEP or BT_SWEEP mode). Minus / bar / plus layout.
constexpr int kTxSweepDwellY  = 214, kTxSweepDwellH = 22;
constexpr int kTxDwBtnW       = 36;
constexpr int kTxDwMinusX     = kTxMargin;
constexpr int kTxDwPlusX      = 320 - kTxMargin - kTxDwBtnW;

// Channel tile layout: 2 rows x 7 cols (14 slots, use 1-13).
static void txChTileRect(int ch, int& x, int& y, int& w, int& h) {
    const int cols = 7;
    const int gap  = 4;
    const int availW = 320 - 2 * kTxMargin;
    w = (availW - (cols - 1) * gap) / cols;
    h = kTxTuneChRowH;
    int idx = ch - 1;
    int col = idx % cols, row = idx / cols;
    x = kTxMargin + col * (w + gap);
    y = kTxTuneChTop + row * (h + kTxTuneChGapY);
}

static void drawTxTuneTab() {
    // ---- BT direct-tune path: when in BT modes, replace HOP+grid with
    // ---- a 1-MHz slider (BT ch 0..78 = 2402..2480 MHz).
    const bool btMode = g_toneSel && (g_cwMode == 3 || g_cwMode == 4);
    if (btMode) {
        // Freq readout -- centered.
        uint8_t  bc  = g_btChan;
        uint16_t mhz = 2402 + bc;
        char fbuf[32];
        snprintf(fbuf, sizeof(fbuf), "BT %u   %u MHz", (unsigned)bc, (unsigned)mhz);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(color::OK_GREEN, color::BG);
        tft.drawString(fbuf, 320 / 2, kTxBtFreqY + kTxBtFreqH / 2, 4);
        // Slider row: minus / bar / plus
        drawGlowButton(kTxBtMinusX, kTxBtSliderY, kTxBtBtnW, kTxBtSliderH, "-", true, 4);
        drawGlowButton(kTxBtPlusX,  kTxBtSliderY, kTxBtBtnW, kTxBtSliderH, "+", true, 4);
        const int barX = kTxBtMinusX + kTxBtBtnW + 8;
        const int barR = kTxBtPlusX - 8;
        const int barW = barR - barX;
        const int barY = kTxBtSliderY + 10;
        const int barH = kTxBtSliderH - 20;
        if (barW > 10) {
            tft.drawRect(barX, barY, barW, barH, color::MID);
            uint32_t rel    = bc;                              // 0..78
            int      filled = (int)((rel * (barW - 2)) / 78);
            if (filled > 0) {
                tft.fillRect(barX + 1, barY + 1, filled, barH - 2, color::OK_GREEN);
            }
            // Thumb marker
            int thumbX = barX + 1 + filled;
            tft.fillRect(thumbX - 2, barY - 3, 5, barH + 6, color::OK_GREEN);
        }
        // fall through to chip rows below (Mode/PA/Pattern/Dwell)
    } else {
        // --- HOP preset button (full-width). Tap to cycle through presets. ---
        const int hopX = kTxMargin, hopW = 320 - 2 * kTxMargin;
        char hopLbl[48];
        const char* base = kHopPresetLbl[g_cwHopPreset];
        if (g_cwHopPreset == HOP_CUSTOM) {
            int n = 0;
            for (int b = 0; b < 13; b++) if (g_cwHopMask & (1u << b)) n++;
            snprintf(hopLbl, sizeof(hopLbl), "%s  (%d ch)", base, n);
        } else {
            snprintf(hopLbl, sizeof(hopLbl), "%s", base);
        }
        drawGlowButton(hopX, kTxTuneHopY, hopW, kTxTuneHopH,
                       hopLbl, g_cwHopPreset != HOP_OFF, 2);

        // --- Channel grid: single-pick when HOP off, multi-pick when HOP on. ---
        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(color::DIM, color::BG);
        const char* chHdr =
            g_cwHopPreset == HOP_OFF    ? "Ch" :
            g_cwHopPreset == HOP_CUSTOM ? "Ch (tap to add/remove)" :
                                           "Ch (preset locked)";
        tft.drawString(chHdr, kTxMargin, kTxTuneChTop - 8, 1);
        for (int ch = 1; ch <= 13; ch++) {
            int x, y, w, h; txChTileRect(ch, x, y, w, h);
            char lbl[6]; snprintf(lbl, sizeof(lbl), "%d", ch);
            bool hop = (g_cwHopPreset != HOP_OFF);
            bool on  = hop ? chanInMask((uint8_t)ch)
                           : (g_txChannel == ch);
            drawGlowButton(x, y, w, h, lbl, on, 2);
        }
    }

    // Shared chip row renderer
    auto drawChipRow = [](const char* label, int rowY, int rowH,
                          const uint16_t* vals, const char* const* lbls,
                          int n, uint16_t sel) {
        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(color::MID, color::BG);
        tft.drawString(label, kTxMargin, rowY + rowH / 2, 2);
        const int chipLeft = 60, chipRight = 320 - 6;
        const int chipGap  = 6;
        const int chipW    = (chipRight - chipLeft - (n - 1) * chipGap) / n;
        for (int i = 0; i < n; i++) {
            int x = chipLeft + i * (chipW + chipGap);
            drawGlowButton(x, rowY, chipW, rowH, lbls[i], sel == vals[i], 2);
        }
    };

    if (g_toneSel) {
        // TONE mode -- Dur/Pace are irrelevant. Show MODE + PA.
        //   TONE  / SWEEP = WiFi radio paths (5 MHz grid)
        //   BT    / BTSWP = Bluetooth radio paths (1 MHz grid, finer)
        const uint16_t modeVals[4] = { 0, 2, 3, 4 };
        const char*    modeLbls[4] = { "TONE", "SWEEP", "BT", "BTSWP" };
        drawChipRow("Mode", kTxTunePaceY2,  kTxTunePaceH2,
                    modeVals, modeLbls, 4, g_cwMode);
        const uint16_t paVals[4] = { 0, 20, 40, 80 };            // backoff * 0.25 dB
        const char*    paLbls[4] = { "MAX", "-5dB", "-10dB", "-20dB" };
        drawChipRow("PA",   kTxTuneDurY2,   kTxTuneDurH2,
                    paVals, paLbls, 4, g_txBackoff);
    } else {
        const uint16_t durs[4] = { 100, 500, 2000, 5000 };
        const char* durL[4]    = { "100ms", "500ms", "2s", "5s" };
        drawChipRow("Dur",  kTxTuneDurY2,   kTxTuneDurH2,   durs, durL, 4, g_txDurMs);

        const uint16_t pcs[4]  = { 0, 300, 1000, 5000 };
        const char* paceL[4]   = { "0us", "300us", "1ms", "5ms" };
        drawChipRow("Pace", kTxTunePaceY2,  kTxTunePaceH2,  pcs,  paceL, 4, g_txIntervalUs);
    }

    // Dwell row context-dependent:
    //   TONE + SWEEP   -> sweep SPAN selector (±1/±2/±3/±4 WiFi ch)
    //   TONE + BT_SWEEP-> pattern selector (FULL / SPAN / BLE / off)
    //   anything else  -> HOP dwell selector (10/50/200/1000 ms per ch)
    if (g_toneSel && g_cwMode == 2) {
        const uint16_t spn[4]  = { 1, 2, 3, 4 };
        const char* spnL[4]    = { "±1ch", "±2ch", "±3ch", "±4ch" };
        drawChipRow("Span", kTxTuneDwellY, kTxTuneDwellH,
                    spn, spnL, 4, g_cwSweepSpan);
    } else if (g_toneSel && g_cwMode == 4) {
        // BT_SWEEP pattern selector
        const uint16_t pat[3]  = { 0, 1, 2 };
        const char* patL[3]    = { "FULL", "SPAN", "BLE" };
        drawChipRow("Pat", kTxTuneDwellY, kTxTuneDwellH,
                    pat, patL, 3, g_btSweepPat);
    }
    // Sweep-dwell slider (only in SWEEP or BT_SWEEP). Shows below the row above.
    if (g_toneSel && (g_cwMode == 2 || g_cwMode == 4)) {
        tft.setTextDatum(ML_DATUM);
        tft.setTextColor(color::MID, color::BG);
        char dlab[20]; snprintf(dlab, sizeof(dlab), "Dwell %uus", (unsigned)g_sweepDwellUs);
        tft.drawString(dlab, kTxMargin, kTxSweepDwellY + kTxSweepDwellH / 2, 2);
        // minus / bar / plus
        drawGlowButton(kTxDwMinusX, kTxSweepDwellY, kTxDwBtnW, kTxSweepDwellH, "-", true, 4);
        drawGlowButton(kTxDwPlusX,  kTxSweepDwellY, kTxDwBtnW, kTxSweepDwellH, "+", true, 4);
        const int barX = kTxDwMinusX + kTxDwBtnW + 70;    // leave room for label
        const int barR = kTxDwPlusX - 8;
        const int barW = barR - barX;
        if (barW > 10) {
            tft.drawRect(barX, kTxSweepDwellY + 6, barW, kTxSweepDwellH - 12, color::MID);
            uint32_t range = kSweepMaxDwellUs - kSweepMinDwellUs;
            uint32_t rel   = (g_sweepDwellUs > kSweepMinDwellUs)
                           ? (g_sweepDwellUs - kSweepMinDwellUs) : 0;
            int filled = (int)((rel * (barW - 2)) / range);
            if (filled > 0) {
                tft.fillRect(barX + 1, kTxSweepDwellY + 7, filled,
                             kTxSweepDwellH - 14, color::OK_GREEN);
            }
        }
    } else {
        const uint16_t dwl[4]  = { 10, 50, 200, 1000 };
        const char* dwlL[4]    = { "10ms", "50ms", "200ms", "1s" };
        drawChipRow(g_txHop ? "Dwell" : "Dwl", kTxTuneDwellY, kTxTuneDwellH,
                    dwl, dwlL, 4, g_txHopDwellMs);
    }
}

static bool txTuneTouch(int tx, int ty) {
    const bool btMode = g_toneSel && (g_cwMode == 3 || g_cwMode == 4);
    if (btMode) {
        // BT slider minus/plus/bar occupies the HOP+grid real estate.
        if (ty >= kTxBtSliderY && ty < kTxBtSliderY + kTxBtSliderH) {
            if (tx >= kTxBtMinusX && tx < kTxBtMinusX + kTxBtBtnW) {
                int next = (int)g_btChan - 1;
                if (next < 0) next = 0;
                g_btChan = (uint8_t)next;
                return true;
            }
            if (tx >= kTxBtPlusX && tx < kTxBtPlusX + kTxBtBtnW) {
                int next = (int)g_btChan + 1;
                if (next > 78) next = 78;
                g_btChan = (uint8_t)next;
                return true;
            }
            const int barX = kTxBtMinusX + kTxBtBtnW + 8;
            const int barR = kTxBtPlusX - 8;
            const int barW = barR - barX;
            if (tx >= barX && tx < barR && barW > 10) {
                int rel = tx - barX;
                int bc  = (rel * 78) / (barW - 2);
                if (bc < 0) bc = 0; else if (bc > 78) bc = 78;
                g_btChan = (uint8_t)bc;
                return true;
            }
        }
        // Also allow taps on the big freq readout to do nothing (eat them, no fallthrough).
        if (ty >= kTxBtFreqY && ty < kTxBtFreqY + kTxBtFreqH) return false;
        // skip HOP + grid hitboxes entirely in BT mode
    } else {
        // HOP preset button: tap cycles through OFF / Custom / 1,6,11 / All / BLE / Video.
        if (ty >= kTxTuneHopY && ty < kTxTuneHopY + kTxTuneHopH
            && tx >= kTxMargin && tx < 320 - kTxMargin) {
            uint8_t next = (uint8_t)((g_cwHopPreset + 1) % HOP_PRESET_COUNT);
            applyHopPreset(next);
            return true;
        }
        // Channel grid: single-pick when HOP off, mask toggle in Custom mode.
        // Preset modes (1/6/11, All, BLE, Video) ignore grid taps so you don't
        // accidentally clobber a preset by touching a tile.
        for (int ch = 1; ch <= 13; ch++) {
            int x, y, w, h; txChTileRect(ch, x, y, w, h);
            if (tx >= x && tx < x + w && ty >= y && ty < y + h) {
                if (g_cwHopPreset == HOP_OFF) {
                    g_txChannel = (uint8_t)ch;
                } else if (g_cwHopPreset == HOP_CUSTOM) {
                    g_cwHopMask ^= (1u << (ch - 1));
                    if (g_cwHopMask == 0) g_cwHopMask = (1u << (ch - 1));
                }
                return true;
            }
        }
    }
    // Chip-row hitboxes (share same geometry as drawChipRow)
    const int chipLeft = 60, chipRight = 320 - 6;
    const int chipGap  = 6;
    const int chipW    = (chipRight - chipLeft - 3 * chipGap) / 4;
    auto hitRow = [&](int rowY, int rowH, const uint16_t* vals,
                      int n, volatile uint16_t* dst) -> bool {
        if (ty < rowY || ty >= rowY + rowH) return false;
        for (int i = 0; i < n; i++) {
            int x = chipLeft + i * (chipW + chipGap);
            if (tx >= x && tx < x + chipW) { *dst = vals[i]; return true; }
        }
        return false;
    };
    if (g_toneSel) {
        // PA row (replaces Dur in TONE mode)
        const uint16_t paVals[4] = { 0, 20, 40, 80 };
        if (ty >= kTxTuneDurY2 && ty < kTxTuneDurY2 + kTxTuneDurH2) {
            // chipLeft/chipW below use the 4-slot layout
            int chipW4 = (chipRight - chipLeft - 3 * chipGap) / 4;
            for (int i = 0; i < 4; i++) {
                int x = chipLeft + i * (chipW4 + chipGap);
                if (tx >= x && tx < x + chipW4) {
                    g_txBackoff = (uint8_t)paVals[i];
                    if (g_cwActive) {
                        // Defer the actual radio re-arm to cwPoll on loop task.
                        g_cwReqNewBo    = g_txBackoff;
                        g_cwReqPaChange = true;
                    }
                    return true;
                }
            }
        }
        // Mode row (replaces Pace in TONE mode): TONE / SWEEP / BT / BTSWP
        if (ty >= kTxTunePaceY2 && ty < kTxTunePaceY2 + kTxTunePaceH2) {
            const uint8_t modeVals[4] = { 0, 2, 3, 4 };
            int chipW4 = (chipRight - chipLeft - 3 * chipGap) / 4;
            for (int i = 0; i < 4; i++) {
                int x = chipLeft + i * (chipW4 + chipGap);
                if (tx >= x && tx < x + chipW4) {
                    uint8_t newMode = modeVals[i];
                    if (newMode != g_cwMode) {
                        if (g_cwActive) g_cwReqStop = true;
                        g_cwMode = newMode;
                    }
                    return true;
                }
            }
        }
    } else {
        const uint16_t durs[4] = { 100, 500, 2000, 5000 };
        if (hitRow(kTxTuneDurY2,  kTxTuneDurH2,  durs, 4, (volatile uint16_t*)&g_txDurMs))
            return true;
        const uint16_t pcs[4]  = { 0, 300, 1000, 5000 };
        if (hitRow(kTxTunePaceY2, kTxTunePaceH2, pcs,  4, (volatile uint16_t*)&g_txIntervalUs))
            return true;
    }
    // Dwell row routes to SPAN in WiFi SWEEP, PATTERN in BT_SWEEP, HOP dwell otherwise.
    if (g_toneSel && g_cwMode == 2) {
        const uint16_t spn[4]  = { 1, 2, 3, 4 };
        if (ty >= kTxTuneDwellY && ty < kTxTuneDwellY + kTxTuneDwellH) {
            for (int i = 0; i < 4; i++) {
                int x = chipLeft + i * (chipW + chipGap);
                if (tx >= x && tx < x + chipW) {
                    g_cwSweepSpan = (uint8_t)spn[i];
                    return true;
                }
            }
        }
    } else if (g_toneSel && g_cwMode == 4) {
        if (ty >= kTxTuneDwellY && ty < kTxTuneDwellY + kTxTuneDwellH) {
            int chipW3 = (chipRight - chipLeft - 2 * chipGap) / 3;
            for (int i = 0; i < 3; i++) {
                int x = chipLeft + i * (chipW3 + chipGap);
                if (tx >= x && tx < x + chipW3) {
                    g_btSweepPat = (uint8_t)i;
                    return true;
                }
            }
        }
    }
    // Sweep-dwell slider -/+ taps (SWEEP or BT_SWEEP only)
    if (g_toneSel && (g_cwMode == 2 || g_cwMode == 4) &&
        ty >= kTxSweepDwellY && ty < kTxSweepDwellY + kTxSweepDwellH) {
        if (tx >= kTxDwMinusX && tx < kTxDwMinusX + kTxDwBtnW) {
            int next = (int)g_sweepDwellUs - 50;
            if (next < (int)kSweepMinDwellUs) next = kSweepMinDwellUs;
            g_sweepDwellUs = (uint16_t)next;
            return true;
        }
        if (tx >= kTxDwPlusX && tx < kTxDwPlusX + kTxDwBtnW) {
            int next = (int)g_sweepDwellUs + 50;
            if (next > (int)kSweepMaxDwellUs) next = kSweepMaxDwellUs;
            g_sweepDwellUs = (uint16_t)next;
            return true;
        }
        // Tap on bar jumps to that position
        const int barX = kTxDwMinusX + kTxDwBtnW + 70;
        const int barR = kTxDwPlusX - 8;
        const int barW = barR - barX;
        if (tx >= barX && tx < barR && barW > 10) {
            uint32_t range = kSweepMaxDwellUs - kSweepMinDwellUs;
            uint32_t rel   = (uint32_t)(tx - barX) * range / (uint32_t)(barW - 2);
            int next = (int)(kSweepMinDwellUs + rel);
            if (next < (int)kSweepMinDwellUs) next = kSweepMinDwellUs;
            if (next > (int)kSweepMaxDwellUs) next = kSweepMaxDwellUs;
            g_sweepDwellUs = (uint16_t)next;
            return true;
        }
    }
    {
        const uint16_t dwl[4]  = { 10, 50, 200, 1000 };
        if (hitRow(kTxTuneDwellY, kTxTuneDwellH, dwl,  4,
                   (volatile uint16_t*)&g_txHopDwellMs))
            return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// TRANSMIT hub screen: two big buttons picking CW/SWEEP vs 802.11 sub-screens.
// Each sub-screen is the existing MODE+TUNE tab UI with its half of the
// controls (TONE-related in TX_CW, beacon-related in TX_80211).
// ---------------------------------------------------------------------------
// Hub button geometry -- 2x2 grid of tiles.
constexpr int kHubTop     = 44;
constexpr int kHubGap     = 8;
constexpr int kHubBtnH    = 84;
constexpr int kHubBtnW    = (320 - 2 * kTxMargin - kHubGap) / 2;
constexpr int kHubRow2    = kHubTop + kHubBtnH + kHubGap;

// Tile color per TX mode: green = benign CW, amber = ham-licensed Morse,
// cyan = self-test/experimental.
struct TxHubTile {
    const char*  big;        // top-row label (title case, size 4)
    const char*  sub1;       // one-liner subtitle
    const char*  sub2;       // optional second subtitle
    uint16_t     tint;       // body color for big label
    Screen       dest;       // sub-screen on tap
    uint8_t      bootId;     // persisted boot-dest id (0 = don't persist)
    bool         isCertTest; // sets g_toneSel + flags path appropriately
    bool         toneSel;    // forwarded to g_toneSel on tap (CW path only)
};

static const TxHubTile kHubTiles[4] = {
    { "CW",     "tone/sweep",   "cert_test",   color::OK_GREEN,    Screen::TX_CW,    2, true,  true  },
    { "802.11", "beacon flood", "ch 1-13 hop", color::OK_GREEN,    Screen::TX_80211, 3, true,  false },
    { "Morse",  "CW keyer",     "ham 13 cm",   color::WARN_AMBER,  Screen::TX_MORSE, 4, false, false },
    { "Hop",    "FHSS test",    "self-check",  color::BRIGHT,      Screen::TX_HOP,   5, false, false },
};

static void drawTxHubTile(int idx) {
    const TxHubTile& t = kHubTiles[idx];
    const int lx = kTxMargin;
    const int rx = kTxMargin + kHubBtnW + kHubGap;
    int x = (idx & 1) ? rx : lx;
    int y = (idx & 2) ? kHubRow2 : kHubTop;
    drawGlowButton(x, y, kHubBtnW, kHubBtnH, "", true, 3);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(t.tint, color::BG);
    tft.drawString(t.big, x + kHubBtnW / 2, y + 24, 4);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString(t.sub1, x + kHubBtnW / 2, y + 50, 2);
    if (t.sub2 && t.sub2[0]) {
        tft.drawString(t.sub2, x + kHubBtnW / 2, y + 66, 2);
    }
}

static void drawTxHubScreen() {
    tft.fillScreen(color::BG);
    drawCornerHome();
    drawGlowTitle("TRANSMIT", tft.width() / 2, 20, 4);
    for (int i = 0; i < 4; i++) drawTxHubTile(i);
}

static void persistBootDest(uint8_t dest);          // fwd decl

static bool txHubTouch(int tx, int ty) {
    const int lx = kTxMargin;
    const int rx = kTxMargin + kHubBtnW + kHubGap;
    for (int i = 0; i < 4; i++) {
        int x = (i & 1) ? rx : lx;
        int y = (i & 2) ? kHubRow2 : kHubTop;
        if (tx < x || tx >= x + kHubBtnW) continue;
        if (ty < y || ty >= y + kHubBtnH) continue;
        const TxHubTile& t = kHubTiles[i];
        if (t.isCertTest) g_toneSel = t.toneSel;
        g_screen = t.dest;
        if (t.bootId) persistBootDest(t.bootId);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Splash + ESP.restart used when leaving TX_CW with g_certTestDirty set.
// cert_test's rftest_init allocates phy buffers that have no deinit path;
// only a full reboot releases them. See signalhound-cyd memory for details.
// ---------------------------------------------------------------------------
static void drawRebootSplash() {
    tft.fillScreen(color::BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::OK_GREEN, color::BG);
    tft.drawString("Clearing radio state", tft.width() / 2, tft.height() / 2 - 20, 4);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString("~2 seconds", tft.width() / 2, tft.height() / 2 + 12, 2);
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString(".  .  .  .", tft.width() / 2, tft.height() / 2 + 40, 2);
}
static void rebootForRadioRelease(uint8_t landOn) {
    persistBootDest(landOn);
    drawRebootSplash();
    delay(400);                                  // let the splash be seen
    Serial.println("[sh] REBOOT: releasing cert_test phy buffers");
    delay(50);                                   // flush serial
    ESP.restart();
}

// ---------------------------------------------------------------------------
// TX_CW sub-screen: existing MODE/TUNE tab UI, forced to TONE mode selector.
// Back button triggers rebootForRadioRelease if cert_test was used.
// ---------------------------------------------------------------------------
static void drawTxCwScreen() {
    tft.fillScreen(color::BG);
    drawCornerBack();
    drawTxTabBar();
    g_toneSel = true;                            // this sub-screen IS tone
    if (g_txTab == TxTab::MODE) drawTxModeTab();
    else                        drawTxTuneTab();
}
static bool txCwTouch(int tx, int ty) {
    TxTab which;
    if (inTxTabBar(tx, ty, which)) {
        if (which != g_txTab) { g_txTab = which; return true; }
        return false;
    }
    return (g_txTab == TxTab::MODE) ? txModeTouch(tx, ty) : txTuneTouch(tx, ty);
}

// ---------------------------------------------------------------------------
// TX_80211 sub-screen: existing UI, forced to 802.11 beacon mode.
// Back button goes straight to Hub -- 802.11 doesn't dirty cert_test.
// ---------------------------------------------------------------------------
static void drawTx80211Screen() {
    tft.fillScreen(color::BG);
    drawCornerBack();
    drawTxTabBar();
    g_toneSel = false;                           // this sub-screen IS 802.11
    if (g_txTab == TxTab::MODE) drawTxModeTab();
    else                        drawTxTuneTab();
}
static bool tx80211Touch(int tx, int ty) {
    TxTab which;
    if (inTxTabBar(tx, ty, which)) {
        if (which != g_txTab) { g_txTab = which; return true; }
        return false;
    }
    return (g_txTab == TxTab::MODE) ? txModeTouch(tx, ty) : txTuneTouch(tx, ty);
}

// Legacy entry point -- the old TRANSMIT screen is now the Hub.
static void drawTransmitScreen() { drawTxHubScreen(); }
static bool transmitTouch(int tx, int ty) { return txHubTouch(tx, ty); }

// ---------------------------------------------------------------------------
// MORSE sub-screen: ham-ready CW keyer. 4 preset messages + WPM/channel
// steppers + GO/STOP. Backend is txOokMorse() from src/tx_ook.cpp.
// ---------------------------------------------------------------------------
static uint8_t  g_morseWpm      = 15;        // 5..30
static uint8_t  g_morseChan     = 1;         // WiFi 1..14
static uint8_t  g_morseLastSent = 0;         // index 0..3
static char     g_morseCall[12] = "DE K1XYZ";  // overwritten by Settings
static const char* kMorsePresetsStatic[4] = {
    "CQ CQ CQ DE K1XYZ",
    "TEST TEST",
    "73 TU EE",
    "VVV VVV K1XYZ BEACON",
};

static constexpr int kMorseBack  = 10;
static constexpr int kMorsePresetY = 54;
static constexpr int kMorsePresetH = 32;
static constexpr int kMorsePresetGap = 4;
static constexpr int kMorseStepperY  = 134;
static constexpr int kMorseStatusY   = 180;
static constexpr int kMorseGoY       = 200;
static constexpr int kMorseGoH       = 32;

static int morseChanFreqMhz(uint8_t ch) {
    if (ch == 14) return 2484;
    return 2407 + 5 * ch;
}

static void drawMorseScreen() {
    tft.fillScreen(color::BG);
    drawCornerBack();
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::WARN_AMBER, color::BG);
    tft.drawString("MORSE CW", tft.width() / 2, 20, 4);

    // header: ch + freq + WPM + dit
    char hdr[48];
    int dit = 1200 / g_morseWpm;
    snprintf(hdr, sizeof(hdr), "ch %u  %d MHz    WPM %u  dit %d ms",
             (unsigned)g_morseChan, morseChanFreqMhz(g_morseChan),
             (unsigned)g_morseWpm, dit);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString(hdr, tft.width() / 2, 40, 2);

    // 4 preset tiles in a 2x2 grid
    const int px0 = kMorseBack;
    const int px1 = kMorseBack + (tft.width() - 2 * kMorseBack - kMorsePresetGap) / 2 + kMorsePresetGap;
    const int pw  = (tft.width() - 2 * kMorseBack - kMorsePresetGap) / 2;
    for (int i = 0; i < 4; i++) {
        int x = (i & 1) ? px1 : px0;
        int y = kMorsePresetY + (i & 2 ? (kMorsePresetH + kMorsePresetGap) : 0) / 2 * ((kMorsePresetH + kMorsePresetGap) / ((kMorsePresetH + kMorsePresetGap) / 2));
        // simpler row layout: two columns, two rows
        y = kMorsePresetY + ((i >> 1) * (kMorsePresetH + kMorsePresetGap));
        bool sel = (g_morseLastSent == (uint8_t)i);
        drawGlowButton(x, y, pw, kMorsePresetH, "", sel, 2);
        tft.setTextColor(sel ? color::WARN_AMBER : color::BRIGHT, color::BG);
        tft.drawString(kMorsePresetsStatic[i], x + pw / 2, y + kMorsePresetH / 2, 2);
    }

    // WPM steppers (left) + CHAN steppers (right)
    auto drawStepper = [](int x, int y, const char* lbl, int val, uint16_t tint) {
        drawGlowButton(x,       y, 32, 28, "-", false, 1);
        drawGlowButton(x + 108, y, 32, 28, "+", false, 1);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(tint, color::BG);
        tft.drawString(lbl, x + 70, y + 10, 2);
        char v[8]; snprintf(v, sizeof(v), "%d", val);
        tft.setTextColor(color::OK_GREEN, color::BG);
        tft.drawString(v, x + 70, y + 24, 2);
    };
    drawStepper(10,  kMorseStepperY, "WPM",  g_morseWpm,  color::MID);
    drawStepper(170, kMorseStepperY, "CHAN", g_morseChan, color::MID);

    // Status
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(color::MID, color::BG);
    bool active = txOokIsActive();
    tft.drawString(active ? "SENDING..." : "IDLE", 12, kMorseStatusY + 6, 2);
    tft.setTextDatum(MR_DATUM);
    tft.drawString(active ? "tap STOP" : "pick a preset to send",
                   tft.width() - 12, kMorseStatusY + 6, 2);

    // GO / STOP button
    tft.setTextDatum(MC_DATUM);
    bool isStop = active;
    drawGlowButton(10, kMorseGoY, tft.width() - 20, kMorseGoH, "",
                   true, 2);
    tft.setTextColor(isStop ? color::ALERT_RED : color::OK_GREEN, color::BG);
    tft.drawString(isStop ? "STOP" : "SEND",
                   tft.width() / 2, kMorseGoY + kMorseGoH / 2, 4);
}

static bool morseTouch(int tx, int ty) {
    // Preset grid
    const int px0 = kMorseBack;
    const int px1 = kMorseBack + (tft.width() - 2 * kMorseBack - kMorsePresetGap) / 2 + kMorsePresetGap;
    const int pw  = (tft.width() - 2 * kMorseBack - kMorsePresetGap) / 2;
    for (int i = 0; i < 4; i++) {
        int x = (i & 1) ? px1 : px0;
        int y = kMorsePresetY + ((i >> 1) * (kMorsePresetH + kMorsePresetGap));
        if (tx >= x && tx < x + pw && ty >= y && ty < y + kMorsePresetH) {
            g_morseLastSent = i;
            txOokStop();                 // flush anything in flight
            delay(30);
            txOokMorse(kMorsePresetsStatic[i], g_morseWpm, g_morseChan, 0);
            return true;
        }
    }
    // WPM steppers
    if (ty >= kMorseStepperY && ty < kMorseStepperY + 28) {
        if (tx >= 10 && tx < 42)        { if (g_morseWpm > 5) g_morseWpm--;  return true; }
        if (tx >= 118 && tx < 150)      { if (g_morseWpm < 30) g_morseWpm++; return true; }
        if (tx >= 170 && tx < 202)      { if (g_morseChan > 1) g_morseChan--; return true; }
        if (tx >= 278 && tx < 310)      { if (g_morseChan < 14) g_morseChan++; return true; }
    }
    // GO / STOP
    if (ty >= kMorseGoY && ty < kMorseGoY + kMorseGoH && tx >= 10 && tx < tft.width() - 10) {
        if (txOokIsActive()) {
            txOokStop();
        } else {
            txOokMorse(kMorsePresetsStatic[g_morseLastSent], g_morseWpm, g_morseChan, 0);
        }
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// HOP sub-screen: FHSS reference emitter stub. Full implementation to follow;
// for now shows "Coming soon" so the tile is tappable without a dead end.
// ---------------------------------------------------------------------------
static void drawHopScreen() {
    tft.fillScreen(color::BG);
    drawCornerBack();
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::BRIGHT, color::BG);
    tft.drawString("HOP test", tft.width() / 2, 20, 4);
    tft.setTextColor(color::MID, color::BG);
    tft.drawString("FHSS reference emitter", tft.width() / 2, 110, 2);
    tft.drawString("coming soon", tft.width() / 2, 132, 2);
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString("CLI preview: pll_chan + tx_cw", tft.width() / 2, 180, 1);
}
static bool hopTouch(int /*tx*/, int /*ty*/) { return false; }

// ----------------------------------------------------------------------------
// SETTINGS screen: brightness + touch recal + factory reset.
// ----------------------------------------------------------------------------
constexpr int kSetMargin   = 10;
constexpr int kBrightLabelY = 44;
constexpr int kBrightBarY   = 60, kBrightBarH = 24;
constexpr int kBrightMinusX = 10,  kBrightPlusX  = 268;
constexpr int kBrightBtnW   = 44,  kBrightBtnH   = 24;
constexpr int kRecalY       = 108, kRecalH       = 36;
constexpr int kResetY       = 156, kResetH       = 36;

static uint8_t g_resetArm = 0;      // 0 idle, 1 "tap again to confirm"

static void drawSettingsScreen() {
    tft.fillScreen(color::BG);
    drawCornerHome();
    drawGlowTitle("SETTINGS", tft.width() / 2, 20, 4);

    // --- Brightness label + bar + nudgers ---
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(color::MID, color::BG);
    char blab[24];
    snprintf(blab, sizeof(blab), "Brightness  %u", (unsigned)g_brightness);
    tft.drawString(blab, kSetMargin, kBrightLabelY + 6, 2);

    drawGlowButton(kBrightMinusX, kBrightBarY, kBrightBtnW, kBrightBtnH, "-", true, 4);
    drawGlowButton(kBrightPlusX,  kBrightBarY, kBrightBtnW, kBrightBtnH, "+", true, 4);
    const int barX = kBrightMinusX + kBrightBtnW + 8;
    const int barR = kBrightPlusX - 8;
    const int barW = barR - barX;
    tft.drawRect(barX, kBrightBarY + 6, barW, kBrightBtnH - 12, color::MID);
    int filled = (g_brightness * (barW - 2)) / 255;
    if (filled > 0) tft.fillRect(barX + 1, kBrightBarY + 7,
                                 filled, kBrightBtnH - 14, color::BRIGHT);

    // --- Touch Recalibrate ---
    drawGlowButton(kSetMargin, kRecalY,
                   320 - 2 * kSetMargin, kRecalH, "Touch Recalibrate", true, 2);

    // --- Factory Reset (two-tap confirm) ---
    drawGlowButton(kSetMargin, kResetY,
                   320 - 2 * kSetMargin, kResetH,
                   g_resetArm ? "TAP AGAIN TO CONFIRM" : "Factory Reset",
                   true, 2);
    if (g_resetArm) {
        // Overlay red-ish text so the armed state reads as a warning.
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(color::WARN_AMBER, color::BG);
        tft.drawString("TAP AGAIN TO CONFIRM",
                       tft.width() / 2, kResetY + kResetH / 2, 2);
    }

    // Footer hint
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(color::DIM, color::BG);
    tft.drawString("Reset clears all settings and reboots",
                   tft.width() / 2, 212, 1);
    tft.drawString("Recalibrate wipes touch cal and reboots",
                   tft.width() / 2, 224, 1);
}

static bool settingsTouch(int tx, int ty) {
    // Brightness nudgers
    if (ty >= kBrightBarY && ty < kBrightBarY + kBrightBtnH) {
        if (tx >= kBrightMinusX && tx < kBrightMinusX + kBrightBtnW) {
            int next = (int)g_brightness - 16;
            if (next < 16) next = 16;         // keep screen readable
            setBrightness((uint8_t)next);
            g_resetArm = 0;
            return true;
        }
        if (tx >= kBrightPlusX && tx < kBrightPlusX + kBrightBtnW) {
            int next = (int)g_brightness + 16;
            if (next > 255) next = 255;
            setBrightness((uint8_t)next);
            g_resetArm = 0;
            return true;
        }
    }
    // Touch Recalibrate -- clear cal + reboot, cal flow runs on next boot.
    if (ty >= kRecalY && ty < kRecalY + kRecalH
        && tx >= kSetMargin && tx < 320 - kSetMargin) {
        Preferences p;
        p.begin(kCalNamespace, false);
        p.clear();
        p.end();
        Serial.println("[sh] touch cal cleared, rebooting");
        saveConfig();                  // persist whatever else is dirty first
        delay(100);
        ESP.restart();
        return true;
    }
    // Factory Reset -- two-tap confirm
    if (ty >= kResetY && ty < kResetY + kResetH
        && tx >= kSetMargin && tx < 320 - kSetMargin) {
        if (g_resetArm == 0) {
            g_resetArm = 1;
            return true;
        }
        // Confirmed: wipe both cal + cfg namespaces, reboot
        Preferences p;
        p.begin(kCalNamespace, false); p.clear(); p.end();
        p.begin(kCfgNamespace, false); p.clear(); p.end();
        Serial.println("[sh] factory reset, rebooting");
        delay(100);
        ESP.restart();
        return true;
    }
    // Any tap elsewhere clears the armed confirm.
    g_resetArm = 0;
    return false;
}

// ----------------------------------------------------------------------------
// Core-1 UI task: screen dispatch + touch
// ----------------------------------------------------------------------------
static void renderCurrentScreen() {
    switch (g_screen) {
        case Screen::HOME:     drawHome();              break;
        case Screen::LIVE:     drawLive();              break;
        case Screen::TUNE:     drawTune();              break;
        case Screen::TRIGGER:  drawTriggerScreen();     break;
        case Screen::TRANSMIT: drawTransmitScreen();    break;
        case Screen::TX_CW:    drawTxCwScreen();        break;
        case Screen::TX_80211: drawTx80211Screen();     break;
        case Screen::TX_MORSE: drawMorseScreen();       break;
        case Screen::TX_HOP:   drawHopScreen();         break;
        case Screen::SETTINGS: drawSettingsScreen();    break;
        default:               drawPlaceholder(g_screen); break;
    }
}

// Returns true if a redraw of the whole screen is needed (nav changed).
static bool handleTouchOnce() {
    int x, y;
    if (!getTouch(x, y)) return false;

    // Debounce: wait for release before accepting the next tap.
    static unsigned long lastTapMs = 0;
    unsigned long now = millis();
    if (now - lastTapMs < 180) return false;
    lastTapMs = now;

    if (g_screen == Screen::HOME) {
        if (homeTouch(x, y)) {
            g_prevScreen = Screen::HOME;        // Back from any sub-screen → Home
            return true;
        }
    } else {
        if (inCornerHome(x, y)) {
            // Corner button -- behavior depends on which screen we're on.
            if (g_screen == Screen::TX_CW) {
                // Back from CW/SWEEP. If cert_test was used this session,
                // reboot to release phy buffers before landing anywhere.
                if (g_certTestDirty) {
                    rebootForRadioRelease(1);     // land on TRANSMIT hub
                    return true;                   // (unreachable, restart)
                }
                g_screen = Screen::TRANSMIT;
                persistBootDest(1);
                return true;
            }
            if (g_screen == Screen::TX_80211) {
                g_screen = Screen::TRANSMIT;
                persistBootDest(1);
                return true;
            }
            if (g_screen == Screen::TX_MORSE) {
                txOokStop();
                g_screen = Screen::TRANSMIT;
                persistBootDest(1);
                return true;
            }
            if (g_screen == Screen::TX_HOP) {
                g_screen = Screen::TRANSMIT;
                persistBootDest(1);
                return true;
            }
            if (g_screen == Screen::TUNE || g_screen == Screen::TRANSMIT) {
                g_screen = g_prevScreen;
                if (g_screen == Screen::HOME) persistBootDest(0);
                return true;
            }
            // Leaving LIVE entirely: restore tune if HOP had taken over.
            if (g_screen == Screen::LIVE && g_liveTab == LiveTab::HOP) hopExitTab();
            g_screen = Screen::HOME;
            persistBootDest(0);
            return true;
        }
        if (g_screen == Screen::LIVE && inLiveTune(x, y)) {
            // Pop out to Tune: let user adjust, but keep HOP's hunt tune
            // in place so re-entering LIVE shows consistent state.
            g_prevScreen = Screen::LIVE;
            g_tuneTab    = TuneTab::TUNE;
            g_screen     = Screen::TUNE;
            return true;
        }
        // LIVE HOP toggle: flip between SPEC and HOP sub-tabs. Save or
        // restore the user's tune state at the boundary so HOP's hunt
        // cycle doesn't leave their SPEC tune mangled.
        if (g_screen == Screen::LIVE && inLiveHopChip(x, y)) {
            if (g_liveTab == LiveTab::SPEC) {
                g_liveTab = LiveTab::HOP;
                hopEnterTab();
            } else {
                hopExitTab();
                g_liveTab = LiveTab::SPEC;
            }
            return true;
        }
        // HOP mode chip row: switch between hunt modes. All seven chips
        // tap: ADV sub-modes retune, placeholders (CONN/WIFI/ZB) switch the
        // mode indicator but freeze the current tune (nothing to retune to
        // yet) and the classifier reports "not yet" so you can see the chip
        // is responsive.
        if (g_screen == Screen::LIVE && g_liveTab == LiveTab::HOP) {
            int chipIdx = -1;
            if (inHopModeChip(x, y, chipIdx) && chipIdx >= 0 && chipIdx < 7) {
                HopMode m = (HopMode)chipIdx;
                if (m != g_hopMode) {
                    hopApplyMode(m);
                    return true;    // triggers full redraw
                }
                return false;
            }
        }
        // LIVE shortcut: tap the AGC pill in the bottom pill strip to open
        // Tune with GAIN tab pre-selected.
        if (g_screen == Screen::LIVE) {
            const int pillY = kPillStripTop, pillH = kPillStripH;
            const int gap = 4;
            const int pw  = (tft.width() - 5 * gap) / 4;
            if (y >= pillY && y < pillY + pillH
                && x >= gap && x < gap + pw) {
                g_prevScreen = Screen::LIVE;
                g_tuneTab    = TuneTab::GAIN;
                g_screen     = Screen::TUNE;
                return true;
            }
        }
        if (g_screen == Screen::TUNE && tuneTouch(x, y)) {
            g_cfgDirty = true;
            return true;
        }
        if (g_screen == Screen::TRIGGER && triggerTouch(x, y)) {
            g_cfgDirty = true;
            return true;
        }
        if (g_screen == Screen::SETTINGS && settingsTouch(x, y)) {
            g_cfgDirty = true;
            return true;
        }
        if (g_screen == Screen::TRANSMIT && transmitTouch(x, y)) {
            g_cfgDirty = true;
            return true;
        }
        if (g_screen == Screen::TX_CW && txCwTouch(x, y)) {
            g_cfgDirty = true;
            return true;
        }
        if (g_screen == Screen::TX_80211 && tx80211Touch(x, y)) {
            g_cfgDirty = true;
            return true;
        }
        if (g_screen == Screen::TX_MORSE && morseTouch(x, y)) {
            return true;
        }
        if (g_screen == Screen::TX_HOP && hopTouch(x, y)) {
            return true;
        }
    }
    return false;
}

// -------- Config helpers (bodies; declarations are earlier in the file) ----
static void loadConfig() {
    Preferences p;
    if (!p.begin(kCfgNamespace, true)) return;
    g_wifiCh      = p.getUChar ("ch",     g_wifiCh);
    g_wantSpan    = p.getUChar ("span",   g_wantSpan);
    g_wantFftN    = p.getUShort("fftN",   g_wantFftN);
    g_wantPasses  = p.getUChar ("pass",   g_wantPasses);
    g_wantAvg     = p.getUChar ("avg",    g_wantAvg);
    g_wantAgc     = p.getBool  ("agc",    g_wantAgc);
    g_txMode      = p.getUChar ("txMode", g_txMode);
    g_txChannel   = p.getUChar ("txCh",   g_txChannel);
    g_txDurMs     = p.getUShort("txDur",  g_txDurMs);
    g_txIntervalUs= p.getUShort("txIvl",  g_txIntervalUs);
    g_txHop       = p.getBool  ("txHop",  g_txHop);
    g_txHopDwellMs= p.getUShort("txHDwl", g_txHopDwellMs);
    uint8_t gn    = p.getUChar ("gain",   0xff);
    if (gn != 0xff) g_wantGain = gn;
    g_trigEnabled   = p.getBool  ("trigEn", g_trigEnabled);
    g_trigThreshold = p.getUChar ("trigTh", g_trigThreshold);
    g_trigBand      = (TrigBand)p.getUChar("trigBd", (uint8_t)g_trigBand);
    g_brightness    = p.getUChar ("bright", g_brightness);
    g_bootDest      = p.getUChar ("tx_boot", 0);
    p.end();
    Serial.printf("[sh] cfg loaded: ch=%u span=%u fftN=%u agc=%d gain=%u "
                  "trigEn=%d trigTh=%u trigBd=%u bright=%u\n",
                  g_wifiCh, g_wantSpan, g_wantFftN, g_wantAgc,
                  (g_wantGain == 0xff ? 0 : g_wantGain),
                  g_trigEnabled, g_trigThreshold, (unsigned)g_trigBand,
                  g_brightness);
}

static void saveConfig() {
    Preferences p;
    if (!p.begin(kCfgNamespace, false)) return;
    p.putUChar ("ch",     g_wifiCh);
    p.putUChar ("span",   g_wantSpan);
    p.putUShort("fftN",   g_wantFftN);
    p.putUChar ("pass",   g_wantPasses);
    p.putUChar ("avg",    g_wantAvg);
    p.putBool  ("agc",    g_wantAgc);
    p.putUChar ("txMode", g_txMode);
    p.putUChar ("txCh",   g_txChannel);
    p.putUShort("txDur",  g_txDurMs);
    p.putUShort("txIvl",  g_txIntervalUs);
    p.putBool  ("txHop",  g_txHop);
    p.putUShort("txHDwl", g_txHopDwellMs);
    if (g_wantGain != 0xff) p.putUChar("gain", g_wantGain);
    p.putBool  ("trigEn", g_trigEnabled);
    p.putUChar ("trigTh", g_trigThreshold);
    p.putUChar ("trigBd", (uint8_t)g_trigBand);
    p.putUChar ("bright", g_brightness);
    p.putUChar ("tx_boot", g_bootDest);
    p.end();
    g_cfgDirty      = false;
    g_cfgLastSaveMs = millis();
    Serial.println("[sh] cfg saved");
}

// Persist boot-destination immediately (independent of the periodic cfg save)
// so a crash or auto-reboot still lands the user back where they were.
static void persistBootDest(uint8_t dest) {
    if (dest == g_bootDest) return;
    g_bootDest = dest;
    Preferences p;
    if (p.begin(kCfgNamespace, false)) {
        p.putUChar("tx_boot", dest);
        p.end();
    }
}

static void uiTask(void*) {
    for (;;) {
        bool navChanged = handleTouchOnce();
        if (navChanged || g_screen != g_screenLast) {
            renderCurrentScreen();
            g_screenLast = g_screen;
        }

        // Pull latest frame (non-blocking); redraw LIVE if we have a new one.
        FftFrame frame;
        if (xQueueReceive(g_fftQueue, &frame, 0) == pdTRUE) {
            g_latestFrame = frame;
            g_haveFrame = true;
            // Hop Trail eats every frame regardless of which screen is up, so
            // the ring stays warm when the user flips to the HOP tab.
            hopLog(frame);
            if (g_screen == Screen::LIVE) {
                if (g_liveTab == LiveTab::SPEC) {
                    drawLiveFrame(0);
                } else {
                    // HOP tab: incremental dirty-rect refresh at frame rate.
                    // Each helper only touches cells that actually changed,
                    // so this stays below the flicker threshold. Cycle tick
                    // handles the ADV_CYCLE retune cadence.
                    hopCycleTick();
                    drawLiveHopTabTick();
                }
            }
        }

        // Redraw TRANSMIT screen when g_txRunning flips (safety-timeout exit,
        // CLI stop, hop channel change readouts) so the button label stays
        // correct without a touch nudge.
        {
            static bool sLastTxRunning = false;
            if (g_screen == Screen::TRANSMIT
                && (bool)g_txRunning != sLastTxRunning) {
                sLastTxRunning = g_txRunning;
                renderCurrentScreen();
            }
        }

        // Debounced config save: once a change is a few seconds idle, flush.
        if (g_cfgDirty && (millis() - g_cfgLastSaveMs) > 3000) {
            saveConfig();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ----------------------------------------------------------------------------
// setup / loop
// ----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("[sh] Signalhound Phase 1 boot");

    // --- TFT ---
    tft.init();
    tft.setRotation(1);       // 320x240 landscape
    tft.fillScreen(color::BG);
    // Backlight on LEDC so we can PWM for brightness from Settings.
    ledcSetup(kBlChannel, kBlFreqHz, kBlResBits);
    ledcAttachPin(kBlPin, kBlChannel);
    ledcWrite(kBlChannel, 255);

    // --- Config load (brightness, retune state, trigger config) ---
    loadConfig();
    setBrightness(g_brightness);

    // --- Touch + calibration ---
    touchInit();
    bool hasCal = loadTouchCalibration();
    Serial.printf("[sh] touch cal %s\n", hasCal ? "LOADED" : "MISSING -- calibrating");
    if (!hasCal) runTouchCalibration();

    // --- SD card (optional). If present, load recent captures into the RAM ring
    // so CAPS tab shows prior-session history across reboots.
    tryMountSd();
    if (g_sdMounted) sdLoadLastEvents(kCapCapacity);

    // --- Queue + tasks ---
    g_fftQueue = xQueueCreate(1, sizeof(FftFrame));
    if (!g_fftQueue) {
        Serial.println("[sh] FATAL: queue alloc failed");
        for (;;) delay(1000);
    }

    // Core-0 is the Arduino default; pin RF to it, UI to Core-1.
    // Radio arbiter mutex -- must exist before cwTask or any START/STOP can run.
    g_radioMtx = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(rfTask, "rf", 8192, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(uiTask, "ui", 8192, nullptr, 1, nullptr, 1);
    // cwTask MUST be on Core 1. Retested on Core 0 after PKT20 removal
    // (2026-10-10): rftest_init still hangs there, confirming the crash is
    // from cert_test's inter-task messaging to Espressif's wifi task
    // (prio 23, Core 0) racing with any caller sharing Core 0.
    // Priority 1 round-robins with uiTask; the SWEEP loop yields every
    // kYieldEvery steps so UI stays responsive.
    xTaskCreatePinnedToCore(cwTask, "cw", 8192, nullptr, 1, nullptr, 1);
    txOokInit();
    Serial.println("[sh] tasks launched; UI on core 1, RF stub on core 0");
    // Note: cert_test init (esp_phy_rftest_init) is deferred to cwTask's
    // first IDLE->TX transition. Called from setup() it hung the boot
    // sequence waiting on the wifi task.

    // Honor persisted boot-destination so a cert_test-release reboot (or a
    // manual power cycle) lands the user back on the last-used Transmit
    // sub-screen instead of HOME. 0=HOME, 1=TRANSMIT hub, 2=TX_CW, 3=TX_80211.
    switch (g_bootDest) {
        case 1: g_screen = Screen::TRANSMIT; g_prevScreen = Screen::HOME; break;
        case 2: g_screen = Screen::TX_CW;    g_prevScreen = Screen::TRANSMIT; g_toneSel = true;  break;
        case 3: g_screen = Screen::TX_80211; g_prevScreen = Screen::TRANSMIT; g_toneSel = false; break;
        default: break;
    }
    Serial.printf("[sh] boot dest = %u (%s)\n", g_bootDest, kScreenName[(int)g_screen]);

    // --- Phase 2.0: bring up the ESP-SDR receive stack ---
    // Deliberately after tasks launch so the UI is responsive even if the
    // SDR init stalls/crashes. If this fails, synthetic frames keep flowing.
    if (!espSdrBegin()) {
        Serial.println("[sh] ESP-SDR init FAILED -- staying on synthetic frames");
    } else {
        Serial.printf("[sh] ESP-SDR ready, gain_max=%u -- rfTask will acquire\n",
                      espSdrGainMax());
    }

    // --- Quick wins from the libphy RE pass -----------------------------
    // Pin max TX power to the chip ceiling (78 * 0.25 dB = 19.5 dBm, the
    // highest LIMIT bound in esp_phy_init_data). Default build may clamp
    // lower depending on CONFIG_ESP_PHY_MAX_TX_POWER. Call after esp-sdr
    // brings up WiFi in NULL mode.
    {
        esp_err_t e = esp_wifi_set_max_tx_power(78);
        Serial.printf("[sh] esp_wifi_set_max_tx_power(78) rc=%d\n", (int)e);
    }
    // Tell libphy we're WiFi-only (no BT coexistence). Frees RF resources
    // the coex scheduler would otherwise reserve, giving CW TX a cleaner
    // uninterrupted carrier.
    phy_set_wifi_mode_only(true);
    Serial.println("[sh] phy_set_wifi_mode_only(true) -- WiFi-only radio");
}

// --- Experimental serial CLI for poking the undocumented TX path --------
// All commands are one line, args space-separated. Lines echo "ok" or "err <msg>".
extern "C" {
    void rom_pbus_workmode(void);
    void rom_pbus_xpd_rx_on(int);
    void rom_pbus_xpd_rx_off(void);
    void rom_pbus_xpd_tx_off(void);
    void rom_pbus_xpd_tx_on(int);
    void rom_set_rxclk_en(int);
    void rom_set_txclk_en(int);
    void rom_rfpll_set_freq(unsigned);
    void rom_start_tx_tone(int, int, int);
    void rom_stop_tx_tone(int);
    void tx_cont_cfg(unsigned);
    void tx_cont_en(void);
    void tx_cont_dis(void);
    // libphy.a internal (phy_chip_v7.o) -- actual tone emitter the RF test
    // firmware's wifiscwout vtable[68] appears to resolve to. Signature from
    // disassembly: 5+ int args, where a3 and a6 are scaled frequency params.
    void ram_start_tx_tone(int, int, int, int, int);
    // Additional libphy symbols for TX path setup (candidates for the
    // phy_fn_A/B/C chain the RF test handler calls BEFORE ram_start_tx_tone).
    void rf_init(void);
    void chip_v7_set_chan(int);
    // Vendored cert_test APIs from esp-phy-lib (IDF v5.5 librftest.a).
    // These are the ones Espressif's RF-test firmware uses for real CW tone.
    void esp_phy_rftest_init(void);
    void esp_phy_rftest_config(unsigned char);
    void esp_phy_wifi_tx_tone(int start, int chan, int backoff);
    // BT CW tone. channel = BT channel index 0..78 (BT_ch_N = 2402 + N MHz).
    // Covers 2402-2480 MHz at 1 MHz steps -- finer than WiFi's 5 MHz grid.
    void esp_phy_bt_tx_tone(int start, int chan, int backoff);
    void esp_phy_test_start_stop(unsigned char value);
    void chip_v7_set_chan_nomac(int, int);
    void ram_pbus_xpd_tx_on(int);
    void ram_rfpll_set_freq(unsigned);
    void i2c_rftx_init(void);
    void set_chanfreq(int);
    void force_txrxoff(void);
    // PA gain control (available in Arduino's libphy)
    void set_tx_gain_table(const void* tbl);     // actual signature TBD
    void target_power_backoff(int dB);
    void phy_close_pa(int en);
    void set_tx_dig_gain(int a, int b);
    void tx_pwctrl_init(void);
    void ram_tx_pwr_backoff(int);
    void phy_wifitx_low_power(int);
    // Calibration functions that might trigger a TX blip. The _fn aliases
    // keep callsites tidy where we need extra decls inside function scope.
    void bb_init(void);
    void tx_pwctrl_cal(void);
    void tx_pwctrl_init(void);                   // referenced as tx_pwctrl_init_fn
    void tx_pwctrl_init_cal(void);
    void set_chan_dig_gain(int);
    static inline void tx_pwctrl_init_fn(void)        { tx_pwctrl_init(); }
    static inline void tx_pwctrl_init_cal_fn(void)    { tx_pwctrl_init_cal(); }
    static inline void set_chan_dig_gain_fn(int c)    { set_chan_dig_gain(c); }

    // One of the above pulls in phy_chip_v7.o::set_rx_gain_testchip_70,
    // which references phy_tx_pwr_track_en (not in Arduino's SDK). Weak
    // stub keeps the linker happy; the function is only called by code
    // paths we don't actually invoke.
    void phy_tx_pwr_track_en(int) __attribute__((weak));
    void phy_tx_pwr_track_en(int) {}
}

#include "esp_wifi.h"

static void cliExec(char* line) {
    // Lightweight tokenization
    char* tok = strtok(line, " \t");
    if (!tok) return;
    auto arg = [](int def) {
        char* a = strtok(nullptr, " \t");
        return a ? (int)strtol(a, nullptr, 0) : def;
    };
    if (!strcmp(tok, "help")) {
        Serial.println("cmds: help | mode null|sta|ap | promisc 0|1 | chan N | rfpll MHz |");
        Serial.println("      pbus_tx on|off | pbus_rx on|off | txclk 0|1 | rxclk 0|1 |");
        Serial.println("      cont_cfg 0|1 | cont_en | cont_dis |");
        Serial.println("      rom_start A B C | rom_stop A |");
        Serial.println("      tone ch ms  (full canned sequence) |");
        Serial.println("      reset_rx    (restore full promiscuous RX)");
        return;
    }
    if (!strcmp(tok, "mode")) {
        const char* a = strtok(nullptr, " \t");
        if (!a) { Serial.println("err mode <null|sta|ap>"); return; }
        wifi_mode_t m = WIFI_MODE_NULL;
        if (!strcmp(a, "sta"))      m = WIFI_MODE_STA;
        else if (!strcmp(a, "ap"))  m = WIFI_MODE_AP;
        else if (!strcmp(a, "null"))m = WIFI_MODE_NULL;
        else { Serial.println("err mode <null|sta|ap>"); return; }
        esp_err_t e = esp_wifi_set_mode(m);
        esp_wifi_start();
        Serial.printf("ok mode %s rc=%d\n", a, (int)e);
        return;
    }
    if (!strcmp(tok, "promisc")) {
        int v = arg(1);
        esp_err_t e = esp_wifi_set_promiscuous(v != 0);
        Serial.printf("ok promisc=%d rc=%d\n", v, (int)e);
        return;
    }
    if (!strcmp(tok, "chan")) {
        int v = arg(6);
        esp_err_t e = esp_wifi_set_channel((uint8_t)v, WIFI_SECOND_CHAN_NONE);
        Serial.printf("ok chan=%d rc=%d\n", v, (int)e);
        return;
    }
    if (!strcmp(tok, "rfpll")) {
        unsigned mhz = (unsigned)arg(2437);
        rom_rfpll_set_freq(mhz);
        Serial.printf("ok rfpll=%u\n", mhz);
        return;
    }
    // PLL sub-offset override (signed MHz, applied via __wrap_RFChannelSel).
    // 0 = pass-through, nonzero = shift carrier by that many MHz on each TX.
    // See src/pll_wrap.cpp.
    if (!strcmp(tok, "pll_offset")) {
        extern volatile int g_pllSubOffsetMhz;
        int v = arg(0);
        g_pllSubOffsetMhz = v;
        Serial.printf("ok pll_offset=%d MHz (0 = pass-through)\n", v);
        return;
    }
    if (!strcmp(tok, "pll_status")) {
        extern volatile int g_pllSubOffsetMhz;
        extern volatile int g_pllWrapMode;
        extern volatile int g_pllWrapHits;
        extern volatile int g_pllWrapLastChan;
        extern volatile int g_pllWrapLastApplied;
        extern volatile int g_pllWrapPreCount;
        extern volatile int g_pllWrapPostCount;
        Serial.printf("mode=%d offset=%d hits=%d last_chan=%d last_applied=%d pre=%d post=%d\n",
                      g_pllWrapMode, g_pllSubOffsetMhz, g_pllWrapHits,
                      g_pllWrapLastChan, g_pllWrapLastApplied,
                      g_pllWrapPreCount, g_pllWrapPostCount);
        return;
    }
    if (!strcmp(tok, "pll_mode")) {
        extern volatile int g_pllWrapMode;
        int v = arg(1);
        if (v < 0 || v > 2) { Serial.println("err pll_mode 0..2"); return; }
        g_pllWrapMode = v;
        const char* n = (v==0)?"PASSTHRU":(v==1)?"LOG_ONLY":"RECALL";
        Serial.printf("ok pll_mode=%d (%s)\n", v, n);
        return;
    }
    if (!strcmp(tok, "pll_chan")) {
        extern volatile int g_pllChanOverride;
        int v = arg(0);
        // Relaxed to full signed-8 range (-64..63) so we can probe past the
        // WiFi 1-14 window and see what libphy/ROM does with unusual values.
        if (v < -64 || v > 63) { Serial.println("err pll_chan -64..63 (0=off)"); return; }
        g_pllChanOverride = v;
        Serial.printf("ok pll_chan=%d (0=pass-through, 1-14=WiFi grid, other=experimental)\n", v);
        return;
    }
    // ---- OOK / Morse keyer ----
    // tx_ook_morse <wpm> <text...>    — Morse CW on WiFi ch 1 at bo=0
    // tx_ook_pattern <bit_ms> <bits>  — raw 0/1 pattern, bit_ms per bit
    // tx_ook_stop                     — flush queue, release carrier
    if (!strcmp(tok, "tx_ook_morse")) {
        int wpm = arg(20);
        const char* text = strtok(NULL, "");         // rest of line
        if (!text) { Serial.println("err tx_ook_morse <wpm> <text>"); return; }
        while (*text == ' ') text++;
        txOokMorse(text, wpm, 1, 0);
        return;
    }
    if (!strcmp(tok, "tx_ook_pattern")) {
        int bit_ms = arg(50);
        const char* bits = strtok(NULL, "");
        if (!bits) { Serial.println("err tx_ook_pattern <bit_ms> <bits>"); return; }
        while (*bits == ' ') bits++;
        txOokPattern(bits, bit_ms, 1, 0);
        return;
    }
    if (!strcmp(tok, "tx_ook_stop")) {
        txOokStop();
        Serial.println("ok tx_ook_stop");
        return;
    }
    if (!strcmp(tok, "pbus_tx")) {
        const char* a = strtok(nullptr, " \t");
        if (!a) { Serial.println("err pbus_tx on|off"); return; }
        if (!strcmp(a, "on"))  { rom_pbus_xpd_tx_on(1); Serial.println("ok tx on"); }
        else                   { rom_pbus_xpd_tx_off(); Serial.println("ok tx off"); }
        return;
    }
    if (!strcmp(tok, "pbus_rx")) {
        const char* a = strtok(nullptr, " \t");
        if (!a) { Serial.println("err pbus_rx on|off"); return; }
        if (!strcmp(a, "on"))  { rom_pbus_xpd_rx_on(1); Serial.println("ok rx on"); }
        else                   { rom_pbus_xpd_rx_off(); Serial.println("ok rx off"); }
        return;
    }
    if (!strcmp(tok, "txclk")) { int v = arg(1); rom_set_txclk_en(v); Serial.printf("ok txclk=%d\n", v); return; }
    if (!strcmp(tok, "rxclk")) { int v = arg(1); rom_set_rxclk_en(v); Serial.printf("ok rxclk=%d\n", v); return; }
    if (!strcmp(tok, "cont_cfg")) { int v = arg(1); tx_cont_cfg((unsigned)v); Serial.printf("ok cont_cfg=%d\n", v); return; }
    if (!strcmp(tok, "cont_en"))  { tx_cont_en();  Serial.println("ok cont_en"); return; }
    if (!strcmp(tok, "cont_dis")) { tx_cont_dis(); Serial.println("ok cont_dis"); return; }
    if (!strcmp(tok, "rom_start")) {
        int a0 = arg(1), a1 = arg(6), a2 = arg(0);
        rom_start_tx_tone(a0, a1, a2);
        Serial.printf("ok rom_start(%d,%d,%d)\n", a0, a1, a2);
        return;
    }
    if (!strcmp(tok, "rom_stop")) {
        int a0 = arg(1);
        rom_stop_tx_tone(a0);
        Serial.printf("ok rom_stop(%d)\n", a0);
        return;
    }
    if (!strcmp(tok, "ram_start")) {
        // 5-arg form matching the ram_start_tx_tone signature inferred from
        // its disassembly. First arg = power/mode, next ones = freq params.
        int a0 = arg(1), a1 = arg(0), a2 = arg(0), a3 = arg(0), a4 = arg(1);
        ram_start_tx_tone(a0, a1, a2, a3, a4);
        Serial.printf("ok ram_start(%d,%d,%d,%d,%d)\n", a0, a1, a2, a3, a4);
        return;
    }
    if (!strcmp(tok, "ram_stop")) {
        int a0 = arg(1);
        rom_stop_tx_tone(a0);
        Serial.printf("ok ram_stop → rom_stop(%d)\n", a0);
        return;
    }
    if (!strcmp(tok, "rf_init"))         { rf_init();         Serial.println("ok rf_init"); return; }
    if (!strcmp(tok, "cv7_chan"))        { int c=arg(11); chip_v7_set_chan(c); Serial.printf("ok cv7_chan=%d\n",c); return; }
    if (!strcmp(tok, "cv7_chan_nomac"))  { int c=arg(11),s=arg(0); chip_v7_set_chan_nomac(c,s); Serial.printf("ok cv7_chan_nomac(%d,%d)\n",c,s); return; }
    if (!strcmp(tok, "ram_tx_on"))       { int v=arg(1); ram_pbus_xpd_tx_on(v); Serial.printf("ok ram_tx_on=%d\n",v); return; }
    if (!strcmp(tok, "rfpll_ram"))       { unsigned m=(unsigned)arg(2462); ram_rfpll_set_freq(m); Serial.printf("ok rfpll_ram=%u\n",m); return; }
    if (!strcmp(tok, "rftx_init"))       { i2c_rftx_init(); Serial.println("ok rftx_init"); return; }
    if (!strcmp(tok, "set_cf"))          { int c=arg(11); set_chanfreq(c); Serial.printf("ok set_cf=%d\n",c); return; }
    if (!strcmp(tok, "txrx_off"))        { force_txrxoff(); Serial.println("ok txrx_off"); return; }
    if (!strcmp(tok, "pwr_backoff"))     { int d=arg(0); target_power_backoff(d); Serial.printf("ok pwr_backoff=%d\n",d); return; }
    if (!strcmp(tok, "close_pa"))        { int e=arg(0); phy_close_pa(e); Serial.printf("ok close_pa=%d\n",e); return; }
    if (!strcmp(tok, "tx_dig_gain"))     { int a=arg(0xff),b=arg(0xff); set_tx_dig_gain(a,b); Serial.printf("ok tx_dig_gain(0x%x,0x%x)\n",a,b); return; }
    if (!strcmp(tok, "pwctrl_init"))     { tx_pwctrl_init(); Serial.println("ok pwctrl_init"); return; }
    if (!strcmp(tok, "ram_pwr_backoff")) { int d=arg(0); ram_tx_pwr_backoff(d); Serial.printf("ok ram_pwr_backoff=%d\n",d); return; }
    if (!strcmp(tok, "wifitx_lp"))       { int e=arg(0); phy_wifitx_low_power(e); Serial.printf("ok wifitx_lp=%d\n",e); return; }
    if (!strcmp(tok, "bb_init"))         { bb_init(); Serial.println("ok bb_init"); return; }
    if (!strcmp(tok, "pwctrl_cal"))      { tx_pwctrl_cal(); Serial.println("ok pwctrl_cal"); return; }
    if (!strcmp(tok, "pwctrl_init2"))    { tx_pwctrl_init_fn();   Serial.println("ok pwctrl_init"); return; }
    if (!strcmp(tok, "pwctrl_cal2"))     { tx_pwctrl_init_cal_fn(); Serial.println("ok pwctrl_init_cal"); return; }
    // txdc_cal removed -- not exported in Arduino libphy (only ram_txdc_cal_v70)
    if (!strcmp(tok, "cdig_gain"))       { int c=arg(11); set_chan_dig_gain_fn(c); Serial.printf("ok cdig_gain(%d)\n",c); return; }
    if (!strcmp(tok, "reboot"))          { Serial.println("ok reboot"); delay(50); ESP.restart(); return; }
    if (!strcmp(tok, "hopthr")) {
        int v = arg(30);
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        g_hopMinLevel = (uint8_t)v;
        Serial.printf("ok hopthr=%u\n", (unsigned)g_hopMinLevel);
        return;
    }
    if (!strcmp(tok, "hopdelta")) {
        int v = arg(18);
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        g_hopDelta = (uint8_t)v;
        Serial.printf("ok hopdelta=%u (above per-bin floor)\n", (unsigned)g_hopDelta);
        return;
    }
    if (!strcmp(tok, "hopfloor")) {
        // Print a condensed view of the per-bin running noise floor so we
        // can see how it's tracking (e.g. WiFi carrier raises floor
        // locally; quiet region stays at baseline).
        int N = g_latestFrame.fftN ? g_latestFrame.fftN : 128;
        Serial.printf("[hop] floor (N=%d, Q4 scaled):\n", N);
        for (int i = 0; i < N; i += 4) {
            if ((i & 31) == 0) Serial.printf(" bin%3d:", i);
            Serial.printf(" %3u", (unsigned)(g_binFloor[i] >> 4));
            if ((i & 31) == 28) Serial.println();
        }
        Serial.println("\nok");
        return;
    }
    if (!strcmp(tok, "hoprun")) {
        int secs = arg(10);
        if (secs < 2)  secs = 2;
        if (secs > 60) secs = 60;
        Serial.printf("[hop] hoprun start -- %d s, hopthr=%u hopdelta=%u"
                      " tune ch=%u span=%u\n",
                      secs, (unsigned)g_hopMinLevel, (unsigned)g_hopDelta,
                      (unsigned)g_wifiCh, (unsigned)g_wantSpan);
        memset((void*)&g_hoprun, 0, sizeof(g_hoprun));
        g_hoprunActive = true;
        uint32_t t0 = millis();
        uint32_t tickMs = t0;
        while ((millis() - t0) < (uint32_t)(secs * 1000)) {
            delay(50);  // yields, cliPoll stays responsive
            uint32_t now = millis();
            if ((now - tickMs) >= 2000) {
                tickMs = now;
                Serial.printf("[hop]  t=%2us total=%u adv:%u/%u/%u conn=%u\n",
                              (unsigned)((now - t0) / 1000),
                              (unsigned)g_hoprun.total,
                              (unsigned)g_hoprun.adv0, (unsigned)g_hoprun.adv24,
                              (unsigned)g_hoprun.adv78, (unsigned)g_hoprun.connCnt);
            }
        }
        g_hoprunActive = false;
        // Summary: totals + rates + top channel + conn interval + ch distribution
        int topCh = 0; uint32_t topCnt = 0; int distinct = 0;
        for (int b = 0; b < 79; b++) {
            if (g_hoprun.perCh[b] > 0) distinct++;
            if (g_hoprun.perCh[b] > topCnt) { topCnt = g_hoprun.perCh[b]; topCh = b; }
        }
        uint32_t avgDt = g_hoprun.connCnt ? (g_hoprun.connDtSum / g_hoprun.connCnt) : 0;
        Serial.printf("[hop] RESULT %ds: total=%u rate=%u/s\n",
                      secs, (unsigned)g_hoprun.total,
                      (unsigned)(g_hoprun.total / secs));
        Serial.printf("[hop]   adv: 0=%u (2402)  24=%u (2426)  78=%u (2480)\n",
                      (unsigned)g_hoprun.adv0, (unsigned)g_hoprun.adv24,
                      (unsigned)g_hoprun.adv78);
        Serial.printf("[hop]   conn: cnt=%u  avgDt=%ums  fastBt=%u\n",
                      (unsigned)g_hoprun.connCnt, (unsigned)avgDt,
                      (unsigned)g_hoprun.fastHop);
        Serial.printf("[hop]   distinctChs=%d  topCh=%d (hits=%u, %u MHz)\n",
                      distinct, topCh, (unsigned)topCnt, 2402 + topCh);
        Serial.print("[hop]   per-ch:");
        for (int b = 0; b < 79; b++) {
            if (g_hoprun.perCh[b] > 0) Serial.printf(" %d=%u", b, (unsigned)g_hoprun.perCh[b]);
        }
        Serial.println();
        Serial.println("ok");
        return;
    }
    if (!strcmp(tok, "hopclear")) {
        g_hopCount = 0;
        g_hopHead  = 0;
        memset(g_scatterRing, 0, sizeof(g_scatterRing));
        g_binFloorPrimed = false;
        Serial.println("ok hopclear (ring + scatter + floor)");
        return;
    }
    if (!strcmp(tok, "specdump")) {
        // Dump current FFT frame as one CSV line for off-device comparison
        // against hackrf_sweep. Format:
        //   spec,<ts_ms>,<wifiCh>,<spanMode>,<fftN>,<bin0>,<bin1>,...,<binN-1>
        int N = g_latestFrame.fftN ? g_latestFrame.fftN : 128;
        uint32_t ts = millis();
        Serial.printf("spec,%lu,%u,%u,%d",
                      (unsigned long)ts, (unsigned)g_wifiCh,
                      (unsigned)g_wantSpan, N);
        for (int i = 0; i < N; i++) {
            Serial.printf(",%u", (unsigned)g_latestFrame.bins[i]);
        }
        Serial.println();
        Serial.println("ok");
        return;
    }
    if (!strcmp(tok, "specstream")) {
        // Stream specdump-format lines for N seconds, one per unique frame.
        int secs = arg(10);
        if (secs < 1)  secs = 1;
        if (secs > 60) secs = 60;
        Serial.printf("# specstream %ds  ch=%u span=%u\n",
                      secs, (unsigned)g_wifiCh, (unsigned)g_wantSpan);
        uint32_t t0 = millis();
        uint32_t lastSeq = 0;
        while ((millis() - t0) < (uint32_t)(secs * 1000)) {
            uint32_t seq = g_latestFrame.seq;
            if (seq != lastSeq) {
                lastSeq = seq;
                int N = g_latestFrame.fftN ? g_latestFrame.fftN : 128;
                uint32_t ts = millis();
                Serial.printf("spec,%lu,%u,%u,%d",
                              (unsigned long)ts, (unsigned)g_wifiCh,
                              (unsigned)g_wantSpan, N);
                for (int i = 0; i < N; i++) {
                    Serial.printf(",%u", (unsigned)g_latestFrame.bins[i]);
                }
                Serial.println();
            }
            delay(5);
        }
        Serial.println("ok");
        return;
    }
    if (!strcmp(tok, "hopadv")) {
        // Dedicated-tune experiment: cycle (or lock to) BLE adv channels at
        // 40 MHz span. The target bin at each tune:
        //   BLE 37 (2402 MHz): ch 1  S40, bin ~32
        //   BLE 38 (2426 MHz): ch 4  S40, bin ~61
        //   BLE 39 (2480 MHz): ch 13 S40, bin ~90
        // Usage: hopadv [mode=0|37|38|39] [secs=10]
        //   mode 0 = cycle all three; 37/38/39 = lock to that one.
        int mode = arg(0);
        int secs = arg(10);
        if (secs < 2)  secs = 2;
        if (secs > 60) secs = 60;
        struct AdvCfg { uint8_t wifiCh; uint8_t span; int bleCh; int targetMhz; };
        static const AdvCfg cfg[3] = {
            { 1,  (uint8_t)SpanMode::S40, 37, 2402 },
            { 4,  (uint8_t)SpanMode::S40, 38, 2426 },
            { 13, (uint8_t)SpanMode::S40, 39, 2480 },
        };
        int cyc[3]; int cycN = 0;
        if      (mode == 37) { cyc[0] = 0; cycN = 1; }
        else if (mode == 38) { cyc[0] = 1; cycN = 1; }
        else if (mode == 39) { cyc[0] = 2; cycN = 1; }
        else                 { cyc[0] = 0; cyc[1] = 1; cyc[2] = 2; cycN = 3; }
        uint8_t saveCh = g_wifiCh, saveSpan = g_wantSpan;
        if (cycN == 1) {
            Serial.printf("[hop] hopadv %ds LOCK to BLE ch %d (%d MHz, WiFi ch %u S40)\n",
                          secs, cfg[cyc[0]].bleCh, cfg[cyc[0]].targetMhz,
                          (unsigned)cfg[cyc[0]].wifiCh);
        } else {
            Serial.printf("[hop] hopadv %ds CYCLE BLE 37/38/39 @ ~300ms each\n", secs);
        }
        memset((void*)&g_hoprun, 0, sizeof(g_hoprun));
        g_hoprunActive = true;
        g_binFloorPrimed = false;    // floor re-primes after each retune
        uint32_t t0 = millis();
        uint32_t tickMs = t0;
        int cIdx = 0;
        while ((millis() - t0) < (uint32_t)(secs * 1000)) {
            g_wifiCh    = cfg[cyc[cIdx]].wifiCh;
            g_wantSpan  = cfg[cyc[cIdx]].span;
            delay(300);   // ~80 ms retune settle + ~220 ms dwell
            cIdx = (cIdx + 1) % cycN;
            uint32_t now = millis();
            if ((now - tickMs) >= 2000) {
                tickMs = now;
                Serial.printf("[hop]  t=%2us adv:%u/%u/%u total=%u\n",
                              (unsigned)((now - t0) / 1000),
                              (unsigned)g_hoprun.adv0, (unsigned)g_hoprun.adv24,
                              (unsigned)g_hoprun.adv78, (unsigned)g_hoprun.total);
            }
        }
        g_hoprunActive = false;
        g_wifiCh   = saveCh;
        g_wantSpan = saveSpan;
        Serial.printf("[hop] RESULT hopadv %ds: total=%u rate=%u/s\n",
                      secs, (unsigned)g_hoprun.total,
                      (unsigned)(g_hoprun.total / secs));
        Serial.printf("[hop]   adv: 0=%u (2402)  24=%u (2426)  78=%u (2480)\n",
                      (unsigned)g_hoprun.adv0, (unsigned)g_hoprun.adv24,
                      (unsigned)g_hoprun.adv78);
        Serial.println("ok");
        return;
    }
    if (!strcmp(tok, "hopdump")) {
        // Dump the hop ring (newest first) + classifier stats, so we can
        // tell what the HOP tab is actually seeing without eyeballing it.
        HopClass cls; classifyHops(cls);
        Serial.printf("[hop] ring=%d/%d  tune ch=%u span=%u  hopthr=%u  hopdelta=%u\n",
                      g_hopCount, kHopRingSize,
                      (unsigned)g_wifiCh, (unsigned)g_wantSpan,
                      (unsigned)g_hopMinLevel, (unsigned)g_hopDelta);
        int vLo, vHi; hopVisibleBtRange(vLo, vHi);
        Serial.printf("[hop] visible BT ch %d..%d (= %d..%d MHz)\n",
                      vLo, vHi, 2402 + vLo, 2402 + vHi);
        Serial.printf("[hop] totalRecent(2s)=%u  rate=%d/s  adv:0=%u 24=%u 78=%u"
                      "  wifi6=%u wifi11=%u  conn=%u(~%lums)  fastBt=%u\n",
                      (unsigned)cls.totalRecent, cls.totalRecent / 2,
                      (unsigned)cls.adv0, (unsigned)cls.adv24, (unsigned)cls.adv78,
                      (unsigned)cls.wifiBand6, (unsigned)cls.wifiBand11,
                      (unsigned)cls.connCount, (unsigned long)cls.connIntvEstMs,
                      (unsigned)cls.fastHop);
        Serial.printf("[hop] distinctChs=%u  topCh=%u (hits=%u, %u MHz)"
                      "  nonTopCount=%u\n",
                      (unsigned)cls.distinctChs, (unsigned)cls.topCh,
                      (unsigned)cls.topChCount, 2402 + (unsigned)cls.topCh,
                      (unsigned)cls.nonTopCount);
        // Per-BT-ch activity, printed only for non-zero chs to keep it short.
        Serial.print("[hop] per-ch 2s: ");
        bool any = false;
        for (int bt = 0; bt < 79; bt++) {
            if (cls.usagePerBtCh[bt] == 0) continue;
            Serial.printf("%d=%u ", bt, (unsigned)cls.usagePerBtCh[bt]);
            any = true;
        }
        if (!any) Serial.print("(none)");
        Serial.println();
        // Last 20 events: ts delta from now, bin, level, computed BT ch.
        // Snapshot head once -- uiTask advances g_hopHead between
        // iterations otherwise and we'd re-read already-printed slots.
        uint32_t now = millis();
        int snapHead  = g_hopHead;
        int snapCount = g_hopCount;
        int showN = (snapCount < 20 ? snapCount : 20);
        Serial.printf("[hop] last %d events:\n", showN);
        for (int i = 0; i < showN; i++) {
            int idx = (snapHead - 1 - i + kHopRingSize) % kHopRingSize;
            const HopEvent& ev = g_hopRing[idx];
            int bt = hopBinToBtCh(ev.binIdx, ev.wifiCh, ev.spanMode, 128);
            if (bt < 0) {
                Serial.printf("  age=%4lums bin=%3u lv=%3u bt=OOB\n",
                              (unsigned long)(now - ev.tsMs),
                              (unsigned)ev.binIdx, (unsigned)ev.level);
            } else {
                Serial.printf("  age=%4lums bin=%3u lv=%3u bt=%2d  (%u MHz)\n",
                              (unsigned long)(now - ev.tsMs),
                              (unsigned)ev.binIdx, (unsigned)ev.level,
                              bt, 2402 + bt);
            }
        }
        Serial.println("ok");
        return;
    }
    if (!strcmp(tok, "wifi_reinit")) {
        // Force WiFi stack teardown + re-init -- same chain that produces
        // the boot-time TX cal blip.
        Serial.println("wifi_reinit: deinit...");
        esp_wifi_set_promiscuous(false);
        esp_wifi_stop();
        esp_wifi_deinit();
        delay(50);
        Serial.println("wifi_reinit: init + start...");
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        esp_wifi_set_mode(WIFI_MODE_NULL);
        esp_wifi_start();
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
        Serial.println("ok wifi_reinit");
        return;
    }
    if (!strcmp(tok, "fullprime")) {
        // Full setup sequence attempt: force off → chan → RF init → TX pbus → tx_cont_cfg/en
        force_txrxoff();
        chip_v7_set_chan(arg(11));
        rf_init();
        i2c_rftx_init();
        ram_pbus_xpd_tx_on(1);
        rom_set_txclk_en(1);
        tx_cont_cfg(1);
        tx_cont_en();
        Serial.println("ok fullprime (chan11, PA primed)");
        return;
    }
    if (!strcmp(tok, "tone")) {
        int ch = arg(11), ms = arg(500);
        Serial.printf("tone ch=%d ms=%d (via espSdrCwTone)\n", ch, ms);
        espSdrCwTone((uint8_t)ch, (uint32_t)ms);
        return;
    }
    if (!strcmp(tok, "tx_cw")) {
        int ch = arg(11);
        int bo = 0;
        char* bo_s = strtok(NULL, " ");
        if (bo_s) bo = atoi(bo_s);
        g_cwReqCh    = (uint8_t)ch;
        g_cwReqBo    = (uint8_t)bo;
        g_cwHop      = false;
        g_cwMode     = 0;                        // TONE
        g_cwReqStart = true;
        Serial.printf("ok tx_cw request posted (ch=%d bo=%d)\n", ch, bo);
        return;
    }
    if (!strcmp(tok, "tx_pkt20")) {
        int ch = arg(11);
        int bo = 0;
        char* bo_s = strtok(NULL, " ");
        if (bo_s) bo = atoi(bo_s);
        g_cwReqCh    = (uint8_t)ch;
        g_cwReqBo    = (uint8_t)bo;
        g_cwHop      = false;
        g_cwMode     = 1;                        // PKT20
        g_cwReqStart = true;
        Serial.printf("ok tx_pkt20 request posted (ch=%d bo=%d)\n", ch, bo);
        return;
    }
    if (!strcmp(tok, "tx_pkt20_hop")) {
        // tx_pkt20_hop [bo] [dwellMs] -- hops across WiFi 1/6/11
        int bo = arg(0);
        char* dw_s = strtok(NULL, " ");
        int dw = dw_s ? atoi(dw_s) : 200;
        g_cwReqCh      = 1;                      // start on ch 1
        g_cwReqBo      = (uint8_t)bo;
        g_cwMode       = 1;                      // PKT20
        g_cwHopMask    = (1u << 0) | (1u << 5) | (1u << 10);  // WiFi 1/6/11
        g_cwHopDwellMs = (uint16_t)dw;
        g_cwHop        = true;                   // enable hop
        g_cwHopPreset  = HOP_WIFI_161;
        g_cwReqStart   = true;
        Serial.printf("ok tx_pkt20_hop request posted (bo=%d dwellMs=%d) mask=1/6/11\n", bo, dw);
        return;
    }
    if (!strcmp(tok, "tx_cw_old")) {
        // Legacy direct path kept for debugging ONLY.
        int ch = arg(11);
        int bo = 0;
        char* bo_s = strtok(NULL, " ");
        if (bo_s) bo = atoi(bo_s);
        Serial.printf("tx_cw_old: init rftest ch=%d bo=%d\n", ch, bo);
        g_cwActive = true;
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_phy_rftest_init();
        esp_phy_rftest_config(0);
        esp_phy_test_start_stop(1);              // BUSY -- keep TX up
        esp_phy_wifi_tx_tone(1, ch, bo);
        Serial.println("ok tx_cw started (ongoing) -- call 'tx_stop' to end");
        return;
    }
    if (!strcmp(tok, "tx_stop")) {
        // Route stop through cwTask too, same as the UI button.
        g_cwReqStop = true;
        Serial.println("ok tx_stop request posted");
        return;
    }
    if (!strcmp(tok, "reset_rx")) {
        rom_pbus_workmode();
        rom_pbus_xpd_tx_off();
        rom_pbus_xpd_rx_on(1);
        rom_set_rxclk_en(1);
        esp_wifi_set_mode(WIFI_MODE_NULL);
        esp_wifi_start();
        esp_wifi_set_promiscuous(true);
        Serial.println("ok reset_rx");
        return;
    }
    // Raw hardware CW test: bypass esp_phy_wifi_tx_tone entirely and drive the
    // TX chain directly from ROM + libphy symbols. Lets us set an arbitrary PLL
    // freq (not just WiFi channels) and see if the carrier follows. Experimental.
    if (!strcmp(tok, "raw_cw")) {
        unsigned mhz = (unsigned)arg(2320);
        Serial.printf("[raw_cw] setup PA path mhz=%u\n", mhz);
        rom_pbus_workmode();
        rom_pbus_xpd_rx_off();
        rom_pbus_xpd_tx_on(1);
        rom_set_rxclk_en(0);
        rom_set_txclk_en(1);
        rom_rfpll_set_freq(mhz);
        // tx_cont_cfg(1) + tx_cont_en() is the libphy "clean" continuous-TX path
        tx_cont_cfg(1);
        tx_cont_en();
        // Belt-and-suspenders: also fire the ROM tone function if the libphy
        // path alone doesn't drive the PA. Args are guesses by analogy with
        // start_tx_tone(1) on other chips.
        rom_start_tx_tone(1, 0, 0);
        Serial.printf("ok raw_cw started at %u MHz\n", mhz);
        return;
    }
    if (!strcmp(tok, "raw_cw_off")) {
        rom_stop_tx_tone(1);
        tx_cont_dis();
        rom_pbus_xpd_tx_off();
        rom_set_txclk_en(0);
        // Restore RX state
        rom_pbus_workmode();
        rom_pbus_xpd_rx_on(1);
        rom_set_rxclk_en(1);
        Serial.println("ok raw_cw_off");
        return;
    }
    Serial.printf("err unknown '%s' -- try 'help'\n", tok);
}

static void cliPoll() {
    static char buf[96];
    static size_t len = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            buf[len] = 0;
            if (len) cliExec(buf);
            len = 0;
            continue;
        }
        if (len < sizeof(buf) - 1) buf[len++] = c;
    }
}

// Dedicated CW task: pinned to Core 0, mirrors the bulb's jamTask shape.
// Owns ALL cert_test phy calls so UI (Core 1) never sees phy-init stalls.
// Bounded stop: if esp_phy_* calls block, the task force-releases state
// after kCwStopTimeoutMs so the UI doesn't hang either.
void cwTask(void*) {
    uint32_t lastHopMs   = 0;
    uint32_t lastHitMs   = 0;
    for (;;) {
            // Guard: STOP wins over START if both are posted in the same frame.
        if (g_cwReqStop) {
            g_cwReqStart = false;
            g_cwReqStop  = false;
            g_txRunning  = false;                        // UI responds instantly
            g_cwHop      = false;                        // freeze hop loop
            uint32_t hitsBefore = g_cwHits;
            radioSwitchMode(RADIO_IDLE, 0, 0);           // arbiter handles teardown
            g_cwActive       = false;
            g_lastTxFinishMs = millis();
            Serial.printf("[tx] CW OFF (hits=%u)\n", (unsigned)hitsBefore);
        }
        // START: ignore if already active (debounce double-taps).
        if (g_cwReqStart) {
            g_cwReqStart = false;
            if (g_cwActive) {
                Serial.println("[tx] (START ignored -- already running; use STOP first)");
            } else {
                uint8_t ch     = g_cwReqCh;
                uint8_t bo     = g_cwReqBo;
                // Modes 3 (BT) and 4 (BT_SWEEP) both enter via RADIO_TX_BT_CW
                // -- for BT modes we override `ch` with the direct BT channel.
                uint8_t target;
                if      (g_cwMode == 1) target = RADIO_TX_PKT20;
                else if (g_cwMode == 3) { target = RADIO_TX_BT_CW; ch = g_btChan; }
                else if (g_cwMode == 4) { target = RADIO_TX_BT_CW; ch = g_btChan; }
                else                    target = RADIO_TX_CW;
                g_cwActive     = true;
                vTaskDelay(pdMS_TO_TICKS(30));
                if (radioSwitchMode(target, ch, bo)) {
                    g_cwHopCurCh    = ch;
                    g_cwSweepLast   = millis();
                    g_cwSweepPos    = 0;
                    g_txRunning     = true;
                    g_certTestDirty = true;    // need reboot before 802.11
                    lastHopMs       = millis();
                } else {
                    g_cwActive = false;
                    Serial.println("[tx] radioSwitchMode FAILED");
                }
            }
        }
        // PA change while live: re-run arbiter at new backoff on current ch.
        if (g_cwReqPaChange) {
            g_cwReqPaChange = false;
            if (g_cwActive) {
                uint8_t target = (g_cwMode == 1) ? RADIO_TX_PKT20 : RADIO_TX_CW;
                radioSwitchMode(target, g_cwHopCurCh, g_cwReqNewBo);
                g_cwReqBo = g_cwReqNewBo;
            }
    }
        // Hop-mode channel stepping across the user-selected mask.
        if (g_cwActive && g_cwHop) {
            uint32_t now = millis();
            if (now - lastHopMs >= g_cwHopDwellMs) {
                lastHopMs = now;
                uint8_t next = nextMaskedChan(g_cwHopCurCh);
                if (next != g_cwHopCurCh) {
                    g_cwHopCurCh = next;
                    // Hop via the arbiter so the full teardown + re-init
                    // cycle runs on every channel change. ~150 ms per hop.
                    uint8_t target = (g_cwMode == 1) ? RADIO_TX_PKT20 : RADIO_TX_CW;
                    radioSwitchMode(target, next, g_cwReqBo);
                    g_cwHits++;    // alive-ticker
                }
            }
        }
        // SWEEP mode: tight loop of chip_v7_set_chan_nomac + delayMicroseconds
        // matching the bulb's NRF jammer architecture. CW tone was already
        // started by the arbiter; we only need to hammer the PLL retune
        // here. Each inner iteration = ~500 us, 50 iters per batch = 25 ms
        // before we yield. That gives UI/other tasks a 1 ms slot every batch.
        // BT_SWEEP mode: tight loop of bt_chan_change at microsecond rate.
        // Pattern 0 (FULL) covers 2402-2480. Pattern 1 (SPAN) sweeps ±N MHz
        // around the WiFi ch center. Pattern 2 (BLE_ADV) hits 0/24/78 only.
        if (g_cwActive && g_cwMode == 4 && !g_cwHop) {
            int pattern  = g_btSweepPat;
            int centerBt = g_btChan;    // direct BT channel from UI
            int lo = 0, hi = 78;
            if (pattern == 1) {
                int sp = g_cwSweepSpan;    // span in "channels" -- for BT = MHz
                if (sp < 1) sp = 1;
                if (sp > 20) sp = 20;
                lo = centerBt - sp;  if (lo < 0)  lo = 0;
                hi = centerBt + sp;  if (hi > 78) hi = 78;
            }
            // BLE_ADV uses fixed array
            static const uint8_t bleAdv[3] = { 0, 24, 78 };
            int cycleLen = (pattern == 2) ? 3 : (hi - lo + 1);
            uint32_t dwellUs = g_sweepDwellUs;
            if (dwellUs < kSweepMinDwellUs) dwellUs = kSweepMinDwellUs;
            if (dwellUs > kSweepMaxDwellUs) dwellUs = kSweepMaxDwellUs;
            for (int i = 0; i < kYieldEvery; i++) {
                g_cwSweepPos = (int8_t)((g_cwSweepPos + 1) % cycleLen);
                int bch = (pattern == 2) ? (int)bleAdv[g_cwSweepPos]
                                         : (lo + g_cwSweepPos);
                bt_chan_change(bch);
                delayMicroseconds(dwellUs);
                if (g_cwReqStop) break;
            }
            g_cwHits++;
            vTaskDelay(1);
            continue;
        }
        if (g_cwActive && g_cwMode == 2) {
            // SWEEP mode -- works solo OR combined with HOP:
            //   HOP OFF: ±span around g_cwReqCh (one wide block)
            //   HOP ON : interleave ±span around EACH masked ch so all hop
            //            centers show as always-on wide blocks (no blink).
            int span = g_cwSweepSpan;
            if (span < 1) span = 1;
            if (span > 6) span = 6;
            // Build hop-center set at loop entry (local copy, cheap).
            uint8_t hopCenters[13];
            int nHops = 0;
            if (g_cwHop) {
                for (int b = 0; b < 13; b++) {
                    if (g_cwHopMask & (1u << b)) hopCenters[nHops++] = (uint8_t)(b + 1);
                }
            }
            if (nHops == 0) {
                hopCenters[0] = (uint8_t)g_cwReqCh;
                nHops = 1;
            }
            int cycleLen = nHops * (2 * span + 1);
            uint32_t dwellUs = g_sweepDwellUs;
            if (dwellUs < kSweepMinDwellUs) dwellUs = kSweepMinDwellUs;
            if (dwellUs > kSweepMaxDwellUs) dwellUs = kSweepMaxDwellUs;
            for (int i = 0; i < kYieldEvery; i++) {
                g_cwSweepPos = (int8_t)((g_cwSweepPos + 1) % cycleLen);
                int hopIdx     = g_cwSweepPos / (2 * span + 1);
                int spanOffset = (g_cwSweepPos % (2 * span + 1)) - span;
                int ch = (int)hopCenters[hopIdx] + spanOffset;
                if (ch < 1)  ch = 1;
                if (ch > 13) ch = 13;
                chip_v7_set_chan_nomac(ch, 0);
                delayMicroseconds(dwellUs);
                if (g_cwReqStop) break;
            }
            g_cwHits++;
            vTaskDelay(1);
            continue;
        }
        // BT CW live retune: slider moves while CW is on -> bt_chan_change.
        // Carrier stays up, PLL hops to the new 1 MHz BT channel.
        if (g_cwActive && g_cwMode == 3) {
            static uint8_t lastSentBt = 0xFF;
            uint8_t cur = g_btChan;
            if (cur != lastSentBt) {
                lastSentBt = cur;
                bt_chan_change(cur);
            }
        }
        // Idle hit bump every ~200ms when active-but-not-hopping, so the
        // UI readout proves the task is scheduled.
        if (g_cwActive && !g_cwHop && g_cwMode != 2) {
            uint32_t now = millis();
            if (now - lastHitMs >= 200) { lastHitMs = now; g_cwHits++; }
        }
        // 10 ms task tick -- generous enough that the UI touch task on Core 1
        // never has to wait on us, tight enough for crisp HOP timing.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void loop() {
    cliPoll();
    static unsigned long last     = 0;
    static uint32_t      lastHits = 0;
    static unsigned long stuckMs  = 0;
    unsigned long now = millis();

    // Auto-recovery watchdog: if cwTask is marked active but hits don't
    // advance, reboot so STOP always succeeds (PKT20's internal stop is
    // unreliable; watchdog is the real stop path). 3s = fast recovery.
    // Also honors an explicit "stop requested but not completing" state:
    // if g_cwReqStop was set and 3s later g_cwActive is still true,
    // reboot immediately.
    if (g_cwActive) {
        if (stuckMs == 0 || g_cwHits != lastHits) {
            lastHits = g_cwHits;
            stuckMs  = now;
        } else if ((now - stuckMs) > 3000) {
            Serial.println("[sh] WATCHDOG: cwTask stuck 3s with cw=ON, rebooting");
            delay(50);
            ESP.restart();
        }
    } else {
        stuckMs  = 0;
        lastHits = g_cwHits;
    }

    if (now - last > 5000) {
        Serial.printf("[sh] alive; screen=%s free=%u cw=%s hits=%u\n",
                      kScreenName[(int)g_screen],
                      (unsigned)ESP.getFreeHeap(),
                      g_cwActive ? "ON" : "off",
                      (unsigned)g_cwHits);
        last = now;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
