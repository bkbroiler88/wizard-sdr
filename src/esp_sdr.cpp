// ============================================================================
//  ESP-SDR debug-path RX driver -- Signalhound port
//
//  Register sequence lifted verbatim from
//    esp-sdr-main/main/targets/esp32/receiver.c
//  with the ESP-SDR team's documentation preserved as comments.
//
//  Phase 2.0 slice: just Wi-Fi init + prepare_rx() + read gain_max. No capture.
// ============================================================================

#include "esp_sdr.h"
#include <Arduino.h>

extern "C" {
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "soc/soc.h"
#include "soc/dport_reg.h"
#include "soc/dport_access.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

#include <string.h>

// ------- Undocumented register addresses (from receiver.c) -------------------
static constexpr uint32_t RX_GAIN     = 0x3ff5c02cu;    // AGC + manual gain
static constexpr uint32_t DUMP_CTRL   = 0x60033d90u;    // (used in Phase 2.1)
static constexpr uint32_t DUMP_STATUS = 0x60033d94u;    // (used in Phase 2.1)
static constexpr uint32_t DUMP_BYTES  = 0x60033dc4u;    // (used in Phase 2.1)

// ------- Undocumented ROM symbols (resolved by Arduino-ESP32's rom.ld) -------
// These live in the ESP32 bootROM. The ESP-SDR team identified them via
// LLM-assisted reverse engineering of ESP-IDF's librftest.a. The symbol
// addresses are baked into rom.ld, which Arduino-ESP32 inherits from IDF.
extern "C" {
    void      rom_pbus_workmode(void);
    void      rom_pbus_xpd_rx_on(int);
    void      rom_pbus_xpd_tx_off(void);
    void      rom_set_rxclk_en(int);
}

static bool    g_ready     = false;
static uint8_t g_gainMax   = 0;
static uint8_t g_currentCh = 6;         // kept in sync with esp_wifi_set_channel
static bool    g_agcOn     = true;      // mirrors RX_GAIN bit 23 (clear = AGC)
static uint8_t g_manGain   = 0;         // current manual gain value

// prepare_rx() verbatim from receiver.c:77-83, minus the LO plan call
// (which belongs in tune_rx and is a Phase 2.1 concern).
static void prepare_rx() {
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
}

// Guarded ESP_ERROR_CHECK equivalent -- logs failure and returns false
// instead of aborting, so a bad init can be visible on the UI.
static bool okOrLog(const char* what, esp_err_t err) {
    if (err == ESP_OK) return true;
    Serial.printf("[sdr] %s FAILED rc=%d (%s)\n", what, (int)err, esp_err_to_name(err));
    return false;
}

bool espSdrBegin() {
    Serial.println("[sdr] espSdrBegin: starting");

    // --- NVS: ESP-IDF needs this; Arduino-ESP32 usually init'd it already
    //    but calling again is safe (returns ESP_OK fast-path). On a brand-new
    //    partition we may get NO_FREE_PAGES / NEW_VERSION — erase and retry.
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("[sdr] NVS reinit: erase + retry");
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    if (!okOrLog("nvs_flash_init", e)) return false;

    // --- Default event loop (idempotent — Arduino may have created it)
    e = esp_event_loop_create_default();
    if (e == ESP_ERR_INVALID_STATE) {
        Serial.println("[sdr] event loop already exists (ok)");
    } else if (!okOrLog("esp_event_loop_create_default", e)) {
        return false;
    }

    // --- Wi-Fi stack init in NULL mode + promiscuous (parks radio in RX)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (!okOrLog("esp_wifi_init",            esp_wifi_init(&cfg))) return false;
    if (!okOrLog("esp_wifi_set_storage",     esp_wifi_set_storage(WIFI_STORAGE_RAM))) return false;
    if (!okOrLog("esp_wifi_set_mode(NULL)",  esp_wifi_set_mode(WIFI_MODE_NULL))) return false;
    if (!okOrLog("esp_wifi_start",           esp_wifi_start())) return false;
    if (!okOrLog("esp_wifi_set_ps(NONE)",    esp_wifi_set_ps(WIFI_PS_NONE))) return false;
    if (!okOrLog("esp_wifi_set_promiscuous", esp_wifi_set_promiscuous(true))) return false;
    // Tune to WiFi ch 6 (2437 MHz) — center of ISM, more likely to catch
    // live traffic than ch 1 which is often avoided.
    if (!okOrLog("esp_wifi_set_channel(6)",  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE))) return false;
    g_currentCh = 6;

    // --- ROM sequence: power up receive path
    prepare_rx();

    // --- Read gain_max from RX_GAIN register. If this reads 0 or 0xFF the
    //     register layout changed or the chip isn't initialized the way we
    //     think. Expected: a small positive integer (e.g. 72–127).
    g_gainMax = (uint8_t)((REG_READ(RX_GAIN) >> 8) & 127);
    Serial.printf("[sdr] RX_GAIN raw=0x%08lx gain_max=%u\n",
                  (unsigned long)REG_READ(RX_GAIN), g_gainMax);

    // Hardware AGC is the default (bit 23 of RX_GAIN clear means AGC on)
    REG_CLR_BIT(RX_GAIN, (1u << 23));
    g_agcOn   = true;
    g_manGain = g_gainMax;    // sensible default if the UI switches to MAN

    g_ready = true;
    Serial.println("[sdr] espSdrBegin: READY");
    return true;
}

uint8_t espSdrGainMax() { return g_gainMax; }
bool    espSdrReady()   { return g_ready; }
bool    espSdrAgc()     { return g_agcOn; }
uint8_t espSdrGain()    { return g_manGain; }

// AGC: bit 23 of RX_GAIN. CLEAR = AGC on (hardware default), SET = manual.
bool espSdrSetAgc(bool on) {
    if (!g_ready) { Serial.println("[sdr] setAgc: not ready"); return false; }
    if (on) {
        REG_CLR_BIT(RX_GAIN, (1u << 23));
    } else {
        REG_SET_BIT(RX_GAIN, (1u << 23));
        // When switching to manual, make sure the gain field reflects the
        // last manual value (don't leave the chip at zero).
        uint32_t v = REG_READ(RX_GAIN) & ~0x7fu;
        REG_WRITE(RX_GAIN, v | (uint32_t)(g_manGain & 0x7f));
    }
    g_agcOn = on;
    Serial.printf("[sdr] AGC %s\n", on ? "ON" : "OFF (manual)");
    return true;
}

// Repeated raw 802.11 TX for durationMs. Our normal RX setup is
// WIFI_MODE_NULL + promiscuous, which has no running TX interface -- the
// 80211_tx call needs STA (or AP) to be the mode. Switch to STA for the
// burst, restore NULL + promiscuous + ROM-level RX after.
uint32_t espSdrTxBurst(const uint8_t* frame, size_t frameLen,
                        uint32_t durationMs, uint32_t intervalUs,
                        uint8_t channel) {
    if (!g_ready || !frame || frameLen < 24 || durationMs == 0) {
        Serial.printf("[sdr] txBurst: bad args ready=%d frame=%p len=%u ms=%lu\n",
                      g_ready, frame, (unsigned)frameLen, (unsigned long)durationMs);
        return 0;
    }
    if (channel < 1 || channel > 14) channel = g_currentCh;
    if (intervalUs > 100000) intervalUs = 100000;    // 100 ms cap
    const uint8_t savedCh = g_currentCh;
    Serial.printf("[sdr] txBurst: start len=%u durMs=%lu intvlUs=%lu ch=%u\n",
                  (unsigned)frameLen, (unsigned long)durationMs,
                  (unsigned long)intervalUs, (unsigned)channel);

    // --- Enter TX mode (STA has a running TX interface; NULL does not) ----
    esp_wifi_set_promiscuous(false);
    esp_err_t e = esp_wifi_set_mode(WIFI_MODE_STA);
    if (e != ESP_OK) {
        Serial.printf("[sdr] txBurst: set_mode(STA) failed rc=%d\n", (int)e);
    }
    esp_wifi_start();
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    // --- Fire ---
    // Yield every ~32 frames so IDLE on Core-0 can run and the task
    // watchdog gets fed. Without this, 2s+ busy loops crash the chip.
    const uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t count = 0, failFirst = 0, yieldCtr = 0;
    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < durationMs) {
        esp_err_t t = esp_wifi_80211_tx(WIFI_IF_STA, frame, frameLen, false);
        if (t == ESP_OK) count++;
        else if (count == 0 && failFirst == 0) failFirst = (uint32_t)t;
        if (intervalUs) esp_rom_delay_us(intervalUs);
        if (++yieldCtr >= 32) {
            yieldCtr = 0;
            vTaskDelay(1);     // 1 tick = ~10 ms with default 100 Hz tick
        }
    }

    // --- Restore RX ---
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(savedCh, WIFI_SECOND_CHAN_NONE);
    prepare_rx();

    if (count == 0) {
        Serial.printf("[sdr] txBurst: 0 frames sent; first-call rc=0x%lx (%s)\n",
                      (unsigned long)failFirst,
                      failFirst ? esp_err_to_name((esp_err_t)failFirst) : "none");
    } else {
        Serial.printf("[sdr] txBurst: sent %lu frames in %lums ch=%u\n",
                      (unsigned long)count, (unsigned long)durationMs,
                      (unsigned)channel);
    }
    return count;
}

// Manual gain: low 7 bits of RX_GAIN. Writing has no effect until AGC is off.
bool espSdrSetGain(uint8_t gain) {
    if (!g_ready) { Serial.println("[sdr] setGain: not ready"); return false; }
    if (g_gainMax && gain > g_gainMax) gain = g_gainMax;
    g_manGain = gain;
    uint32_t v = REG_READ(RX_GAIN) & ~0x7fu;
    REG_WRITE(RX_GAIN, v | (uint32_t)(gain & 0x7f));
    return true;
}

// WiFi channel centers: ch 1 = 2412 MHz, each next channel +5 MHz,
// except ch 14 (2484 MHz, Japan-only — included for completeness).
uint32_t espSdrFreqMHz() {
    if (g_currentCh < 1 || g_currentCh > 14) return 0;
    return (g_currentCh == 14) ? 2484u : (2407u + 5u * (uint32_t)g_currentCh);
}

bool espSdrSetChannel(uint8_t ch) {
    if (!g_ready) { Serial.println("[sdr] setChannel: not ready"); return false; }
    if (ch < 1 || ch > 14) {
        Serial.printf("[sdr] setChannel: bad ch=%u\n", (unsigned)ch);
        return false;
    }
    esp_err_t e = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (e != ESP_OK) {
        Serial.printf("[sdr] setChannel ch=%u FAILED rc=%d (%s)\n",
                      (unsigned)ch, (int)e, esp_err_to_name(e));
        return false;
    }
    // Re-assert the RX path. esp_wifi_set_channel may toggle calibration
    // and knock the ROM-level RX enable clear; cheap to redo.
    prepare_rx();
    g_currentCh = ch;
    Serial.printf("[sdr] retuned to ch %u (%lu MHz)\n",
                  (unsigned)ch, (unsigned long)espSdrFreqMHz());
    return true;
}

// =============================================================================
// Phase 2.1: snapshot capture
//
// Ported verbatim from esp-sdr-main/main/targets/esp32/receiver.c:87-125.
// Hardware writes to a FIXED SRAM bank at 0x3ffe8000 whenever DUMP_MODE=3
// is selected; we have no choice over the destination. In the original IDF
// project this region is carved out of heap via SOC_RESERVE_MEMORY_REGION.
// Here (Arduino) it isn't — if the heap handed any of that 64 KB to anyone,
// we clobber it. One capture should still succeed because the hardware
// writes complete before anything else can read, and we copy the result out
// right away; sustained use will need the heap reservation (see Phase 2.2).
// =============================================================================

static constexpr unsigned CAPACITY    = 16384u;
static constexpr unsigned MAX_SAMPLES = CAPACITY - 4;
static constexpr uint32_t SENTINEL    = 0xa55aa55au;
static uint32_t* const    g_samples   = (uint32_t*)0x3ffe8000u;

bool espSdrAcquireBurst(uint32_t* out, size_t n, uint8_t clockIdx, uint32_t* elapsedUs) {
    if (!g_ready) { Serial.println("[sdr] acquire: not ready"); return false; }
    // Minimum raised from the reference's hardcoded 256 so we can run a
    // 128-point FFT when the UI requests it.
    if (n < 64 || n > MAX_SAMPLES || clockIdx > 2 || !out) {
        Serial.printf("[sdr] acquire: bad args n=%u clk=%u\n", (unsigned)n, (unsigned)clockIdx);
        return false;
    }

    // Phase 2.2 note: the ESP-SDR original primes the entire 64 KB SRAM bank
    // with a SENTINEL pattern here, then verifies that only `n` words got
    // overwritten by the hardware. That's a diagnostic for short writes, NOT
    // required for operation -- and in Arduino (no SOC_RESERVE) that 64 KB
    // write hits heap memory the Wi-Fi stack is actively using, causing
    // boot-loops at 60 Hz capture rate. We only need the `n` words the
    // hardware will overwrite, and we memcpy them out immediately after.
    REG_WRITE(DUMP_CTRL, 0);

    // --- Switch SRAM bank 2 into MAC dump mode 3 (full 64 KB aperture).
    //     Preserve the previous owner + byte-select so Wi-Fi doesn't choke
    //     after we hand them back.
    uint32_t ownerSave = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
    uint32_t byteSave  = REG_READ(DUMP_BYTES);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,
        (ownerSave & ~DPORT_MAC_DUMP_MODE_M) | (3u << DPORT_MAC_DUMP_MODE_S));
    REG_WRITE(DUMP_BYTES, 0x03020100u);         // Four distinct byte lanes

    // --- Program CTRL: count | source=0 | clock bits. BIT(31) = enable.
    //     clockIdx 0 -> 16 MS/s (BIT 16 set); 1 -> 80 MS/s (both clear);
    //     clockIdx 2 -> 40 MS/s (BIT 15 set).
    uint32_t ctrl = (uint32_t)n
                  | (0u << 20)
                  | (((uint32_t)clockIdx == 0 ? 1u : 0u) << 16)
                  | (((uint32_t)clockIdx == 2 ? 1u : 0u) << 15);
    REG_WRITE(DUMP_CTRL, ctrl | BIT(31));

    // --- Kick the capture (BIT 19 is a one-shot trigger).
    int64_t t0 = esp_timer_get_time();
    REG_SET_BIT(DUMP_CTRL, BIT(19));
    REG_CLR_BIT(DUMP_CTRL, BIT(19));

    // --- Spin on done bit (BIT 18). 20 ms upper bound; at 16 MS/s a
    //     256-sample capture completes in <20 us.
    while (!(REG_READ(DUMP_CTRL) & BIT(18)) && esp_timer_get_time() - t0 < 20000) {}
    uint32_t result = REG_READ(DUMP_CTRL);
    uint32_t status = REG_READ(DUMP_STATUS);
    uint32_t elapsed = (uint32_t)(esp_timer_get_time() - t0);

    // --- Tear down: disable DUMP, hand SRAM mode + byte-lane back to Wi-Fi.
    REG_WRITE(DUMP_CTRL, 0);
    esp_rom_delay_us(10);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, ownerSave);
    REG_WRITE(DUMP_BYTES, byteSave);

    if (!(result & BIT(18)) || (status & 0x7fffu) != n) {
        Serial.printf("[sdr] acquire TIMEOUT result=0x%08lx status=0x%08lx elapsed=%luus\n",
                      (unsigned long)result, (unsigned long)status, (unsigned long)elapsed);
        return false;
    }

    // --- Copy out to the caller's buffer immediately.
    memcpy(out, g_samples, n * sizeof(uint32_t));
    if (elapsedUs) *elapsedUs = elapsed;
    return true;
}
