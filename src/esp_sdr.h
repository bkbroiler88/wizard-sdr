// ============================================================================
//  ESP-SDR debug-path RX driver -- Signalhound port, Phase 2.0 shim
//
//  Thin C++ wrapper over the register-level RX code from the ESP-SDR project
//  (https://espargos.net/espsdr/). The implementation lives in esp_sdr.cpp.
//
//  Phase 2.0 scope: bring up Wi-Fi in NULL + promiscuous mode (parks the radio
//  in RX), run prepare_rx(), read gain_max from the undocumented RX_GAIN
//  register. No sample acquisition yet (that's Phase 2.1). The purpose of this
//  phase is to prove the ROM symbol linking works and the stack init doesn't
//  crash on our board.
// ============================================================================
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Initialize the receive side of the Wi-Fi radio:
//   - nvs_flash_init() / event loop
//   - esp_wifi_init() + WIFI_MODE_NULL + start
//   - set promiscuous true, lock channel 1
//   - prepare_rx() ROM sequence (pbus workmode, RX on, clock on)
//   - read gain_max from RX_GAIN register (0x3ff5c02c >> 8 & 127)
//
// Returns true on success, false if any ESP-IDF call failed. Logs to Serial.
bool espSdrBegin();

// Phase 2.0 inspection: report the gain_max value read at init.
uint8_t espSdrGainMax();

// Phase 2.0 inspection: has espSdrBegin() been called and succeeded?
bool espSdrReady();

// Phase 2.1: snapshot capture via the DUMP_CTRL register path.
// Grabs `n` uint32 I/Q words (10-bit I, 10-bit Q + metadata, packed in the low
// 20 bits of each word) into `out`. The hardware writes to a fixed SRAM bank
// at 0x3ffe8000; we copy from there to `out` immediately after capture to
// avoid holding that region while other code runs.
//   n:          64..16380. Must be a positive sample count.
//   clockIdx:   0 = 16 MS/s, 1 = 80 MS/s, 2 = 40 MS/s
//   out:        caller buffer, at least n uint32s
//   elapsedUs:  filled with wall-time of the capture in microseconds (nullable)
// Returns false on timeout or arg error; logs reason to Serial.
bool espSdrAcquireBurst(uint32_t* out, size_t n, uint8_t clockIdx, uint32_t* elapsedUs);

// Phase 2.3: retune the receiver to a different WiFi channel. Must be called
// from the same task that captures (Core-0 RF task). Chan is 1..14.
// Returns false if the channel is out of range or esp_wifi_set_channel fails.
bool espSdrSetChannel(uint8_t ch);

// Phase 2.3: center frequency the capture is currently tuned to, in MHz.
uint32_t espSdrFreqMHz();

// Phase A TX: transmit a crafted raw 802.11 frame repeatedly for `durationMs`.
// Uses esp_wifi_80211_tx on the currently tuned channel. Works while in
// promiscuous RX, but the receiver will stop producing new captures during
// the burst because the radio can only do one thing at a time.
//
// Returns the number of frames actually transmitted. 0 means the TX path
// refused the call (not ready, bad args, or hardware).
uint32_t espSdrTxBurst(const uint8_t* frame, size_t frameLen,
                        uint32_t durationMs, uint32_t intervalUs,
                        uint8_t channel);

// Phase 2.4: receiver gain control.
//
// The ESP32's integrated receive path has a programmable gain stage. In AGC
// mode (default) the chip adjusts gain automatically. In manual mode you set
// a value in 0..gainMax (where gainMax is what espSdrGainMax() reports --
// typically 72). Higher = more sensitive but easier to overload with a
// nearby strong carrier.
//
// Must be called from the capture task (Core-0). Returns true on success.
bool    espSdrSetAgc(bool on);
bool    espSdrSetGain(uint8_t gain);
bool    espSdrAgc();          // current AGC state
uint8_t espSdrGain();         // current manual gain (meaningful only when AGC off)
