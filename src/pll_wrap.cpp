// Linker --wrap intercept for libphy.a's RFChannelSel().
//
// Dig-deeper variant: can be run in three modes depending on g_pllWrapMode:
//   0 = PASSTHRU  : just call __real_RFChannelSel, no re-call (safe baseline)
//   1 = LOG_ONLY  : call real, then log what args came in; don't re-call
//                   (proves wrap infrastructure is stable)
//   2 = RECALL    : call real, then re-invoke set_channel_rfpll_freq with
//                   g_pllSubOffsetMhz as the 3rd arg (previously crashed)
//
// Classic ESP32 call graph:
//   esp_phy_wifi_tx_tone(start, chan, bo)
//     -> wifiscwout(start, chan, bo)                [rf_test.o]
//       -> RFChannelSel(chan, 0)                     <-- WE WRAP THIS
//         -> chip_v7_set_chan(chan, 0)              [phy_chip_v7_ana.o]

#include <stdint.h>
#include <Arduino.h>

extern "C" {
    int  __real_RFChannelSel(int chan, int zero);
    int  __wrap_RFChannelSel(int chan, int zero);
    int  set_channel_rfpll_freq(int chan, int mode, int sub_offset);
}

// Written by main.cpp CLI handlers.
volatile int g_pllSubOffsetMhz = 0;
volatile int g_pllWrapMode     = 1;   // default: LOG_ONLY (safe)
// If > 0: override the chan arg passed to __real_RFChannelSel with this value
// (1..14). 0 means pass the caller's chan through unmodified.
volatile int g_pllChanOverride = 0;

// Telemetry.
volatile int g_pllWrapHits        = 0;
volatile int g_pllWrapLastChan    = 0;
volatile int g_pllWrapLastApplied = 0;
volatile int g_pllWrapPreCount    = 0;
volatile int g_pllWrapPostCount   = 0;

extern "C" int __wrap_RFChannelSel(int chan, int zero) {
    g_pllWrapHits++;
    g_pllWrapLastChan = chan;

    // Optionally substitute the chan arg so __real_RFChannelSel configures
    // the hardware for a different WiFi channel than the caller requested.
    // Full signed range -64..63 (RFChannelSel sign-extends to 7 bits). We
    // treat 0 as "pass through" by convention; use pll_chan 15, 20, -1, etc.
    // to probe past the WiFi grid.
    int effChan = (g_pllChanOverride != 0) ? g_pllChanOverride : chan;

    // Always let the real one run (possibly with substituted chan).
    int ret = __real_RFChannelSel(effChan, zero);

    switch (g_pllWrapMode) {
    case 0:
        // Pure pass-through, no logging, no re-call.
        break;
    case 1:
        // LOG_ONLY. Serial.printf is called from arduino task context inside
        // tx_cw request path -- safe. If this mode crashes, --wrap itself is
        // the problem; if not, the crash is specifically from re-calling.
        Serial.printf("[wrap] RFChannelSel caller_chan=%d eff_chan=%d zero=%d ret=%d\n",
                      chan, effChan, zero, ret);
        break;
    case 2:
        // RECALL. Previously crashed with offset=-92. Try smaller offsets to
        // characterize whether crash is offset-magnitude or re-call itself.
        {
            int ov = g_pllSubOffsetMhz;
            Serial.printf("[wrap] pre-recall chan=%d ov=%d\n", chan, ov);
            g_pllWrapPreCount++;
            if (ov != 0) {
                set_channel_rfpll_freq(chan, 0, ov);
                g_pllWrapLastApplied = ov;
            }
            g_pllWrapPostCount++;
            Serial.printf("[wrap] post-recall ok applied=%d\n", g_pllWrapLastApplied);
        }
        break;
    }

    return ret;
}
