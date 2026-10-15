// OOK / Morse keyer for classic ESP32 Signalhound.
//
// Toggles esp_phy_wifi_tx_tone() on/off from a dedicated task at ms resolution
// to produce on-off-keyed carriers. Primary use: Morse CW TX on 2.4 GHz WiFi
// channels (including ham-band overlap at ch 1-5 for the 13cm amateur
// allocation, 2300-2450 MHz).
//
// Secondary: arbitrary bit patterns (digital ASK, test signals, FHSS
// building block when combined with pll_chan).
//
// The keyer owns the radio while a queue is draining. Callers should tx_stop
// any active CW first; main.cpp CLI handlers do this automatically.

#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
    void esp_phy_wifi_tx_tone(int start, int chan, int backoff);
}

struct OokStep {
    uint16_t ms;
    uint8_t  on;    // 1 = carrier on, 0 = off
};

static constexpr int kOokQueueSize = 2048;
static OokStep        g_ookQueue[kOokQueueSize];
static volatile int   g_ookHead = 0;
static volatile int   g_ookTail = 0;
static volatile uint8_t g_ookChan = 1;
static volatile uint8_t g_ookBo   = 0;
static volatile bool  g_ookStopReq = false;
static volatile bool  g_ookActive  = false;
static TaskHandle_t   g_ookTask = nullptr;

static bool ookPush(uint16_t ms, uint8_t on) {
    int next = (g_ookHead + 1) % kOokQueueSize;
    if (next == g_ookTail) return false;        // full
    g_ookQueue[g_ookHead].ms = ms;
    g_ookQueue[g_ookHead].on = on;
    g_ookHead = next;
    return true;
}

static int ookPending() {
    int h = g_ookHead, t = g_ookTail;
    return (h >= t) ? (h - t) : (kOokQueueSize - t + h);
}

static void ookTaskBody(void*) {
    bool carrierOn = false;
    for (;;) {
        if (g_ookStopReq) {
            if (carrierOn) { esp_phy_wifi_tx_tone(0, 0, 0); carrierOn = false; }
            g_ookTail = g_ookHead;      // flush
            g_ookStopReq = false;
            g_ookActive  = false;
            Serial.println("[ook] stopped (queue flushed)");
        }
        if (g_ookTail != g_ookHead) {
            if (!g_ookActive) g_ookActive = true;
            OokStep s = g_ookQueue[g_ookTail];
            if (s.on && !carrierOn) {
                esp_phy_wifi_tx_tone(1, g_ookChan, g_ookBo);
                carrierOn = true;
            } else if (!s.on && carrierOn) {
                esp_phy_wifi_tx_tone(0, 0, 0);
                carrierOn = false;
            }
            vTaskDelay(pdMS_TO_TICKS(s.ms));
            g_ookTail = (g_ookTail + 1) % kOokQueueSize;
            // If this was the last step, make sure carrier releases.
            if (g_ookTail == g_ookHead && carrierOn) {
                esp_phy_wifi_tx_tone(0, 0, 0);
                carrierOn = false;
                g_ookActive = false;
                Serial.println("[ook] done");
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// ---- Morse table ---------------------------------------------------------
static const char* morseFor(char c) {
    c = (char)toupper((unsigned char)c);
    switch (c) {
        case 'A': return ".-";    case 'B': return "-...";  case 'C': return "-.-.";
        case 'D': return "-..";   case 'E': return ".";     case 'F': return "..-.";
        case 'G': return "--.";   case 'H': return "....";  case 'I': return "..";
        case 'J': return ".---";  case 'K': return "-.-";   case 'L': return ".-..";
        case 'M': return "--";    case 'N': return "-.";    case 'O': return "---";
        case 'P': return ".--.";  case 'Q': return "--.-";  case 'R': return ".-.";
        case 'S': return "...";   case 'T': return "-";     case 'U': return "..-";
        case 'V': return "...-";  case 'W': return ".--";   case 'X': return "-..-";
        case 'Y': return "-.--";  case 'Z': return "--..";
        case '0': return "-----"; case '1': return ".----"; case '2': return "..---";
        case '3': return "...--"; case '4': return "....-"; case '5': return ".....";
        case '6': return "-...."; case '7': return "--..."; case '8': return "---..";
        case '9': return "----.";
        case '.': return ".-.-.-"; case ',': return "--..--"; case '?': return "..--..";
        case '/': return "-..-.";  case '-': return "-....-"; case '=': return "-...-";
        case ' ': return " ";       // word break sentinel
        default:  return nullptr;
    }
}

// ---- Public entry points -------------------------------------------------
extern "C" {

void txOokInit() {
    if (g_ookTask) return;
    xTaskCreatePinnedToCore(ookTaskBody, "ook", 4096, nullptr, 1, &g_ookTask, 1);
}

void txOokStop() {
    g_ookStopReq = true;
}

bool txOokIsActive() {
    return g_ookActive || (g_ookTail != g_ookHead);
}

// Encode `text` as PARIS-timed Morse at `wpm` and queue it. Standard timing:
//   dit = 1 unit, dah = 3 units, intra-char gap = 1, inter-char gap = 3,
//   inter-word gap = 7. dit_ms = 1200 / wpm.
void txOokMorse(const char* text, int wpm, uint8_t chan, uint8_t bo) {
    if (!text) return;
    if (wpm <= 4)  wpm = 5;
    if (wpm > 60)  wpm = 60;
    int dit = 1200 / wpm;
    g_ookChan = chan ? chan : 1;
    g_ookBo   = bo;
    int queued = 0;
    for (const char* p = text; *p; p++) {
        const char* code = morseFor(*p);
        if (!code) continue;
        if (code[0] == ' ') {
            // Add 4 extra dits of silence to make inter-word gap total 7
            // (we already left 3 at the end of the previous char).
            ookPush((uint16_t)(4 * dit), 0);
            continue;
        }
        for (const char* s = code; *s; s++) {
            uint16_t sym_len = (uint16_t)((*s == '-') ? 3 * dit : dit);
            ookPush(sym_len, 1);
            ookPush((uint16_t)dit, 0);        // intra-char gap
        }
        // Convert the final intra-char gap into an inter-char gap by adding
        // 2 more dits (dit + 2*dit = 3*dit = inter-char).
        ookPush((uint16_t)(2 * dit), 0);
        queued++;
    }
    Serial.printf("[ook] queued morse '%s' chars=%d wpm=%d dit=%dms pending=%d\n",
                  text, queued, wpm, dit, ookPending());
}

// Queue a raw bit pattern. Each '0' or '1' in `bits` becomes a step of
// `bit_ms` with the corresponding carrier state. Other characters are
// skipped so you can space the string for readability.
void txOokPattern(const char* bits, int bit_ms, uint8_t chan, uint8_t bo) {
    if (!bits || bit_ms < 1) return;
    g_ookChan = chan ? chan : 1;
    g_ookBo   = bo;
    int queued = 0;
    for (const char* p = bits; *p; p++) {
        if (*p == '0' || *p == '1') {
            ookPush((uint16_t)bit_ms, (uint8_t)(*p - '0'));
            queued++;
        }
    }
    Serial.printf("[ook] queued pattern bits=%d bit_ms=%d chan=%u bo=%u pending=%d\n",
                  queued, bit_ms, g_ookChan, g_ookBo, ookPending());
}

}  // extern "C"
