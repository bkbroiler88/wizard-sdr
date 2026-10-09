# Signalhound CYD

Standalone 2.4 GHz spectrum watcher on the Cheap Yellow Display.

- **Classic CYD** (ESP32-WROOM-32, 4 MB flash, no PSRAM) — this project
- **S3 CYD** — later; see `specs/signalhound-s3.html` for the planned deltas

## Design docs

- `specs/signalhound-classic.html` — Classic spec (dual-core architecture, 9 screens, feature list)
- `specs/signalhound-s3.html` — S3 variant plan

## Phase 1 scope (what this skeleton proves)

- CYD TFT (ILI9341 320×240 landscape) up with the project scanner palette
- XPT2046 touch via dedicated VSPI, 3-point L calibration, NVS-persisted
  (same pattern as `cyd-bulb-controller`)
- **Dual-core shape:** Core-0 runs the RF task, Core-1 runs the UI task,
  with a 1-slot FreeRTOS queue (`xQueueOverwrite`) between them so capture
  never waits on UI and UI never blocks on capture
- Home screen with six tiles; LIVE screen renders a synthetic FFT trace
  + 36-row waterfall at ~60 Hz; other screens are "Phase 2" placeholders
  that return to Home from the back strip
- Dirty-rect redraws only (per the project no-flicker rule)

Phase 1 uses a **synthetic FFT generator** so the UI validates independently
of the ESP-SDR driver port.

## Phase 2 scope (next)

Port the ESP-SDR debug-path receiver from the ESP-IDF `esp-sdr` project into
the Core-0 RF task so synthetic frames become real ones. See
`C:\Users\bkenn\Downloads\esp-sdr-main\esp-sdr-main\main\targets\esp32\` for
the register-level RX code to port. The queue + frame struct here are
already shaped to drop that payload in.

## Build + flash

```
pio run -t erase && pio run -t upload
pio run -t monitor
```

Erase-before-flash is a project-wide rule — never upload without erasing NVS
first, or stale touch calibration from a previous build will boot in.

## Pinout (Classic CYD 2.8")

| Role | Pins |
|---|---|
| TFT (HSPI) | MOSI 13 · MISO 12 · SCLK 14 · CS 15 · DC 2 · BL 21 |
| Touch (VSPI) | SCK 25 · MOSI 32 · MISO 39 · CS 33 · IRQ 36 |
| SD (VSPI shared) | MOSI 23 · MISO 19 · SCLK 18 · CS 5 |

SD is reserved for Phase 2 capture storage; not touched by Phase 1.
