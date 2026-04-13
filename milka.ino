#include <HX711.h>
#include <WiFi.h>
#include "ESPTelnet.h"

// --- SETTINGS ---
const char* ssid = "NU";
const char* password = "1234512345";
const char* firmware_url = "https://github.com/Abylkanov/milka/raw/refs/heads/main/build/esp32.esp32.esp32/milka.ino.bin";

const String FIRMWARE_VERSION = "1.0.10";

const int LOADCELL_DOUT_PIN = 22;
const int LOADCELL_SCK_PIN  = 21;

ESPTelnet telnet;
HX711 scale;

bool isMeasuring = false;
unsigned long previousMillis = 0;
const long interval = 1000;

float calibration_factor = 420.0;
long  calibration_offset  = 0;

// 2-point calibration state
float raw_1 = 0, weight_1 = 0;
float raw_2 = 0, weight_2 = 0;

// ─── Diagnostics ────────────────────────────────────────────────────────────

void runDiagnostics() {
  telnet.println("\n======= HX711 DIAGNOSTICS =======");

  // 1. Presence check
  bool ready = scale.is_ready();
  telnet.println(ready ? "[OK]   HX711 responds (DOUT pulled LOW)"
                       : "[FAIL] HX711 not ready – check wiring/power");

  if (!ready) {
    telnet.println("       DOUT pin: " + String(LOADCELL_DOUT_PIN) +
                   "  SCK pin: "  + String(LOADCELL_SCK_PIN));
    telnet.println("=================================\n");
    return;
  }

  // 2. Collect 20 raw samples, count failures
  const int N = 20;
  long samples[N];
  int failures = 0;
  telnet.print("[INFO] Reading " + String(N) + " raw samples: ");
  for (int i = 0; i < N; i++) {
    if (scale.is_ready()) {
      samples[i] = scale.read();
      telnet.print(".");
    } else {
      samples[i] = 0;
      failures++;
      telnet.print("X");
    }
    delay(80);
  }
  telnet.printf(" done (%d failures)\n", failures);

  if (failures > N / 2) {
    telnet.println("[FAIL] More than half of reads failed – unstable power or broken HX711");
  }

  // 3. Statistics
  long vmin = samples[0], vmax = samples[0];
  long long sum = 0;
  for (int i = 0; i < N; i++) {
    if (samples[i] < vmin) vmin = samples[i];
    if (samples[i] > vmax) vmax = samples[i];
    sum += samples[i];
  }
  long avg   = sum / N;
  long noise = vmax - vmin;

  telnet.printf("[RAW]  min=%-12ld  max=%-12ld\n", vmin, vmax);
  telnet.printf("[RAW]  avg=%-12ld  noise(p-p)=%ld\n", avg, noise);

  // 4. Noise verdict
  if      (noise <  1000)  telnet.println("[OK]   Noise excellent  (< 1 000)");
  else if (noise < 10000)  telnet.println("[OK]   Noise acceptable (< 10 000)");
  else if (noise < 50000)  telnet.println("[WARN] Noise high       (< 50 000) – check power/shielding");
  else                     telnet.println("[FAIL] Noise excessive  (>= 50 000) – likely wiring issue");

  // 5. Saturation check with specific diagnostics
  if (avg == 0) {
    telnet.println("[WARN] avg=0 – DOUT may be floating (no load cell?)");
  } else if (avg == -8388608) {
    telnet.println("[FAIL] ADC at minimum (-8388608 = 0x800000)");
    telnet.println("       CAUSE: load cell not connected OR E+/E- swapped OR A+/A- swapped");
    telnet.println("       FIX 1: Check all 4 wires (Red=E+, Black=E-, Green=A+, White=A-)");
    telnet.println("       FIX 2: Try swapping A+ and A- on the HX711");
    telnet.println("       FIX 3: Verify 3.3V-5V stable on HX711 VCC");
    telnet.println("       TIP:   Run 'gain 32' or 'gain 64' to test other channels");
  } else if (avg == 8388607) {
    telnet.println("[FAIL] ADC at maximum (+8388607 = 0x7FFFFF)");
    telnet.println("       CAUSE: overloaded or E+/E- swapped");
  } else {
    telnet.println("[OK]   ADC not saturated");
  }

  // 6. Current config summary
  telnet.printf("[CFG]  factor=%.4f  offset=%ld\n", calibration_factor, calibration_offset);
  telnet.printf("[CFG]  DOUT=%d  SCK=%d\n", LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  telnet.println("=================================\n");
}

// ─── Noise/stability test ────────────────────────────────────────────────────

void noiseTest(int n) {
  if (n < 5) n = 5;
  if (n > 200) n = 200;

  telnet.printf("[Noise] Sampling %d readings (no scale/offset)...\n", n);
  long vmin = LONG_MAX, vmax = LONG_MIN;
  long long sum = 0;
  for (int i = 0; i < n; i++) {
    long v = scale.read();
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
    sum += v;
    delay(30);
  }
  long avg = sum / n;
  telnet.printf("[Noise] avg=%ld  min=%ld  max=%ld  peak-to-peak=%ld\n",
                avg, vmin, vmax, vmax - vmin);
}

// ─── Setup ──────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[System] IP: " + WiFi.localIP().toString());

  // ── Telnet callbacks ─────────────────────────────────────────────────────

  telnet.onConnect([](String ip) {
    telnet.println("\n--- ESP32 Weight Station v" + FIRMWARE_VERSION + " ---");
    telnet.println("MEASUREMENT : start | stop | tare");
    telnet.println("CALIBRATION : cal_tare | cal_weight <g> | calib <factor> | factor");
    telnet.println("DIAGNOSTICS : diag | raw [n] | noise [n] | gain <128|64|32> | wiring | status");
    telnet.println("SYSTEM      : update");
    telnet.print("> ");
  });

  telnet.onInputReceived([](String str) {
    str.trim();

    // ── Measurement ────────────────────────────────────────────────────────
    if (str == "start") {
      isMeasuring = true;
      telnet.println("[System] Measurement started.");

    } else if (str == "stop") {
      isMeasuring = false;
      telnet.println("[System] Measurement stopped.");

    } else if (str == "tare") {
      scale.tare();
      telnet.println("[Scale] Zero set (tared).");

    // ── Simple calibration (direct factor) ────────────────────────────────
    } else if (str.startsWith("calib ")) {
      calibration_factor = str.substring(6).toFloat();
      scale.set_scale(calibration_factor);
      telnet.printf("[Scale] Factor set to %.4f\n", calibration_factor);

    } else if (str == "factor") {
      telnet.printf("[Scale] factor=%.4f  offset=%ld\n",
                    calibration_factor, (long)scale.get_offset());

    // ── 2-point calibration ────────────────────────────────────────────────
    //   Step 1: cal_tare           – nothing on the scale → records zero point
    //   Step 2: cal_weight <grams> – known weight on scale → computes factor
    } else if (str == "cal_tare") {
      raw_1    = scale.read_average(15);
      weight_1 = 0;
      calibration_offset = (long)raw_1;
      scale.set_offset(calibration_offset);
      telnet.printf("[Calib] Zero point recorded. RAW = %.0f\n", raw_1);
      telnet.println("[Calib] Now place a known weight and run: cal_weight <grams>");

    } else if (str.startsWith("cal_weight ")) {
      weight_2 = str.substring(11).toFloat();
      if (weight_2 <= 0) {
        telnet.println("[Calib] Error: weight must be > 0");
      } else {
        raw_2 = scale.read_average(15);
        calibration_factor = (raw_2 - raw_1) / weight_2;
        scale.set_scale(calibration_factor);
        telnet.printf("[Calib] Point 2: weight=%.1fg  RAW=%.0f\n", weight_2, raw_2);
        telnet.printf("[Calib] New factor = %.4f\n", calibration_factor);
        float check = scale.get_units(10);
        telnet.printf("[Calib] Verification read: %.2f g  (expected ~%.1f)\n", check, weight_2);
        telnet.println("[Calib] Done! Run 'factor' to confirm, 'tare' if needed.");
      }

    // Legacy 2-point (point1/point2) – kept for compatibility
    } else if (str.startsWith("point1 ")) {
      weight_1 = str.substring(7).toFloat();
      raw_1    = scale.read_average(10);
      telnet.printf("[Calib] Point1: weight=%.2f  RAW=%.0f\n", weight_1, raw_1);

    } else if (str.startsWith("point2 ")) {
      weight_2 = str.substring(7).toFloat();
      raw_2    = scale.read_average(10);
      calibration_factor = (raw_2 - raw_1) / (weight_2 - weight_1);
      calibration_offset = (long)raw_1;
      scale.set_scale(calibration_factor);
      scale.set_offset(calibration_offset);
      telnet.printf("[Calib] Point2: weight=%.2f  RAW=%.0f\n", weight_2, raw_2);
      telnet.printf("[Calib] New factor=%.4f  offset=%ld\n",
                    calibration_factor, calibration_offset);

    // ── Diagnostics ────────────────────────────────────────────────────────
    // ── Gain / channel switching ───────────────────────────────────────────
    } else if (str.startsWith("gain ")) {
      int g = str.substring(5).toInt();
      if (g == 128 || g == 64 || g == 32) {
        scale.set_gain(g);
        // dummy read to apply gain
        if (scale.is_ready()) scale.read();
        delay(100);
        long v = scale.is_ready() ? scale.read() : -999999999L;
        telnet.printf("[Gain] Set to %d. First read: %ld%s\n", g, v,
                      v == -8388608 ? "  <- still saturated" :
                      v == -999999999L ? "  <- not ready" : "  <- looks alive!");
      } else {
        telnet.println("[Gain] Valid values: 128 (ch A), 64 (ch A), 32 (ch B)");
      }

    // ── Wiring guide ─────────────────────────────────────────────────────
    } else if (str == "wiring") {
      telnet.println("\n=== Load Cell Wiring Guide ===");
      telnet.println("HX711 side:  E+  E-  A+  A-  (or INA+ INA- on some boards)");
      telnet.println("Load cell:   Excitation (+/-) and Signal (+/-)");
      telnet.println("");
      telnet.println("Typical wire colors:");
      telnet.println("  Red   -> E+   (excitation positive)");
      telnet.println("  Black -> E-   (excitation negative)");
      telnet.println("  Green -> A+   (signal positive)");
      telnet.println("  White -> A-   (signal negative)");
      telnet.println("");
      telnet.println("If reading is -8388608 (minimum):");
      telnet.println("  1. Verify all 4 wires are inserted (common: white wire missing)");
      telnet.println("  2. Swap A+ and A- – if value becomes +8388607 you had it backwards");
      telnet.println("  3. Swap E+ and E- – same idea");
      telnet.println("  4. Measure VCC-GND with multimeter (need stable 3.3-5V, >= 10mA)");
      telnet.println("==============================\n");

    } else if (str == "diag") {
      runDiagnostics();

    } else if (str == "raw" || str.startsWith("raw ")) {
      int n = 5;
      if (str.length() > 4) n = str.substring(4).toInt();
      if (n < 1) n = 1; if (n > 50) n = 50;
      telnet.printf("[RAW] Averaging %d readings...\n", n);
      long raw = scale.read_average(n);
      float units = scale.get_units(n);
      telnet.printf("[RAW] raw=%ld  scaled=%.2f  factor=%.4f  offset=%ld\n",
                    raw, units, calibration_factor, (long)scale.get_offset());

    } else if (str == "noise" || str.startsWith("noise ")) {
      int n = 30;
      if (str.length() > 6) n = str.substring(6).toInt();
      noiseTest(n);

    } else if (str == "status") {
      telnet.println("[Status] Version : " + FIRMWARE_VERSION);
      telnet.println("[Status] IP      : " + WiFi.localIP().toString());
      telnet.println("[Status] HX711   : " + String(scale.is_ready() ? "OK" : "NOT READY"));
      telnet.printf( "[Status] Factor  : %.4f\n", calibration_factor);
      telnet.printf( "[Status] Offset  : %ld\n",  (long)scale.get_offset());
      telnet.println("[Status] Measure : " + String(isMeasuring ? "running" : "stopped"));

    // ── OTA ───────────────────────────────────────────────────────────────
    } else if (str == "update") {
      startOTA();

    } else if (str != "") {
      telnet.println("[Unknown] " + str);
    }

    telnet.print("> ");
  });

  telnet.begin();
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
  telnet.loop();

  if (isMeasuring && telnet.isConnected()) {
    unsigned long now = millis();
    if (now - previousMillis >= interval) {
      previousMillis = now;

      if (scale.is_ready()) {
        float weight = scale.get_units(15);
        long  raw    = scale.read();
        telnet.printf("\r[Weight] %8.2f g   (raw: %ld)      ", weight, raw);
      } else {
        telnet.println("\n[Error] HX711 not ready!");
      }
    }
  }
}
