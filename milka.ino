#include <HX711.h>
#include <WiFi.h>
#include <Preferences.h>
#include "ESPTelnet.h"

// --- SETTINGS ---
const char* ssid     = "NU";
const char* password = "1234512345";
const char* firmware_url =
  "https://github.com/Abylkanov/milka/raw/refs/heads/main/build/esp32.esp32.esp32/milka.ino.bin";

const String FIRMWARE_VERSION = "1.1.0";

const int LOADCELL_DOUT_PIN = 22;
const int LOADCELL_SCK_PIN  = 21;

ESPTelnet   telnet;
HX711       scale;
Preferences prefs;

// ─── Настройки (хранятся в NVS) ──────────────────────────────────────────────
float    calibration_factor  = 420.0f;
long     calibration_offset  = 0;
int      num_samples         = 15;     // кол-во усредняемых семплов
bool     autoZeroEnabled     = false;
float    autoZeroThreshold   = 10.0f; // г: считается «пустые весы»
uint32_t autoZeroHoldMs      = 8000;  // мс стабильности до авто-тары

// ─── Состояние ────────────────────────────────────────────────────────────────
bool          isMeasuring    = false;
unsigned long previousMillis = 0;
const long    interval       = 1000;

// 2-точечная калибровка
float raw_1 = 0, weight_1 = 0;
float raw_2 = 0, weight_2 = 0;

// Автообнуление
unsigned long nearZeroSince = 0;
bool          wasNearZero   = false;

// Угловая коррекция (FL=0, FR=1, BL=2, BR=3)
float       cornerVal[4] = {0, 0, 0, 0};
bool        cornerSet[4] = {false, false, false, false};
const char* CORNER[4]    = {"FL", "FR", "BL", "BR"};

// ─── NVS: сохранение / загрузка ──────────────────────────────────────────────

void saveSettings() {
  prefs.begin("scale", false);
  prefs.putFloat("factor",  calibration_factor);
  prefs.putLong( "offset",  calibration_offset);
  prefs.putInt(  "samples", num_samples);
  prefs.putBool( "az_en",   autoZeroEnabled);
  prefs.putFloat("az_thr",  autoZeroThreshold);
  prefs.putUInt( "az_time", autoZeroHoldMs);
  prefs.end();
  telnet.println("[NVS] Settings saved to flash.");
}

void loadSettings() {
  prefs.begin("scale", true);
  calibration_factor  = prefs.getFloat("factor",  420.0f);
  calibration_offset  = prefs.getLong( "offset",  0);
  num_samples         = prefs.getInt(  "samples", 15);
  autoZeroEnabled     = prefs.getBool( "az_en",   false);
  autoZeroThreshold   = prefs.getFloat("az_thr",  10.0f);
  autoZeroHoldMs      = prefs.getUInt( "az_time", 8000);
  prefs.end();
  Serial.printf("[NVS] Loaded: factor=%.4f offset=%ld samples=%d\n",
                calibration_factor, calibration_offset, num_samples);
}

// ─── Медианный фильтр ────────────────────────────────────────────────────────
// Возвращает медиану из n считанных сырых значений (без scale/offset)
// Медиана отбрасывает выбросы лучше среднего.

static void insertionSort(long* arr, int n) {
  for (int i = 1; i < n; i++) {
    long key = arr[i];
    int  j   = i - 1;
    while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
    arr[j + 1] = key;
  }
}

// Усреднённое + медианно-отфильтрованное значение в граммах
float getFilteredWeight(int n) {
  if (n < 3) n = 3;
  if (n > 64) n = 64;

  long buf[64];
  int  got = 0;
  for (int i = 0; i < n; i++) {
    if (scale.is_ready()) buf[got++] = scale.read();
    delayMicroseconds(500);
  }
  if (got == 0) return 0.0f;

  insertionSort(buf, got);

  // Отбрасываем нижние и верхние 20% (медианный trimming)
  int drop = got / 5;
  int from = drop, to = got - drop;
  if (from >= to) { from = 0; to = got; }

  long long sum = 0;
  for (int i = from; i < to; i++) sum += buf[i];
  long avg = sum / (to - from);

  // Применяем scale и offset вручную (тот же расчёт что в HX711 library)
  return (float)(avg - scale.get_offset()) / calibration_factor;
}

// ─── Угловая коррекция ───────────────────────────────────────────────────────

int cornerIndex(const String& name) {
  String s = name;
  s.toUpperCase();
  for (int i = 0; i < 4; i++)
    if (s == CORNER[i]) return i;
  return -1;
}

void cornerReport() {
  int nSet = 0;
  for (int i = 0; i < 4; i++) if (cornerSet[i]) nSet++;
  if (nSet < 2) {
    telnet.println("[Corner] Нужно замерить хотя бы 2 угла.");
    telnet.println("  corner_test FL  (FR / BL / BR)  — положи гирю в угол, запусти команду");
    return;
  }

  float ref = 0;
  for (int i = 0; i < 4; i++) if (cornerSet[i] && cornerVal[i] > ref) ref = cornerVal[i];

  telnet.println("\n========= Угловая коррекция =========");
  telnet.printf("  Эталон (максимум): %.2f г\n\n", ref);
  for (int i = 0; i < 4; i++) {
    if (!cornerSet[i]) {
      telnet.printf("  %s: не замерен\n", CORNER[i]);
      continue;
    }
    float diff = cornerVal[i] - ref;
    float pct  = (ref > 0.1f) ? (diff / ref) * 100.0f : 0.0f;
    if (fabsf(pct) < 0.3f) {
      telnet.printf("  %s: %7.2f г  [OK]\n", CORNER[i], cornerVal[i]);
    } else if (diff < 0) {
      telnet.printf("  %s: %7.2f г  [МАЛО на %.1f%% = %.2f г]  -> увеличь подстроечник %s\n",
                    CORNER[i], cornerVal[i], fabsf(pct), fabsf(diff), CORNER[i]);
    } else {
      telnet.printf("  %s: %7.2f г  [МНОГО на %.1f%% = %.2f г]  -> уменьши подстроечник %s\n",
                    CORNER[i], cornerVal[i], pct, diff, CORNER[i]);
    }
  }
  telnet.println("\n  После подстройки: повтори corner_test для скорректированных углов,");
  telnet.println("  затем corner_report. Когда все [OK] — перекалибруй (cal_tare + cal_weight).");
  telnet.println("=====================================\n");
}

// ─── Диагностика HX711 ───────────────────────────────────────────────────────

void runDiagnostics() {
  telnet.println("\n======= HX711 DIAGNOSTICS =======");

  bool ready = scale.is_ready();
  telnet.println(ready ? "[OK]   HX711 отвечает (DOUT=LOW)"
                       : "[FAIL] HX711 не отвечает – проверь питание и провода");
  if (!ready) {
    telnet.printf("       DOUT=%d  SCK=%d\n", LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    telnet.println("=================================\n");
    return;
  }

  const int N = 20;
  long samples[N];
  int  failures = 0;
  telnet.printf("[INFO] Читаю %d семплов: ", N);
  for (int i = 0; i < N; i++) {
    if (scale.is_ready()) { samples[i] = scale.read(); telnet.print("."); }
    else                  { samples[i] = 0; failures++; telnet.print("X"); }
    delay(80);
  }
  telnet.printf(" готово (%d ошибок)\n", failures);
  if (failures > N / 2)
    telnet.println("[FAIL] Больше половины чтений провалились – нестабильное питание");

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

  if      (noise <  1000)  telnet.println("[OK]   Шум отличный   (< 1 000)");
  else if (noise < 10000)  telnet.println("[OK]   Шум допустимый (< 10 000)");
  else if (noise < 50000)  telnet.println("[WARN] Шум высокий    (< 50 000) – проверь питание/экран");
  else                     telnet.println("[FAIL] Шум огромный   (>= 50 000) – проблема с проводкой");

  if (avg == 0) {
    telnet.println("[WARN] avg=0 – DOUT висит в воздухе (нет датчика?)");
  } else if (avg <= -8388607) {
    telnet.println("[FAIL] ADC насыщён (минимум -8388608)");
    telnet.println("       ПРИЧИНА: датчик не подключён, или перепутаны E+/E-, или A+/A-");
    telnet.println("       FIX 1: проверь 4 провода (Red=E+, Black=E-, Green=A+, White=A-)");
    telnet.println("       FIX 2: поменяй местами A+ и A-");
    telnet.println("       FIX 3: проверь питание HX711 мультиметром (нужно 3.3-5В)");
    telnet.println("       TIP:   попробуй 'gain 64' или 'gain 32'");
  } else if (avg >= 8388607) {
    telnet.println("[FAIL] ADC насыщён (максимум +8388607) – перегруз или перепутаны E+/E-");
  } else {
    telnet.println("[OK]   ADC не насыщён");
  }

  telnet.printf("[CFG]  factor=%.4f  offset=%ld  samples=%d\n",
                calibration_factor, calibration_offset, num_samples);
  telnet.printf("[CFG]  DOUT=%d  SCK=%d\n", LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  telnet.printf("[CFG]  AutoZero=%s  thr=%.1fg  hold=%ums\n",
                autoZeroEnabled ? "ON" : "OFF", autoZeroThreshold, autoZeroHoldMs);
  telnet.println("=================================\n");
}

void noiseTest(int n) {
  if (n < 5) n = 5;
  if (n > 200) n = 200;
  telnet.printf("[Noise] Читаю %d сырых семплов...\n", n);
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
  // В граммах
  float g_min  = (float)(vmin - scale.get_offset()) / calibration_factor;
  float g_max  = (float)(vmax - scale.get_offset()) / calibration_factor;
  float g_noise = g_max - g_min;
  telnet.printf("[Noise] avg=%ld  p-p raw=%ld\n", avg, vmax - vmin);
  telnet.printf("[Noise] В граммах: min=%.3f  max=%.3f  шум=%.3f г\n", g_min, g_max, g_noise);
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  // Загружаем настройки из NVS
  loadSettings();

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_gain(128);
  scale.set_scale(calibration_factor);

  if (calibration_offset != 0) {
    // Используем сохранённый offset — не сбрасываем в 0
    scale.set_offset(calibration_offset);
    Serial.println("[Scale] Loaded saved offset, skipping tare.");
  } else {
    scale.tare();
    calibration_offset = scale.get_offset();
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[System] IP: " + WiFi.localIP().toString());

  // ── Telnet ───────────────────────────────────────────────────────────────

  telnet.onConnect([](String ip) {
    telnet.println("\n--- ESP32 Weight Station v" + FIRMWARE_VERSION + " ---");
    telnet.println("MEASUREMENT  : start | stop | tare | samples <n>");
    telnet.println("CALIBRATION  : cal_tare | cal_weight <g> | calib <factor> | factor");
    telnet.println("CORNER CORR  : corner_test <FL|FR|BL|BR> | corner_report | corner_clear");
    telnet.println("AUTO-ZERO    : autozero on|off | az_thr <g> | az_time <ms>");
    telnet.println("DIAGNOSTICS  : diag | raw [n] | noise [n] | gain <128|64|32> | wiring");
    telnet.println("SYSTEM       : save | status | update");
    telnet.print("> ");
  });

  telnet.onInputReceived([](String str) {
    str.trim();

    // ── Измерение ─────────────────────────────────────────────────────────
    if (str == "start") {
      isMeasuring = true;
      telnet.println("[System] Измерение запущено.");

    } else if (str == "stop") {
      isMeasuring = false;
      telnet.println("[System] Измерение остановлено.");

    } else if (str == "tare") {
      scale.tare();
      calibration_offset = scale.get_offset();
      telnet.println("[Scale] Обнулено (tare). Запусти 'save' чтобы сохранить.");

    } else if (str.startsWith("samples ")) {
      int n = str.substring(8).toInt();
      if (n >= 1 && n <= 64) {
        num_samples = n;
        telnet.printf("[Scale] Семплов на усреднение: %d\n", num_samples);
      } else {
        telnet.println("[Scale] Допустимо: 1..64");
      }

    // ── Калибровка ────────────────────────────────────────────────────────
    } else if (str.startsWith("calib ")) {
      calibration_factor = str.substring(6).toFloat();
      scale.set_scale(calibration_factor);
      telnet.printf("[Scale] Коэффициент установлен: %.4f\n", calibration_factor);

    } else if (str == "factor") {
      telnet.printf("[Scale] factor=%.4f  offset=%ld  samples=%d\n",
                    calibration_factor, (long)scale.get_offset(), num_samples);

    } else if (str == "cal_tare") {
      raw_1    = scale.read_average(20);
      weight_1 = 0;
      calibration_offset = (long)raw_1;
      scale.set_offset(calibration_offset);
      telnet.printf("[Calib] Нулевая точка: RAW=%.0f\n", raw_1);
      telnet.println("[Calib] Положи гирю и введи: cal_weight <граммы>");

    } else if (str.startsWith("cal_weight ")) {
      weight_2 = str.substring(11).toFloat();
      if (weight_2 <= 0) {
        telnet.println("[Calib] Ошибка: вес должен быть > 0");
      } else {
        raw_2 = scale.read_average(20);
        calibration_factor = (raw_2 - raw_1) / weight_2;
        scale.set_scale(calibration_factor);
        telnet.printf("[Calib] Точка 2: вес=%.1fг  RAW=%.0f\n", weight_2, raw_2);
        telnet.printf("[Calib] Новый коэффициент: %.4f\n", calibration_factor);
        float check = getFilteredWeight(num_samples);
        telnet.printf("[Calib] Контрольное чтение: %.2f г  (ожидалось ~%.1f г)\n", check, weight_2);
        telnet.println("[Calib] Готово! Введи 'save' чтобы сохранить в flash.");
      }

    // Legacy совместимость
    } else if (str.startsWith("point1 ")) {
      weight_1 = str.substring(7).toFloat();
      raw_1    = scale.read_average(10);
      telnet.printf("[Calib] Точка1: вес=%.2f  RAW=%.0f\n", weight_1, raw_1);

    } else if (str.startsWith("point2 ")) {
      weight_2 = str.substring(7).toFloat();
      raw_2    = scale.read_average(10);
      calibration_factor = (raw_2 - raw_1) / (weight_2 - weight_1);
      calibration_offset = (long)raw_1;
      scale.set_scale(calibration_factor);
      scale.set_offset(calibration_offset);
      telnet.printf("[Calib] factor=%.4f  offset=%ld\n", calibration_factor, calibration_offset);

    // ── Угловая коррекция ─────────────────────────────────────────────────
    } else if (str.startsWith("corner_test ")) {
      String cname = str.substring(12);
      cname.trim();
      int idx = cornerIndex(cname);
      if (idx < 0) {
        telnet.println("[Corner] Неизвестный угол. Используй: FL FR BL BR");
      } else {
        telnet.printf("[Corner] Читаю угол %s (%d семплов)...\n", CORNER[idx], num_samples);
        cornerVal[idx] = getFilteredWeight(num_samples);
        cornerSet[idx] = true;
        telnet.printf("[Corner] %s = %.2f г\n", CORNER[idx], cornerVal[idx]);
        // Показываем все замеренные углы
        for (int i = 0; i < 4; i++)
          if (cornerSet[i])
            telnet.printf("         %s=%.2f ", CORNER[i], cornerVal[i]);
        telnet.println();
      }

    } else if (str == "corner_report") {
      cornerReport();

    } else if (str == "corner_clear") {
      for (int i = 0; i < 4; i++) { cornerSet[i] = false; cornerVal[i] = 0; }
      telnet.println("[Corner] Данные угловых замеров сброшены.");

    // ── Автообнуление ─────────────────────────────────────────────────────
    } else if (str == "autozero on") {
      autoZeroEnabled = true;
      telnet.printf("[AutoZero] Включено. Порог=%.1fг  Время=%ums\n",
                    autoZeroThreshold, autoZeroHoldMs);

    } else if (str == "autozero off") {
      autoZeroEnabled = false;
      wasNearZero = false;
      telnet.println("[AutoZero] Выключено.");

    } else if (str.startsWith("az_thr ")) {
      autoZeroThreshold = str.substring(7).toFloat();
      telnet.printf("[AutoZero] Порог установлен: %.1f г\n", autoZeroThreshold);

    } else if (str.startsWith("az_time ")) {
      autoZeroHoldMs = (uint32_t)str.substring(8).toInt();
      telnet.printf("[AutoZero] Время удержания: %u мс\n", autoZeroHoldMs);

    // ── Диагностика ───────────────────────────────────────────────────────
    } else if (str.startsWith("gain ")) {
      int g = str.substring(5).toInt();
      if (g == 128 || g == 64 || g == 32) {
        scale.set_gain(g);
        if (scale.is_ready()) scale.read(); // применяем gain
        delay(150);
        long v = scale.is_ready() ? scale.read() : LONG_MIN;
        if (v == LONG_MIN)
          telnet.printf("[Gain] %d: HX711 не ответил\n", g);
        else
          telnet.printf("[Gain] %d: raw=%ld%s\n", g, v,
                        v <= -8388607 ? "  <- всё ещё насыщён (минимум)" :
                        v >= 8388607  ? "  <- насыщён (максимум)" : "  <- сигнал есть!");
      } else {
        telnet.println("[Gain] Допустимые значения: 128 (кан.A), 64 (кан.A), 32 (кан.B)");
      }

    } else if (str == "wiring") {
      telnet.println("\n=== Подключение датчиков нагрузки ===");
      telnet.println("HX711: E+  E-  A+  A-");
      telnet.println("Типичные цвета проводов:");
      telnet.println("  Красный  -> E+  (питание тензодатчика +)");
      telnet.println("  Чёрный   -> E-  (питание тензодатчика -)");
      telnet.println("  Зелёный  -> A+  (сигнал +)");
      telnet.println("  Белый    -> A-  (сигнал -)");
      telnet.println("");
      telnet.println("При чтении -8388608:");
      telnet.println("  1. Проверь все 4 провода — часто белый не до конца вставлен");
      telnet.println("  2. Поменяй местами A+ и A- на HX711");
      telnet.println("  3. Измерь VCC-GND мультиметром (нужно стабильное 3.3-5В)");
      telnet.println("=====================================\n");

    } else if (str == "diag") {
      runDiagnostics();

    } else if (str == "raw" || str.startsWith("raw ")) {
      int n = 5;
      if (str.length() > 4) n = constrain(str.substring(4).toInt(), 1, 50);
      long raw   = scale.read_average(n);
      float filt = getFilteredWeight(n);
      float plain = scale.get_units(n);
      telnet.printf("[RAW] raw=%ld  plain=%.3fg  filtered=%.3fg\n", raw, plain, filt);
      telnet.printf("      factor=%.4f  offset=%ld\n",
                    calibration_factor, (long)scale.get_offset());

    } else if (str == "noise" || str.startsWith("noise ")) {
      int n = 30;
      if (str.length() > 6) n = str.substring(6).toInt();
      noiseTest(n);

    // ── Система ───────────────────────────────────────────────────────────
    } else if (str == "save") {
      calibration_offset = scale.get_offset();
      saveSettings();

    } else if (str == "status") {
      telnet.println("[Status] Версия    : " + FIRMWARE_VERSION);
      telnet.println("[Status] IP        : " + WiFi.localIP().toString());
      telnet.println("[Status] HX711     : " + String(scale.is_ready() ? "OK" : "НЕ ГОТОВ"));
      telnet.printf( "[Status] Factor    : %.4f\n", calibration_factor);
      telnet.printf( "[Status] Offset    : %ld\n",  (long)scale.get_offset());
      telnet.printf( "[Status] Samples   : %d\n",   num_samples);
      telnet.println("[Status] Измерение : " + String(isMeasuring ? "запущено" : "остановлено"));
      telnet.printf( "[Status] AutoZero  : %s  thr=%.1fg  hold=%ums\n",
                     autoZeroEnabled ? "ON" : "OFF", autoZeroThreshold, autoZeroHoldMs);
      if (scale.is_ready()) {
        float w = getFilteredWeight(5);
        telnet.printf("[Status] Сейчас    : %.2f г\n", w);
      }

    } else if (str == "update") {
      startOTA();

    } else if (str != "") {
      telnet.println("[Unknown] " + str);
    }

    telnet.print("> ");
  });

  telnet.begin();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  telnet.loop();

  // ── Автообнуление ─────────────────────────────────────────────────────────
  // Срабатывает только когда весы НЕ в режиме измерения (не мешать показаниям)
  if (autoZeroEnabled && !isMeasuring && scale.is_ready()) {
    static unsigned long lastAZCheck = 0;
    unsigned long now = millis();
    if (now - lastAZCheck >= 500) {
      lastAZCheck = now;
      float w = getFilteredWeight(3);
      bool  nearZero = fabsf(w) < autoZeroThreshold;
      if (nearZero && !wasNearZero) {
        nearZeroSince = now;
        wasNearZero   = true;
      } else if (!nearZero) {
        wasNearZero = false;
      } else if (nearZero && (now - nearZeroSince >= autoZeroHoldMs)) {
        scale.tare();
        calibration_offset = scale.get_offset();
        nearZeroSince = now; // сброс таймера
        if (telnet.isConnected())
          telnet.println("\n[AutoZero] Авто-обнуление выполнено.");
        Serial.println("[AutoZero] Tared.");
      }
    }
  }

  // ── Вывод веса ────────────────────────────────────────────────────────────
  if (isMeasuring && telnet.isConnected()) {
    unsigned long now = millis();
    if (now - previousMillis >= interval) {
      previousMillis = now;
      if (scale.is_ready()) {
        float weight = getFilteredWeight(num_samples);
        telnet.printf("\r[Weight] %8.2f г      ", weight);
      } else {
        telnet.println("\n[Error] HX711 не готов!");
      }
    }
  }
}
