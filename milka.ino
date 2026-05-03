#include <HX711.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ESPTelnet.h"

// --- SETTINGS ---
const char* ssid     = "NU";
const char* password = "1234512345";
const char* firmware_url =
  "https://github.com/Abylkanov/milka/raw/refs/heads/main/build/esp32.esp32.esp32/milka.ino.bin";

const String FIRMWARE_VERSION = "1.1.2";
const int WIFI_LED = 2;

const int LOADCELL_DOUT_PIN = 34;
const int LOADCELL_SCK_PIN  = 23;

// ─── Дисплей SSD1306 ─────────────────────────────────────────────────────────
#define OLED_SDA   14
#define OLED_SCL   4
#define OLED_ADDR  0x3C
#define OLED_W     128
#define OLED_H     64

ESPTelnet        telnet;
HX711            scale;
Preferences      prefs;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

// ─── Настройки (NVS) ─────────────────────────────────────────────────────────
float    calibration_factor = 420.0f;
long     calibration_offset = 0;
int      num_samples        = 15;
bool     autoZeroEnabled    = false;
float    autoZeroThreshold  = 10.0f;
uint32_t autoZeroHoldMs     = 8000;

// ─── Состояние ────────────────────────────────────────────────────────────────
bool          isMeasuring    = false;
unsigned long previousMillis = 0;
const long    interval       = 1000;

float raw_1 = 0, weight_1 = 0;
float raw_2 = 0, weight_2 = 0;

unsigned long nearZeroSince = 0;
bool          wasNearZero   = false;

float       cornerVal[4] = {0, 0, 0, 0};
bool        cornerSet[4] = {false, false, false, false};
const char* CORNER[4]    = {"FL", "FR", "BL", "BR"};

// ─── FreeRTOS / защита HX711 ─────────────────────────────────────────────────
//
//  Core 0: WiFi-стек ESP-IDF (системные задачи, прерывания TCP/IP)
//  Core 1: loopTask (Telnet, AutoZero) + hx711Task (чтение HX711)
//
//  WiFi-прерывания живут на Core 0 → они физически не могут разрушить
//  bit-bang на Core 1. portENTER_CRITICAL дополнительно блокирует
//  любые оставшиеся Core-1 прерывания (таймеры и т.п.) на ~100–200 мкс
//  — ровно на время клокирования 24 бит.
//
//  scaleMutex (FreeRTOS mutex) защищает кольцевой буфер rawBuf[]
//  от одновременного доступа hx711Task и loopTask.

#define RAW_BUF_SIZE 64

portMUX_TYPE      scaleMux        = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t scaleMutex      = NULL;
TaskHandle_t      hx711TaskHandle = NULL;

volatile long rawBuf[RAW_BUF_SIZE];
volatile int  rawHead  = 0;   // индекс следующей записи
volatile int  rawCount = 0;   // сколько семплов накоплено

// ─── Примитивы защищённого чтения ────────────────────────────────────────────

// Ждём готовности ВНЕ критической секции, потом читаем ВНУТРИ.
// Так portENTER_CRITICAL держится только ~100-200 мкс (время клокирования),
// а не всё время ожидания конверсии (100 мс при 10 Гц).
long safeRead() {
  while (!scale.is_ready()) vTaskDelay(pdMS_TO_TICKS(1));
  portENTER_CRITICAL(&scaleMux);
  long v = scale.read();   // wait_ready() внутри вернётся мгновенно
  portEXIT_CRITICAL(&scaleMux);
  return v;
}

// Управление задачей HX711 (для калибровки)
void hx711Pause()  { if (hx711TaskHandle) vTaskSuspend(hx711TaskHandle); }
void hx711Resume() { if (hx711TaskHandle) vTaskResume(hx711TaskHandle);  }

// Среднее из n семплов для калибровки (приостанавливает hx711Task)
long safeReadAvg(int n) {
  hx711Pause();
  long long sum = 0;
  for (int i = 0; i < n; i++) sum += safeRead();
  hx711Resume();
  return (long)(sum / n);
}

// Тара: вычисляем offset вручную через safeRead (не вызываем scale.tare())
void safeTare(int n = 15) {
  hx711Pause();
  long long sum = 0;
  for (int i = 0; i < n; i++) sum += safeRead();
  scale.set_offset((long)(sum / n));
  hx711Resume();
}

// ─── Задача HX711 (Core 1) ────────────────────────────────────────────────────
// Непрерывно заполняет кольцевой буфер. Главный поток читает из него
// без блокировки — не нужно ждать HX711 в loop().
void hx711Task(void* pv) {
  for (;;) {
    if (scale.is_ready()) {
      long v = safeRead();
      if (xSemaphoreTake(scaleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rawBuf[rawHead] = v;
        rawHead  = (rawHead + 1) % RAW_BUF_SIZE;
        if (rawCount < RAW_BUF_SIZE) rawCount++;
        xSemaphoreGive(scaleMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Гц опрос; HX711 выдаёт 10/80 Гц
  }
}

// ─── Дисплей ─────────────────────────────────────────────────────────────────
//
//  Макет 128×64:
//  ┌────────────────────────────┐
//  │ IP: 10.101.x.x             │  строка 0, шрифт 1×
//  │                            │
//  │   1234.56 г                │  строка 2–5, шрифт 2×
//  │                            │
//  │ [MEASURING] / [IDLE]       │  строка 7, шрифт 1×
//  └────────────────────────────┘

void updateDisplay(float weight) {
  display.clearDisplay();

  // Строка 1: IP-адрес
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("IP: ");
  display.print(WiFi.localIP().toString());

  // Разделитель
  display.drawLine(0, 10, OLED_W - 1, 10, SSD1306_WHITE);

  // Центральный блок: вес крупным шрифтом
  display.setTextSize(2);
  char wbuf[16];
  if (fabsf(weight) < 10000.0f)
    snprintf(wbuf, sizeof(wbuf), "%.2f g", weight);
  else
    snprintf(wbuf, sizeof(wbuf), "%.0f g", weight);

  // Вычисляем ширину для центрирования (каждый символ 2× = 12 пикс)
  int16_t x1, y1; uint16_t tw, th;
  display.getTextBounds(wbuf, 0, 0, &x1, &y1, &tw, &th);
  int cx = (OLED_W - (int)tw) / 2;
  if (cx < 0) cx = 0;
  display.setCursor(cx, 20);
  display.print(wbuf);

  // Строка внизу: статус
  display.setTextSize(1);
  display.setCursor(0, 56);
  if (isMeasuring)
    display.print("[ MEASURING ]");
  else if (autoZeroEnabled && fabsf(weight) < autoZeroThreshold)
    display.print("[ AUTOZERO... ]");
  else
    display.print("[ IDLE ]");

  display.display();
}

// ─── NVS ─────────────────────────────────────────────────────────────────────

void saveSettings() {
  prefs.begin("scale", false);
  prefs.putFloat("factor",  calibration_factor);
  prefs.putLong( "offset",  calibration_offset);
  prefs.putInt(  "samples", num_samples);
  prefs.putBool( "az_en",   autoZeroEnabled);
  prefs.putFloat("az_thr",  autoZeroThreshold);
  prefs.putUInt( "az_time", autoZeroHoldMs);
  prefs.end();
  telnet.println("[NVS] Сохранено в flash.");
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
  Serial.printf("[NVS] factor=%.4f offset=%ld samples=%d\n",
                calibration_factor, calibration_offset, num_samples);
}

// ─── Медианный фильтр (читает из rawBuf, не из HX711) ────────────────────────

static void insertionSort(long* arr, int n) {
  for (int i = 1; i < n; i++) {
    long key = arr[i]; int j = i - 1;
    while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
    arr[j + 1] = key;
  }
}

// Берёт последние n семплов из буфера, применяет trimmed mean (±20%).
// Не вызывает scale.read() — не блокирует основной поток.
float getFilteredWeight(int n) {
  if (n < 3) n = 3;
  if (n > RAW_BUF_SIZE) n = RAW_BUF_SIZE;

  long snap[RAW_BUF_SIZE];
  int  snapN;

  xSemaphoreTake(scaleMutex, portMAX_DELAY);
  int avail = (rawCount < n) ? rawCount : n;
  for (int i = 0; i < avail; i++)
    snap[i] = rawBuf[(rawHead - 1 - i + RAW_BUF_SIZE) % RAW_BUF_SIZE];
  snapN = avail;
  xSemaphoreGive(scaleMutex);

  if (snapN == 0) return 0.0f;

  insertionSort(snap, snapN);

  int drop = snapN / 5;
  int from = drop, to = snapN - drop;
  if (from >= to) { from = 0; to = snapN; }

  long long sum = 0;
  for (int i = from; i < to; i++) sum += snap[i];
  long avg = (long)(sum / (to - from));

  return (float)(avg - scale.get_offset()) / calibration_factor;
}

// ─── Угловая коррекция ───────────────────────────────────────────────────────

int cornerIndex(const String& name) {
  String s = name; s.toUpperCase();
  for (int i = 0; i < 4; i++) if (s == CORNER[i]) return i;
  return -1;
}

void cornerReport() {
  int nSet = 0;
  for (int i = 0; i < 4; i++) if (cornerSet[i]) nSet++;
  if (nSet < 2) {
    telnet.println("[Corner] Нужно замерить хотя бы 2 угла.");
    telnet.println("  corner_test FL  (FR / BL / BR)");
    return;
  }
  float ref = 0;
  for (int i = 0; i < 4; i++) if (cornerSet[i] && cornerVal[i] > ref) ref = cornerVal[i];

  telnet.println("\n========= Угловая коррекция =========");
  telnet.printf("  Эталон (максимум): %.2f г\n\n", ref);
  for (int i = 0; i < 4; i++) {
    if (!cornerSet[i]) { telnet.printf("  %s: не замерен\n", CORNER[i]); continue; }
    float diff = cornerVal[i] - ref;
    float pct  = (ref > 0.1f) ? (diff / ref) * 100.0f : 0.0f;
    if      (fabsf(pct) < 0.3f) telnet.printf("  %s: %7.2f г  [OK]\n", CORNER[i], cornerVal[i]);
    else if (diff < 0)           telnet.printf("  %s: %7.2f г  [МАЛО  %.1f%% = %.2f г]  -> увеличь подстроечник %s\n",
                                               CORNER[i], cornerVal[i], fabsf(pct), fabsf(diff), CORNER[i]);
    else                         telnet.printf("  %s: %7.2f г  [МНОГО %.1f%% = %.2f г]  -> уменьши подстроечник %s\n",
                                               CORNER[i], cornerVal[i], pct, diff, CORNER[i]);
  }
  telnet.println("\n  Когда все [OK] — перекалибруй: cal_tare → cal_weight.");
  telnet.println("=====================================\n");
}

// ─── Диагностика HX711 ───────────────────────────────────────────────────────

void runDiagnostics() {
  telnet.println("\n======= HX711 DIAGNOSTICS =======");

  // Сначала останавливаем задачу — иначе она уже прочитала данные,
  // DOUT поднялся HIGH, и быстрая проверка is_ready() всегда даёт false.
  hx711Pause();

  // Ждём следующую конверсию HX711 (до 200 мс при 10 Гц)
  bool ready = false;
  for (int i = 0; i < 40; i++) {
    if (scale.is_ready()) { ready = true; break; }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  telnet.println(ready ? "[OK]   HX711 отвечает (DOUT=LOW)"
                       : "[FAIL] HX711 не отвечает – проверь питание и провода");
  if (!ready) {
    telnet.printf("       DOUT=%d  SCK=%d\n", LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    telnet.println("=================================\n");
    hx711Resume();
    return;
  }

  const int N = 20;
  long samples[N];
  int  failures = 0;
  telnet.printf("[INFO] Читаю %d семплов: ", N);
  for (int i = 0; i < N; i++) {
    if (scale.is_ready()) { samples[i] = safeRead(); telnet.print("."); }
    else                  { samples[i] = 0; failures++; telnet.print("X"); }
    vTaskDelay(pdMS_TO_TICKS(120));
  }
  hx711Resume();
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
  long avg   = (long)(sum / N);
  long noise = vmax - vmin;

  telnet.printf("[RAW]  min=%-12ld  max=%-12ld\n", vmin, vmax);
  telnet.printf("[RAW]  avg=%-12ld  noise(p-p)=%ld\n", avg, noise);

  if      (noise <  1000)  telnet.println("[OK]   Шум отличный   (< 1 000)");
  else if (noise < 10000)  telnet.println("[OK]   Шум допустимый (< 10 000)");
  else if (noise < 50000)  telnet.println("[WARN] Шум высокий    (< 50 000) – проверь питание/экран");
  else                     telnet.println("[FAIL] Шум огромный   (>= 50 000) – проблема с проводкой");

  if      (avg == 0)         telnet.println("[WARN] avg=0 – DOUT висит в воздухе (нет датчика?)");
  else if (avg <= -8388607)  {
    telnet.println("[FAIL] ADC насыщён (минимум -8388608)");
    telnet.println("       ПРИЧИНА: нет датчика, перепутаны E+/E- или A+/A-");
    telnet.println("       FIX 1: проверь 4 провода (Red=E+, Black=E-, Green=A+, White=A-)");
    telnet.println("       FIX 2: поменяй местами A+ и A-");
    telnet.println("       FIX 3: проверь питание HX711 (нужно 3.3-5В)");
    telnet.println("       TIP:   попробуй 'gain 64' или 'gain 32'");
  } else if (avg >= 8388607) telnet.println("[FAIL] ADC насыщён (максимум +8388607) – перегруз или E+/E- перепутаны");
  else                       telnet.println("[OK]   ADC не насыщён");

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
  telnet.printf("[Noise] Читаю %d сырых семплов (hx711Task приостановлена)...\n", n);
  long vmin = LONG_MAX, vmax = LONG_MIN;
  long long sum = 0;
  hx711Pause();
  for (int i = 0; i < n; i++) {
    long v = safeRead();
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
    sum += v;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  hx711Resume();
  long avg = (long)(sum / n);
  float g_noise = (float)(vmax - vmin) / fabsf(calibration_factor);
  telnet.printf("[Noise] avg=%ld  p-p raw=%ld  шум=%.3f г\n", avg, vmax - vmin, g_noise);
}

void broadcastIpTask(void *pvParameters) {
    WiFiUDP udp;
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            udp.beginPacket("255.255.255.255", 4210);
            udp.printf("MILKA_STATION:%s", WiFi.localIP().toString().c_str());
            udp.endPacket();
        }
        // Спим 30 секунд, не занимая процессор
        vTaskDelay(pdMS_TO_TICKS(30000)); 
    }
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  loadSettings();

  // Создаём mutex до первого использования
  scaleMutex = xSemaphoreCreateMutex();

  // I2C на нестандартных пинах (SDA=4, SCL=14)
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[Display] SSD1306 not found — check wiring (SDA=4, SCL=14, addr=0x3C)");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("  Weight Station");
    display.setCursor(0, 36);
    display.println("  Connecting WiFi...");
    display.display();
  }

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_gain(128);
  scale.set_scale(calibration_factor);

  if (calibration_offset != 0) {
    scale.set_offset(calibration_offset);
    Serial.println("[Scale] Offset загружен из NVS, tare пропущена.");
  } else {
    safeTare(15);
    calibration_offset = scale.get_offset();
  }
  pinMode(WIFI_LED, OUTPUT);

  WiFi.disconnect(true); // Сброс старых сессий и настроек
    delay(200);
    WiFi.mode(WIFI_STA);   // Явная установка режима клиента
    WiFi.begin(ssid, password);
    
    Serial.print("[WiFi] Connecting");
    int attempts = 0;
    // Ждем подключения не более 10 секунд (20 попыток по 500мс)
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500); 
      Serial.print(".");
      attempts++;
      
      // Если на 10-й попытке всё еще нет связи, пробуем "пнуть" инициализацию еще раз
      if (attempts == 10) {
        WiFi.begin(ssid, password);
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[System] IP: " + WiFi.localIP().toString());
      xTaskCreatePinnedToCore(broadcastIpTask, "IpTask", 2048, NULL, 1, NULL, 0);
      digitalWrite(WIFI_LED, HIGH);
    } else {
      Serial.println("\n[WiFi] Не удалось подключиться сразу. Попытки продолжатся в фоне.");
    }
  updateDisplay(0.0f);  // покажем IP сразу после подключения

  // Запускаем задачу HX711 на Core 1 (WiFi-стек на Core 0)
  xTaskCreatePinnedToCore(
    hx711Task,        // функция
    "hx711Task",      // имя
    2048,             // стек (байт)
    NULL,             // параметр
    2,                // приоритет (выше loop=1, ниже WiFi=19)
    &hx711TaskHandle, // handle
    1                 // Core 1
  );

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
      safeTare(15);
      calibration_offset = scale.get_offset();
      telnet.println("[Scale] Обнулено. Введи 'save' чтобы сохранить.");

    } else if (str.startsWith("samples ")) {
      int n = str.substring(8).toInt();
      if (n >= 1 && n <= 64) { num_samples = n; telnet.printf("[Scale] Семплов: %d\n", n); }
      else                     telnet.println("[Scale] Допустимо: 1..64");

    // ── Калибровка ────────────────────────────────────────────────────────
    } else if (str.startsWith("calib ")) {
      calibration_factor = str.substring(6).toFloat();
      scale.set_scale(calibration_factor);
      telnet.printf("[Scale] Коэффициент: %.4f\n", calibration_factor);

    } else if (str == "factor") {
      telnet.printf("[Scale] factor=%.4f  offset=%ld  samples=%d\n",
                    calibration_factor, (long)scale.get_offset(), num_samples);

    } else if (str == "cal_tare") {
      raw_1 = (float)safeReadAvg(20);
      calibration_offset = (long)raw_1;
      scale.set_offset(calibration_offset);
      telnet.printf("[Calib] Нулевая точка RAW=%.0f. Положи гирю → cal_weight <г>\n", raw_1);

    } else if (str.startsWith("cal_weight ")) {
      weight_2 = str.substring(11).toFloat();
      if (weight_2 <= 0) {
        telnet.println("[Calib] Ошибка: вес должен быть > 0");
      } else {
        raw_2 = (float)safeReadAvg(20);
        calibration_factor = (raw_2 - raw_1) / weight_2;
        scale.set_scale(calibration_factor);
        telnet.printf("[Calib] RAW=%.0f  Новый factor=%.4f\n", raw_2, calibration_factor);
        float check = getFilteredWeight(num_samples);
        telnet.printf("[Calib] Контроль: %.2f г  (ожидалось ~%.1f г)\n", check, weight_2);
        telnet.println("[Calib] Готово! Введи 'save'.");
      }

    // Legacy
    } else if (str.startsWith("point1 ")) {
      weight_1 = str.substring(7).toFloat();
      raw_1    = (float)safeReadAvg(10);
      telnet.printf("[Calib] Точка1: вес=%.2f  RAW=%.0f\n", weight_1, raw_1);

    } else if (str.startsWith("point2 ")) {
      weight_2 = str.substring(7).toFloat();
      raw_2    = (float)safeReadAvg(10);
      calibration_factor = (raw_2 - raw_1) / (weight_2 - weight_1);
      calibration_offset = (long)raw_1;
      scale.set_scale(calibration_factor);
      scale.set_offset(calibration_offset);
      telnet.printf("[Calib] factor=%.4f  offset=%ld\n", calibration_factor, calibration_offset);

    // ── Угловая коррекция ─────────────────────────────────────────────────
    } else if (str.startsWith("corner_test ")) {
      String cname = str.substring(12); cname.trim();
      int idx = cornerIndex(cname);
      if (idx < 0) {
        telnet.println("[Corner] Используй: FL FR BL BR");
      } else {
        telnet.printf("[Corner] Читаю %s (%d семплов)...\n", CORNER[idx], num_samples);
        cornerVal[idx] = getFilteredWeight(num_samples);
        cornerSet[idx] = true;
        telnet.printf("[Corner] %s = %.2f г\n", CORNER[idx], cornerVal[idx]);
        for (int i = 0; i < 4; i++)
          if (cornerSet[i]) telnet.printf("         %s=%.2f ", CORNER[i], cornerVal[i]);
        telnet.println();
      }

    } else if (str == "corner_report") {
      cornerReport();

    } else if (str == "corner_clear") {
      for (int i = 0; i < 4; i++) { cornerSet[i] = false; cornerVal[i] = 0; }
      telnet.println("[Corner] Данные сброшены.");

    // ── Автообнуление ─────────────────────────────────────────────────────
    } else if (str == "autozero on") {
      autoZeroEnabled = true;
      telnet.printf("[AutoZero] Включено. Порог=%.1fг  Время=%ums\n",
                    autoZeroThreshold, autoZeroHoldMs);

    } else if (str == "autozero off") {
      autoZeroEnabled = false; wasNearZero = false;
      telnet.println("[AutoZero] Выключено.");

    } else if (str.startsWith("az_thr ")) {
      autoZeroThreshold = str.substring(7).toFloat();
      telnet.printf("[AutoZero] Порог: %.1f г\n", autoZeroThreshold);

    } else if (str.startsWith("az_time ")) {
      autoZeroHoldMs = (uint32_t)str.substring(8).toInt();
      telnet.printf("[AutoZero] Время: %u мс\n", autoZeroHoldMs);

    // ── Диагностика ───────────────────────────────────────────────────────
    } else if (str.startsWith("gain ")) {
      int g = str.substring(5).toInt();
      if (g == 128 || g == 64 || g == 32) {
        hx711Pause();
        scale.set_gain(g);
        if (scale.is_ready()) safeRead(); // применяем gain
        vTaskDelay(pdMS_TO_TICKS(150));
        long v = scale.is_ready() ? safeRead() : LONG_MIN;
        hx711Resume();
        if (v == LONG_MIN)
          telnet.printf("[Gain] %d: HX711 не ответил\n", g);
        else
          telnet.printf("[Gain] %d: raw=%ld%s\n", g, v,
                        v <= -8388607 ? "  <- насыщён (мин)" :
                        v >= 8388607  ? "  <- насыщён (макс)" : "  <- сигнал есть!");
      } else {
        telnet.println("[Gain] Допустимо: 128 / 64 / 32");
      }

    } else if (str == "wiring") {
      telnet.println("\n=== Подключение тензодатчиков ===");
      telnet.println("HX711: E+  E-  A+  A-");
      telnet.println("  Красный  -> E+"); telnet.println("  Чёрный   -> E-");
      telnet.println("  Зелёный  -> A+"); telnet.println("  Белый    -> A-");
      telnet.println("\nПри -8388608:");
      telnet.println("  1. Проверь все 4 провода (белый часто не вставлен до конца)");
      telnet.println("  2. Поменяй A+ и A-");
      telnet.println("  3. Измерь VCC-GND мультиметром (нужно 3.3-5В стабильных)");
      telnet.println("=================================\n");

    } else if (str == "diag") {
      runDiagnostics();

    } else if (str == "raw" || str.startsWith("raw ")) {
      int n = 5;
      if (str.length() > 4) n = constrain(str.substring(4).toInt(), 1, 50);
      long raw_avg = safeReadAvg(n);
      float filt   = getFilteredWeight(num_samples);
      telnet.printf("[RAW] avg_raw=%ld  filtered=%.3fg  factor=%.4f  offset=%ld\n",
                    raw_avg, filt, calibration_factor, (long)scale.get_offset());

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
      // Ждём готовности под паузой задачи, чтобы не поймать HIGH после её чтения
      hx711Pause();
      bool hxOk = false;
      for (int i = 0; i < 40 && !hxOk; i++) { hxOk = scale.is_ready(); if (!hxOk) vTaskDelay(pdMS_TO_TICKS(5)); }
      hx711Resume();
      telnet.println("[Status] HX711     : " + String(hxOk ? "OK" : "НЕ ГОТОВ"));
      telnet.printf( "[Status] Factor    : %.4f\n", calibration_factor);
      telnet.printf( "[Status] Offset    : %ld\n",  (long)scale.get_offset());
      telnet.printf( "[Status] Samples   : %d\n",   num_samples);
      telnet.println("[Status] Измерение : " + String(isMeasuring ? "запущено" : "остановлено"));
      telnet.printf( "[Status] AutoZero  : %s  thr=%.1fg  hold=%ums\n",
                     autoZeroEnabled ? "ON" : "OFF", autoZeroThreshold, autoZeroHoldMs);
      {
        xSemaphoreTake(scaleMutex, portMAX_DELAY);
        int cnt = rawCount;
        xSemaphoreGive(scaleMutex);
        telnet.printf("[Status] Буфер HX711: %d / %d семплов\n", cnt, RAW_BUF_SIZE);
      }
      float w = getFilteredWeight(num_samples);
      telnet.printf("[Status] Сейчас    : %.2f г\n", w);

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

  // ── Автообнуление ─────────────────────────────────────────────────────────
  if (autoZeroEnabled && !isMeasuring) {
    static unsigned long lastAZCheck = 0;
    unsigned long now = millis();
    if (now - lastAZCheck >= 500) {
      lastAZCheck = now;
      // Читаем из буфера — не блокируем поток
      float w = getFilteredWeight(5);
      bool  nearZero = fabsf(w) < autoZeroThreshold;
      if      (nearZero && !wasNearZero)                          { nearZeroSince = now; wasNearZero = true; }
      else if (!nearZero)                                          { wasNearZero = false; }
      else if (nearZero && (now - nearZeroSince >= autoZeroHoldMs)) {
        safeTare(15);
        calibration_offset = scale.get_offset();
        nearZeroSince = now;
        if (telnet.isConnected()) telnet.println("\n[AutoZero] Авто-обнуление выполнено.");
        Serial.println("[AutoZero] Tared.");
      }
    }
  }

  // ── Вывод веса (Telnet + дисплей) ────────────────────────────────────────
  {
    static unsigned long lastDisplay = 0;
    unsigned long now = millis();
    if (now - lastDisplay >= 500) {          // дисплей обновляем каждые 500 мс
      lastDisplay = now;
      float weight = getFilteredWeight(num_samples);
      updateDisplay(weight);

      if (isMeasuring && telnet.isConnected()) {
        if (now - previousMillis >= interval) {
          previousMillis = now;
          telnet.printf("\r[Weight] %8.2f г      ", weight);
        }
      }
    }
  }
}
