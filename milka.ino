#include <HX711.h>
#include <WiFi.h>
#include "ESPTelnet.h"

// --- SETTINGS ---
const char* ssid = "NU";
const char* password = "1234512345";
const char* firmware_url = "https://github.com/Abylkanov/milka/raw/refs/heads/main/build/esp32.esp32.esp32/milka.ino.bin"; 

const String FIRMWARE_VERSION = "1.0.8"; 

const int LOADCELL_DOUT_PIN = 22;
const int LOADCELL_SCK_PIN = 21;

ESPTelnet telnet;
HX711 scale;

bool isMeasuring = false;
unsigned long previousMillis = 0;
const long interval = 1000; // Интервал отправки веса (1 секунда)
float calibration_factor = 420.0;
float raw_1 = 0, weight_1 = 0; 
float raw_2 = 0, weight_2 = 0;

void setup() {
  Serial.begin(115200);

// Инициализация весов
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor); 
  scale.tare(); // Сброс в 0 при включении

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[System] IP: " + WiFi.localIP().toString());

  // Telnet Setup
  telnet.onConnect([](String ip) {
    telnet.println("\n--- ESP32 Weight Station ---");
    telnet.println("Version: " + FIRMWARE_VERSION);
    telnet.println("Commands: 'start', 'stop', 'tare', 'calib <val>', 'update', 'status'");
    telnet.print("> ");
  });

  telnet.onInputReceived([](String str) {
    str.trim();
    
    if (str == "start") {
      isMeasuring = true;
      telnet.println("[System] Measurement started.");
    } 
    else if (str == "stop") {
      isMeasuring = false;
      telnet.println("[System] Measurement stopped.");
    } 
    else if (str == "tare") {
      scale.tare();
      telnet.println("[Scale] Zero set (Tared).");
    } 
    
    // 2. Шаг первый: узнать "сырое" значение без коэффициента
    else if (str == "calib_step1") {
      telnet.println("[Calib] Remove weight, type 'tare', then put known weight and wait...");
      long raw = scale.get_value(10); // Получаем среднее сырое значение
      telnet.printf("[Calib] RAW Value: %ld\n", raw);
      telnet.println("[Calib] Calculation: FACTOR = RAW / KNOWN_WEIGHT");
    }
    else if (str.startsWith("point1 ")) {
     weight_1 = str.substring(7).toFloat(); // Вводим реальный вес (например 0)
     raw_1 = scale.read_average(10);     // Читаем "сырое"
     telnet.printf("Point 1 saved: Weight %.2f = Raw %.0f\n", weight_1, raw_1);
}
else if (str.startsWith("point2 ")) {
  weight_2 = str.substring(7).toFloat(); // Вводим реальный вес гири (например 500)
  raw_2 = scale.read_average(10);     // Читаем "сырое"
  telnet.printf("Point 2 saved: Weight %.2f = Raw %.0f\n", weight_2, raw_2);
  
  // Вычисляем новый коэффициент
  calibration_factor = (raw_2 - raw_1) / (weight_2 - weight_1);
  scale.set_scale(calibration_factor);
  scale.set_offset(raw_1); // Устанавливаем базу
  telnet.printf("New Factor: %.2f. Calibration complete!\n", calibration_factor);
}
    // 3. Шаг второй: применить вычисленный коэффициент
    else if (str.startsWith("calib_set ")) {
      float factor = str.substring(10).toFloat();
      calibration_factor = factor;
      scale.set_scale(calibration_factor);
      telnet.printf("[Scale] Factor updated to: %.2f\n", calibration_factor);
    }
    else if (str.startsWith("calib ")) {
      // Пример: 'calib 1234.5'
      String valStr = str.substring(6);
      calibration_factor = valStr.toFloat();
      scale.set_scale(calibration_factor);
      telnet.println("[Scale] New calibration factor: " + String(calibration_factor));
    }
    else if (str == "update") {
      startOTA();
    } 
    else if (str == "status") {
      telnet.println("[Status] Version: " + FIRMWARE_VERSION);
      telnet.println("[Status] Scale Ready: " + String(scale.is_ready() ? "Yes" : "No"));
    } 
    else if (str != "") {
      telnet.println("[Unknown] " + str);
    }
    telnet.print("> ");
  });

  telnet.begin();
}

void loop() {
  telnet.loop();

  if (isMeasuring && telnet.isConnected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      if (scale.is_ready()) {
        // Читаем сразу усредненный вес
        float weight = scale.get_units(15); 
        
        // Выводим в одну строку, чтобы не спамить
        telnet.printf("\r[Weight] %.2f units (Raw: %ld)      ", weight, scale.read());
      } else {
        telnet.println("\n[Error] HX711 disconnected!");
      }
    }
  }
}