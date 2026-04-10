#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

void startOTA() {
  telnet.println("\n[OTA] Connection to GitHub...");
  
  WiFiClientSecure client;
  client.setInsecure(); // Отключаем проверку SSL сертификата (нужно для GitHub)
  client.setHandshakeTimeout(30); // Таймаут для медленного интернета

  HTTPClient http;
  
  // КРИТИЧНО: Разрешаем редиректы для проверки файла (код 302)
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  if (http.begin(client, firmware_url)) {
    int httpCode = http.GET();
    telnet.printf("[OTA] HTTP Check Code: %d\n", httpCode);
    
    // После активации setFollowRedirects, при успешном переходе по 302
    // сервер в итоге вернет 200 OK.
    if (httpCode == 200) {
      telnet.println("[OTA] File found. Starting download...");
      http.end(); // Закрываем проверочное соединение, чтобы освободить память

      // Настройка процесса прошивки
      httpUpdate.rebootOnUpdate(false);
      // КРИТИЧНО: Разрешаем редиректы и для самого процесса обновления
      httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

      t_httpUpdate_return ret = httpUpdate.update(client, firmware_url);

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          telnet.printf("[OTA] Update Failed! Error (%d): %s\n", 
                        httpUpdate.getLastError(), 
                        httpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          telnet.println("[OTA] No updates available.");
          break;

        case HTTP_UPDATE_OK:
          telnet.println("[OTA] Success! Rebooting in 2 seconds...");
          delay(2000); 
          ESP.restart();
          break;
      }
    } else {
      telnet.printf("[OTA] Error: Server returned code %d\n", httpCode);
      http.end();
    }
  } else {
    telnet.println("[OTA] Unable to connect to server.");
  }
}