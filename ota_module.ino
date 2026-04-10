#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>


void startOTA() {
  telnet.println("\n[OTA] Connection to server...");
  
  WiFiClientSecure client;
  client.setInsecure(); 

  HTTPClient http;
  
  if (http.begin(client, firmware_url)) {
    int httpCode = http.GET();
    telnet.printf("[OTA] HTTP Check Code: %d\n", httpCode);
    http.end(); 
    
    if (httpCode == 200) {
      telnet.println("[OTA] File found. Starting download...");
      
      // We disable auto-reboot to try and send the final message
      httpUpdate.rebootOnUpdate(false);

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
    }
  } else {
    telnet.println("[OTA] Unable to connect to server.");
  }
}