#include "wifi_function.h"

const char* hostname = "wf";
AsyncWebServer server(80);
AsyncWebServer server_ota(81);

/**
 * @brief Handle for receive data from WebSerial server
 * 
 * @param data : data to be received
 * @param len : length of data to be received
 */
void WF_Handler(uint8_t *data, size_t len){
  WebSerial.println("Data Received!");
  String Data = "";
  for(int i=0; i < len; i++){
    Data += char(data[i]);
  }
  WebSerial.println(Data);
  //If send string "10112002" in WebSerial ESP32 reset
  if(Data == "10112002")
  {
    WebSerial.println("Esp32 reset after 1 second");
    delay(1000);
    ESP.restart();
  }
  if(Data == "LED ON")
  {
    digitalWrite(LED_BUILTIN, 1);
    WebSerial.println("-------- Da Bat Den ---------");
  }
  if(Data == "LED OFF")
  {
    digitalWrite(LED_BUILTIN, 0);
    WebSerial.println("-------- Da Tat Den ---------");
  }
}

/**
 * @brief Setup
 * Connect to wifi, if can't connect last wifi -> connect  AutoConnectAP wifi station/ pass: 12345678 and config new wifi 
 * Connect to Sever for WebSerial
 * Connect to Sever for OTA 
 */
void WF_Setup(void) {



  // //WiFiManager
  // //Local intialization. Once its business is done, there is no need to keep it around

      WiFiManager wm;
    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result
    bool res;
    res = wm.autoConnect("AutoConnectAP","12345678"); // password protected ap

    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }
  //setup host name for easy connect without ip address
  //form: http://hostname.local
  if (!MDNS.begin(hostname)) { 
    while (1) {
      delay(1000);
    }
  }

  /**setup for WebSerial, port 80
  access http://hostname.local/webserial
  or access http://ip/webserial -> 192.168.0.178/webserial
  */
  WebSerial.begin(&server);
  WebSerial.msgCallback(WF_Handler);
  server.begin();

  /**setup for ota, port 81
  access //http://hostname.local:81/update
  or access http://ip:81/update -> 192.168.0.178:81/update
  */
  server_ota.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I ESP32.");
  });
  AsyncElegantOTA.begin(&server_ota); 
  server_ota.begin();
}
