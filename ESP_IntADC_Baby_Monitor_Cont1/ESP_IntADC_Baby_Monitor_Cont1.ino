// Based on ESP_MCP3201_SPI


#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include "wifi_params.h"
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
// ===============================================

// ------------------PIN Definitions-------------
#define LDRPIN A0       // Only analogue pin on ESP8266
// ------------------Light sensor-----------------
char lightString[6] = "test";
// -----------------------------------------------
#define ONE_WIRE_BUS D4

// -----------------------------------------------
#define red 15
#define blue 13
#define green 12
#define button 4
// -----------------------------------------------

WiFiUDP udp;
const int udp_recv_port = 45990; // for command&control
const int udp_target_port = 45990; // sound transfer
const IPAddress IP_target_device(10, 1, 1, 106);
const IPAddress IP_target_PC(10, 1, 1, 107);
IPAddress IP_target = IP_target_device;

uint16_t adc_buf[2][700]; // ADC data buffer, double buffered
int current_adc_buf; // which data buffer is being used for the ADC (the other is being sent)
unsigned int adc_buf_pos; // position in the ADC data buffer
int send_samples_now; // flag to signal that a buffer is ready to be sent

#define SILENCE_EMA_WEIGHT 1024
#define ENVELOPE_EMA_WEIGHT 2
int32_t silence_value = 2048; // computed as an exponential moving average of the signal
uint16_t envelope_threshold = 150; // envelope threshold to trigger data sending

uint32_t log_serial_when, send_sound_util = 0; // date until sound transmission ends after an envelope threshold has triggered sound transmission

int enable_highpass_filter = 0;

void ota_onstart(void)
{
  // Disable timer when an OTA happens
  timer1_detachInterrupt();
  timer1_disable();
}

void ota_onprogress(unsigned int sz, unsigned int total)
{
  Serial.print("OTA: "); Serial.print(sz); Serial.print("/"); Serial.print(total);
  Serial.print("="); Serial.print(100*sz/total); Serial.println("%%");
}

void ota_onerror(ota_error_t err)
{
  Serial.print("OTA ERROR:"); Serial.println((int)err);
}


void setup(void)
{ 
  Serial.begin(115200);
  Serial.println("I was built on " __DATE__ " at " __TIME__ "");

  WiFi.setOutputPower(10); // reduce power to 10dBm = 10mW
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);

  Serial.print("Connecting to wifi");
  // Wait for connection
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  Serial.println ( "" );
  Serial.print ( "Cnnectd to " );
  Serial.println ( ssid );
  Serial.print ( "IP " );
  Serial.println ( WiFi.localIP() );

  ArduinoOTA.onStart(ota_onstart);
  ArduinoOTA.onError(ota_onerror);
  ArduinoOTA.onProgress(ota_onprogress);
  ArduinoOTA.setHostname("bb-xmit");
  ArduinoOTA.begin();

  Serial.println("setup done");

  send_sound_util = millis()+60000;
  log_serial_when = millis() +1000;
  udp.begin(udp_recv_port);
}

void loop() 
{
// ------------------LDR sensor -----------------
  int light = analogRead(LDRPIN);
  itoa(light, lightString, 10);
//  Serial.print("Light ");
//  Serial.println(lightString);

  ArduinoOTA.handle();
  if (millis() < send_sound_util) {
      udp.beginPacket(IP_target, udp_target_port);
      udp.write((const uint8_t )light);
      udp.endPacket();
      send_sound_util = millis() +1000;
      if (log_serial_when <= millis()){
        log_serial_when = millis() +1000;
        Serial.print("Send UDP ");
        Serial.print(lightString);
        Serial.print(" - To ");
        Serial.println(IP_target_device);
      }
  }
  if (udp.parsePacket()) {
    // Command and control packets
    char buf[32];
    char *ptr = &buf[0];
    udp.read(&buf[0], 31);
    buf[31] = 0;
    udp.print("sending for 15 sec");
    udp.endPacket(); 
    Serial.print("receive UDP ");
    Serial.println(buf);    
  }
    delay(2);
}

