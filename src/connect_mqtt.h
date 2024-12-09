#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <soc/rtc_cntl_reg.h>
#include "soc/soc.h"
#include <WiFiClientSecure.h>



const char* mqtt_server = "4838f20abef342ccba6a129cecb3ebe8.s1.eu.hivemq.cloud";
String clientID = "esp32_cam";
const char* mqtt_username = "testaccount";
const char* mqtt_password = "Test12345";

const int mqtt_port = 8883;



const char* topic_PHOTO = "take_picture_cam/3/5";
const char* topic_PUBLISH = "send_picture_cam/3/5";
const char* topic_FLASH = "setFlash";
extern const int MAX_PAYLOAD;
// MQTT client
WiFiClientSecure espClient;
PubSubClient client(espClient);

void sendMQTT(const uint8_t * buf, uint32_t len){
  Serial.println("Sending picture...");
  if(len>MAX_PAYLOAD){
    Serial.print("Picture too large, increase the MAX_PAYLOAD value");
  }else{
    Serial.print("Picture sent ? : ");
    Serial.println(client.publish(topic_PUBLISH, buf, len, false));
  }
  
}


void connectMQTT() {
        while (!client.connected()) { 
            clientID = clientID + String(random(0xffff),HEX);
            if (client.connect(clientID.c_str(),mqtt_username, mqtt_password)) {
                Serial.println("Connected to MQTT broker!");
                // Resubscribe to topics after connection
                client.subscribe(topic_PHOTO);
                client.subscribe(topic_FLASH);
            } else {
                Serial.print("MQTT connection failed: ");
                Serial.println(client.state());
                delay(5000); // Retry every 5 seconds
            }
        }

    }


