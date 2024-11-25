#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <soc/rtc_cntl_reg.h>
#include "soc/soc.h"

WiFiClient espClient;
PubSubClient client(espClient);

const char* mqttServer = "bf9e78d9019447609bada8c1a9b76912.s1.eu.hivemq.cloud";
const char* clientID = "esp32_cam";

const char* mqttUser = "test_MQQT";
const char* mqttPassword = "123ABC456abc";
const int port_mqtt = 8883;



const char* topic_PHOTO = "takePicture";
const char* topic_PUBLISH = "sendPicture";
const char* topic_FLASH = "setFlash";
extern const int MAX_PAYLOAD;

void sendMQTT(const uint8_t *buf, uint32_t len)
{
  Serial.println("Sending picture...");
  if(len > MAX_PAYLOAD)
  {
    Serial.print("Picture too large, increase the MAX_PAYLOAD value");
  }
  else
  {
    Serial.print("Picture sent? : ");
    Serial.println(client.publish(topic_PUBLISH, buf, len, false));
  }
}

void connectMQTT() {
    if (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        unsigned long startAttemptTime = millis();

        // Attempt to connect with retries
        while (!client.connected() && millis() - startAttemptTime < 30000) { // 30s timeout
            if (client.connect(clientID, mqttUser, mqttPassword)) {
                Serial.println("Connected to MQTT broker!");
                // Resubscribe to topics after connection
                //client.subscribe(topic_PHOTO);
                //client.subscribe(topic_FLASH);
            } else {
                Serial.print("MQTT connection failed: ");
                Serial.println(client.state());
                delay(5000); // Retry every 5 seconds
            }
        }

        if (!client.connected()) {
            Serial.println("Failed to connect to MQTT within timeout.");
        }
    }
}

