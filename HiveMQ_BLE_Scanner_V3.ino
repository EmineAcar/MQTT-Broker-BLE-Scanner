#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEEddystoneURL.h>
#include <BLEEddystoneTLM.h>
#include <BLEBeacon.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "MQTT_Config.h"

#define DATA_SEND 60000
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0xFF) << 8))

int scanTime = 5; //In seconds
static BLEAddress *pMAC_Address;
BLEScan *pBLEScan;
BLEAdvertisedDevice advertisedDevice;
long last_time = 0;

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      if (advertisedDevice.haveName())
      {
        Serial.print("MAC Adress: ");
        pMAC_Address = new BLEAddress(advertisedDevice.getAddress());
        Serial.println(pMAC_Address->toString().c_str());
        Serial.print("RSSI: ");
        Serial.println(advertisedDevice.getRSSI());
        Serial.println("");
      }
     }
};

void connectToWiFi() {
  Serial.print("Connecting to..");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("Connected to the WiFi.");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Callback - ");
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.print("\n");
}

void setupMQTT() {
  wifiClient.setCACert(ca_cert);
  mqttClient.setServer(MQTT_SERVER_NAME, MQTT_PORT);
  // set the callback function
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(60);
}


void setup() {
  Serial.begin(9600);
  Serial.println("Scanning...");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value
  
  connectToWiFi();
  setupMQTT();
}

void reconnectToTheBroker() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    Serial.println("Reconnecting to MQTT Broker..");
    if (mqttClient.connect(CLIENT_ID, MQTT_USER_NAME, MQTT_PASSWORD)) {
      Serial.println("Connected.");
      // subscribe to topic
      mqttClient.subscribe("/o1/desk1/esp32-1/cm");
    }
    else {
      Serial.print("Connection failed, rc=");
      Serial.print(mqttClient.state());
    }
  }
}


void loop() {
  
  // put your main code here, to run repeatedly:
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory
  delete pMAC_Address;
  pMAC_Address = NULL;
  delay(2000);
  
  if (!mqttClient.connected()) {
    Serial.println("Reconnecting to the broker..");
    reconnectToTheBroker();
  }
  mqttClient.loop();
  long now = millis();
  if (now - last_time > DATA_SEND) {

    // Publishing data through MQTT
    //publish byte in terms of integer.
    char rssi[10];
    sprintf(rssi, "%d",advertisedDevice.getRSSI());
    Serial.println(rssi[0]);
    mqttClient.publish("/o1/m1/esp32-1/scn-dvc", rssi, 1); //1 byte.

    
    //publish 
    char MAC[10];
    sprintf(MAC, "%s", pMAC_Address->toString().c_str());
    Serial.println(MAC[0]);
    mqttClient.publish("/o1/m1/esp32-1/info", MAC);
    last_time = now;
  }

}
