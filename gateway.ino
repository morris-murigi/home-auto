#include <ESP8266WiFi.h>
#include <SPI.h>
#include <LoRa.h>

// WiFi network details
const char* ssid = " ";
const char* password = " ";

// Blynk authentication token
#define BLYNK_TEMPLATE_ID "TMPL2eJutAnKH"
#define BLYNK_TEMPLATE_NAME "LoRa Ra 02 with ESP8266"
#define BLYNK_AUTH_TOKEN "ZHais_e9FkAFjqzTVa-gX37mCt8dYfeU"

#include <BlynkSimpleEsp8266.h>

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 4;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 5000;          // interval between sends

float temperature = 0.0;
float humidity = 0.0;
byte motionState = 0;

void setup() {
  Serial.begin(9600);                   // initialize serial
  //while (!Serial);

  Serial.println("Gateway LoRa Duplex with callback");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(868E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("LoRa init succeeded.");
}

void loop() {
  //Serial.println(1);
  if (millis() - lastSendTime > interval) {
    //Serial.println(2);
    byte message = 10;   // send a message
    //Serial.println(3);
    sendMessage(message);
    //Serial.println(4);
    Serial.println("Sending " + message);
    //Serial.println(5);
    lastSendTime = millis();            // timestamp the message
    //Serial.println(6);
    interval = random(1000) + 5000;     // 2-3 seconds
    //Serial.println(7);
    LoRa.receive();                     // go back into receive mode
  }
}

void sendMessage(byte outgoing) {
  Serial.println(31);
  LoRa.beginPacket();                   // start packet
  Serial.println(32);
  LoRa.write(destination);              // add destination address
  Serial.println(33);
  LoRa.write(localAddress);             // add sender address
  Serial.println(34);
  LoRa.write(msgCount);                 // add message ID
  Serial.println(35);
  //LoRa.write(1);
  //LoRa.write(byte(sizeof(outgoing)));        // add payload length
  Serial.println(36);
  //LoRa.write(512);
  LoRa.write(outgoing);                 // add payload
  Serial.println(37);
  LoRa.endPacket();                     // finish packet and send it
  Serial.println(38);
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  //byte incomingLength = LoRa.read();    // incoming msg length

  byte high_t = LoRa.read();        // payload of packet
  byte low_t = LoRa.read();
  byte high_h = LoRa.read();
  byte low_h = LoRa.read();
  motionState = LoRa.read();

  /*
    if (incomingLength != sizeof(incoming)) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
    }
  */

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  int t = high_t;
  t = (high_t << 8) + low_t;
  int h = high_h;
  h = (high_h << 8) + low_h;
  temperature = float(t) / 100.0;
  humidity = float(h) / 100.0;

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  //Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: ");
  Serial.println(t);
  Serial.println(h);
  Serial.println(high_t);
  Serial.println(low_t);
  Serial.println(high_h);
  Serial.println(low_h);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Motion State: ");
  Serial.println(motionState);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

  // Send To Blynk
  Blynk.virtualWrite(V0, temperature); // Virtual pin V0 for temperature
  Blynk.virtualWrite(V4, humidity);    // Virtual pin V4 for humidity
  Blynk.virtualWrite(V3, motionState); // Virtual pin V3 for motion state

}

BLYNK_WRITE(V1) {
  if (param.asInt() == 1) {
    Serial.println("Button 1 pressed");
    sendMessage(10);
  }
  else {
    sendMessage(11);
  }
}

BLYNK_WRITE(V2) {
  if (param.asInt() == 1) {
    Serial.println("Button 2 pressed");
    sendMessage(20);
  }
  else {
    sendMessage(21);
  }
}
