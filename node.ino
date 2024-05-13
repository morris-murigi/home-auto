#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

// Define LoRa module pins
const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 2;         // change for your board; must be a hardware interrupt pin

//Define the parameters
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 5000;          // interval between sends

float temperature = 0.0;
float humidity = 0.0;
byte motionDetected = 0.0;

// Define relay pins
byte RelayFan = 5;
byte RelayBulb = 6;

// Define DHT sensor setup
#define DHTPIN 7            // DHT11 data pin
#define DHTTYPE DHT11       // DHT11 sensor type
DHT dht(DHTPIN, DHTTYPE);

// Define PIR Motion Sensor pin
const int pirPin = 8;       // PIR motion sensor pin

void setup() {
  Serial.begin(9600);                   // initialize serial
  //while (!Serial);

  pinMode(RelayBulb, OUTPUT);
  pinMode(RelayFan, OUTPUT);
  // Initialize DHT sensor
  dht.begin();
  // Initialize PIR motion sensor
  pinMode(pirPin, INPUT);

  Serial.println("Node LoRa Duplex with callback");

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
    //uint32_t message = ((uint16_t(temperature * 100)) << 16)  + (uint16_t(humidity * 100)) ;   // send a message
    //Serial.println(3);
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    motionDetected = digitalRead(pirPin);
    sendMessage(int(temperature * 100.0), int(humidity * 100.0), motionDetected);
    // Print sensor readings
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.print("%, Motion Detected: ");
    Serial.println(motionDetected ? "Yes" : "No");
    //Serial.println(4);
    //Serial.println("Sending :" + message);
    //Serial.println(5);
    lastSendTime = millis();            // timestamp the message
    //Serial.println(6);
    interval = random(1000) + 5000;     // 2-3 seconds
    //Serial.println(7);
    LoRa.receive();                     // go back into receive mode
  }
}

void sendMessage(int temperature, int humidity, byte motionDetected) {
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
  //temperature *= 100.0;
  //humidity *= 100.0;
  LoRa.write(highByte(temperature));                 // add payload
  LoRa.write(lowByte(temperature));
  LoRa.write(highByte(humidity));
  LoRa.write(lowByte(humidity));
  LoRa.write(motionDetected);
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
  byte incoming = LoRa.read();                 // payload of packet

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

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  //Serial.println("Message length: " + String(incomingLength));
  Serial.print("Message: ");
  Serial.println(incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();

  // Actuate Relays
  if (incoming == 10)
  {
    digitalWrite(RelayFan, LOW);
  }
  else if (incoming == 11)
  {
    digitalWrite(RelayFan, HIGH);
  }
  else if (incoming == 20)
  {
    digitalWrite(RelayBulb, LOW);
  }
  else if (incoming == 21)
  {
    digitalWrite(RelayBulb, HIGH);
  }
}
