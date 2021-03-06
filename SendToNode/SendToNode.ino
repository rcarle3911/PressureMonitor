/**
 * This sketch is demonstrates sending from one node to another over LoRa
 */
#include <SPI.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#define TXPOWER 5 // TX power in dbm. (Range of 5 - 23)

#ifdef ESP8266
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#else
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#include <Ethernet.h>
#define W5100_CS  10
#define SDCARD_CS 4

#endif

#define RF95_FREQ 915.0
#define NODEID 2
#define SEND_TO_NODE_ID 3
#define PRESSURE_READ_PIN A1

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t rx_buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t rx_len = sizeof(rx_buf);
unsigned long mid = 0;
StaticJsonBuffer<RH_RF95_MAX_MESSAGE_LEN> jsonBuffer;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Send-To-Node program running"));
  
  radio_init();
}

void loop() {
  JsonObject& root = jsonBuffer.createObject();
  
  float data = readPressure(PRESSURE_READ_PIN);

  root[F("to")] = SEND_TO_NODE_ID;
  root[F("fr")] = NODEID;
  root[F("pd")] = data;
  root[F("m")] = mid++;

  Serial.print("Sending ");Serial.print(data);Serial.print("  to ");Serial.print(SEND_TO_NODE_ID);Serial.print(" from ");Serial.println(NODEID);

  root.printTo((char*)rx_buf, rx_len);
  jsonBuffer.clear();
  
  rf95.send(rx_buf, rx_len);
  if (rf95.waitPacketSent(500)) {
    Serial.println("Sent");
  } else {
    Serial.println("Failed to send");
  }
  delay(2000);
}

float readPressure(uint8_t _pin) {  
  // 0.5V = 0 psi = 102
  // 4.5V = 150 psi = 922
  // 0.5 - 4.5 = 0.183 psi per 1 step
  uint16_t val = analogRead(_pin);
  if (val <= 102) {
    return 0.0;
  } else if (val >= 922) {
    return 150.0;
  } else {
    return (val - 102) * 0.183;
  }
}

void radio_init() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1) {
      delay(100);
    }
  }
  Serial.println(F("LoRa radio init OK!"));

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1) {
      delay(100);
    }
  }
  Serial.print(F("Set Freq to: ")); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(TXPOWER, false);
}
