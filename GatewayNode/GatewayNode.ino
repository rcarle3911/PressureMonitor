/**
 * This sketch is for loading into the node connected to the Gateway.
 */
#include <SPI.h>
#include <RH_RF95.h>
#define TXPOWER 5 // TX power in dbm. (Range of 5 - 23)

#define MSG_MAX_LEN 7 // [Opcode: 1][ToNode: 1][FromNode: 1][Payload: 4] 

// Opcodes
#define SEEK_GATE 100
#define GATE_LINK 101
#define PAYLOAD 0 // 0-99 Reserved for payload if needed for future

#ifdef ESP8266
#define RFM95_CS 15
#define RFM95_RST 16
#define RFM95_INT 5
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
char ssid[] = "MexiNet";
char pass[] = "PeterAnswersIsNotARealTarot3496";
#else
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
#define W5100_CS  10
#define SDCARD_CS 4

#endif

char auth[] = "61f7084688b54ebf8e4c749b2436a0ae"; //Replace with your auth key from blynk

#define RF95_FREQ 915.0
#define NODEID 0 // Gateway node id is 0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t buf[MSG_MAX_LEN];
uint8_t buf_len = sizeof(buf);

typedef union
{
    float num;
    byte bytes[4];
} FLOAT_ARRAY;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Gateway Node program running"));
  radio_init();
  blynk_init();
}

void loop() {
  if (rf95.waitAvailableTimeout(10)) {
    if (rf95.recv(buf, &buf_len)) {
      uint8_t opcode = buf[0];
      if ( opcode == SEEK_GATE ) {
        linkToGate(buf[1]);
      } else if ( opcode == PAYLOAD ) {
        if (sentToMe(buf)) {
          decodePayload(buf);
        }
      }
    }
  }
  Blynk.run();
}

void decodePayload(uint8_t *packet) {
  
  FLOAT_ARRAY pyld;
  
  pyld.bytes[0] = packet[3];
  pyld.bytes[1] = packet[4];
  pyld.bytes[2] = packet[5];
  pyld.bytes[3] = packet[6];

  Serial.print("Received payload: ");Serial.println(pyld.num);

  Blynk.virtualWrite(packet[2], pyld.num);
}

void linkToGate(uint8_t toNode) {
  // Build response message
  buf[0] = GATE_LINK;
  buf[1] = toNode;
  buf[2] = NODEID;
  buf[3] = rf95.lastRssi();
  buf[4] = 0;

  rf95.send(buf, buf_len);
  if (rf95.waitPacketSent(50)) {
    Serial.println(F("Sent gate link"));
  } else {
    Serial.println(F("Failed to send"));
  }
}

bool sentToMe (uint8_t *packet) {
  return packet[1] == NODEID;
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

void blynk_init() {
#if defined(ESP8266)
  Blynk.begin(auth, ssid, pass);
#else
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); // Deselect the SD card
  Blynk.begin(auth);  
#endif
}
