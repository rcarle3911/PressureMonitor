/**
 * This sketch is for loading into the node connected to the Gateway.
 */

#define NODEID 0 // Gateway node id is always 0

#include <RHReliableDatagram.h>
#include <SPI.h>
#include <RH_RF95.h>
#define TXPOWER 5 // TX power in dbm. (Range of 5 - 23)

#define MSG_MAX_LEN 6 // [Opcode: 1][Origin: 1][Payload: 4] 

// Opcodes
#define SEEK_GATE 100
#define GATE_LINK 101
#define PAYLOAD_MIN 0 // 0-99 Reserved for payload if needed for future
#define PAYLOAD_MAX 99

// Singleton instance of the radio driver
#define RFM95_CS 10
#define RFM95_INT 2
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, NODEID);

//#define BLYNK_PRINT Serial
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
#define W5100_CS 5
#define SDCARD_CS 4

char auth[] = "61f7084688b54ebf8e4c749b2436a0ae"; //Replace with your auth key from blynk

#define RF95_FREQ 915.0

#define DEBUG_ENABLED 0 // Set to 0 to turn off print statements.
#define SEND_WAIT_TIMEOUT 500

uint8_t buf[MSG_MAX_LEN];
uint8_t buf_len = sizeof(buf);

typedef union
{
    float num;
    uint8_t bytes[4];
} FLOAT_ARRAY;

void setup() {
  
  pinMode(W5100_CS, OUTPUT);
  pinMode(RFM95_CS, OUTPUT);
  
  Serial.begin(115200);
  delay(100);
  Ethernet.init(5);
  Serial.println(F("Gate running..."));

  selectRadio();
  radio_init();

  // Put radio to sleep while Blynk connects
  rf95.sleep();

  selectEthernet();
  blynk_init();
}

void loop() {
  selectRadio();
  if ( manager.waitAvailableTimeout(200) ) {
    uint8_t from;
    uint8_t to;
    uint8_t id;
    if ( manager.recvfromAck( buf, &buf_len, &from, &to, &id )) {

      uint8_t opcode = buf[0];

      if ( opcode == SEEK_GATE ) {

        if ( linkToGate( from ) ) {
          if (DEBUG_ENABLED) {
            Serial.println(F("Linked to gate"));
          }
        } else {
          if (DEBUG_ENABLED) {
            Serial.println(F("Failed to link"));
          }
        }

      } else if ( opcode >= PAYLOAD_MIN && opcode <= PAYLOAD_MAX ) {
        
        uint8_t b_pin = decodeBlinkPin(buf);
        float b_val = decodePayload(buf);
        rf95.setModeIdle();
        selectEthernet();
        Blynk.virtualWrite(b_pin, b_val);

      }
    }
  }
  rf95.setModeIdle();
  selectEthernet();
  Blynk.run();
}

float decodePayload(uint8_t *packet) {

  FLOAT_ARRAY pyld;
  
  pyld.bytes[0] = packet[2];
  pyld.bytes[1] = packet[3];
  pyld.bytes[2] = packet[4];
  pyld.bytes[3] = packet[5];

  if (DEBUG_ENABLED) {
    Serial.print(F("Received payload: "));Serial.println(pyld.num);Serial.flush();
  }
  return pyld.num;
}

uint8_t decodeBlinkPin( uint8_t *packet ) {
  return packet[1] + packet[0]; //Blink pin is originID plus payloadID
}

bool linkToGate(uint8_t toNode) {
  // Build response message
  buf[0] = GATE_LINK;
  buf[1] = rf95.lastRssi();
  buf[2] = 0;

  return sendDataPacket(buf, buf_len, toNode);

}

bool sendDataPacket(uint8_t *packet, uint8_t packetSize, uint8_t to) {

  if (manager.sendtoWait(packet, packetSize, to)) {

    if (DEBUG_ENABLED) {
      Serial.println(F("Sent"));
    }

    return true;
    
  } else {
    if (DEBUG_ENABLED) {
      Serial.println(F("Failed"));
    }
    return false;
  }

}

void radio_init() {
  while (!manager.init()) {
    Serial.println(F("radio fail"));
    while (1) {
      delay(100);
    }
  }
  Serial.println(F("radio OK!"));
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFreq fail"));
    while (1) {
      delay(100);
    }
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(TXPOWER, false);

  manager.setTimeout(SEND_WAIT_TIMEOUT);
  Serial.flush();
}

void blynk_init() {
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); // Deselect the SD card
  Blynk.begin(auth);
  Serial.println(F("Blynk OK"));Serial.flush();
}

void selectRadio() {
  digitalWrite(W5100_CS, HIGH);
  digitalWrite(RFM95_CS, LOW);
}

void selectEthernet() {
  digitalWrite(RFM95_CS, HIGH);
  digitalWrite(W5100_CS, LOW);
}
