/**
 * Generic Node sketch to load on arduino's with pressure monitoring system. Will need NODEID configured.
 */

#define NODEID 1 // Make sure this is unique and less than 128
 
#include <SPI.h>
#include <RH_RF95.h>
#define TXPOWER 5 // TX power in dbm. (Range of 5 - 23)

#define MSG_MAX_LEN 7 // [Opcode: 1][ToNode: 1][FromNode: 1][Payload: 4]

// Opcodes
#define SEEK_GATE 100
#define GATE_LINK 101
#define PAYLOAD 0 // 0-99 Reserved for payload if needed for future

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define PRESSURE_READ_PIN A1
#define DEBUG_ENABLED 1 // Set to 0 to turn off print statements.
#define SEND_WAIT_TIMEOUT 500 // In milliseconds
#define UPDATE_MIN_TIME 3000 // In milliseconds. 300000 = 5 minutes

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint8_t buf[MSG_MAX_LEN];
uint8_t buf_len = sizeof(buf);

unsigned long lastSentTime = 0;
uint8_t nextNode;
uint8_t hops;

typedef union
{
    float num;
    byte bytes[4];
} FLOAT_ARRAY;

void setup() {
  Serial.begin(115200);
  delay(100);
  if (NODEID > 127 || NODEID < 0) {
    Serial.println(F("Invalid NODEID configured"));
    while(1) {
      delay(500);
    }
  }
  Serial.print(F("Water Pressure Node "));Serial.print(NODEID);Serial.println(F(" running..."));
  radio_init();
  seekGate();  
}

void seekGate() {

  unsigned long seekTime = 5000; // Milliseconds

  bool gateLinkFound = false;

  while (!gateLinkFound) {
    
    bool sent = false;
  
    while (!sent) {
      buf[0] = SEEK_GATE;
      buf[1] = NODEID;
    
      rf95.send(buf, buf_len);
      if (rf95.waitPacketSent(500)) {
        sent = true;
      }
    }

    unsigned long startTime = millis();

    int8_t rssiValues[255] = {-128}; // RSSI map to each possible node id
    uint8_t hopValues[255] = {255}; // Hops to gate from each node

    while (millis() - startTime < seekTime) { // Gather information for nearby nodes
      if (rf95.waitAvailableTimeout(500)) {
        if (rf95.recv(buf, &buf_len)) {
          if(buf[0] == GATE_LINK) {
            if (sentToMe(buf)) {
              rssiValues[buf[2]] = buf[3]; // Save rssi value
              hopValues[buf[2]] = buf[4]; // Save hops

              if (DEBUG_ENABLED) {
                Serial.print(F("NODE: "));Serial.println(buf[2]);
                Serial.print(F("RSSI: "));Serial.println(rssiValues[buf[2]], DEC);
                Serial.print(F("Hops: "));Serial.println(hopValues[buf[2]]);
              }
            }
          }
        }
      }
    }

    if (DEBUG_ENABLED) {
      Serial.println(F("Seek information gathered"));
    }
    
    if (rssiValues[0] > -100) { // Send directly to gate if > -100 Rssi
      gateLinkFound = true;
      nextNode = 0;
      hops = 1;
    } else {
      uint8_t i = 0;
      uint8_t maxNode = 0;
      while ( i < 255 ) {
        if (rssiValues[maxNode] < rssiValues[i] && hopValues[i] <= hopValues[maxNode]) {          
          maxNode = i;                
        }
        i++;
      }
      if (rssiValues[maxNode] > -128) { // -128 is the lowest value. Look again.
        gateLinkFound = true;
        nextNode = maxNode;
        hops = hopValues[maxNode]++;
      } else {
        Serial.println(F("Failed to link to gate. Trying again..."));
      }
    }
  }

  if (DEBUG_ENABLED) {
    Serial.println(F("Route to gate found"));
  }
}

void loop() {

  checkIncoming(); // Waits for incoming messages and forwards them to the next node
  sendPressureData(NODEID, readPressure(PRESSURE_READ_PIN)); // Sends own pressure data
  
}

bool sentToMe (uint8_t *packet) {
  return packet[1] == NODEID;
}

void checkIncoming() {

  do {
    if (rf95.waitAvailableTimeout(UPDATE_MIN_TIME)) {
      if (rf95.recv(buf, &buf_len)) {

        uint8_t opcode = buf[0];

        if (opcode == SEEK_GATE) {
          linkToGate(buf[1]);
        } else if (opcode == PAYLOAD) {
          if (sentToMe(buf)) {
            decodePayload(buf);
          }
        }
      }
    }
  } while (millis() < lastSentTime + UPDATE_MIN_TIME);
}

void linkToGate(uint8_t toNode) {
  // Build response message
  buf[0] = GATE_LINK;
  buf[1] = toNode;
  buf[2] = NODEID;
  buf[3] = rf95.lastRssi();
  buf[4] = hops;

  rf95.send(buf, buf_len);
  if (rf95.waitPacketSent(50)) {
    Serial.println(F("Sent gate link"));
  } else {
    Serial.println(F("Failed to send"));
  }
}

void decodePayload(uint8_t *packet) {
  
  FLOAT_ARRAY pyld;
  
  pyld.bytes[0] = packet[3];
  pyld.bytes[1] = packet[4];
  pyld.bytes[2] = packet[5];
  pyld.bytes[3] = packet[6];

  if (DEBUG_ENABLED) {
    Serial.print(F("Received payload: "));Serial.println(pyld.num);
  }
  sendPressureData(packet[2], pyld.num);
}

void sendPressureData(uint8_t from, float data) {

  // Build packet
  buf[0] = PAYLOAD;
  buf[1] = nextNode;
  buf[2] = from;

  FLOAT_ARRAY pyld;

  pyld.num = data;

  buf[3] = pyld.bytes[0];
  buf[4] = pyld.bytes[1];
  buf[5] = pyld.bytes[2];
  buf[6] = pyld.bytes[3];
  
  rf95.send(buf, buf_len);
  if (rf95.waitPacketSent(SEND_WAIT_TIMEOUT)) {
    if (from == NODEID) {
      lastSentTime = millis();
    }
    if (DEBUG_ENABLED) {
      Serial.println(F("Sent"));
    }
  } else {
    Serial.println(F("Failed to send"));
  }
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
