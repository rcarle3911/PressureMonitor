/**
 * Generic Node sketch to load on arduino's with pressure monitoring system. Will need NODEID configured.
 */

#define NODEID 10 // Make sure this is unique and less than 128

#include <RHReliableDatagram.h>
#include <SPI.h>
#include <RH_RF95.h>
#define TXPOWER 5 // TX power in dbm. (Range of 5 - 23)

#define MSG_MAX_LEN 7 // [Opcode: 1][Origin: 1][Payload: 4][RSSI: 1]

// Opcodes
#define SEEK_GATE 100
#define GATE_LINK 101
#define PAYLOAD_MIN 0 // 0-99 Reserved for payload if needed for future
#define PAYLOAD_MAX 99

#define RF95_FREQ 915.0

#define DEBUG_ENABLED 1 // Set to 0 to turn off print statements.
#define SEND_WAIT_TIMEOUT 500 // In milliseconds
#define UPDATE_MIN_TIME 6000 // In milliseconds. 300000 = 5 minutes

// Singleton instance of the radio driver
RH_RF95 rf95;
RHReliableDatagram manager(rf95, NODEID);

uint8_t buf[MSG_MAX_LEN];
uint8_t buf_len = sizeof(buf);

unsigned long lastSentTime = 0;
uint8_t nextNode;
uint8_t hops;

typedef union
{
    float num;
    uint8_t bytes[4];
} FLOAT_ARRAY;


// 0.5V = 0 psi = 102
// 4.5V = 150 psi = 922
// 0.5 - 4.5 = 0.183 psi per 1 step
float sensor0ToFloat( uint16_t val ) {
  if (val <= 102) {
    return 0.0;
  } else if (val >= 922) {
    return 150.0;
  } else {
    return (val - 102) * 0.183;
  }
}

// A different calibration function.
// Analog read returns a value from 0 to 1024.
// This function should convert that into a decimal depending on the sensor specs.
float sensor1ToFloat( uint16_t val ) {
  if (val <= 50) {
    return 0.0;
  } else if (val >= 1015) {
    return 200.0;
  } else {
    return (val - 50) * 0.207;
  }
}

struct SensorMap {
  uint8_t pin;
  float (*toPressure)(uint16_t);
  unsigned long lastSent;
} sensors[] = {

  // The following contains the pin and associated function to run to convert the result
  // of analogRead into a floating point (decimal)
  {
    pin: A1,
    toPressure: sensor0ToFloat,
    lastSent: 0
  },
  {
    pin: A2,
    toPressure: sensor1ToFloat,
    lastSent: 0
  }

};

void setup() {
  Serial.begin(115200);
  delay(100);

  if (NODEID > 127 || NODEID < 0) {
    Serial.println(F("Invalid NODEID configured"));
    while(1) {
      delay(500);
    }
  }

  if ( sizeof(sensors) / sizeof(SensorMap) > PAYLOAD_MAX ) {
    Serial.print(F("Too many pressure pins. Must be less than "));Serial.println(PAYLOAD_MAX + 1);
    while(1) {
      delay(500);
    }
  }

  Serial.print(F("Water Pressure Node "));Serial.print(NODEID);Serial.println(F(" intializing..."));
  radio_init();
  seekGate();
  Serial.print(F("Main program starting..."));
}

void seekGate() {

  unsigned long seekTime = 10000; // Milliseconds

  bool gateLinkFound = false;
  
  int8_t rssiValues[255];// RSSI map to each possible node id
  uint8_t hopValues[255];// Hops to gate from each node

  for (byte i = 0; i < 255; i++) {
    rssiValues[i] = -128; // Lowest signal possible
    hopValues[i] = 255;   // Max hops allowed
  }  
  
  while (!gateLinkFound) {
  
    buf[0] = SEEK_GATE;
  
    manager.sendtoWait( buf, buf_len, 255); //Broadcast to all

    unsigned long startTime = millis();
    lastSentTime = millis();
    
    while (millis() - startTime < seekTime) { // Gather information for nearby nodes
      if ( manager.waitAvailableTimeout( SEND_WAIT_TIMEOUT )) {
        
        uint8_t from;
        uint8_t to;
        uint8_t id;

        if ( manager.recvfromAck( buf, &buf_len, &from, &to, &id )) {
          if(buf[0] == GATE_LINK) {
            
            rssiValues[from] = buf[1]; // Save rssi value
            hopValues[from] = buf[2]; // Save hops

            if (DEBUG_ENABLED) {
              Serial.print(F("NODE: "));Serial.println(buf[2]);
              Serial.print(F("RSSI: "));Serial.println(rssiValues[buf[2]], DEC);
              Serial.print(F("Hops: "));Serial.println(hopValues[buf[2]]);
            }            
          }
        }
      }
      if ( millis() - lastSentTime > UPDATE_MIN_TIME) {
        buf[0] = SEEK_GATE;
        manager.sendtoWait( buf, buf_len, 255); //Broadcast to all
        lastSentTime = millis();
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
      } else if (DEBUG_ENABLED) {
        Serial.println(F("Failed to link to gate. Trying again..."));
      }
    }
  }

  if (DEBUG_ENABLED) {
    Serial.println(F("Route to gate found"));
    Serial.print(F("NODE: "));Serial.println(nextNode);
    Serial.print(F("RSSI: "));Serial.println(rssiValues[nextNode]);
    Serial.print(F("Hops: "));Serial.println(hops);
    
  }
}

void loop() {

  checkIncoming(); // Waits for incoming messages and forwards them to the next node
  // Loops through all sensors and sends their data.
  for ( byte i = 0; i < sizeof(sensors)/sizeof(SensorMap); i++) {
    if (millis() - sensors[i].lastSent > UPDATE_MIN_TIME) {
      if (sendPressureData( i, sensors[i].toPressure( analogRead( sensors[i].pin ) ) )) {
        sensors[i].lastSent = millis();
        break;
      }
    }
  }
}

void checkIncoming() {

  if ( manager.available() ) {
    uint8_t from;
    uint8_t to;
    uint8_t id;
    if ( manager.recvfromAck( buf, &buf_len, &from, &to, &id )) {

      uint8_t opcode = buf[0];

      if (opcode == SEEK_GATE) {
        if ( linkToGate( from )) {
          if (DEBUG_ENABLED) {
            Serial.println(F("Successful link to gate"));
          }
        } else if (DEBUG_ENABLED) {
          Serial.println(F("Failed to link to gate"));
        }
      } else if (opcode >= PAYLOAD_MIN && opcode <= PAYLOAD_MAX) {
        attachRSSI( buf, from );
        sendDataPacket( buf, buf_len, nextNode );
      }
    }
  }
}

void attachRSSI( uint8_t *packet, uint8_t from ) {
  // Attach RSSI if sender is origin
  if (packet[1] == from) {
    packet[6] = rf95.lastRssi();
  }
}

bool linkToGate( uint8_t toNode ) {
  // Build response message
  buf[0] = GATE_LINK;
  buf[1] = rf95.lastRssi();
  buf[2] = hops;

  return sendDataPacket( buf, buf_len, toNode );
}

bool sendPressureData( uint8_t payloadID, float data ) {

  // Build packet
  buf[0] = PAYLOAD_MIN + payloadID;

  FLOAT_ARRAY pyld;

  pyld.num = data;
  buf[1] = NODEID;
  buf[2] = pyld.bytes[0];
  buf[3] = pyld.bytes[1];
  buf[4] = pyld.bytes[2];
  buf[5] = pyld.bytes[3];

  return sendDataPacket(buf, buf_len, nextNode);
}

bool sendDataPacket(uint8_t *packet, uint8_t packetSize, uint8_t to) {

  if ( manager.sendtoWait( packet, packetSize, to )) {
    if (DEBUG_ENABLED) {
      Serial.println(F("Sent"));
    }
    return true;
  } else if (DEBUG_ENABLED) {
    Serial.println(F("Failed to send"));
    return false;
  }
}

void radio_init() {
  
  while (!manager.init()) {
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

  manager.setTimeout(SEND_WAIT_TIMEOUT);
}
