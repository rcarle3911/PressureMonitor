/**
 * This sketch is for loading into the node connected to the Gateway.
 */

#define NODEID 0 // Gateway node id is always 0

#include <RHReliableDatagram.h>
#include <SPI.h>
#include <RH_RF95.h>
#define TXPOWER 5 // TX power in dbm. (Range of 5 - 23)

#define MSG_MAX_LEN 11 // [Opcode: 1][Origin: 1][Payload: 4][RSSI: 1][VBATT: 4]

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

const char auth[] = "61f7084688b54ebf8e4c749b2436a0ae"; //Replace with your auth key from blynk

unsigned long lastSent = 0;

#define RF95_FREQ 915.0

#define DEBUG_ENABLED 0 // Set to 0 to turn off print statements.
#define SEND_WAIT_TIMEOUT 500
#define UPDATE_MIN_TIME 20000 // In milliseconds
#define NOTIFY_TIME 360 // 360 seconds = 6 minutes
#define ICON_OFF_TIME 120 // 120 seconds = 2 minutes

#include <EthernetUdp.h>
#include <NTPClient.h>

EthernetUDP Udp;
NTPClient timeClient(Udp, "time-a-g.nist.gov", 0, 60000);

typedef union
{
    float num;
    uint8_t bytes[4];
} FLOAT_ARRAY;

struct NodeMap {
  // NodeID is mapped to table's rowID
  uint8_t nodeID;
  // Last time a message was received
  unsigned long lastMsg;
  bool selected;
} nodes[] = {

  {
    nodeID: 10,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 20,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 30,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 40,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 50,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 60,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 70,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 80,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 90,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 100,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 105,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 110,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 115,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 120,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 125,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 1,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 2,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 3,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 4,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 5,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 6,
    lastMsg: 0,
    selected: false
  },
  {
    nodeID: 7,
    lastMsg: 0,
    selected: false
  }
};

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
  timeClient.begin();
  timeClient.update();
}

void loop() {
  selectRadio();
  if ( manager.waitAvailableTimeout(200) ) {
    uint8_t buf[MSG_MAX_LEN];
    uint8_t buf_len = sizeof(buf);
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
        attachRSSI( buf, from );
        
        rf95.setModeIdle();
        selectEthernet();
        NodeMap *node = getNodeByID(decodeNodeID(buf));
        if (node) {
          node->lastMsg = timeClient.getEpochTime();
          
          // RSSI value and node reference
          updateBlynkNode((int8_t)buf[6], decodeVBatt(buf), node);
          
          selectNode(node);
        } else if (DEBUG_ENABLED) {
          Serial.println(F("Node not mapped"));
        }
        Blynk.virtualWrite(decodeBlinkPin(buf), decodePayload(buf));
      }
    }
  }
  rf95.setModeIdle();
  selectEthernet();
  timeClient.update();
  if (millis() - lastSent > UPDATE_MIN_TIME) {
    checkMessageTimes();
    lastSent = millis();
  }
  Blynk.run();
  
}

NodeMap* getNodeByID(uint8_t nID) {
  for (byte i = 0; i < sizeof(nodes)/sizeof(NodeMap); i++) {
    if (nodes[i].nodeID == nID) {
      return &nodes[i];
    }
  }
  
  return NULL;
}

char* floatToString(float f) {
  char buff[6];
  //4 width, 2 precision
  dtostrf(f, 4, 2, buff);
  return buff;
}

void hour24toAMPM(uint8_t h, uint8_t m, char *buf) {
  
  // 0 12AM, 1 1AM 12PM
  if (h == 0) {
    buf[0] = '1';
    buf[1] = '2';
    buf[6] = 'A';
    buf[7] = 'M';
  } else if (h < 10) {
    buf[0] = '0';    
    buf[1] = (char)('0' + h);
    buf[6] = 'A';
    buf[7] = 'M';
  } else if (h < 13) {
    buf[0] = '1';
    buf[1] = (char)('0' + h%10);
    if (h == 12) {
      buf[6] = 'P';
      buf[7] = 'M';
    } else {
      buf[6] = 'A';
      buf[7] = 'M';
    }
  } else if (h < 24) {
    h = h - 12;
    buf[0] = (char)('0' + h/10);
    buf[1] = (char)('0' + h%10);
    buf[6] = 'P';
    buf[7] = 'M';
  }

  buf[2] = ':';

  if (m < 10) {
    buf[3] = '0';
    buf[4] = (char)('0' + m);
  } else {
    buf[3] = (char)('0' + m/10);
    buf[4] = (char)('0' + m%10);
  }

  buf[5] = ' ';
  buf[8] = '\0';
}

void updateBlynkNode(int8_t rssi, float vbat, NodeMap *node) {
  // Node XXX
  char nodeName[10];
  // TIME, VBATT, RSSI
  // XX:XX:XX, XX.XXV, -XXX RSSI
  char infoLine[30];
  char timeStr[10];
  hour24toAMPM(timeClient.getHours(), timeClient.getMinutes(), timeStr);
  //timeClient.getFormattedTime().toCharArray(timeStr, 10);
  sprintf_P(nodeName, PSTR("Node %d"), node->nodeID);
  sprintf_P(infoLine, PSTR("%s, %sV, %3d RSSI"), timeStr, floatToString(vbat), rssi);
  
  Blynk.virtualWrite(V0, F("update"), node->nodeID, infoLine, nodeName);
}

void checkMessageTimes() {
  
  for (byte i = 0; i < sizeof(nodes)/sizeof(NodeMap); i++) {
    if (nodes[i].lastMsg > 0 && timeClient.getEpochTime() - nodes[i].lastMsg > NOTIFY_TIME) {
      char msg[20];
      sprintf_P(msg, PSTR("Node %d Offline!"), nodes[i].nodeID);
      Blynk.notify(msg);
      nodes[i].lastMsg = 0;
    }
    if (timeClient.getEpochTime() - nodes[i].lastMsg > ICON_OFF_TIME) {
      deselectNode(&nodes[i]);
    }
  }
}

void selectNode(NodeMap *node) {
  if (!node->selected) {
    Blynk.virtualWrite(V0, F("select"), node->nodeID);
    node->selected = true;
  }
}

void deselectNode(NodeMap *node) {
  Blynk.virtualWrite(V0, F("deselect"), node->nodeID);
  node->selected = false;
}

float decodePayload(uint8_t *packet) {

  FLOAT_ARRAY pyld;
  
  pyld.bytes[0] = packet[2];
  pyld.bytes[1] = packet[3];
  pyld.bytes[2] = packet[4];
  pyld.bytes[3] = packet[5];

//  if (DEBUG_ENABLED) {
//    Serial.print(F("Received payload: "));Serial.println(pyld.num);Serial.flush();
//  }
  return pyld.num;
}

float decodeVBatt(uint8_t *packet) {

  FLOAT_ARRAY vbat;

  vbat.bytes[0] = packet[7];
  vbat.bytes[1] = packet[8];
  vbat.bytes[2] = packet[9];
  vbat.bytes[3] = packet[10];

  return vbat.num;
}

uint8_t decodeBlinkPin( uint8_t *packet ) {
  return packet[1] + packet[0]; //Blink pin is originID plus payloadID
}

uint8_t decodeNodeID( uint8_t *packet ) {
  return packet[1];
}

void attachRSSI( uint8_t *packet, uint8_t from ) {
  // Attach RSSI if sender is origin
  if (packet[1] == from) {
    packet[6] = rf95.lastRssi();
  }
}

bool linkToGate(uint8_t toNode) {
  // Build response message
  uint8_t buf[MSG_MAX_LEN];
  uint8_t buf_len = sizeof(buf);
  
  buf[0] = GATE_LINK;
  buf[1] = rf95.lastRssi();
  buf[2] = 0;// 0 hops

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

  //Setup info table
  Blynk.virtualWrite(V0, F("clr"));
  char nodeName[9];
  for (byte i = 0; i < sizeof(nodes)/sizeof(NodeMap); i++) {
    sprintf_P(nodeName, PSTR("Node %d"), nodes[i].nodeID);
    Blynk.virtualWrite(V0, F("add"), nodes[i].nodeID, F("Unknown"), nodeName);
    Blynk.virtualWrite(V0, F("deselect"), nodes[i].nodeID);
  }
  
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
