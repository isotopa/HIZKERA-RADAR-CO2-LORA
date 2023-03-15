#include <Arduino.h>
#include <SensirionI2CScd4x.h>   // AÑADIDO SILVIA PARA CO2
#include <Wire.h>                // AÑADIDO SILVIA PARA CO2

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <60ghzfalldetection.h>
#include <esp_now.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <functions.h>
#include <settings.h>
#include "RunningAverage.h"

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6


SensirionI2CScd4x scd4x;   // AÑADIDO SILVIA PARA CO2
FallDetection_60GHz radar;

float temperature;         // AÑADIDO SILVIA PARA CO2
float humidity; 
uint16_t co2;          // AÑADIDO SILVIA PARA CO2
float caidita;
unsigned char data[7];



#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

void do_send(osjob_t* j);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xa5, 0x88, 0xec, 0xd3, 0xe5, 0x64, 0xd2, 0x5e };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x62, 0x57, 0x39, 0xe7, 0xa6, 0x7a, 0x89, 0x05, 0xc9, 0xf3, 0xed, 0x2b, 0xa8, 0x60, 0xca, 0xbf };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//static uint8_t mydata[] = "HIZKERA - OK!";
//static uint8_t mydata2[] = {temperature};  //AÑADIDO SILVIA CO2
static osjob_t sendjob;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

RunningAverage RAC(10);
int samples = 0;
int oldavgco2 = 0;

void bufferInit(void) {
  Serial.println(__FILE__);
  Serial.print("Version: ");
  Serial.println(RUNNINGAVERAGE_LIB_VERSION);
  // explicitly start clean
  RAC.clear(); 
}


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

//AÑADIDO SILVIA PARA CO2
void printUint16Hex(uint16_t value) {
   Serial.print(value < 4096 ? "0" : "");
   Serial.print(value < 256 ? "0" : "");
   Serial.print(value < 16 ? "0" : "");
   Serial.print(value, HEX);
}
//AÑADIDO SILVIA PARA CO2
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
   Serial.print("Serial: 0x");
   printUint16Hex(serial0);
   printUint16Hex(serial1);
   printUint16Hex(serial2);
   Serial.println();
}

// EspNow REVEIVER
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  int x;
  int y;
  int z;
}struct_message;
// Create a struct_message called myData
struct_message myData;
// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;
// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2, board3};
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].x = myData.x;
  boardsStruct[myData.id-1].y = myData.y;
  boardsStruct[myData.id-1].z = myData.z;
  Serial.printf("x value: %d \n", boardsStruct[myData.id-1].x);
  Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
  Serial.printf("z value: %d \n", boardsStruct[myData.id-1].z);
  Serial.println();
}

void EspNowInit() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
/// Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
}

void EspNowLoop() {
   // Acess the variables for each board
  int board1X = boardsStruct[0].x;
  int board1Y = boardsStruct[0].y;
  int board1Z = boardsStruct[0].z;
  int board2X = boardsStruct[1].x;
  int board2Y = boardsStruct[1].y;
  int board2Z = boardsStruct[1].z;
  int board3X = boardsStruct[2].x;
  int board3Y = boardsStruct[2].y;
  int board3Z = boardsStruct[2].z;
  delay(5000);
}

void radarInit() {
    radar.SerialInit();
}

void radarloop()
{
radar.recvRadarBytes();                       //Receive radar data and start processing
        if (radar.newData == true) {                  //The data is received and transferred to the new list dataMsg[]
    byte dataMsg[radar.dataLen+3] = {0x00};
    dataMsg[0] = 0x53;                         //Add the header frame as the first element of the array
    for (byte n = 0; n < radar.dataLen; n++)dataMsg[n+1] = radar.Msg[n];  //Frame-by-frame transfer
    dataMsg[radar.dataLen+1] = 0x54;
    dataMsg[radar.dataLen+2] = 0x43;
    radar.newData = false;                     //A complete set of data frames is saved
    
    radar.ShowData(dataMsg);                 //Serial port prints a set of received data frames
    radar.Fall_Detection(dataMsg);         //Use radar built-in algorithm to output human motion status
    radar.Situation_judgment(dataMsg);  //**************

        if (FALL_DETECTION = true){

            int16_t estado = (int16_t)(caidita);
            data[6] = estado >> 8;
	        data[7] = estado & 0xFF;
        }
  }

 }

void leerSCD41(){
    uint16_t error;
    char errorMessage[256];
    
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else if (co2 == 0) {
        Serial.println("Invalid sample detected, skipping.");
    } else {
        Serial.print("Co2:");
        Serial.print(co2);
        Serial.print("\t");
        Serial.print("Temperatura:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("% Humedad:");
        Serial.println(humidity);
    }  

    // Conversion 
    int16_t intTemp = (int16_t)(temperature*100);
    int16_t intHume = (int16_t)(humidity*100);
    int16_t intDioxido = (int16_t)(co2);
   // static uint8_t mydata[] = "HIZKERA - OK!";
   //int16_t intEstado = (int16_t)strtol(estado, NULL, 26);

	// shift bits and store the value in bytes	
    data[0] = intTemp >> 8;
	  data[1] = intTemp & 0xFF;
	  data[2] = intHume >> 8;
	  data[3] = intHume & 0xFF;
	  data[4] = intDioxido >> 8;
	  data[5] = intDioxido & 0xFF;
    //data[6] = estado >> 8;
	//data[7] = estado & 0xFF;
}

//static uint8_t mydata[] = "HIZKERA - OK!";

// void radarInit() {
//     radar.SerialInit();
// }

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
               
        //static uint8_t mydata[] = "Hizkera OK!";
         
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);

        leerSCD41();
        radarloop();
        

        LMIC_setTxData2 (1, data, sizeof (data)-1, 0);
        Serial.println(F("Packet queued"));
   } 
}
     
   
void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable Brownout Detector
    Serial.begin(115200);
    Serial.println(F("Starting Hizkera system ..."));
    
    while (!Serial); {                  // wait for Serial to be initialized
       delay(100); 
       }    

       #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

 Wire.begin();
    uint16_t error;  
    char errorMessage[256];
    scd4x.begin(Wire);

 // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (30 sec)");
   
  //EspNowInit();
  radarInit();
  bufferInit();
  Wire.begin();
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    do_send(&sendjob);
}

void loop() {

    os_runloop_once();

}