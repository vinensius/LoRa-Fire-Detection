
#include <CayenneLPP.h>
#include <DHTesp.h>
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>



// LoRaWAN NwkSKey, network session key

static const PROGMEM u1_t NWKSKEY[16] = { 0xB3, 0x0D, 0x4B, 0x4E, 0xD5, 0xF1, 0xCD, 0x56, 0x78, 0x73, 0xAD, 0xA4, 0x55, 0x4E, 0xFD, 0x9B };
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x78, 0xF6, 0xB3, 0x7E, 0x1A, 0x15, 0xC2, 0xC2, 0x04, 0x15, 0x15, 0xE6, 0xB2, 0xC3, 0x5B, 0x09 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR =  0x2604167C ;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;
// Schedule data trasmission in every this many seconds (might become longer due to duty
// cycle limitations).
// we set 10 seconds interval
const unsigned TX_INTERVAL = 10; // Fair Use policy of TTN requires update interval of at least several min. We set update interval here of 1 min for testing

// Pin mapping according to Cytron LoRa Shield RFM
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

/** Pin number for DHT11 data pin */
#define DHTPIN 17
#define DHTTYPE DHT22

//mq2 flame
int mq2sensor = 35;
int flamesensor = 34;
int flame_detected;
int smoke_detected;


// Adafruit GPS
#define SERIAL1_RXPIN 12
#define SERIAL1_TXPIN 13

HardwareSerial gpsSerial(2);  // use ESP32's 2nd serial port (default serial 1 is for debug)
Adafruit_GPS GPS(&gpsSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
//#define GPSECHO  true

DHTesp dht;

CayenneLPP lpp(51);



void onEvent (ev_t ev) 
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) 
  {
    case EV_TXCOMPLETE:
      Serial.printf("EV_TXCOMPLETE (includes waiting for RX windows)\r\n");
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;  
    case EV_RXCOMPLETE:
      if (LMIC.dataLen)
      {
        Serial.printf("Received %d bytes\n", LMIC.dataLen);
      }
      break;
    default:
      Serial.printf("Unknown event\r\n");
      break;
  }
}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) 
  {
    Serial.printf("OP_TXRXPEND, not sending\r\n");
  } 
  else
  { 
    TempAndHumidity newValues = dht.getTempAndHumidity();
    smoke_detected = digitalRead(mq2sensor);
    flame_detected = digitalRead(flamesensor);
    
    lpp.reset();
    lpp.addTemperature(1, newValues.temperature);
    lpp.addRelativeHumidity(2, newValues.humidity);
    lpp.addDigitalInput(3,smoke_detected);
    lpp.addDigitalInput(4,flame_detected);

     
    Serial.printf("Temperature : %.2f, Humidity : %.2f\r\n", newValues.temperature, newValues.humidity);
    if (flame_detected == 1)
  {
    Serial.println("Flame detected...!");
  }
  else
  {
    Serial.println("No flame detected.");
  }
  if (smoke_detected == 0 )
  {
    Serial.println("Smoke detected...! ");
  }
  else
  {
    Serial.println("No smoke detected.");
  }
  
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
         
    Serial.printf("Packet queued\r\n");
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() 
{ 
  Serial.begin(115200);
  gpsSerial.begin(115200, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  Serial.printf("Starting...\r\n");
  dht.setup(DHTPIN, DHTesp::DHT22);
  pinMode(flamesensor, INPUT);
  pinMode(mq2sensor, INPUT);
  
#if 0
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
#endif
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  // Select frequencies range
  LMIC_selectSubBand(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF10,14);
  Serial.printf("LMIC setup done!\r\n");
  // Start job
  do_send(&sendjob);
}

void loop() 
{
#if 0
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //if ((c) && (GPSECHO))
  //  Serial.write(c);
     
   // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) 
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      ; //return;  // we can fail to parse a sentence in which case we should just wait for another
  } 
#endif    
  // Make sure LMIC is ran too
  os_runloop_once();
}
