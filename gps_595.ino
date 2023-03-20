//#include <Keyboard.h>

#include <SoftwareSerial.h>

// Connect the GPS RX/TX 
SoftwareSerial gps (D7,D8);

// connect 4digit 7segment display
#define DATA_PIN   D2
#define RCLK_PIN   D3
#define SCLK_PIN   D4

#include <SevenSegmentsDisp.h>
Disp595 disp(DATA_PIN, SCLK_PIN, RCLK_PIN);

#define R D6 //Red pin
#define G D5 //Green pin
#define B D0 //Blue pin
byte In=150; //Intencity for RGB led

bool deBug = false;           // debug on Serial, main

unsigned long lastScreenUpdate = 0;
unsigned long DispUpdate = 99; // Redraw 4-digit 7-segments display
int Speed, Fix, Sat, hAcc;
bool gps_led ;                  // led for gps messages status

// 0-100kmh timer
unsigned long timerMeterSpeed = 20000; // wait for 100kmh for first 20 seconds.
unsigned long timerShowSpeed  =  3000; // show speed for 3 seconds.
unsigned long startMillis = 0;         // Start of speedup.
unsigned long currentMillis = 0;       // Current time.
unsigned long lastDisplaySpeedup = 0;  // Show speedup time.
bool start100 = false;                 // Start race
float accel100;                        // 0-100kmh time
bool met100;                           // true if complete 0-100kmh speedup.


// header
const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };
const unsigned char UBX_NAV_PVT[]       = { 0x01, 0x07 };

const unsigned char UBLOX_INIT_SPD[] PROGMEM = {
  0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x91,0x84, // UBX only, 38400
};
const unsigned char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // TIM-TP message to sync time with GPS
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x0D,0x01,0x00,0x01,0x00,0x00,0x00,0x00,0x1E,0x17,

  // Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x3C,0x00,0x01,0x00,0x01,0x00,0x52,0x22, // 16.67Hz (60Ms)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x48,0x00,0x01,0x00,0x01,0x00,0x5E,0x6A, // 13.89Hz (72Ms)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x50,0x00,0x01,0x00,0x01,0x00,0x66,0x9A, // 12.5Hz (80Ms)
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, // 10Hz (100Ms)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, // 5Hz (500Ms)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, // 1Hz (1000Ms)

  // 5 sattelites min
  0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x16,0x04,0x00,0x05,0xFF,0x00,0x01,0x00,0x00,0x01,0x01,0x01,0x03,0x00,0x01,0x00,0x00,0x01,0x05,0x00,0x03,0x00,0x00,0x00,0x00,0x01,0x06,0x08,0xFF,0x00,0x00,0x00,0x00,0x01,0xA6,0x58,

  // automotive,Fix 3D
  0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x04,0x02,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4F,0x83,

  // Sbas autoscan
  0xB5,0x62,0x06,0x16,0x08,0x00,0x01,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x2B,0xB9,

};


// UBX-CFG-PRT
 //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xA0,0xA9, // UBX only, 9600
 //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x4B,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x46,0x4B, // UBX only, 19200
 //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x96,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x91,0x84, // UBX only, 38400
 //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xDC,0xBD, // UBX only, 57600
 //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xBE,0x72, // UBX only, 115200

struct NAV_PVT {
  unsigned char cls;           // 1/
  unsigned char id;            // 1/
  unsigned short len;          // 2/
  unsigned long iTOW;          // 4/ GPS time of week of the navigation epoch (ms)

  unsigned short year;         // 2/ Year (UTC)
  unsigned char month;         // 1/ Month, range 1..12 (UTC)
  unsigned char day;           // 1/ Day of month, range 1..31 (UTC)
  unsigned char hour;          // 1/ Hour of day, range 0..23 (UTC)
  unsigned char minute;        // 1/ Minute of hour, range 0..59 (UTC)
  unsigned char second;        // 1/ Seconds of minute, range 0..60 (UTC)
  char valid;                  // 1/ Validity Flags (see graphic below)
  unsigned long tAcc;          // 4/ Time accuracy estimate (UTC) (ns)
  long nano;                   // 4/ Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // 1/ GNSSfix Type, range 0..5
  char flags;                  // 1/ Fix Status Flags
  unsigned char reserved1;     // 1/ Flags2
  unsigned char numSV;         // 1/ Number of satellites used in Nav Solution

  long lon;                    // 4/ Longitude (deg)
  long lat;                    // 4/ Latitude (deg)
  long height;                 // 4/ Height above Ellipsoid (mm)
  long hMSL;                   // 4/ Height above mean sea level (mm)
  unsigned long hAcc;          // 4/ Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // 4/ Vertical Accuracy Estimate (mm)

  long velN;                   // 4/ NED north velocity (mm/s)
  long velE;                   // 4/ NED east velocity (mm/s)
  long velD;                   // 4/ NED down velocity (mm/s)
  long gSpeed;                 // 4/ Ground Speed (2-D) (mm/s)
  long heading;                // 4/ Heading of motion 2-D (deg)
  unsigned long sAcc;          // 4/ Speed Accuracy Estimate
  unsigned long headingAcc;    // 4/ Heading Accuracy Estimate
  unsigned short pDOP;         // 2/ Position dilution of precision
  short reserved2;             // 2/ Additional flags3
  unsigned long reserved3;     // 4/ Reserved
  unsigned char dummy[10];     // 6/ extra bytes. UBX6 has 0, UBX7 - 0, UBX8 - 6.
};

NAV_PVT pvt;

void gpsSetup() {
  // send configuration data in UBX protocol
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    gps.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}

byte processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  int payloadSize = sizeof(NAV_PVT);
  int calcSize = 0;
  
  while ( gps.available() ) {
    byte c = gps.read();
    //Serial.print(","); Serial.print(c,HEX); 
    
    if ( fpos < 2 ) {
      // looking for UBX sync bytes.
      if ( c == UBX_HEADER[fpos] ) {
        fpos++;
        }
      else { fpos = 0; }
    }
    else if ( fpos == 2 ) { 
      // only accept NAV_PVT header, byte 0.
      if ( c == UBX_NAV_PVT[0] ) { ((unsigned char*)(&pvt))[fpos-2] = c; fpos++;  }
      else { fpos = 0; return 6;}
    }
    else if ( fpos == 3 ) { 
      // only accept NAV_PVT header, byte 1.
      if ( c == UBX_NAV_PVT[1] ) { ((unsigned char*)(&pvt))[fpos-2] = c; fpos++;  }
      else { fpos = 0; return 7; }
    }
    else if ( fpos > 3 && fpos < 6 ) { ((unsigned char*)(&pvt))[fpos-2] = c; fpos++;
    }
    else {
      // here we can calculate payload size from gps data
      calcSize = ((unsigned char*)(&pvt))[2] + 256 * ((unsigned char*)(&pvt))[3] + 4;
      if ( calcSize < payloadSize )  payloadSize = calcSize; 

      if ( (fpos-2) < payloadSize ) 
        ((unsigned char*)(&pvt))[fpos-2] = c; 

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        //calcChecksum(checksum);
        memset(checksum, 0, 2);
        for (int i = 0; i < payloadSize; i++) {
          checksum[0] += ((unsigned char*)(&pvt))[i];
          checksum[1] += checksum[0];
        }
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] ) { fpos = 0; return 2; }
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) { 
          return 1; // full package received
        } else { fpos = 0; return 3; }
      }
      else if ( fpos > (payloadSize+4) ) { fpos = 0; return 4; }
    }
  }
  return 0; // no more bytes in serial
}

void setup() {
  delay(500); // wait 0.5 sec to allow esp8266 to boot and start serial port
  Serial.begin(38400);
  // display
  Serial.println("Display start");
  disp.tick(); // Таймер уже встроен в .tick()
  disp.displayBytes( _i, _n, _i, _t );
  disp.tick();

  // RGB led
  Serial.println("RGB start");
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  digitalWrite(R, 0); digitalWrite(G, 0); digitalWrite(B, 0); // Black
 
  // gps
  // Switch from 9600 to 38400
  Serial.println("GPS speed start");
  gps.begin(9600);
  for (int i = 0; i < sizeof(UBLOX_INIT_SPD); i++) {
    gps.write( pgm_read_byte(UBLOX_INIT_SPD + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
    disp.tick();
  }
  gps.end();
  
  // send configuration data in UBX protocol
  gps.begin(38400);
  Serial.println("GPS setup protocols");
  for (int i = 0; i < sizeof(UBLOX_INIT); i++) {
    gps.write( pgm_read_byte(UBLOX_INIT + i) );
    delay(10); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
    disp.tick();
  }

  Serial.println("Done");
  disp.displayBytes( _empty, _empty, _empty, _empty ); 
  disp.tick();
}

void loop() {
  unsigned long now = millis();
  disp.tick();

  /*byte gps_result=processGPS();
  if ( gps_result == 0 ) { }
  else if ( gps_result == 1 ) { Serial.println("Correct packet."); }
  else if ( gps_result == 2 ) { Serial.println("CRC 0 error."); }
  else if ( gps_result == 3 ) { Serial.println("CRC 1 error."); }
  else if ( gps_result == 4 ) { Serial.println("Packet bigger, that expected."); }
  else if ( gps_result == 5 ) { Serial.println("UBX sync not found"); }
  else if ( gps_result == 6 ) { Serial.println("NAV_PVT header byte 0 not found."); }
  else if ( gps_result == 7 ) { Serial.println("NAV_PVT header byte 1 not found."); }
  */
  
  if ( processGPS() == 1 ) {
    Sat = pvt.numSV;
    Speed = pvt.gSpeed * 0.0036; // Переводим в км/ч    
    Fix = pvt.fixType;

    // start metering. if we start moving
    currentMillis = millis();
    
    if (Speed > 0) {                                                // This is started just now:
      if (!start100) {                                              // We don't have speedup time yet
        start100 = true;
        startMillis = currentMillis;
        met100 = false;
      }
      // save result:
      if (!met100 ) {                                               // if 0-100 not metered
        if ( currentMillis - startMillis < timerMeterSpeed ) {      // if past not more 20 seconds
          if ( Speed >= 100 ) {                                     // we reach 100kmh speed
            accel100 = (float)(currentMillis - startMillis) / 1000; //  Time speedup 0-100kmh
            met100 = true; 
            lastDisplaySpeedup = currentMillis;
          }
        }
      }
    } else if (start100 && Speed == 0 ) {                               // If we stopped
      start100 = false; }
    // end metering  
 
    if ( Fix == 3 ) {
      // don't show status
      if ( gps_led) {
        // only if dot true
        gps_led=false;
        disp.point(0, gps_led);
      }
    } else {
      // show status led
      gps_led=!gps_led;
      disp.point(0, gps_led);
    }

    if (deBug) {
      Serial.print("NAV_PVT ");
      Serial.print("#Sat:");      Serial.print(pvt.numSV);      Serial.print(",");
      Serial.print("fixType:");   Serial.print(pvt.fixType);    Serial.print(",");
      Serial.print("Speed:");     Serial.print(Speed);          Serial.print(",");
      Serial.print("hAcc:");      Serial.print(pvt.hAcc/1000);  Serial.print(",");
      Serial.println();
    }
  } 

  // Show result on display only if X milliseconds pass.
  if ( now - lastScreenUpdate > DispUpdate ) {
    updateScreen();
    lastScreenUpdate = now;
  }

}

void updateScreen() {
  // check, should we show time or speed
  if ( met100  &&  millis() - lastDisplaySpeedup < timerShowSpeed ) {
    // show speedup time
    disp.displayFloat(accel100);
    disp.displayByte(_t,0);
  } else {
  // show usual speeed display
    switch ( Fix ) {
      case 3:
      case 2:
        disp.displayInt(Speed);
        if (deBug) { Serial.println("3d/2d fix."); }
        break;
      default:
        // No fixation, show number of satelites.
        // disp.displayInt(Sat); 
        uint8_t data[4] = {_empty, _empty, _empty, _empty};
        disp.displayBytes(data, sizeof(data)); 
        break;
    }
    // RGB status led.
    if ( Sat > 5 )       { digitalWrite(R, 0); analogWrite(G, In-100); digitalWrite(B, 0); } // Green
    else if ( Sat == 5 ) { analogWrite(R, In); analogWrite(G, In-100); digitalWrite(B, 0); } // Yellow
    else if ( Sat == 4 ) { analogWrite(R, In); digitalWrite(G, 0);     analogWrite(B, In); } // Magenta
    else if ( Sat == 3 ) { digitalWrite(R, 0); digitalWrite(G, 0);     analogWrite(B, In); } // Blue
    else                 { analogWrite(R, In); digitalWrite(G, 0);     digitalWrite(B, 0); } // Red
  }   
}
