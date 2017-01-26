
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 13

String imuString = "";

#include <Adafruit_GPS.h>

#define mySerial Serial3

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  false

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

String gpsString = "";
String gpsString1 = "";

String ROBOT="1";

String odoString = "";

boolean stringComplete = false;
boolean stringComplete2 = false;
String sendPayload = "";
String sendPayload2 = "";
String inputString = "";

void setup(void) {
  
gpsString.reserve(500);
gpsString1.reserve(200);

  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println("MainController");
  Serial.println();
  inputString.reserve(200);
GPS.begin(9600);
  mySerial.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif


    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    accelgyro.initialize();

    // configure Arduino LED for
    imuString.reserve(200);
    sendPayload.reserve(200);
    sendPayload2.reserve(200);
}
#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  

#endif
}

void useInterrupt(boolean v) {
  if (v) {

    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
  Serial.println("ax,ay,az,gx,gy,gz,e,robotID,time m_controller,x,y,theta,hour,minutes,second,day,month,year,fix,fix_quality,latitude,lat,longitude,lon,speed angle,altitude,satelite");
}
#endif //#ifdef__AVR__

uint32_t timer = millis();
uint32_t timer1 = millis();
int odo = 0 ;

void loop(void) {
if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
     if (c) Serial.print(c);
  }

  if (GPS.newNMEAreceived()) {
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
// approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 1000) { 
    timer = millis(); // reset the timer
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    imuString=String(ax)+","+String(ay)+","+String(az)+","+String(gx)+","+String(gy)+","+String(gz);
        if(odo==1)
        { odoString=  "," + sendPayload + ","; odo  = 0;}
        else
        { odoString=  ", , , , , , ,"; odo=0;}
    gpsString=String(GPS.hour, DEC)+','+String(GPS.minute, DEC)+','+String(GPS.seconds, DEC)+","+String(GPS.day, DEC)+','+String(GPS.month, DEC)+","+String(GPS.year, DEC)+","+String((int)GPS.fix)+","+String(((int)GPS.fixquality));
    gpsString1=","+String(GPS.latitude, 8)+String(GPS.lat)+","+String(GPS.longitude, 8)+String(GPS.lon)+","+String(GPS.speed,4)+","+String(GPS.angle,4)+","+String(GPS.altitude,4)+","+String((int)GPS.satellites);
    gpsString=imuString+odoString+gpsString+gpsString1;
    
    Serial.println(gpsString);
}
  else if(millis() - timer1 > 20) { 
    timer1 = millis(); // reset the timer}
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    imuString=String(ax)+","+String(ay)+","+String(az)+","+String(gx)+","+String(gy)+","+String(gz);
    if(odo==1)
        {odoString=","+sendPayload; odo=0;}
    else
        { odoString=  ""; odo=0;}
    Serial.println(imuString+odoString);
  }
}

void serialEvent1() {
  sendPayload = "";
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    if (inChar != '\n')
    sendPayload += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
      odo=0;
    if(sendPayload[0]=='e')
      {
        odo=1;
        }
      stringComplete = false;
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
     sendPayload2 += inChar;
    if (inChar == '\n') {
      stringComplete2 = true;
    }
  }
    if (stringComplete2) {
      Serial.print(sendPayload2);
    if(sendPayload2.startsWith("1")){
      sendPayload2=sendPayload2.substring(2);
      Serial.print(sendPayload2);
      if (sendPayload2.startsWith("MOTOR")) {
        sendPayload2=sendPayload2.substring(6);
        Serial1.print(sendPayload2);
        Serial.print(sendPayload2);
      }
    }
  // else{Serial.print("Hello World");}
    sendPayload2="";
    stringComplete2 = false;
  }
}
