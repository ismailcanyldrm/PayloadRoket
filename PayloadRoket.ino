
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "LoRa_E32.h"

TinyGPS gps;
Adafruit_BMP085 bmp;
SoftwareSerial ss(6,5);
File dosya;
LoRa_E32 E32(&Serial);

int gaz1;
int gaz2;
static void smartdelay(unsigned long ms);
float p,h,t ,e ,b,be,bes;
struct Signal {

    byte gaz1[4];
    byte gaz2[4];
    byte t[4];
    byte p[4];
    byte h[4];
    byte gpsE[4];
    byte gpsB[4];
    byte gpsH[4];
    byte gpsS[4];
} data;



void setup() {
    pinMode(A0, OUTPUT);
    ss.begin(9600);
    Serial.begin(9600); 
    SD.begin(10); 
    bmp.begin();
    E32.begin();
}
void loop() {
    mq135();
    mq4();
    bmp180();
    gpsSpeed();
    gpsAltitude();
    gpsKonum();
    smartdelay(250);

    *(int*)(data.gaz1) = gaz1 ;
    *(int*)(data.gaz2) = gaz2 ;

    *(float*)(data.t) = t ;
    *(float*)(data.p) = p ;
    *(float*)(data.h) = h ;

    *(float*)(data.gpsE) = e ;
    *(float*)(data.gpsB) = b ;
    *(float*)(data.gpsH) = be ;
    *(float*)(data.gpsS) = bes ;
  
    ResponseStatus rs = E32.sendFixedMessage(0, 61, 21, &data, sizeof(Signal));
    buzzer(1,15);
    dosya = SD.open("test.txt", FILE_WRITE); 
    if (dosya) {
      dosya.println(t);
      dosya.println(p);
      dosya.println(h);
      dosya.println(gaz1);
      dosya.println(gaz2);
      dosya.println(e);
      dosya.println(b);
      dosya.println(be);
      dosya.println(bes);
      dosya.close();
  } 

}
    
void mq4()
{
  gaz1=random(100,200);
  int mq4[1]= {0};
  mq4[0] = gaz1;
}
void mq135()
{
  gaz2=analogRead(A1);
  int mq135[1]= {0};
  mq135[0] = gaz2;
}

void bmp180()
{
    float bmp180[3]={0.0,0.0,0.0};
    bmp180[0] = bmp.readTemperature();
    bmp180[1] = bmp.readPressure()/100.00;
    bmp180[2] = bmp.readAltitude();

    t = bmp180[0];
    p = bmp180[1];
    h = bmp180[2];
}
void gpsKonum() {
  float gpsk[2]= {0.000000,0.000000};
  float flat, flon, invalid;
  gps.f_get_position(&flat, &flon);
  invalid = TinyGPS::GPS_INVALID_F_ANGLE;
  
  if (flat == invalid || flon == invalid) {
    gpsk[0]= 0.000000;
    gpsk[1]= 0.000000;
    e = gpsk[0];
    b = gpsk[1];
  } else {
    gpsk[0]= float(flat),6;
    gpsk[1]= float(flon),6;
    e = gpsk[0];
    b = gpsk[1]; 
  }
}
void gpsAltitude(){
  float gpsa[1]= {0.00};
  float gpsAltitude , invalid;
  gpsAltitude = gps.f_altitude() ;
  invalid   = TinyGPS::GPS_INVALID_F_ALTITUDE;
  if (gpsAltitude == invalid) {
    gpsa[0]= 0.00;
    be = gpsa[0];
  } else {
    gpsa[0]= float(gpsAltitude);
    be = gpsa[0];
    }
}
void gpsSpeed(){
  float gpss[1]= {0.00};
  float gpsSpeed , invalid;
  gpsSpeed = gps.f_speed_kmph() ;
  invalid   = TinyGPS::GPS_INVALID_F_SPEED;
  if (gpsSpeed == invalid) {
    gpss[0]= 0.00;
    bes = gpss[0];
  } else {
    gpss[0]= float(gpsSpeed);
    bes = gpss[0];
    }
}
static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void buzzer(int tekrar, int sure) {
  for (int i = 0; i < tekrar; i++) {
    analogWrite(A0, 1023);
    delay(sure);
    analogWrite(A0, 0);
    delay(sure);
  }
}
