#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "LoRa_E22.h"
SFE_BMP180 pressure;
int pinSensor = A0;
int THRESHOLD = 250;
int pinCS = 4;
#define ALTITUDE 1655.0 
TinyGPS gps;
SoftwareSerial serialgps(5,6);
SoftwareSerial mySerial(10, 11); //rx tx
LoRa_E22 E22(&mySerial);

int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;
int sensorValue;

struct veriler {
  GPSS();
  hava();
  bmp();
  mq();
} data;

void setup()
{
   Serial.begin(9600);
   E22.begin();
   delay(500);

    Serial.begin(9600);
      serialgps.begin(9600);
    Serial.println("");
    Serial.println(" ...Uydu Bağlantısı Bekleniyor... ");
    

  if (pressure.begin())
    Serial.println("BMP180'e Bağlanıldı");
  else
  {
    Serial.println("BMP180'e bağlanılamadı");
    while(1);
  }// sets the serial port to 9600
  pinMode(pinSensor, INPUT);

 }

void loop()
{
 Serial.println("---------GPS VERİLERİ---------");
 GPSS();
 Serial.println("---------MQ135 VERİLERİ---------");
 hava();
 Serial.println("---------BMP180 VERİLERİ---------");
  bmp();
 Serial.println("---------MQ4 VERİLERİ---------");
 mq();

  ResponseStatus rs = E22.sendFixedMessage(0, 0, 23, &data, sizeof(veriler));
  Serial.println(rs.getResponseDescription());
}
void GPSS()
{
   while(serialgps.available())
    {
        int c = serialgps.read();
        if(gps.encode(c))
        {
            float latitude, longitude;
            gps.f_get_position(&latitude, &longitude);
            Serial.print("Enlem/Boylam: ");
            Serial.print(latitude,5);
            Serial.print(" / ");
            Serial.println(longitude,5);
            gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
            Serial.print("Gün: ");
            Serial.print(day, DEC);
            Serial.print("Ay: ");
            Serial.print(month, DEC);
            Serial.print("Yıl: ");
            Serial.print(year);
            Serial.print(" Saat: ");
            Serial.print((hour+3), DEC);
            Serial.print(":");
            Serial.print(minute, DEC);
            Serial.print(":");
            Serial.print(second, DEC);
            Serial.print(".");
            Serial.println(hundredths, DEC);
            Serial.print("Yükseklik (meters): ");
            Serial.println((gps.f_altitude()/1229.0000));
            Serial.print("Rota (degrees): ");
            Serial.println(gps.f_course());
            Serial.print("Hız(kmph): ");
            Serial.println(gps.f_speed_kmph());
            Serial.print("Uydu Sayısı: ");
            Serial.println(gps.satellites());
            Serial.println();
            
        }
    }
}
  
  void hava(){
  sensorValue = analogRead(0);       // read analog input pin 0
Serial.print("Hava Kalitesi=");
Serial.print(sensorValue, DEC);               // prints the value read
Serial.println(" PPM");

if ( sensorValue <400)
 {
 Serial.println("Temiz Hava");
 }

 else if( sensorValue >400 && sensorValue <550){
 
 Serial.println("Havalandırılması Gerekli");
 }
 else if(  sensorValue >550 && sensorValue <900)
 {
    Serial.println("Kirli Hava");
 }
 else {
  Serial.println("Kritik Hava Seviyesi");
 }
  

Serial.println("       "); 
Serial.print("  ");
delay(250);                                   // wait 100ms for next reading
}

double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);



        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
void bmp()
{
    double a,P;
  
  double  baseline = getPressure();
  
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb"); 

  P = getPressure();

  a = pressure.altitude(P,baseline);
  
  Serial.print("relative altitude: ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a,1);
  Serial.print(" meters, ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a*3.28084,0);
  Serial.println(" feet");
  
  delay(500);
}
void mq()
{
  int analogValue = analogRead(pinSensor);
Serial.println("Val: " +analogValue);
if (analogValue >= THRESHOLD) {
Serial.print("Metan Gazı Miktarı Yüksek.");
}
else {
Serial.println("Metan Gazı Miktarı Alçak.");
}

}
