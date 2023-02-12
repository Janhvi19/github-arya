/***********************************************Tasks to be done********************************************/
//Storing values from BMP280 in EEPROM
//Finding the tilt angles and the orientation of the CanSat
//Finding coordinates of the CanSat using GPS
//Storing values in SDcard using Teensy4.1
//Measuring UTC time using Teensy 4.1
//voltage divider using Teensy4.1   

/********************************************************Library********************************************/
#include <EEPROM.h> //EEPROM library
#include <Adafruit_BMP280.h> //adafruit BMP280 library
#include <TimeLib.h> //UTC time library
#include <SD.h> //SD card library0
#include <SPI.h> //SPI interface
#include <Adafruit_GPS.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <Servo.h>

/********************************************************Variable********************************************/
float referenceAltitude,actualAltitude,realAltitude,tiltx,tilty,orientation,i,t,volts;
double temperature,gAltitude,pressure,latitude_val, longitude_val;
int period=1000, flag,s , sats, Actualtime,UTCtime,UTCtime1,UTCtime2,a,time_HS=0,packetCount,packetFlag,EPC;
//char lat_dir,lng_dirn;
uint8_t hr, mins, sec;
unsigned long time_now=0;
char storedArray[20], buffers[150],HS = 'N',PC = 'N',Mast = 'N';
//const int chipSelect = BUILTIN_SDCARD;
uint32_t timer = millis();
const float referenceVolts = 9;
const float R1 = 1000; 
const float R2 = 4700;
const float resistorFactor = 1023.0 * (R2/(R1 + R2));
const int batteryPin = A0;
int val = analogRead(batteryPin);  
 
/********************************************************Define********************************************/
#define MPU9250_ADDR 0x68
#define GPSSerial Serial
#define GPSECHO false
#define TIME_HEADER  "T"   // Header tag for serial time sync message
#define chipSelect BUILTIN_SDCARD//FOR ACCESSING THE SD CARD SLOT OF TEENSY 4.1

/********************************************************Object********************************************/
Adafruit_BMP280 bmp; // I2C Interface
File myFile;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
Adafruit_GPS GPS(&GPSSerial);
TinyGPSPlus gps;
Servo HServo;
Servo PServo;
Servo MServo;
/********************************************************User Defined Functions********************************************/
void bmp280();
void tpa();

void mpu_init();
void mpu_tilt();

void gps_init();
void gps_loop();

//void sdcard(); //SDcard initialization
//void Program();//Function for saving the program in SDcard and reading the saved program in SDcard
void sdcard_1();

void rtc_set();
void rtc_loop();  

void VD();
void buffer_fun();

void packetc_loop();

void HS_deployed();
void HS_deployed();
void HS_deployed();
/********************************************************Setup********************************************/
void setup() {
  Serial.begin(9600);
  bmp280();
  mpu_init();
  gps_init();
  //sdcard();
  sdcard_1();
  rtc_set();
}

/********************************************************Loop********************************************/
void loop() 
{
  if(millis() > time_now+period)
  {
  buffer_fun();
  time_now= millis();
}
}

/********************************************************User Defined Function Description********************************************/

//_____________________________BMP280 EEPROM________________________________________
void bmp280(){
  Serial.println(F("BMP280 test"));
  if (!bmp.begin(0x76))
   {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
   }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
   
  flag=EEPROM.read(25);
  if(flag==0)
   {
    referenceAltitude = bmp.readAltitude();
    EEPROM.write(20,referenceAltitude);
    flag = 9;
    EEPROM.write(25,flag);
   }

  else{
    referenceAltitude = EEPROM.read(20);
   }

  Serial.print("The reference Altitude is : ");
  Serial.println(referenceAltitude);
 }

void tpa()
{
    temperature=bmp.readTemperature();
    pressure=bmp.readPressure()/10; //displaying the Pressure in kPa, you can change the unit
    actualAltitude=(bmp.readAltitude());
    realAltitude=actualAltitude-referenceAltitude;
}

//_____________________________Tilt angles and orientation using MPU9250________________________________________
void mpu_init() {
Wire.begin();
if(!myMPU9250.init()){
Serial.println("MPU9250 does not respond");
}
else{
Serial.println("MPU9250 is connected");
}

Serial.println("Position you MPU9250 flat and don’t move it – Calibrating...");
delay(1000);
myMPU9250.autoOffsets();
Serial.println("Done!");
myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);
myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
myMPU9250.enableAccDLPF(true);
myMPU9250.setAccDLPF(MPU9250_DLPF_6);
}

void mpu_tilt() {
xyzFloat angle = myMPU9250.getAngles();
tiltx =angle.x;
tilty = angle.y;
//orientation = myMPU9250.getOrientationAsString(); 
}     

//__________________________________________GPS________________________________________
void gps_init()
{
 Serial.println("Adafruit GPS library basic parsing test!");
 GPS.begin(9600);
 GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
 GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
 delay(1000);
 GPSSerial.println(PMTK_Q_RELEASE); 
}

void gps_loop ()
{
  hr = gps.time.hour(); 
  mins= gps.time.minute();  
  sec= gps.time.second();  
        
 latitude_val= (GPS.latitude, 4); 
 //lat_dir= (GPS.lat);
 longitude_val= (GPS.longitude, 4); 
 //lng_dirn=(GPS.lon);
 gAltitude = GPS.altitude;
 sats = (int)GPS.satellites;
 }

//__________________________________________SDcard________________________________________
void sdcard_1()
  {
    while (!Serial)
    { }
    if (!SD.begin(chipSelect)) {
      Serial.println("Initialization failed!");
      return;
    }
    myFile = SD.open("CanSat.csv", FILE_WRITE);
    // dataFile.close();
   sprintf(storedArray, "1033, %d:%d:%d, %f, %lf, %f, %lf, %02d:%02d:%02d, %lf, %lf, %lf, %d, %f,%f ",
          UTCtime1, UTCtime2, s, realAltitude,temperature, volts, pressure,hr, mins, sec, 
          gAltitude, latitude_val, longitude_val, sats, tiltx, tilty);
  //Load program
  int j=20;
  char array[j];
  myFile = SD.open("CanSat.csv");
  myFile.read(&array[j],array[j]);
  myFile.close(); //CHANGE THE FILE NAME BASED ON THE ONE PRESENT ON SD CARD
  }
/*void sdcard(){
  if (!SD.begin(chipSelect)) {
  Serial.println("Initialization failed!");
   return;
   }
 Serial.println("Initialization done!");
}
void Program(){
  //For saving data in SDcard
  dataFile = SD.open("sdcard_t.csv", FILE_WRITE);
  sprintf(storedArray, "1033, %d:%d:%d, %f, %lf, %f, %lf, %02d:%02d:%02d, %lf, %lf%c, %lf%c, %d, %f,%f ",
          UTCtime1, UTCtime2, s, realAltitude,temperature, volts, pressure,hr, mins, sec, 
          gAltitude, latitude_val, lat_dir, longitude_val, lng_dirn, sats, tiltx, tilty);   
  for (a = 0; a < 20; a++){
        dataFile.print(storedArray);
        dataFile.println();
  }
  dataFile.close();
  //Load program
  int j=20;
  char array[j];
  dataFile = SD.open("sd_card.csv");
  dataFile.read(&array[j],array[j]);
  dataFile.close(); 
}    
*/
//__________________________________________RTC________________________________________
void rtc_set(){
  setSyncProvider(getTeensy3Time);
  //while (!Serial);
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("According to UTC Time");
  }
}

void rtc_loop()
{ // digital clock display of the time
  Actualtime=hour()*60+minute();//conversion of hour to min
  UTCtime=Actualtime-(5*60+30);
  UTCtime1=UTCtime/60;//conversion of min to hour
  UTCtime2=UTCtime%60;//modulus formation
   
   s = second();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

//__________________________________________Voltage Divider________________________________________
void VD()
{
 volts = (val / resistorFactor) * referenceVolts; // calculate the ratio
}

/**************************Buffer function*****************************/
void buffer_fun()
{
  tpa();
  mpu_tilt();
  gps_loop();  
//  Program();
  rtc_loop();
  packetc_loop();
  VD(); 
sprintf(buffers,"1033, %d:%d:%d,%d, %f, %lf, %f, %lf, %02d:%02d:%02d, %lf, %lf, %lf, %d, %f,%f ",
                 UTCtime1, UTCtime2, s,EPC, realAltitude,temperature, volts, pressure,hr, mins, sec, 
                 gAltitude, latitude_val, longitude_val, sats, tiltx, tilty); 
Serial.println(buffers); 

}    

void HS_deployed()
{
  HServo.write(0);
  time_HS = millis();
  if(millis()- time_HS >= 5000)
  {
    HServo.write(90);
    HS = 'P';
  }
}
void PC_deployed()
{
  PServo.write(90);
  PC = 'C';
}
void Mast_raised()
{
  MServo.write(180);
  Mast = 'M';
}

//_________________________________________EEPROM PACKET COUNT____________________________________________________________
void packetc_setup() {
 Serial.begin(9600);
 
 packetCount=EEPROM.read(5);
 packetFlag=EEPROM.read(8);
 
 if(packetFlag==0)
   {
      packetCount=0;
      EEPROM.write(5,packetCount);
      //EEPROM.write(5,packetCount++);
      
      packetFlag=1;
      EEPROM.write(8,flag);
   }
 else
    {
      packetCount=EEPROM.read(5);
      packetCount++;
      EEPROM.write(5,packetCount);
      
    }
 
}

void packetc_loop()
  {
    EPC = packetCount; 
    packetCount++;
    EEPROM.write(5,packetCount);
    delay(1000);
  }
