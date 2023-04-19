/* 
 Tracker based upon:
 - arduino nano
 - GPS module to recover longitude, latitude, altitude, date and time
 - Epheride library that provides sunrise, sundown, azimuth of sun according to longitude, latitude, altitude, date and time
 - Compass module to recover the azimuth of the tracker
 - Computation of the movement of tracker to determine where to go
 - 2 relays module to send 220V to turn the motor to go to Est or West according previous computation

 Tricks:
 - GPS module is incompatible with arduino delay() and too long Serial.print() as it is needed to pool regulary the serial link of GPS module
 - serialSoftware libray cannot be used. Conflict between I2C of compass and serial link of GPS. Workaround: Usage of AltSoftSerial library.
   https://arduino.stackexchange.com/questions/26240/which-pins-of-an-arduino-uno-can-be-used-as-tx-and-rx-pins-for-connecting-to-gsm#26277
 - Factorization of GPS to recover only once a day the date, time, longitude, latitude, altitude. Computation of time locally on arduino via millis().
 - Usage of arduino watchdog timer to secure freezing potential bug and also to reset the board every day at 3:00 UTC to resynchronize the time with GPS data 
 - A forbiden zone is checked to prevent the tracker to turn arround and to mess the electric wires connected to my house 
   In this zone, a manual intervention is needed to move the tracker and arduino is in permanent reset
 - Be careful with 2 relays module to isolate ground from arduino via a separated power supply on JD VCC
 - Be carefull with electromagnetic interference: Put arduino, GPS and compass into aluminium box
 - Put a capacitor on each 220V wires of the motor to avoid electrical arc on the relays.  
 - Compass need to be calibrated. See calibration sketch to find to min/max values for XYZ.
*/


// Serial libray compatible GPS serial and Compass I2C
#include <AltSoftSerial.h>
// GPS library
#include <TinyGPS.h> 
// QMC5883L Compass Library
#include <QMC5883LCompass.h>
// Ephemeride library
#include <Ephem_Soleil.h>
// watchdog
#include <avr/wdt.h>
// lcd
#include <LiquidCrystal_I2C.h> // Library for LCD in I2C


//#define debug

// LCD variables
LiquidCrystal_I2C lcd(0x27, 16, 2);// Define the type of LCD (addr I2C, number of columns, number of lines) 
byte customBackslash[8] = {
  0b00000,
  0b10000,
  0b01000,
  0b00100,
  0b00010,
  0b00001,
  0b00000,
  0b00000
};

int keepAlive = 0;

int azimuth;
int sunAzimuth;
bool sundownFlag;
char lcdInfo[16];

void setupLcd() {
  lcd.init(); // Initialization of LCD
  lcd.createChar(7, customBackslash); // Create special character backslash
}

// LCD functions
void lcdPrint() {
  char buffer[4];

  lcd.clear();
  lcd.backlight();

  lcd.setCursor(0, 0);
  if (sundownFlag) {
    lcd.print("Nuit S:");
  } 
  else {
    lcd.print("Jour S:");
  }
  
  sprintf(buffer,"%d",sunAzimuth);
  lcd.setCursor(7, 0);
  lcd.print(buffer);

  // Keep alive
  lcd.setCursor(10, 0);
  if (keepAlive == 0) { 
    lcd.print("/");
  }
  else if (keepAlive == 1) {
    lcd.print("-");
  }
  else if (keepAlive == 2) {
    lcd.write(byte(7));
  }
  else if (keepAlive == 3) {
    lcd.print("|");
  }
  else {
    lcd.print("E");
  }
  keepAlive = (keepAlive + 1)%4;
  
  lcd.setCursor(11, 0);
  lcd.print(F("A:"));
  sprintf(buffer,"%d",azimuth);
  lcd.setCursor(13, 0);
  lcd.print(buffer);

  lcd.setCursor(0, 1);
  lcd.print(lcdInfo);
}

// Rtracker=2,5m leads to 15,707m of tracker perimeter for 360 degrees
// Rwheel=0,27m  leads to 1,696m of wheel perimeter
// Rtracker/Rwheel=9,261 wheel revolution per 360 degrees
// d=11 pinion tooth on the motor
// D=132 pinion tooth on the wheel
// D/d=12 motor revolutions for 1 wheel revolution
// Motor speed=3,998 revolution per sec
// Evaluation of seconds of motor for 10 degrees is 8,1 second
//#define TIME_FOR_1_DEGREES_IN_MS 810
#define TIME_FOR_1_DEGREES_IN_MS 600
#define MAX_TIME_OF_MOTOR_IN_MS 30000

QMC5883LCompass compass; // Compass object
#define COMPASS_CORRECTION 0

float lat = 0,lon = 0; // create variable for latitude and longitude object

#define TOLERANCE 5
#define MAX_AZIMUTH 262 
#define MIN_AZIMUTH 60 
#define MIN_DEGREE 5 

#define MYLATITUDE 48.796
#define MYLONGITUDE -3.414
#define MYALTITUDE 50
unsigned long initialTime;
unsigned long resetTime;
float mylatitude = MYLATITUDE, mylongitude = MYLONGITUDE, myaltitude = MYALTITUDE;
int year;
byte month, day;

TinyGPS gps; // create gps object 
AltSoftSerial ss;  // create AltSoftSerial serial link object for GPS module

// Smart functions to print, delay ...
static void smartdelay(unsigned long ms, boolean gpsflag);
static void print_str(const char *str, boolean gpsflag);

// Relay 1 is on/off 220V to relay 2
// Relay 2 is to go Est or West
#define RELAY1 6
#define RELAY2 7

// Motor relay initialization
void setupMotor() {
  // Initialize the relays pins
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  // Relay 1 is off and relay 2 is Est by default
  digitalWrite(RELAY1, HIGH); // Led is off on relay 1
  digitalWrite(RELAY2, HIGH); // Led is off on relay 2
}

void gotoEst(unsigned long degree, unsigned long coolingInMs) 
{
  unsigned long t, d;
  int azimuth;
  int sunAzimuth;
  
  digitalWrite(RELAY2, HIGH);           // Relay 2 led is off means go to Est
  smartdelay(200, false);               // Wait to be sure that relay 2 is set
  digitalWrite(RELAY1, LOW);            // Relay 1 led is on means 220V to relay 2 

  if (degree < MIN_DEGREE) {
    d = MIN_DEGREE;
  }
  else {
    d = degree;
  }
  
  t = d * TIME_FOR_1_DEGREES_IN_MS;
  if (t > MAX_TIME_OF_MOTOR_IN_MS) t = MAX_TIME_OF_MOTOR_IN_MS;

  smartdelay(t, false);                 // Wait ms
  digitalWrite(RELAY1, HIGH);           // Relay 1 led is off means 0V to relay 2
  
  // gps info
  sunAzimuth = getSunAzimuth();
  azimuth = getAzimuth();
  
  smartdelay(coolingInMs, false);
}

void gotoWest(unsigned long degree, unsigned long coolingInMs) 
{
  unsigned long t, d;
  int azimuth;
  int sunAzimuth;
  
  digitalWrite(RELAY2, LOW);            // Relay 2 led is on means go to West
  smartdelay(200, false);               // Wait to be sure that relay 2 is set
  digitalWrite(RELAY1, LOW);            // Relay 1 led is on means 220V to relay 2

  if (degree < MIN_DEGREE) {
    d = MIN_DEGREE;
  }
  else {
    d = degree;
  }
  
  t = d * TIME_FOR_1_DEGREES_IN_MS;
  if (t > MAX_TIME_OF_MOTOR_IN_MS) t = MAX_TIME_OF_MOTOR_IN_MS;

  smartdelay(t, false);                 // Wait ms
  digitalWrite(RELAY1, HIGH);           // Relay 1 led is off means 0V to relay 2
  smartdelay(200, false);               // Wait to be sure that relay 1 is set
  digitalWrite(RELAY2, HIGH);           // Relay 2 led is off means go to Est by default
  
  // gps info
  sunAzimuth = getSunAzimuth();
  azimuth = getAzimuth();
  
  smartdelay(coolingInMs, false);
}

bool gpsInitData() {
  unsigned long age;
  byte hour, minute, second, hundredths;
  float flat, flon, falt;
  char sz[32];

  gps.f_get_position(&flat, &flon, &age);
  if (flat == TinyGPS::GPS_INVALID_F_ANGLE || flon == TinyGPS::GPS_INVALID_F_ANGLE) {
    print_str("Error detected in latitude or longitude...\n", true);
    return true;
  }

  mylatitude = flat;
  mylongitude = flon;
  
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  
  Serial.print(F("Initial date and time: "));
  sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d\n", day, month, year, hour, minute, second);
  Serial.print(sz);

  // Note that age value may be correct while month, day or year might be in failure (Erroneous value seen was month=day=0 and year=2000)
  // Workaround: Check also month, day, year, hour, minute and second boundaries...
  if ((age == TinyGPS::GPS_INVALID_AGE) || (month < 1) || (month > 12) || (day < 1) || (day > 31) || (year < 2023) || (year > 2063) || (hour > 23) || (minute > 59) || (second > 59)) {
    print_str("Error detected in date and time...\n", true);
    return true;
  }
  
  initialTime = (unsigned long)hour*3600000 + (unsigned long)minute*60000 + (unsigned long)second*1000 - (unsigned long)millis();

  if (initialTime <= 3*3600000) {
    resetTime = 3*3600000;
  }
  else {
    resetTime = 27*3600000;
  }
    
  falt = gps.f_altitude();
  if (falt == TinyGPS::GPS_INVALID_F_ALTITUDE) {
    print_str(" Error detected in altitude...\n", true);
    return true;
  }

  myaltitude = flat;

  return false; 
}

unsigned long getTime() {
  unsigned long t;
  t = millis() + initialTime;
  return t;
}

int getSunAzimuth() {
  char sz[32];
  double ha;
  double az;
  String lS, mS, cS;
  byte hr, mn, sec;

  unsigned long t = getTime();

  hr = trunc(t / 3600000);
  mn = trunc((t - hr*3600000) / 60000);
  sec = trunc((t - hr*3600000 - mn*60000) / 1000);
  hr = hr % 24;

  sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d\n", day, month, year, (byte)hr, (byte)mn, (byte)sec);
  
  posSoleil(String(sz), 0, (double)mylatitude, (double)mylongitude, &ha, &az);

  return (int)az;
}

bool isSundown() {
  char sz[32];
  double lS, mS, cS;
  bool ret = false;
  byte hr, mn, sec;

  unsigned long t = getTime();

  hr = trunc(t / 3600000);
  mn = trunc((t - hr*3600000) / 60000);
  sec = trunc((t - hr*3600000 - mn*60000) / 1000);
  hr = hr % 24;

  sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d\n", day, month, year, (byte)hr, (byte)mn, (byte)sec);
  
  lmvSoleil(sz, 0, 0, (double)mylatitude, (double)mylongitude, &lS, &mS, &cS, (int)myaltitude); 

#ifdef debug
  Serial.print(F("isSundown: t="));
  Serial.print(t/60000);
  Serial.print(F(" cS: "));
  Serial.print(cS);
  Serial.print(F(" lS: "));
  Serial.print(lS);
  Serial.print(F("\n"));
#endif

  // cS and lS are in minutes while t is in ms
  if ((t/60000) > cS || (t/60000) < lS) {
    ret = true;
  }

  return ret;
}


void getSunriseSundown(String *sunrise, String *sundown) {
  char sz[32];
  String lS, mS, cS;
  byte hr, mn, sec;

  unsigned long t = getTime();

  hr = trunc(t / 3600000);
  mn = trunc((t - hr*3600000) / 60000);
  sec = trunc((t - hr*3600000 - mn*60000) / 1000);
  hr = hr % 24;

  sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d\n", day, month, year, (byte)hr, (byte)mn, (byte)sec);
  
  lmvSoleil(sz, 0, 0, (double)mylatitude, (double)mylongitude, &lS, &mS, &cS, (int)myaltitude); 

#ifdef debug
  Serial.print(F("Sunrise: "));
  Serial.print(lS);
  Serial.print(F(" Sundown: "));
  Serial.print(cS);
  Serial.print(F("\n"));
#endif

  *sunrise = lS;
  *sundown = cS;
}

// GPS initialization
void setupGps() {
  bool err;
  ss.begin(9600); // connect gps sensor
  do {
    err = gpsInitData();
    smartdelay(1000, true);
  } while(err);
}

int getAzimuth() {
  int az;
  // Read compass values
  compass.read();

  // Return Azimuth reading
  az = compass.getAzimuth() + COMPASS_CORRECTION;
  return az;
}

// Serial initialization
void setupSerial() {
  Serial.begin(9600); // connect serial 
}

// Compass initialization
void setupCompass() {
  // Initialize the Compass.
  compass.init();
  // Calibration values in my house
  //compass.setCalibration(-1475, 1325, -1542, 1350, -1382, 1246);
  // Calibration values in my garden
  compass.setCalibration(-1490, 1360, -1563, 1282, -1437, 1243);
}

// Watchdog initialization
void setupWatchdog() {
  // Initialize the watchdog to 8s.
  wdt_enable(WDTO_8S);
}

// Reset the board
void resetBoard(bool force) {
  unsigned long t = getTime();
  int i;

  // Note that the board should be reseted between 03:00 and 03:15 to recover from GPS the new date and reinitialize the time...
  if (force || (t > resetTime && t < (resetTime + 900000))) {
    // Let the watchdog reset the board
    while(true) {
      i++;
    }
  }
  else {
    wdt_reset();
  }
}

void setup(){ 
  setupSerial();
  setupMotor();
  setupCompass();
  setupGps();
  setupWatchdog();
  setupLcd();
  Serial.print(F("End of tracker setup\n"));
} 

void loop(){
  int delta;
  double expectedAzimuth;
  bool err;
  String sunrise, sundown;
  
  // gps info
  sunAzimuth = getSunAzimuth();

  sundownFlag = isSundown();

  getSunriseSundown(&sunrise, &sundown);

  azimuth = getAzimuth();

  if (sunAzimuth < MIN_AZIMUTH) {
    delta = MIN_AZIMUTH - azimuth;
  }
  else if (sunAzimuth > MAX_AZIMUTH) {
    delta = MAX_AZIMUTH - azimuth;
  }
  else {
    delta = sunAzimuth - azimuth;
  }
  
  if (azimuth > (MAX_AZIMUTH + TOLERANCE) || azimuth < (MIN_AZIMUTH - TOLERANCE)) {
#ifdef debug
    Serial.print(F(" Error: Tracker is out of authorized space. Reset board...\n"));
#endif
    strcpy(lcdInfo, "Tracker error!!!");
    lcdPrint();
    resetBoard(true);
  }
  else if (sundownFlag && (azimuth - (MIN_AZIMUTH + TOLERANCE)) > 0) {
#ifdef debug
    Serial.print(F(" Sundown was at "));
    Serial.print(sundown);
    Serial.print(F(", tracker azimuth "));  
    Serial.print(azimuth);  
    Serial.print(F(", move tracker "));
    Serial.print(azimuth - (MIN_AZIMUTH + TOLERANCE));
    Serial.print(F(" degrees Est to sleep...\n"));
#endif
    strcpy(lcdInfo, "Go Est to sleep");
    lcdPrint();
    gotoEst(azimuth - (MIN_AZIMUTH + TOLERANCE), 600000);
  }
  else if (sundownFlag) {
#ifdef debug
    Serial.print(F("Tracker azimuth "));  
    Serial.print(azimuth);  
    Serial.print(F(", Tracker will sleep until "));
    Serial.print(sunrise);
    Serial.print(F(" ...\n"));
#endif
    strcpy(lcdInfo, "Sleeping...");
    lcdPrint();
  }
  else if (abs(delta) <= TOLERANCE) {
#ifdef debug
    Serial.print(F("Tracker azimuth "));  
    Serial.print(azimuth);  
    Serial.print(F(" is already pointed to sun "));  
    Serial.print(sunAzimuth);  
    Serial.print(F(", don't move tracker...\n"));  
#endif
    strcpy(lcdInfo, "Don't move...");
    lcdPrint();
  }
  else if(delta > 0) {
#ifdef debug
    Serial.print(F("Tracker azimuth: "));
    Serial.print(azimuth);
    Serial.print(F(" Sun azimuth: "));
    Serial.print(sunAzimuth);
    Serial.print(F("; Tracker needs to go "));
    Serial.print(abs(delta));
    Serial.print(F(" degrees West\n"));
#endif
    strcpy(lcdInfo, "Go to West...");
    lcdPrint();
    gotoWest(abs(delta), 1000);
  }
  else {
#ifdef debug
    Serial.print(F("Tracker azimuth: "));
    Serial.print(azimuth);
    Serial.print(F(" Sun azimuth: "));
    Serial.print(sunAzimuth);
    Serial.print(F("; Tracker needs to go "));
    Serial.print(abs(delta));
    Serial.print(F(" degrees Est\n"));
#endif
    strcpy(lcdInfo, "Go to Est...");
    lcdPrint();
    gotoEst(abs(delta), 1000);
  }

  resetBoard(false);
  
  smartdelay(1000, false); 
} 

static void smartdelay(unsigned long ms, boolean gpsflag)
{
  unsigned long start = millis();
  unsigned long lcdprint = start + 2000;
  do 
  {
    if (gpsflag) {
      while (ss.available()) gps.encode(ss.read());
    }
    wdt_reset();
    if (millis() > lcdprint) {
      azimuth = getAzimuth();
      lcdPrint();
      lcdprint = millis() + 2000;
    }
  } while (millis() - start < ms);
}

static void print_str(const char *str, boolean gpsflag)
{
  int slen = strlen(str);

  for (int i=0; i<slen; ++i)
    Serial.print(str[i]);
  smartdelay(0, gpsflag);
}
