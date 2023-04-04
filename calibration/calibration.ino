/*
QMC5883LCompass.h Library Calibration Example Sketch
Learn more at [https://github.com/mprograms/QMC5883LCompass]

Upload this calibration sketch onto your Arduino to provide calibration for your QMC5883L chip.
After upload, run the serial monitor and follow the directions.
When prompted, copy the last line into your project's actual sketch.

===============================================================================================================
Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]
===============================================================================================================











                                          UTILITY SKETCH
                                    NO SERVICABLE PARTS BELOW












*/
#include <QMC5883LCompass.h>
// lcd
#include <LiquidCrystal_I2C.h> // Library for LCD in I2C

QMC5883LCompass compass;
LiquidCrystal_I2C lcd(0x27, 16, 2);// Define the type of LCD (addr I2C, number of columns, number of lines) 

int calibrationData[3][2] = {{32767, -32767}, {32767, -32767}, {32767, -32767}};
bool changed = false;
bool done = false;
unsigned long t = 0;
unsigned long c = 0;

void lcdPrint() {
  char buffer[6];

  lcd.clear();
  lcd.backlight();

  lcd.setCursor(0, 0);
  sprintf(buffer,"%i",calibrationData[0][0]);
  lcd.print(buffer);
  lcd.setCursor(6, 0);
  sprintf(buffer,"%i",calibrationData[0][1]);
  lcd.print(buffer);
  lcd.setCursor(11, 0);
  sprintf(buffer,"%i",calibrationData[1][0]);
  lcd.print(buffer);
  lcd.setCursor(0, 1);
  sprintf(buffer,"%i",calibrationData[1][1]);
  lcd.print(buffer);
  lcd.setCursor(5, 1);
  sprintf(buffer,"%i",calibrationData[2][0]);
  lcd.print(buffer);
  lcd.setCursor(11, 1);
  sprintf(buffer,"%i",calibrationData[2][1]);
  lcd.print(buffer);
}

void setup() {
  Serial.begin(9600);
  compass.init();
  lcd.init(); // Initialization of LCD
  
  Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);
  c = millis();
}

void loop() {
  int x, y, z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  changed = false;

  if(x < calibrationData[0][0]) {
    calibrationData[0][0] = x;
    changed = true;
  }
  if(x > calibrationData[0][1]) {
    calibrationData[0][1] = x;
    changed = true;
  }

  if(y < calibrationData[1][0]) {
    calibrationData[1][0] = y;
    changed = true;
  }
  if(y > calibrationData[1][1]) {
    calibrationData[1][1] = y;
    changed = true;
  }

  if(z < calibrationData[2][0]) {
    calibrationData[2][0] = z;
    changed = true;
  }
  if(z > calibrationData[2][1]) {
    calibrationData[2][1] = z;
    changed = true;
  }

  if (changed && !done) {
    Serial.println("CALIBRATING... Keep moving your sensor around.");
    lcdPrint();
    c = millis();
  }
    t = millis();
  
  
  if ( (t - c > 60000) && !done) {
    done = true;
    Serial.println("DONE. Copy the line below and paste it into your projects sketch.);");
    Serial.println();
      
    Serial.print("compass.setCalibration(");
    Serial.print(calibrationData[0][0]);
    Serial.print(", ");
    Serial.print(calibrationData[0][1]);
    Serial.print(", ");
    Serial.print(calibrationData[1][0]);
    Serial.print(", ");
    Serial.print(calibrationData[1][1]);
    Serial.print(", ");
    Serial.print(calibrationData[2][0]);
    Serial.print(", ");
    Serial.print(calibrationData[2][1]);
    Serial.println(");");
    }
  
 
}
