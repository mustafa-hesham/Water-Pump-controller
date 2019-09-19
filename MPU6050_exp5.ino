
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
//#endif
int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}

int I2C_ClearBusLoop() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(1500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
void Mainscreen ();
void Unitscreen ();
void Pots ();
void chageunits ();
void buttonstatus ();
void rightleft ();
void movecursor ();
void MainscreenBar ();
void MainscreenPsi ();
void buttonstatuslong ();
void updown ();
void setpressure ();
void setpressurebar ();
void setpressurepsi ();
void setlowerprbar ();
void sethigherbar ();
void setlowerprpsi ();
void sethigherpsi ();
void setlowerprpsi ();
void calibration ();
void calibrating ();
void setupcalibration ();
void unit2 ();
void unit7 ();
void pressurectrl ();
void lowpressure();
void resetting ();
void defaults ();
void setangle ();
void angle ();
void door ();
void pumprun ();

int bluetoothTx = A0;  // TX-O pin of bluetooth mate, Arduino A1
int bluetoothRx = A1;  // RX-I pin of bluetooth mate, Arduino A0
int dataFromBt;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float x;
float pressure;
float actpressure1;
int initialypr2;
float t;
float k;
float r;
float w;
float m;
float y;
int d;
int temp;
int psi;
int potPin1 = A2;    // select the input pin for the potentiometer
int val1 = 0;       // variable to store the value coming from the sensor
int potPin2 = A3;    // select the input pin for the potentiometer
int val2 = 0;       // variable to store the value coming from the sensor
int pushb = 7;
int val3 = 0;
int unit;
int item;
float lowerprbar;
float higherprbar;
int lowerprpsi;
int higherprpsi;
int plus = 1;
int minus = -1;
float actpressurepsi;
int kaddress = 60;
int initialypr2addr = 45;
int firstcalibre;
int firstcalibreaddr = 37;
int actpressure1addr = 50;
int pressureaddr = 4;
int unitaddr = 26;
int lowpsiaddr = 12;
int highpsiaddr = 30;
int lowbar = 10;
int highbar = 11;
int angleaddr = 80;
int relay2 = 10;
int relay = 9;
float ps, pm , pk , pq , pu;
float aroudlowerprpsi;
char pumpstatus[] = "OFF";


byte menuCursor[8] = {
  B01000, //  *
  B00100, //   *
  B00010, //    *
  B00001, //     *
  B00010, //    *
  B00100, //   *
  B01000, //  *
  B00000  //
};

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
#define LED_PIN 13
bool blinkState = false;

void setup() {
    Serial.begin(38400);
int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } else { // bus clear
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }
  Serial.println("setup finished");


  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
  pinMode (relay, OUTPUT);
  pinMode (relay2, OUTPUT);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay, HIGH);



  lcd.createChar(0, menuCursor);
  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(pushb, INPUT);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Water pump");
  lcd.setCursor(3, 1);
  lcd.print("controller");
  delay (1000);
  EEPROM.get(angleaddr, x);
  //EEPROM.get(highbar, higherprbar);
  //EEPROM.get(lowbar, lowerprbar);
  EEPROM.get(lowpsiaddr, lowerprpsi);
  EEPROM.get(highpsiaddr, higherprpsi);
  EEPROM.get(unitaddr, unit);
  EEPROM.get(kaddress, k);
  EEPROM.get(initialypr2addr, initialypr2);
  EEPROM.get(firstcalibreaddr, firstcalibre);
  //EEPROM.get(actpressure1addr, actpressure1);
  //EEPROM.get(pressureaddr, pressure);

  higherprbar = higherprpsi / 14.5;
  lowerprbar = lowerprpsi / 14.5;
  if (lowerprbar <= 0) {
    lowerprbar = 1;
  }

  if (unit != 2 && unit != 7) {
    unit = 7;
  }

  if (x > 600 || x < 20  || x <= 0) {
    x = 95;
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Angle is set to");
    lcd.setCursor(0, 2);
    lcd.print("default  value");
    delay (2000);
    lcd.clear();
  }
 
  setupcalibration ();
  aroudlowerprpsi = lowerprpsi - 10;
  lcd.clear();
}

void loop() {
  
  dataFromBt = bluetooth.read();
  // blink LED to indicate activity
  blinkState = !blinkState;
  
  digitalWrite(LED_PIN, blinkState);
  

  aroudlowerprpsi = lowerprpsi - 10;
  
  Mainscreen ();


  d = 300;

  for (int aya = 1 ; aya <= d ; aya++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    pressure = (ax - initialypr2) / x;
    m = pressure + k;
    y += m;
  }
  t = y / d;
  y = 0;
  m = 0;
  if (t < 0) {
    t = 0;
  }


  psi = t * 14.5;

  
  if (relay2 == LOW) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(14, 0);
    lcd.print("ON");
    pumpstatus[1] = 'N';
    pumpstatus[2] = 0;
  }
  if (relay2 == HIGH) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(13, 0);
    lcd.print("OFF");
    pumpstatus[1] = 'F';
    pumpstatus[2] = 'F';
  }
  
  temp = (accelgyro.getTemperature() / 340.) + 36.53;

  
  pressurectrl ();
  door ();
  pumprun ();
  

  //Serial.print(t, 1); Serial.println(" Bar");
  //Serial.print(t * 14.5 , 1); Serial.println(" PSI");
  //Serial.print (temp);
  //Serial.println(" C");
  //Serial.print (ax);
  //Serial.println(" X axis");
  //Serial.println("-----");
  
  bluetooth.print(temp);
  bluetooth.print(" C");
  bluetooth.print(",");
  bluetooth.print(psi);
  bluetooth.print(" Psi");
  bluetooth.print(",");
  bluetooth.print(t);
  bluetooth.print(" Bar");
  bluetooth.print(",");
  bluetooth.print(pumpstatus);
  bluetooth.print(";");


  Pots ();
  
  int rtn = I2C_ClearBusLoop(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } else { // bus clear
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }
}

void Mainscreen () {
  EEPROM.put(unitaddr, unit);
  switch (unit) {
    case 7:
      MainscreenPsi ();
    case 2:
      MainscreenBar ();
  }
  if (val3 == HIGH) {
    Unitscreen ();
  }

}

void MainscreenBar () {
  if (unit == 7) {
    return;
  }
  if (relay2 == LOW) {
    lcd.clear();
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(14, 0);
    lcd.print("ON");
    pumpstatus[1] = 'N';
    pumpstatus[2] = 0;
  }
  if (relay2 == HIGH) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(13, 0);
    lcd.print("OFF");
    pumpstatus[1] = 'F';
    pumpstatus[2] = 'F';
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(temp);
  lcd.setCursor(2, 0);
  lcd.print((char)223);
  lcd.setCursor(3, 0);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(t);
  lcd.setCursor(5, 1);
  lcd.print("Bar");
}

void MainscreenPsi () {
  if (unit == 2) {
    return;
  }
  if (relay2 == LOW) {
    lcd.clear();
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(14, 0);
    lcd.print("ON");
    pumpstatus[1] = 'N';
    pumpstatus[2] = 0;
  }
  if (relay2 == HIGH) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(13, 0);
    lcd.print("OFF");
    pumpstatus[1] = 'F';
    pumpstatus[2] = 'F';
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(temp);
  lcd.setCursor(2, 0);
  lcd.print((char)223);
  lcd.setCursor(3, 0);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("        ");
  lcd.setCursor(0, 1);
  lcd.print(psi);
  lcd.setCursor(3, 1);
  lcd.print("Psi");
}

void Unitscreen () {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Pressure  unit");
  delay (1500);
  updown ();
  if (val2 < 400) {
    setangle ();
  }
  updown ();
  if (val2 > 600) {
    setpressure ();
  }

  buttonstatuslong ();
  if (val3 == HIGH) {
    chageunits ();
  }

  else {
    lcd.clear();
    return;
  }
}


void chageunits () {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Choose  unit");
  lcd.setCursor(3, 1);
  lcd.print("Bar");
  lcd.setCursor(8, 1);
  lcd.print("Psi");
  movecursor ();
}
void setpressure () {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Set Pressure");
  delay (1500);
  updown ();
  if (val2 < 400) {
    Unitscreen ();
  }
  updown ();
  if (val2 > 600) {
    calibration ();
  }
  buttonstatuslong ();
  if (val3 == HIGH) {
    setpressurebar ();
  }
  else {
    lcd.clear();
    return;
  }
}

void calibration () {
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Calibration");
  delay (1500);
  updown ();
  if (val2 < 400) {
    setpressure ();
  }
  updown ();
  if (val2 > 600) {
    resetting ();
  }
  buttonstatuslong ();
  if (val3 == HIGH) {
    calibrating ();
  }
  else {
    lcd.clear();
    return;
  }
}

void resetting () {
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Reset to");
  lcd.setCursor(4, 1);
  lcd.print("defaults");
  delay (1500);
  updown ();
  if (val2 < 400) {
    calibration ();
  }
  updown ();
  if (val2 > 600) {
    setangle ();
  }
  buttonstatuslong ();
  if (val3 == HIGH) {
    defaults ();
  }
  else {
    lcd.clear();
    return;
  }

}

void defaults () {
  lcd.clear();
  firstcalibre = 0;
  k = 0;
  initialypr2 = 0;
  actpressure1 = 0;
  pressure = 0;
  //higherprbar = 0;
  //lowerprbar =0;
  lowerprpsi = 55;
  higherprpsi = 65;
  x = 95;
  EEPROM.put(angleaddr, x);
  EEPROM.put(firstcalibreaddr, firstcalibre);
  EEPROM.put(kaddress, k);
  EEPROM.put(initialypr2addr, initialypr2);
  EEPROM.put(actpressure1addr, actpressure1);
  //EEPROM.put(pressureaddr, pressure);
  //EEPROM.put(highbar, higherprbar);
  //EEPROM.put(lowbar, lowerprbar);
  EEPROM.put(lowpsiaddr, lowerprpsi);
  EEPROM.put(highpsiaddr, higherprpsi);
  lcd.setCursor(3, 0);
  lcd.print("Resetting");
  lcd.setCursor(3, 1);
  lcd.print("complete!");
  delay (2000);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Restart device");
  lcd.setCursor(0, 1);
  lcd.print("to take  effect");
  delay (4000);
  return;
}

void setangle () {

  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Set  angle");
  delay (1500);
  updown ();
  if (val2 < 400) {
    resetting ();
  }
  updown ();
  if (val2 > 600) {
    Unitscreen ();
  }
  buttonstatuslong ();
  if (val3 == HIGH) {
    angle ();
  }
  else {
    lcd.clear();
    return;
  }

}

void angle () {
  
   if (x > 600 || x < 20  || x <= 0) {
    x = 95;
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Angle is set to");
    lcd.setCursor(0, 2);
    lcd.print("default  value");
    delay (2000);
    lcd.clear();
  }
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Enter  angle");

  for (int pw = 1 ; pw <= 600 ; pw++) {
    lcd.setCursor(5, 1);
    lcd.print(x);
    delay (600);
    lcd.setCursor(5, 1);
    lcd.print("      ");
    updown ();
    if (val2 < 400) {
      x = x + plus;
      if (x > 320 || x < 20) {
        x = 95;
      }
      lcd.setCursor(5, 1);
      lcd.print(x);
    }
    delay (200);
    updown ();
    if (val2 > 600) {
      x = x + minus;
      if (x > 320 || x < 20) {
        x = 95;
      }
      lcd.setCursor(5, 1);
      lcd.print(x);
    }
    buttonstatuslong ();
    if (val3 == HIGH) {
      pw = 301;
    }
  }
  EEPROM.put(angleaddr, x);
  lcd.clear();
  lcd.setCursor(6 , 0);
  lcd.write("Saved");
  delay (1000);
  lcd.clear();
  (Mainscreen);
}

void Pots () {
  val1 = analogRead(potPin1);
  val2 = analogRead(potPin2);
  val3 = digitalRead(pushb);
}

void buttonstatus () {
  for (int b = 1; b <= 10000; b++) {
    val3 = digitalRead(pushb);
    if (val3 == HIGH) {
      b = 10001;
    }
  }
}

void buttonstatuslong () {
  for (int lb = 1; lb <= 30000; lb++) {
    val3 = digitalRead(pushb);
    if (val3 == HIGH) {
      lb = 30001;
    }
  }
}

void rightleft () {

  for (int pots = 1; pots <= 10000; pots++) {
    val1 = analogRead(potPin1);
    if (val1 < 400 || val1 > 600) {
      pots = 15001;
    }
  }
}

void movecursor () {

  lcd.setCursor(unit, 1);
  lcd.write(byte(0));
  for (int o = 1; o <= 60; o++) {

    switch (unit) {

      case 7:
        unit7 ();

      case 2:
        unit2 ();

    }

    delay (1500);
    buttonstatus ();
    if (val3 == HIGH) {
      o = 61;
    }
  }

  lcd.clear();
  lcd.setCursor(6 , 0);
  lcd.write("Saved");
  delay (1000);
  lcd.clear();
  (Mainscreen);
}



void setpressurebar () {
  switch (unit) {
    case 2:
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("H");
      lcd.setCursor(7, 0);
      lcd.print(higherprbar);
      lcd.setCursor(5, 1);
      lcd.print("L");
      lcd.setCursor(7, 1);
      lcd.print(lowerprbar);
      delay (5000);
      buttonstatuslong ();
      if (val3 == HIGH) {
        setlowerprbar ();
        sethigherbar ();
      }
      break;
    case 7:
      lcd.clear();
      lcd.setCursor(5, 0);
      lcd.print("H");
      lcd.setCursor(7, 0);
      lcd.print(higherprpsi);
      lcd.setCursor(5, 1);
      lcd.print("L");
      lcd.setCursor(7, 1);
      lcd.print(lowerprpsi);
      delay (5000);
      buttonstatuslong ();
      if (val3 == HIGH) {
        setlowerprpsi ();
        sethigherpsi ();
      }
      break;
  }

  lcd.clear();
  lcd.setCursor(6 , 0);
  lcd.write("Saved");
  delay (1000);
  lcd.clear();
  (Mainscreen);
}

void updown () {
  for (int upots = 1; upots <= 10000; upots++) {
    val2 = analogRead(potPin2);
    if (val2 < 400 || val2 > 600) {
      upots = 15001;
    }
  }

}

void setlowerprbar () {

  for (int lw = 1 ; lw <= 30 ; lw++) {
    lcd.setCursor(7, 1);
    lcd.print(lowerprbar);
    delay (300);
    lcd.setCursor(7, 1);
    lcd.print("      ");
    updown ();
    if (val2 < 400) {
      lowerprbar = lowerprbar + plus;
      lcd.setCursor(7, 1);
      lcd.print(lowerprbar);
    }
    delay (200);
    updown ();
    if (val2 > 600) {
      lowerprbar = lowerprbar + minus;
      if (lowerprbar > 13 || lowerprbar < 0) {
        lowerprbar = 0;
      }
      lcd.setCursor(7, 1);
      lcd.print(lowerprbar);
      lowerprpsi = lowerprbar * 14.5;
    }
    buttonstatuslong ();
    if (val3 == HIGH) {
      lw = 31;
    }
  }
}

void sethigherbar () {
  lcd.setCursor(7, 1);
  lcd.print(lowerprbar);
  //EEPROM.put(lowbar, lowerprbar);
  EEPROM.put(lowpsiaddr, lowerprpsi);
  for (int lw = 1 ; lw <= 30 ; lw++) {
    lcd.setCursor(7, 0);
    lcd.print(higherprbar);
    delay (300);
    lcd.setCursor(7, 0);
    lcd.print("      ");
    updown ();
    if (val2 < 400) {
      higherprbar = higherprbar + plus;
      lcd.setCursor(7, 0);
      lcd.print(higherprbar);
    }
    delay (200);
    updown ();
    if (val2 > 600) {
      higherprbar = higherprbar + minus;
      if (higherprbar > 13 || higherprbar < 0) {
        higherprbar = 0;
      }
      lcd.setCursor(7, 0);
      lcd.print(higherprbar);
      higherprpsi = higherprbar * 14.5;
    }
    buttonstatuslong ();
    if (val3 == HIGH) {
      //EEPROM.put(highbar, higherprbar);
      EEPROM.put(highpsiaddr, higherprpsi);
      lw = 31;
    }

  }
}

void setlowerprpsi () {

  for (int pw = 1 ; pw <= 300 ; pw++) {
    lcd.setCursor(7, 1);
    lcd.print(lowerprpsi);
    delay (300);
    lcd.setCursor(7, 1);
    lcd.print("      ");
    updown ();
    if (val2 < 400) {
      lowerprpsi = lowerprpsi + plus;
      lcd.setCursor(7, 1);
      lcd.print(lowerprpsi);
    }
    delay (200);
    updown ();
    if (val2 > 600) {
      lowerprpsi = lowerprpsi + minus;
      if (lowerprpsi > 180 || lowerprpsi < 0) {
        lowerprpsi = 0;
      }
      lcd.setCursor(7, 1);
      lcd.print(lowerprpsi);
      lowerprbar = lowerprpsi / 14.5;
    }
    buttonstatuslong ();
    if (val3 == HIGH) {
      pw = 301;
    }

  }
}

void sethigherpsi () {
  lcd.setCursor(7, 1);
  lcd.print(lowerprpsi);
  EEPROM.put(lowpsiaddr, lowerprpsi);
  for (int pw = 1 ; pw <= 300 ; pw++) {
    lcd.setCursor(7, 0);
    lcd.print(higherprpsi);
    delay (300);
    lcd.setCursor(7, 0);
    lcd.print("      ");
    updown ();
    if (val2 < 400) {
      higherprpsi = higherprpsi + plus;
      if (higherprpsi > 180 || higherprpsi < lowerprpsi) {
        higherprpsi = lowerprpsi;
      }
      lcd.setCursor(7, 0);
      lcd.print(higherprpsi);
    }
    delay (200);
    updown ();
    if (val2 > 600) {
      higherprpsi = higherprpsi + minus;
      lcd.setCursor(7, 0);
      lcd.print(higherprpsi);
      higherprbar = higherprpsi / 14.5;
    }
    buttonstatuslong ();
    if (val3 == HIGH) {
      EEPROM.put(highpsiaddr, higherprpsi);
      pw = 301;
    }
  }
}

void calibrating () {
  r = 0;
  lcd.setCursor(1, 0);
  lcd.print("Enter pressure");
  lcd.setCursor(5, 1);
  lcd.print("in Psi");
  delay (2000);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Enter pressure");
  
  for (int pw = 1 ; pw <= 300 ; pw++) {
    lcd.setCursor(7, 1);
    lcd.print(actpressurepsi);
    delay (300);
    lcd.setCursor(7, 1);
    lcd.print("      ");
    updown ();
    if (val2 < 400) {
      actpressurepsi = actpressurepsi + plus;
      lcd.setCursor(7, 1);
      lcd.print(actpressurepsi);
      actpressure1 = actpressurepsi / 14.5;
    }
    delay (200);
    updown ();
    if (val2 > 600) {
      actpressurepsi = actpressurepsi + minus;
      lcd.setCursor(7, 1);
      lcd.print(actpressurepsi);
      actpressure1 = actpressurepsi / 14.5;

    }
    buttonstatuslong ();
    if (val3 == HIGH) {
      pw = 301;
    }
  }
   for (int q = 1; q <= 600; q++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    r += ax;
  }
  w = r / 600;
  r = 0;
  initialypr2 = w;
  pressure = (w - initialypr2) / x;
  k = actpressure1 - pressure;
  EEPROM.put(kaddress, k);
  EEPROM.put(initialypr2addr, initialypr2);
  EEPROM.put(actpressure1addr, actpressure1);
  //EEPROM.put(pressureaddr, pressure);
  lcd.clear();
  lcd.setCursor(6 , 0);
  lcd.write("Saved");
  delay (1000);
  lcd.clear();
  (Mainscreen);
}

void setupcalibration () {
  if (firstcalibre != 0) {
    return;
  }
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Calibrating");
  lcd.setCursor(2, 1);
  lcd.print("Pressure = 0");
  delay (1500);
  for (int s = 1; s <= 600; s++) {

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    r += ax;
  }
  w = r / 600;
  r=0;
  //Serial.println ("Enter current pressure ");
  //while (Serial.available() == 0);
  //float actpressure1 = Serial.parseFloat();
  initialypr2 = w;
  pressure = (w - initialypr2) / x;
  k = actpressure1 - pressure;
  EEPROM.put(kaddress, k);
  EEPROM.put(initialypr2addr, initialypr2);
  EEPROM.put(actpressure1addr, actpressure1);
  EEPROM.put(pressureaddr, pressure);
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Set Pressure");
  delay (2500);
  lcd.clear();
  setpressurebar ();
  aroudlowerprpsi = lowerprpsi - 10;
  firstcalibre = 1;
  EEPROM.put(firstcalibreaddr, firstcalibre);
}

void unit7 () {
  rightleft ();
  if (val1 < 400 || val1 > 600) {
    lcd.setCursor(unit , 1);
    lcd.write(" ");
    item = 2;
    unit = 2;
    lcd.setCursor(item, 1);
    lcd.write(byte(0));
    delay (500);
    rightleft ();
    if (val1 < 400 || val1 > 600) {
      lcd.setCursor(item , 1);
      lcd.write(" ");
      item = 7;
      unit = 7;
      lcd.setCursor(item, 1);
      lcd.write(byte(0));
    }

  }
}

void unit2 () {

  rightleft ();
  if (val1 < 400 || val1 > 600) {
    lcd.setCursor(unit , 1);
    lcd.write(" ");
    item = 7;
    unit = 7;
    lcd.setCursor(item, 1);
    lcd.write(byte(0));
  }
  delay (500);
  rightleft ();
  if (val1 < 400 || val1 > 600) {
    lcd.setCursor(item , 1);
    lcd.write(" ");
    item = 2;
    unit = 2;
    lcd.setCursor(item, 1);
    lcd.write(byte(0));
  }
}


void pressurectrl () {
  if (relay2 == LOW) {
    lcd.clear();
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(14, 0);
    lcd.print("ON");
    pumpstatus[1] = 'N';
    pumpstatus[2] = 0;
  }
  if (relay2 == HIGH) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(13, 0);
    lcd.print("OFF");
    pumpstatus[1] = 'F';
    pumpstatus[2] = 'F';
  }


  if (psi > aroudlowerprpsi && psi <= lowerprpsi) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    digitalWrite(relay2, LOW);
    lcd.setCursor(14, 0);
    lcd.print("ON");
    pumpstatus[1] = 'N';
    pumpstatus[2] = 0;
  }
  if (psi > higherprpsi || psi < aroudlowerprpsi) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    digitalWrite(relay2, HIGH);
    lcd.setCursor(13, 0);
    lcd.print("OFF");
    pumpstatus[1] = 'F';
    pumpstatus[2] = 'F';
  }
  if (psi < aroudlowerprpsi) {
    lowpressure();
  }
}

void lowpressure() {
  if (psi > aroudlowerprpsi) {
    return;
  }
  if (relay2 == LOW) {
    lcd.clear();
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(14, 0);
    lcd.print("ON");
    pumpstatus[1] = 'N';
    pumpstatus[2] = 0;
  }
  if (relay2 == HIGH) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    lcd.setCursor(13, 0);
    lcd.print("OFF");
    pumpstatus[1] = 'F';
    pumpstatus[2] = 'F';
  }
  temp = (accelgyro.getTemperature() / 340.) + 36.53;
  lcd.setCursor(0, 0);
  lcd.print(temp);
  lcd.setCursor(2, 0);
  lcd.print((char)223);
  lcd.setCursor(3, 0);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("             ");
  lcd.setCursor(0, 1);
  lcd.print("Low pressure!");

  for (int lp = 1 ; lp <= 100 ; lp++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    pressure = (ax - initialypr2) / x;
    m = pressure + k;
    y += m;
  }
  ps = y / 100;
  y = 0;
  m = 0;
  if (ps < 0) {
    ps = 0;
  }
  pm = ps * 14.5;
  ps = 0;
  delay (2000);
  temp = (accelgyro.getTemperature() / 340.) + 36.53;
  lcd.setCursor(0, 0);
  lcd.print(temp);
  lcd.setCursor(2, 0);
  lcd.print((char)223);
  lcd.setCursor(3, 0);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("Low pressure!");
  y = 0;
  for (int lp = 1 ; lp <= 100 ; lp++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    pressure = (ax - initialypr2) / x;
    m = pressure + k;
    y += m;
  }
  pk = y / 100;
  y = 0;
  m = 0;
  if (pk < 0) {
    pk = 0;
  }
  pq = pk * 14.5;
  pk = 0;
  pu = pq - pm;

  if (pu > 3) {
    lcd.setCursor(13, 0);
    lcd.print("   ");
    digitalWrite(relay2, LOW);
    lcd.setCursor(14, 0);
    lcd.print("ON");
    pumpstatus[1] = 'N';
    pumpstatus[2] = 0;
    delay (10000);
  }

}

void door () {

  if (dataFromBt == '1') {
    digitalWrite(relay, LOW);
    delay (1000);
    digitalWrite(relay, HIGH);
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("DOOR");
    lcd.setCursor(6, 1);
    lcd.print("OPEN");
    delay (2000);
    lcd.clear();
  }

}

void pumprun () {

  if (dataFromBt == '2') {
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("PUMP IS");
    lcd.setCursor(4, 1);
    lcd.print("RUNNING");
    digitalWrite(relay2, LOW);
    delay (10000);
    digitalWrite(relay2, HIGH);
    lcd.clear();
  }

}


