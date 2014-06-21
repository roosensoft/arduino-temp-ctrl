#include <LiquidCrystal.h>
#include <OneWire.h>

#define INTERVAL 10000UL
#define MAX_SENSORS 3

struct sensor_data {
  byte addr[8];
  float cur;
  float high;
  float low;
  byte firstData;
};

const int sensorPin = A0;    // select the input pin for the potentiometer
const int displLedPin = 10;

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);
OneWire ds(2);

int firstScan = 1;
int numSensors = 0;
int selSensor = 0;
int scanSensor = 0;
unsigned long previousMillis = 0;

struct sensor_data sensor[MAX_SENSORS];


static void displayOff(void)
{
  digitalWrite(displLedPin, LOW);
}

static void displayOn(void)
{
  digitalWrite(displLedPin, HIGH);
}

static void displaySetup(void)
{
  pinMode(displLedPin, OUTPUT);
  displayOn();
}

static void search_sensors(void)
{
  numSensors = 0;
  while (numSensors < MAX_SENSORS) {
    if (ds.search(sensor[numSensors].addr)) {
      sensor[numSensors].firstData = 1;
      numSensors++;
    } else {
      break;
    }
  }
}

static void updateDisplay(int sensor_id)
{
  struct sensor_data *pS = &sensor[sensor_id];

  lcd.setCursor(0, 0);
  lcd.print("Sensor");
  lcd.print(sensor_id);
  lcd.print(": ");
  lcd.print(pS->cur);
  lcd.print((char)223); // °
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(pS->low);
  lcd.print((char)223); // °
  lcd.print("C  ");
  lcd.print(pS->high);
  lcd.print((char)223); // °
  lcd.print("C");
}

void setup()
{
  firstScan = 1;

  displaySetup();
  Serial.begin(9600);
  lcd.clear();
  lcd.begin(16, 2);
  previousMillis = millis();

  search_sensors();
}

void loop()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  float celsius, fahrenheit;
  int buttonVal;
  struct sensor_data *pS;

  buttonVal = analogRead(sensorPin);
  if (buttonVal < 850) {
    previousMillis = millis();
    displayOn();
    selSensor = (selSensor + 1) % MAX_SENSORS;
  }
  if (millis() - previousMillis > INTERVAL) {
    previousMillis = millis();
    displayOff();
  }

  updateDisplay(selSensor);

  if (!numSensors)
    return;

  pS = &sensor[scanSensor % numSensors];
  
  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(pS->addr[i], HEX);
  }

  if (OneWire::crc8(pS->addr, 7) != pS->addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (pS->addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(pS->addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(pS->addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");

  if (pS->firstData) {
    pS->firstData = 0;
    pS->high = celsius;
    pS->low = celsius;
  }
  if (celsius > pS->high) {
    pS->high = celsius;
  }
  if (celsius < pS->low) {
    pS->low = celsius;
  }
  pS->cur = celsius;

  scanSensor++;

  firstScan = 0;
}

