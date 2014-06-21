#include <LiquidCrystal.h>
#include <OneWire.h>

#define AUTOSCAN
#define INTERVAL 10000UL
#define MAX_SENSORS 3

struct sensor_data {
  byte addr[8];
  byte firstData;
  float cur;
  float high;
  float low;
};

const int sensorPin = A0;    // select the input pin for the potentiometer
const int displLedPin = 10;

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);
OneWire ds(2);

int numSensors = 0;
int selSensor = 0;
int scanSensor = 0;
unsigned long previousMillis = 0;

#ifdef AUTOSCAN
static struct sensor_data sensor[MAX_SENSORS];
#else
static struct sensor_data sensor[] =
{
  {{ 0x28, 0x21, 0x5C, 0xDA, 0x05, 0x00, 0x00, 0x04 }, 1, .0, .0, .0 }
};
#endif


static void display_off(void)
{
  digitalWrite(displLedPin, LOW);
}

static void display_on(void)
{
  digitalWrite(displLedPin, HIGH);
}

static void display_setup(void)
{
  pinMode(displLedPin, OUTPUT);
  display_on();
  lcd.clear();
  lcd.begin(16, 2);
}

static void display_update(int sensor_id)
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

static void sensor_setup(void)
{
#ifdef AUTOSCAN
  struct sensor_data *pS;

  numSensors = 0;
  while (numSensors < MAX_SENSORS) {
    int found;
    pS = &sensor[numSensors];
    found = ds.search(pS->addr);
    if (found) {
      byte id = pS->addr[0];
      // only temp sensors
      if (id != 0x10 && id != 0x28 && id != 0x22)
        continue;

      pS->firstData = 1;
      numSensors++;
    } else {
      break;
    }
  }
#else
  numSensors = sizeof(sensor) / sizeof(sensor[0]);
#endif
}


void setup()
{
  display_setup();
  sensor_setup();
  Serial.begin(9600);
  previousMillis = millis();
}

void loop()
{
  byte i;
  byte present = 0;
  byte data[12];
  float celsius, fahrenheit;
  int buttonVal;
  struct sensor_data *pS;

  buttonVal = analogRead(sensorPin);
  if (buttonVal < 850) {
    previousMillis = millis();
    display_on();
    selSensor = (selSensor + 1) % numSensors;
  }
  if (millis() - previousMillis > INTERVAL) {
    previousMillis = millis();
    display_off();
  }

  display_update(selSensor);

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
  if (pS->addr[0] == 0x10) {
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
}

