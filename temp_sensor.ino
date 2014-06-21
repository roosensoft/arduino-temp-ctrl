#include <LiquidCrystal.h>
#include <OneWire.h>

#define CONFIG_SENSOR_AUTOSCAN
#define CONFIG_MAX_SENSORS 3

#define DISPLAY_ON_TIME        20000UL
#define SENSOR_CONVERSION_TIME 1000UL

#define STATE_INIT       0
#define STATE_CONVERTING 1

struct sensor_data {
  byte addr[8];
  byte firstData;
  byte state;
  float cur;
  float high;
  float low;
  unsigned long t;
};

const int button_pin = A0;    // select the input pin for the potentiometer
const int displ_bcklght_pin = 10;

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7);
OneWire ds(2);

static int numSensors = 0;
static int selSensor = 0;
static int scanSensor = 0;

static int display_state;
static unsigned long displ_last_trig_time = 0;

#ifdef CONFIG_SENSOR_AUTOSCAN
static struct sensor_data sensor[CONFIG_MAX_SENSORS];
#else
static struct sensor_data sensor[] =
{
  {{ 0x28, 0x21, 0x5C, 0xDA, 0x05, 0x00, 0x00, 0x04 }, 1, 0, .0, .0, .0, 0 }
};
#endif


static void display_off(void)
{
  digitalWrite(displ_bcklght_pin, LOW);
  display_state = 0;
}

static void display_on(void)
{
  digitalWrite(displ_bcklght_pin, HIGH);
  display_state = 1;
}

static int is_display_on(void)
{
  return display_state;
}

static void display_trigger(void)
{
  display_on();
  displ_last_trig_time = millis();
}

static void display_setup(void)
{
  pinMode(displ_bcklght_pin, OUTPUT);
  lcd.clear();
  lcd.begin(16, 2);
  display_trigger();
}

static void display_process(int sensor_id)
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

  // backlight timeout
  if (is_display_on() && (millis() - displ_last_trig_time) > DISPLAY_ON_TIME) {
    display_off();
  }
}

static void sensor_setup(void)
{
#ifdef CONFIG_SENSOR_AUTOSCAN
  struct sensor_data *pS;

  numSensors = 0;
  while (numSensors < CONFIG_MAX_SENSORS) {
    int found;
    pS = &sensor[numSensors];
    found = ds.search(pS->addr);
    if (found) {
      byte id = pS->addr[0];

      // crc has to be valid
      if (OneWire::crc8(pS->addr, 7) != pS->addr[7])
        continue;

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

static int sensor_start_conversion(struct sensor_data *pS)
{
  if (!ds.reset())
    return -1;

  ds.select(pS->addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  return 0;
}

static void sensor_read_data(struct sensor_data *pS)
{
  byte i;
  byte present;
  byte data[9];
  float celsius;

  present = ds.reset();
  if (present) {
    ds.select(pS->addr);
    ds.write(0xBE);         // Read Scratchpad

    for ( i = 0; i < 9; i++)             // we need 9 bytes
      data[i] = ds.read();

    if (OneWire::crc8(data, 8) == data[8])
    {
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
        if (cfg == 0x00)
          raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
          raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
          raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }
      celsius = (float)raw / 16.0;

      // update data structure
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

      // output to serial
      for (i = 0; i < 8; i++) {
        Serial.print(pS->addr[i], HEX);
        Serial.write(' ');
      }
      for ( i = 0; i < 9; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
}

static void sensor_process(struct sensor_data *pS)
{
  switch (pS->state) {
    case STATE_INIT:
      if (sensor_start_conversion(pS) == 0) {
        pS->t = millis();
        pS->state = STATE_CONVERTING;
      }
      break;
    case STATE_CONVERTING:
      if (millis() - pS->t > SENSOR_CONVERSION_TIME) {
        sensor_read_data(pS);
        pS->state = STATE_INIT;
      }
      break;
    default:
      break;
  }
}

void button_process(void)
{
  int buttonVal;

  buttonVal = analogRead(button_pin);
  if (buttonVal < 850) {
    display_trigger();
    selSensor = (selSensor + 1) % numSensors;
  }
}

void setup()
{
  display_setup();
  sensor_setup();
  Serial.begin(9600);
}

void loop()
{
  struct sensor_data *pS;

  button_process();
  display_process(selSensor);

  if (!numSensors)
    return;

  pS = &sensor[scanSensor % numSensors];
  sensor_process(pS);

  scanSensor++;
}

