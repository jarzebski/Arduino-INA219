/*
    INA219 Zero-Drift, Bi-directional Current/Power Monitor. Simple Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/cyfrowy-czujnik-pradu-mocy-ina219.html
    GIT: https://github.com/jarzebski/Arduino-INA219
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <INA219.h>

INA219 ina;

void checkConfig()
{
  Serial.print("Mode:                 ");
  switch (ina.getMode())
  {
    case INA219_MODE_POWER_DOWN:      Serial.println("Power-Down"); break;
    case INA219_MODE_SHUNT_TRIG:      Serial.println("Shunt Voltage, Triggered"); break;
    case INA219_MODE_BUS_TRIG:        Serial.println("Bus Voltage, Triggered"); break;
    case INA219_MODE_SHUNT_BUS_TRIG:  Serial.println("Shunt and Bus, Triggered"); break;
    case INA219_MODE_ADC_OFF:         Serial.println("ADC Off"); break;
    case INA219_MODE_SHUNT_CONT:      Serial.println("Shunt Voltage, Continuous"); break;
    case INA219_MODE_BUS_CONT:        Serial.println("Bus Voltage, Continuous"); break;
    case INA219_MODE_SHUNT_BUS_CONT:  Serial.println("Shunt and Bus, Continuous"); break;
    default: Serial.println("unknown");
  }
  
  Serial.print("Range:                ");
  switch (ina.getRange())
  {
    case INA219_RANGE_16V:            Serial.println("16V"); break;
    case INA219_RANGE_32V:            Serial.println("32V"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Gain:                 ");
  switch (ina.getGain())
  {
    case INA219_GAIN_40MV:            Serial.println("+/- 40mV"); break;
    case INA219_GAIN_80MV:            Serial.println("+/- 80mV"); break;
    case INA219_GAIN_160MV:           Serial.println("+/- 160mV"); break;
    case INA219_GAIN_320MV:           Serial.println("+/- 320mV"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus resolution:       ");
  switch (ina.getBusRes())
  {
    case INA219_BUS_RES_9BIT:         Serial.println("9-bit"); break;
    case INA219_BUS_RES_10BIT:        Serial.println("10-bit"); break;
    case INA219_BUS_RES_11BIT:        Serial.println("11-bit"); break;
    case INA219_BUS_RES_12BIT:        Serial.println("12-bit"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Shunt resolution:     ");
  switch (ina.getShuntRes())
  {
    case INA219_SHUNT_RES_9BIT_1S:    Serial.println("9-bit / 1 sample"); break;
    case INA219_SHUNT_RES_10BIT_1S:   Serial.println("10-bit / 1 sample"); break;
    case INA219_SHUNT_RES_11BIT_1S:   Serial.println("11-bit / 1 sample"); break;
    case INA219_SHUNT_RES_12BIT_1S:   Serial.println("12-bit / 1 sample"); break;
    case INA219_SHUNT_RES_12BIT_2S:   Serial.println("12-bit / 2 samples"); break;
    case INA219_SHUNT_RES_12BIT_4S:   Serial.println("12-bit / 4 samples"); break;
    case INA219_SHUNT_RES_12BIT_8S:   Serial.println("12-bit / 8 samples"); break;
    case INA219_SHUNT_RES_12BIT_16S:  Serial.println("12-bit / 16 samples"); break;
    case INA219_SHUNT_RES_12BIT_32S:  Serial.println("12-bit / 32 samples"); break;
    case INA219_SHUNT_RES_12BIT_64S:  Serial.println("12-bit / 64 samples"); break;
    case INA219_SHUNT_RES_12BIT_128S: Serial.println("12-bit / 128 samples"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Max possible current: ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current:          ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage:    ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power:            ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
}

void setup() 
{
  Serial.begin(115200);

  Serial.println("Initialize INA219");
  Serial.println("-----------------------------------------------");

  // Default INA219 address is 0x40
  ina.begin();

  // Configure INA219
  ina.configure(INA219_RANGE_32V, INA219_GAIN_320MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);

  // Calibrate INA219. Rshunt = 0.1 ohm, Max excepted current = 2A
  ina.calibrate(0.1, 2);

  // Display configuration
  checkConfig();

  Serial.println("-----------------------------------------------");
}

void loop()
{
  Serial.print("Bus voltage:   ");
  Serial.print(ina.readBusVoltage(), 5);
  Serial.println(" V");

  Serial.print("Bus power:     ");
  Serial.print(ina.readBusPower(), 5);
  Serial.println(" W");


  Serial.print("Shunt voltage: ");
  Serial.print(ina.readShuntVoltage(), 5);
  Serial.println(" V");

  Serial.print("Shunt current: ");
  Serial.print(ina.readShuntCurrent(), 5);
  Serial.println(" A");

  Serial.println("");
  delay(1000);
}
