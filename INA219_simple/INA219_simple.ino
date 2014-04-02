/*
    L3G4200D Triple Axis Gyroscope. Simple Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-l3g4200d.html
    GIT: https://github.com/jarzebski/Arduino-L3G4200D
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <INA219.h>

INA219 ina;

void checkConfig()
{

  Serial.print("Range: ");
  switch (ina.getRange())
  {
    case INA219_RANGE_16V: Serial.println("16V"); break;
    case INA219_RANGE_32V: Serial.println("32V"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Gain: ");
  switch (ina.getGain())
  {
    case INA219_GAIN_40MV: Serial.println("+/- 40mV"); break;
    case INA219_GAIN_80MV: Serial.println("+/- 80mV"); break;
    case INA219_GAIN_160MV: Serial.println("+/- 160mV"); break;
    case INA219_GAIN_320MV: Serial.println("+/- 3200mV"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus resolution: ");
  switch (ina.getBusRes())
  {
    default: Serial.println("unknown");
  }

  Serial.print("Shunt resolution: ");
  switch (ina.getShuntRes())
  {
    default: Serial.println("unknown");
  }

  Serial.print("Max possible current: ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max possible current: ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current: ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage: ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power: ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
]

void setup() 
{
  Serial.begin(115200);

  Serial.println("Initialize INA219");
  Serial.println("-----------------");

  // Default INA219 address is 0x40
  ina.begin();

  // Configure INA219
  ina.configure(INA219_RANGE_32V, INA219_GAIN_320MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);

  // Calibrate INA219. Rshunt = 0.1 ohm, Max excepted current = 2A
  ina.calibrate(0.1, 2);

  // Check configuration
  checkConfig();
}

void loop()
{
  Serial.print("Bus voltage:   ");
  Serial.print(ina.busVoltage(), 5);
  Serial.println(" V");

  Serial.print("Bus power:     ");
  Serial.print(ina.busPower(), 5);
  Serial.println(" W");


  Serial.print("Shunt voltage: ");
  Serial.print(ina.shuntVoltage(), 5);
  Serial.println(" V");

  Serial.print("Shunt current: ");
  Serial.print(ina.shuntCurrent(), 5);
  Serial.println(" A");

  Serial.println("");
  delay(100);
}
