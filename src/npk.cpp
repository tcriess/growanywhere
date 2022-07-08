#include "Arduino.h"

#define Modbus Serial2

const byte nitro[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

byte values[11];

byte nitrogen()
{
  delay(10);
  if (Modbus.write(nitro, sizeof(nitro)) == 8)
  {
    for (byte i = 0; i < 7; i++)
    {
      // Serial.print(Modbus.read(),HEX);
      values[i] = Modbus.read();
      // Serial.print(values[i],HEX);
    }
//    Serial.println();
  }
  return values[4];
}

byte phosphorous()
{
  delay(10);
  if (Modbus.write(phos, sizeof(phos)) == 8)
  {
    for (byte i = 0; i < 7; i++)
    {
      // Serial.print(Modbus.read(),HEX);
      values[i] = Modbus.read();
      // Serial.print(values[i],HEX);
    }
//    Serial.println();
  }
  return values[4];
}

byte potassium()
{
  delay(10);
  if (Modbus.write(pota, sizeof(pota)) == 8)
  {
    for (byte i = 0; i < 7; i++)
    {
      // Serial.print(Modbus.read(),HEX);
      values[i] = Modbus.read();
      // Serial.print(values[i],HEX);
    }
//    Serial.println();
  }
  return values[4];
}
