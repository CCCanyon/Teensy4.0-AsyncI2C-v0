#include "AsyncI2C.h"

// test with VL53L0X
#define ADDRESS_DEFAULT 0b0101001
#define IDENTIFICATION_MODEL_ID 0xC0
#define RESULT_RANGE_STATUS 0x14
uint32_t counter = 0;
uint32_t now = 0;

void usr();

void setup()
{
  Wire.begin();
  Wire.setClock(400000);

  AsyncWire.begin();
  AsyncWire.USR(usr);
  AsyncWire.bidask(ADDRESS_DEFAULT, IDENTIFICATION_MODEL_ID, 1);
  syncAsk();
}
void loop()
{
  if(micros() - now > 2000000)
  {
    now = micros();
    Serial.print("\n\n"); Serial.print(counter++); Serial.print(":");
    syncAsk();
    AsyncWire.bidask(ADDRESS_DEFAULT, IDENTIFICATION_MODEL_ID, 1);
  }
}

void usr()
{
  uint8_t v = AsyncWire.read<uint8_t>();
  Serial.print("usr:"); Serial.println(v, HEX); // 0xEE
}




void syncAsk()
{
  Wire.beginTransmission(ADDRESS_DEFAULT);
  Wire.write(IDENTIFICATION_MODEL_ID);
  Wire.endTransmission();

  Wire.requestFrom(ADDRESS_DEFAULT, 1);
  uint8_t id = Wire.read();
  Serial.print("\nsync:"); Serial.println(id, HEX); // 0xEE
}
