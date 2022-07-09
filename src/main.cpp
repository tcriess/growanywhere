#include <Arduino.h>
#include <M5Stack.h>
#include <M5_DLight.h>
#include <TCA9548A.h>
#include "M5_ENV.h"
#include "npk.h"

uint8_t mode = 0;
uint8_t temp = 32;
uint16_t light = 23;
uint8_t humid = 85;

// M5_DLight dlight;

TCA9548A hub;

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.fillScreen(BLACK);

  Wire.begin(SDA, SCL, 4000000UL);

  hub.begin();

  hub.openChannel(TCA_CHANNEL_0);
  // hub.writeRegister(CONTINUOUSLY_H_RESOLUTION_MODE);

  Wire.beginTransmission(0x23);
  Wire.write((byte)POWER_ON);
  Wire.endTransmission();

  Wire.beginTransmission(0x23);
  Wire.write((byte)CONTINUOUSLY_H_RESOLUTION_MODE);
  Wire.endTransmission();
  // dlight.setMode(CONTINUOUSLY_H_RESOLUTION_MODE);
  hub.closeChannel(TCA_CHANNEL_0);
}

uint8_t buffer[2];

void fetchData(){
  hub.openChannel(TCA_CHANNEL_0);
  //light = dlight.getLUX();
  Wire.requestFrom(0x23, 2);
  buffer[0] = Wire.read();
  buffer[1] = Wire.read();
  light = (uint16_t)buffer[0] << 8 | (uint16_t)buffer[1];
  hub.closeChannel(TCA_CHANNEL_0);
}

void refreshDisplay(){

  switch(mode){
    case 0:

      // first menu
      M5.Lcd.setTextSize(3);

      M5.Lcd.drawString("Temperature: ", 0, 0);
      M5.Lcd.setCursor(210,0);
      M5.Lcd.printf("%i", temp);

      M5.Lcd.drawString("Light: ", 0, 25);
      M5.Lcd.setCursor(210,25);
      M5.Lcd.printf("%i", light);

      M5.Lcd.drawString("Humidity: ", 0, 50);
      M5.Lcd.setCursor(210,50);
      M5.Lcd.printf("%i", humid);

      M5.Lcd.drawString("Phosphorous: ", 0, 75);
      M5.Lcd.setCursor(210,75);
      M5.Lcd.printf("%i", phosphorous());

      M5.Lcd.drawString("Potassium: ", 0, 100);
      M5.Lcd.setCursor(210,100);
      M5.Lcd.printf("%i", potassium());

      M5.Lcd.drawString("nitrogen:", 0, 125);
      M5.Lcd.setCursor(210,125);
      M5.Lcd.printf("%i", nitrogen());

      break;
    case 1:
      
      //second menu
      int address;
    int error;
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("scanning Address [HEX]");
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(
            address);  // Data transmission to the specified device address
                       // starts.   开始向指定的设备地址进行传输数据
        error = Wire.endTransmission(); /*Stop data transmission with the slave.
                  停止与从机的数据传输 0: success.  成功 1: The amount of data
                  exceeds the transmission buffer capacity limit.
                  数据量超过传送缓存容纳限制 return value:              2:
                  Received NACK when sending address.  传送地址时收到 NACK 3:
                  Received NACK when transmitting data.  传送数据时收到 NACK
                                             4: Other errors.  其它错误 */
        if (error == 0) {
            M5.Lcd.print(address, HEX);
            M5.Lcd.print(" ");
        } else
            M5.Lcd.print(".");

        delay(10);
    }

      break;
    case 2:
      
      //third menu


      break;
  }
}

void listenBtns(){
  temp = random(100);
  humid = random(100);
}

void loop() {
  
  fetchData();
  refreshDisplay();
  listenBtns();
  delay(2500);

}

