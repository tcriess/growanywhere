#include <M5Stack.h>
#include <M5_DLight.h>
#include "M5_ENV.h"
#include "npk.h"

uint8_t mode = 0;
uint8_t temp = 32;
uint16_t light = 23;
uint8_t humid = 85;
float pressure = 1.1;

int ms;

M5_DLight dlight;

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.begin();    
  M5.Lcd.fillScreen(BLACK);

  Wire.begin();
  dlight.begin();
  dlight.setMode(CONTINUOUSLY_H_RESOLUTION_MODE);

}

void fetchData(){
  light = dlight.getLUX();
  temp = random(100);
  humid = random(100);
}

void refreshDisplay(){

  switch(mode){
    case 0:

      // first menu

      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Temperature", 0, 0);
      M5.Lcd.setCursor(25,40);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i", temp);
      

      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Light", 220, 0);
      M5.Lcd.setCursor(205,40);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i", light);


      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Humidity", 0, 120);
      M5.Lcd.setCursor(30,160);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i", humid);


      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Pressure", 220, 120);
      M5.Lcd.setCursor(230,160);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%f", std::round(pressure));

      M5.Lcd.fillRect(0,220,105,40,WHITE);


      break;
    case 1:
      
      //second menu
      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Phosporous", 0, 0);
      M5.Lcd.setCursor(25,40);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i", phosphorous());
      

      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Potassium", 210, 0);
      M5.Lcd.setCursor(230,40);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i", potassium());


      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Nitrogen", 0, 120);
      M5.Lcd.setCursor(30,160);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i", nitrogen());


      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("4th val", 220, 120);
      M5.Lcd.setCursor(230,160);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i", phosphorous());

      M5.Lcd.fillRect(105,220,105,40,WHITE);


      break;
    case 2:
      
      //third menu
      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("GPS", 50, 0);
      M5.Lcd.setCursor(0,40);
      M5.Lcd.setTextSize(2);
      M5.Lcd.printf("Lat: %f", 47.684090);
      M5.Lcd.setCursor(0,80);
      M5.Lcd.printf("Lon: %f", 9.168429);

      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Growing:", 220, 0);
      M5.Lcd.setCursor(230,40);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i h", 4);

      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Grown in:", 30, 120);
      M5.Lcd.setCursor(30,160);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("%i d", 12);


      M5.Lcd.setTextSize(2);
      M5.Lcd.drawString("Health", 220, 120);
      M5.Lcd.setCursor(230,160);
      M5.Lcd.setTextSize(4);
      M5.Lcd.printf("OK");


      M5.Lcd.fillRect(210,220,105,40,WHITE);

      break;
  }
}

void listenBtns(){
  M5.update();
  if (M5.BtnA.isPressed() && mode != 0) {
        mode = 0;
        M5.Lcd.clear();
    } else if (M5.BtnB.isPressed() && mode != 1) {
        mode = 1;
        M5.Lcd.clear();
    } else if (M5.BtnC.isPressed() && mode != 2) {
        mode = 2;
        M5.Lcd.clear();
    }
}

void loop() {

  listenBtns();
  refreshDisplay();
  if(ms + 10000 < millis()){
    fetchData();
    ms = millis();
  }
}

