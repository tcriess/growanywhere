// #include <Arduino.h>
#include <M5Stack.h>
#include <M5_DLight.h>
#include <TCA9548A.h>
#include <M5_ENV.h>
#include <M5_ADS1100.h>
#include <esp32ModbusRTU.h>
#include <SoftwareSerial.h>
#include <IoT_BASE_SIM7080.h>
#include <algorithm>
#define TINY_GSM_DEBUG SerialMon
#include <TinyGsmClient.h> // after IoT_BASE_SIM7080.h (the modem is defined there)
#include <TinyGPSPlus.h>

/*
The following pins are in use
I2C: 21 (SDA), 22 (SCL)
RS485: 13 (RX), 15 (TX)  <- HardwareSerial 2 8N1, 9600
SIM7080: 12 (Enable, needs to be LOW for 2 seconds, then HIGH), 35 (RX), 0 (TX) <- HardwareSerial 1, alias SerialAT (?)
GPS: 16 (RX), 17 (TX) <- SoftwareSerial
PH: -not working: 36 (analog input) (because it is only 0..3.3v, but the pH sensor is -theoretically- 0..5v)-
*/

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

TinyGPSPlus gps;

TinyGsm modem(SerialAT);
const char apn[]      = "hologram";
const char gprsUser[] = "";
const char gprsPass[] = "";

uint8_t mode = 0;
uint8_t temp = 0;
uint16_t light = 0;
uint8_t humid = 0;
float press = 0.0;
float ph = 0.0;

float lon = 0.0, lat = 0.0;

uint16_t nitrogen=0, phosphorus=0, potassium=0;

M5_DLight *dlight;
SHT3X *sht30;
QMP6988 *qmp6988; // this will not work, because it has the same address (0x70) as the tca9548 hub :(
ADS1100 *ads;

TCA9548A hub(0x74);

esp32ModbusRTU modbus(&Serial2);

SoftwareSerial gpsSerial(GPS_RX);

void nbConnect(void);

void log(String info) {
    //canvas.println(info);
    //canvas.pushSprite(0, 0);
    SerialMon.println(info);
}

void setup() {
  Serial.write("Start.");
  Wire.setPins(SDA, SCL);
  M5.begin(true, false, true, false);
  M5.Power.begin();

  iotBaseInit(); // enable SIM7080

  M5.Lcd.fillScreen(BLACK);

  M5.Lcd.fillScreen(WHITE);
  delay(500);
  M5.Lcd.fillScreen(RED);
  delay(500);
  M5.Lcd.fillScreen(GREEN);
  delay(500);
  M5.Lcd.fillScreen(BLUE);
  delay(500);
  M5.Lcd.fillScreen(BLACK);
  delay(500);

  Serial.write("After fillScreen");

  // RS486 modbus
  Serial2.begin(9600, SERIAL_8N1, IoT_BASE_RS485_RX, IoT_BASE_RS485_TX, false);
  modbus.onData([](uint8_t serverAddress, esp32Modbus::FunctionCode fc,  uint16_t address, uint8_t* data, size_t length) {
    Serial.printf("id 0x%02x fc 0x%02x len %u: 0x", serverAddress, fc, length);
    for (size_t i = 0; i < length; ++i) {
      Serial.printf("%02x", data[i]);
    }
    // fix endianness
    std::reverse(data, data+2);
    std::reverse(data+2, data+4);
    std::reverse(data+4, data+6);
    // convert the 3 values to uints
    nitrogen = *reinterpret_cast<uint16_t*>(data);
    phosphorus = *reinterpret_cast<uint16_t*>(data+2);
    potassium = *reinterpret_cast<uint16_t*>(data+4);
  });
  modbus.onError([](esp32Modbus::Error error) {
    Serial.printf("error: 0x%02x\n\n", static_cast<uint8_t>(error));
  });
  modbus.begin();

  // Open the i2c hub
  hub.begin();
  hub.openAll(); // the sensors (currently) have no conflicting addresses, so we can have all channels activated
  // it is important to initialize the sensors *after* the hub channels are open, otherwise the devices are not found!
  dlight = new M5_DLight();
  dlight->begin();
  dlight->setMode(CONTINUOUSLY_H_RESOLUTION_MODE);
  qmp6988 = new QMP6988();
  qmp6988->init();
  sht30 = new SHT3X();
  ads = new ADS1100();
  ads->getAddr_ADS1100(ADS1100_DEFAULT_ADDRESS);  // 0x48, 1001 000 (ADDR = GND)
  // ads->setGain(GAIN_ONE);  // 1x gain(default)
  ads->setGain(GAIN_TWO);       // 2x gain
  // ads.setGain(GAIN_FOUR);      // 4x gain
  // ads.setGain(GAIN_EIGHT);     // 8x gain

  ads->setMode(MODE_CONTIN);  // Continuous conversion mode (default)
  // ads.setMode(MODE_SINGLE);    // Single-conversion mode

  ads->setRate(RATE_8);  // 8SPS (default)
  // ads.setRate(RATE_16);        // 16SPS
  // ads.setRate(RATE_32);        // 32SPS
  // ads.setRate(RATE_128);       // 128SPS

  ads->setOSMode(OSMODE_SINGLE);  // Set to start a single-conversion.
  ads->begin();
  gpsSerial.begin(GPS_BAUD, SWSERIAL_8N1, GPS_RX);
  SerialAT.begin(SIM7080_BAUDRATE, SERIAL_8N1, IoT_BASE_SIM7080_RX, IoT_BASE_SIM7080_TX);
  // nbConnect(); // connect to CatM - disabled for now, need to find out if the SIM card actually works.
}

uint8_t buffer[2];

void fetchData(){
  light = dlight->getLUX();
  press = qmp6988->calcPressure() / 100.0; // hPa

  if (sht30->get() == 0) {
    temp = sht30->cTemp;
    humid = sht30->humidity;
  }
  
  int16_t ph_voltage = ads->Measure_Differential();
  log("ph_v = " + String(ph_voltage));

  lon = (float)gps.location.lng();
  lat = (float)gps.location.lat();

  log("lon = " + String(lon));
  log("lat = " + String(lat));
  log("dt = " + String(gps.date.value()));
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

      M5.Lcd.drawString("Pressure: ", 0, 75);
      M5.Lcd.setCursor(210,75);
      M5.Lcd.printf("%.2f", press);

      M5.Lcd.drawString("Phosphorous: ", 0, 100);
      M5.Lcd.setCursor(210,100);
      M5.Lcd.printf("%d", phosphorus);

      M5.Lcd.drawString("Potassium: ", 0, 125);
      M5.Lcd.setCursor(210,125);
      M5.Lcd.printf("%d", potassium);

      M5.Lcd.drawString("nitrogen:", 0, 150);
      M5.Lcd.setCursor(210,150);
      M5.Lcd.printf("%d", nitrogen);

      M5.Lcd.drawString("pos:", 0, 175);
      M5.Lcd.setCursor(210,175);
      M5.Lcd.printf("%.4f %.4f", lon, lat);

      break;
    case 1:
      

      break;
    case 2:
      
      //third menu


      break;
  }
}

/*
void nbConnect(void) {
    unsigned long start = millis();
    log("Initializing modem...");
    while (!modem.init()) {
        log("waiting...." + String((millis() - start) / 1000) + "s");
    };

    start = millis();
    log("Waiting for network...");
    while (!modem.waitForNetwork()) {
        log("waiting...." + String((millis() - start) / 1000) + "s");
    }

    log("Waiting for GPRS connect...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        log("waiting...." + String((millis() - start) / 1000) + "s");
    }
    log("success");
}
*/

void listenBtns(){
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available()) {
      byte b = gpsSerial.read();
      gps.encode(b);
    }
  } while (millis() - start < ms);
}

void loop() {
  
  fetchData();
  refreshDisplay();
  listenBtns();

  Serial.print("sending Modbus request...\n");
  modbus.readHoldingRegisters(0x01, 0x1e, 3);

  smartDelay(2500);

}
