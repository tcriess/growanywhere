// #include <Arduino.h>
#include <M5Stack.h>
#include <M5_DLight.h>
#include <TCA9548A.h>
// #include <M5_ENV.h>
// ENVII hat
#include <SHT3X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <bmm150.h>
#include <bmm150_defs.h>
// end ENVII hat
#include <M5_KMeter.h>
#include <M5_ADS1100.h>
#include <esp32ModbusRTU.h>
#include <SoftwareSerial.h>
#include <IoT_BASE_SIM7080.h>
#include <Unit_Sonic.h>
#include <LoRaWanFixed.h> // from: https://github.com/Bjoerns-TB/LoRaWAN-M5Stack
#include <CayenneLPP.h>
#include <algorithm>
#define TINY_GSM_DEBUG SerialMon
#include <TinyGsmClient.h> // after IoT_BASE_SIM7080.h (the modem is defined there)
#include <TinyGPSPlus.h>
#include <ArduinoHttpClient.h>
#include <UNIT_4RELAY.h> // the globally defined "addr" clashes with the aduinojson lib used in cayennelpp...
#include "secrets.h"

/*
The following pins are in use
I2C: 21 (SDA), 22 (SCL)
RS485: 13 (RX), 15 (TX)  <- HardwareSerial 2 8N1, 9600
SIM7080: 12 (Enable, needs to be LOW for 2 seconds, then HIGH), 35 (RX), 0 (TX) <- HardwareSerial 1, alias SerialAT (?)
GPS: 36 (RX), 26 (TX) <- SoftwareSerial
PH: either: 36 (analog input) (0..3.3v, but the pH sensor is -theoretically- 0..5v, but the relevant range is typically ~3v), or via I2C ADC
LoRaWan: 16 (RX), 17 (TX) <- SoftwareSerial
*/

/*
ModBus RS485 addresses:
0x01: EC sensor
0x02: NPK sensor (default: 0x01)
*/

/*
I2C addresses:
hub: 0x74 (default: 0x70)
ambient light: 0x23
ADC for pH sensor: 0x48
Ultrasonic distance measurement: 0x57
(Environment sensor EnvIII: SHT30 (0x44), QMP6988(0x70))
Environment sensor EnvII: SHT30(0x44), BMP280(0x76), BMM150(0x10)
Stepmotor drivers: 0x70, 0x71
Relais: 0x26
K-Meter: 0x66 (water temperature)
*/

#define CAMADDR 0x17

#define LORA_RX 16
#define LORA_TX 17

// SoftwareSerial loraSerial(LORA_RX, LORA_TX);

#define GPS_RX 36
#define GPS_TX 26

#define GPS_BAUD 9600

TinyGPSPlus gps;

TinyGsm modem(SerialAT);

TinyGsmClient tcpclient(modem);
// HttpClient http(tcpclient, "growanywhere.de", 8086);
const char tcpaddr[] = "157.245.23.172";
const uint16_t tcpport = 8094;

const char apn[]      = GPRS_APN;
const char gprsUser[] = GPRS_USER;
const char gprsPass[] = GPRS_PASSWORD;

uint8_t mode = 0;

time_t unixtime = 0;

bool temp_valid = false;
float temp = -270;
uint16_t light = 0;
float humid = -1;
float press = 0.0;
float ph = 0.0;
float water_temp = -270;
float lon = 0.0, lat = 0.0, alt = 0.0, hdop = 0.0;

uint16_t nitrogen=0, phosphorus=0, potassium=0;

float ec = 0.0;
float distance = 0.0;

M5_DLight *dlight;
SHT3X *sht30;
// QMP6988 *qmp6988; // this will not work if using an unmodified hub, because it has the same address (0x70) as the tca9548 hub :(
Adafruit_BMP280 *bme;
ADS1100 *ads;
M5_KMeter *kmeter;
// ADS1100 *ads2;
UNIT_4RELAY *unit_4relay;
SONIC_I2C *sonic;

struct DataRecord {

  float temperature; // degree celcius
  uint16_t light;
  float humidity;
  float pressure;
  float pH;
  float longitude, latitude, altitude, hdop;
  uint16_t nitrogen, phosphorus, potassium;
  float ec;
};

// the NPK and the EC sensors both come with a factory setting of address 0x01, which conflicts.
// after some trial and error, i could set the NPK sensor address to 0x02 via
// modbus.writeSingleHoldingRegister(0x01, 0x0100, 0x0002);
// this seems to persist!
#define MODBUS_ADDR_NPK 0x02
#define MODBUS_ADDR_EC 0x01

#define ADC_VCC 12.0
#define ADC_GAIN GAIN_ONE
#define ADC_FACTOR 1.0
// one of RATE_8, RATE16, RATE_32, RATE_128 (this determines the accuracy and thus the min value)
#define ADC_RATE RATE_8
// depending on the bit rate (-32768), (-16384), (-8192), (-2048)
#define ADC_MIN (-32768)

// the pH value depends on the measured voltage, the manual gives three reference values:
// pH  voltage
// 4   3.071
// 7   2.535
// 10  2.066
//
// fitting a quadratic curve leads to (it is very flat though)
// f(x) = a * x**2 + b * x + c
// a = 0.795596
// b = -10.0571
// c = 27.3821
#define PH_FIT_A 0.795596
#define PH_FIT_B (-10.0571)
#define PH_FIT_C 27.3821

TCA9548A hub(0x74);

esp32ModbusRTU modbus(&Serial2);

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

void nbConnect(void *);
void m5update(void *);

char lorabuffer[256];

double compute_ph(double voltage) {
  return voltage > 0 ? PH_FIT_A * voltage * voltage + PH_FIT_B * voltage + PH_FIT_C : 0;
}

void log(String info) {
    //canvas.println(info);
    //canvas.pushSprite(0, 0);
    SerialMon.println(info);
}

char *devEui = DEVICE_EUI;
char *appEui = APP_EUI;
char *appKey = APP_KEY;

bool hasCard = false;

uint8_t img[10240];


void setup() {
  Serial.write("Start.");
  disableCore0WDT(); // disable the watchdog on core 0, this only causes issues (it is disabled on core 1 by default)

  Wire.setPins(SDA, SCL);
  M5.begin(true, true, true, false);
  M5.Power.begin();

  if (!SD.begin()) {  // Initialize the SD card.
      M5.Lcd.println(
          "Card failed, or not present");  // Print a message if the SD card
                                            // initialization fails or if the
                                            // SD card does not exist
      delay(2000);
  } else {
    hasCard = true;
  }

  // iotBaseInit(); // enable SIM7080 -> this is now part of the modemReset function

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

  // RS485 modbus
  Serial2.begin(9600, SERIAL_8N1, IoT_BASE_RS485_RX, IoT_BASE_RS485_TX, false);
  modbus.onData([](uint8_t serverAddress, esp32Modbus::FunctionCode fc,  uint16_t address, uint8_t* data, size_t length) {
    if (address == 0x1e && length == 6 && serverAddress == MODBUS_ADDR_NPK) { // npk
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
      log("nitrogen: "+String(nitrogen));
      log("phosphorus: "+String(phosphorus));
      log("potassium: "+String(potassium));
    }
    if (address == 0x0100 && length > 0 && serverAddress == MODBUS_ADDR_NPK) {
      Serial.printf("id 0x%02x fc 0x%02x len %u: 0x", serverAddress, fc, length);
      for (size_t i = 0; i < length; ++i) {
        Serial.printf("%02x", data[i]);
      }
    }
    if (address == 0x00 && length == 4 && serverAddress == MODBUS_ADDR_EC) {
      // value 1 is a calibration value in uS/cm (or also in 10 uS/cm?), value 2 is the EC in 10 uS/cm
      Serial.printf("id 0x%02x fc 0x%02x len %u: 0x", serverAddress, fc, length);
      for (size_t i = 0; i < length; ++i) {
        Serial.printf("%02x", data[i]);
      }
      // fix endianness
      std::reverse(data, data+2);
      std::reverse(data+2, data+4);
      // convert the 2 values to uints
      // cal = *reinterpret_cast<uint16_t*>(data);
      uint16_t ec_raw = *reinterpret_cast<uint16_t*>(data+2);
      ec = (float)ec_raw / 10.0;
      log("ec="+String(ec,1));
    }
  });
  modbus.onError([](esp32Modbus::Error error) {
    Serial.printf("error: 0x%02x\n\n", static_cast<uint8_t>(error));
  });
  modbus.begin();
  // register 0x0100 of the NPK sensor hold the address and the baud rate, both changeable
  // modbus.readHoldingRegisters(0x02, 0x0100, 1);
  // f.e. if the current device address is 0x01 and the baud rate is 0x00 (9600) and the address should be changed to 0x02
  // the command is (the 0x0002 -> 1st byte is baud rate 0x00 and second byte is address 0x02)
  // modbus.writeSingleHoldingRegister(0x01, 0x0100, 0x0002);

  // Open the i2c hub
  hub.begin();
  hub.openAll(); // the sensors (currently) have no conflicting addresses, so we can have all channels activated
  // hub.closeChannel(4); // ADC - water
  // hub.closeChannel(5); // ADC - pH sensor
  // it is important to initialize the sensors *after* the hub channels are open, otherwise the devices are not found!
  dlight = new M5_DLight();
  dlight->begin();
  dlight->setMode(CONTINUOUSLY_H_RESOLUTION_MODE);

  //qmp6988 = new QMP6988();
  //qmp6988->init();

  kmeter = new M5_KMeter();
  kmeter->begin();

  sht30 = new SHT3X();

  bme = new Adafruit_BMP280();
  if (!bme->begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      // while (1);
  }
  Serial.print("\n\rCalibrate done..");

  /// hub.openChannel(5); // pH
  ads = new ADS1100();
  ads->getAddr_ADS1100(ADS1100_DEFAULT_ADDRESS);  // 0x48, 1001 000 (ADDR = GND)
  // ads->setGain(GAIN_ONE);  // 1x gain(default)
  // ads->setGain(GAIN_TWO);       // 2x gain
  // ads.setGain(GAIN_FOUR);      // 4x gain
  // ads.setGain(GAIN_EIGHT);     // 8x gain
  ads->setGain(ADC_GAIN); 

  ads->setMode(MODE_CONTIN);  // Continuous conversion mode (default)
  // ads->setMode(MODE_SINGLE);    // Single-conversion mode

  // ads->setRate(RATE_8);  // 8SPS (default)
  // ads.setRate(RATE_16);        // 16SPS
  // ads.setRate(RATE_32);        // 32SPS
  // ads.setRate(RATE_128);       // 128SPS
  ads->setRate(ADC_RATE);

  ads->setOSMode(OSMODE_SINGLE);  // Set to start a single-conversion.
  ads->begin();
  // hub.closeChannel(5);

  /*
  // hub.openChannel(4); // water
  ads2 = new ADS1100();
  ads2->getAddr_ADS1100(ADS1100_DEFAULT_ADDRESS);  // 0x48, 1001 000 (ADDR = GND)
  // ads->setGain(GAIN_ONE);  // 1x gain(default)
  // ads->setGain(GAIN_TWO);       // 2x gain
  // ads.setGain(GAIN_FOUR);      // 4x gain
  // ads.setGain(GAIN_EIGHT);     // 8x gain
  ads2->setGain(ADC_GAIN); 

  ads2->setMode(MODE_CONTIN);  // Continuous conversion mode (default)
  // ads->setMode(MODE_SINGLE);    // Single-conversion mode

  // ads->setRate(RATE_8);  // 8SPS (default)
  // ads.setRate(RATE_16);        // 16SPS
  // ads.setRate(RATE_32);        // 32SPS
  // ads.setRate(RATE_128);       // 128SPS
  ads2->setRate(ADC_RATE);

  ads2->setOSMode(OSMODE_SINGLE);  // Set to start a single-conversion.
  ads2->begin();
  //hub.closeChannel(4);
  */

  unit_4relay = new UNIT_4RELAY();
  unit_4relay->Init(1); // mode 0: async, 1: sync

  // unit_4relay->relayWrite(0, true);

  sonic = new SONIC_I2C();
  sonic->begin();

  gpsSerial.begin(GPS_BAUD, SWSERIAL_8N1, GPS_RX, GPS_TX);
  
  lora_fixed.init();
  delay(2000);  // must delay for lorawan power on

  memset(lorabuffer, 0, 256);
  lora_fixed.getVersion(lorabuffer, 256, 1);
  Serial.print(lorabuffer);

  memset(lorabuffer, 0, 256);
  lora_fixed.getId(lorabuffer, 256, 1);
  Serial.print(lorabuffer);

  // void setId(char *DevAddr, char *DevEUI, char *AppEUI);
  // DevAddr: 00:02:B9:EF
  lora_fixed.setId(NULL, devEui, appEui);
  // setKey(char *NwkSKey, char *AppSKey, char *AppKey);
  lora_fixed.setKey(NULL, NULL, appKey);

  lora_fixed.setDeviceMode(LWOTAA);
  lora_fixed.setDataRate(DR5, EU868);

  lora_fixed.setChannel(0, 868.1);
  lora_fixed.setChannel(1, 868.3);
  lora_fixed.setChannel(2, 868.5);
  lora_fixed.setChannel(3, 867.1);
  lora_fixed.setChannel(4, 867.3);
  lora_fixed.setChannel(5, 867.5);
  lora_fixed.setChannel(6, 867.7);
  lora_fixed.setChannel(7, 867.9);

  lora_fixed.setReceiveWindowFirst(0, 868.1);
  lora_fixed.setReceiveWindowSecond(869.525, DR3);

  lora_fixed.setPower(14);
  lora_fixed.setPort(1);
  lora_fixed.setAdaptiveDataRate(true);
  //lora.setDutyCycle(false);
  //lora.setJoinDutyCycle(false);
  lora_fixed.setOTAAJoin(JOIN, 10);
  
  SerialAT.begin(SIM7080_BAUDRATE, SERIAL_8N1, IoT_BASE_SIM7080_RX, IoT_BASE_SIM7080_TX);
  xTaskCreatePinnedToCore(
    nbConnect,    // Function that should be called
    "Connect to network",  // Name of the task (for debugging)
    4096,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL,             // Task handle
    0 // core id
  );
  //nbConnect(); // connect to CatM - disabled for now, need to find out if the SIM card actually works.

  xTaskCreatePinnedToCore(
    m5update,    // Function that should be called
    "m5update",  // Name of the task (for debugging)
    4096,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL,             // Task handle
    1 // core id
  );

  if (hasCard) {
    if (SD.exists("/data.csv")) {  // Check if the file exists
        log("data.csv exists.");
    } else {
        log("hello.txt doesn't exist.");
        File myFile = SD.open("/data.csv",
                          FILE_WRITE);  // Create a new file "/hello.txt".
        if (myFile) {
          myFile.println("unixtime,temperature,humidity,pressure,light,ph,ec,nitrogen,phosphorus,potassium");
          myFile.close();
        }
    }
  }
}

void save_image() {
  uint8_t lenbuf[2];
  lenbuf[0] = 0;
  lenbuf[1] = 0;
  Wire.requestFrom(CAMADDR, 10240); // always request 10kB
  for (int i=0; i<2; i++) {
    if (!Wire.available()) break;
    lenbuf[i] = Wire.read();
  }
  uint16_t length = (uint16_t) (lenbuf[0]);
  length = (length << 8) + lenbuf[1];
  Serial.printf("len: %d", length);
  if (length>0 && length<10240) {
    for (int i=0; i<length; i++) {
      if (!Wire.available()) break;
      img[i] = Wire.read();
    }
    if (hasCard) {
      File myFile = SD.open("/img.jpg", FILE_WRITE);
      if (myFile) {
        myFile.write(img, length);
      }
      myFile.close();
    }
  }
}

CayenneLPP lpp(160);
uint8_t buffer[2];
unsigned long lastSent = 0;
byte coords[12];
void sendobject() {
  if (millis() - lastSent < 15 * 60000) {
    return;
  }
  save_image();
  lastSent = millis();
  bool result = false;

  Serial.println("Sending");

  lpp.reset();
  if(unixtime > 943920000L) {
    // this kills the ttn decoder
    // lpp.addUnixTime(1, (uint32_t)unixtime);
    // this also kills the ttn decoder
    // lpp.addFrequency(1, (uint32_t)unixtime);
    // there is no supported 32-bit integer format...
    // possible solutions:
    // - ignore timestamp, always use the ttn time
    // - use float (very inprecise, + the telegraf parser probably will not accept it)
    // - write a ttn payload parser
  }
  lpp.addLuminosity(4, (uint32_t)light);
  if (press > 0) {
    lpp.addBarometricPressure(3, press);
  }
  if (temp_valid) {
    lpp.addTemperature(3, temp);
  }
  if (water_temp > 0) {
    lpp.addTemperature(11, water_temp);
  }
  if (humid >= 0) {
    lpp.addRelativeHumidity(3, humid);
  }
  if (ph > 0) {
    lpp.addAnalogInput(5, ph);
  }
  if (nitrogen > 0) {
    lpp.addDigitalInput(6, (uint32_t)nitrogen);
  }
  if (phosphorus > 0) {
    lpp.addDigitalInput(7, (uint32_t)phosphorus);
  }
  if (potassium > 0) {
    lpp.addDigitalInput(8, (uint32_t)potassium);
  }
  if (ec > 0) {
    lpp.addTemperature(9, ec); // we use "temperature" here as this uses a factor of 10 (instead of a factor of 100 used by the generic analog input)
  }
  if (distance > 0 && distance < 4500.0) {
    lpp.addTemperature(10, distance); // we use "temperature" here as this uses a factor of 10 (instead of a factor of 100 used by the generic analog input)
  }
  if (lon != 0 || lat != 0) {
    lpp.addGPS(1, lon, lat, alt);
  }
  uint8_t *b = lpp.getBuffer();
  String hexstring = "";
  for(int i=0; i<lpp.getSize(); i++) {
    String s(b[i], HEX);
    if (hexstring != "") {
      hexstring += ":" + s;
    } else {
      hexstring += s;
    }
  }
  log("send to ttn: ");
  log(hexstring);
  result = lora_fixed.transferPacket(lpp.getBuffer(), lpp.getSize(), 5);

/*
  int32_t lat = lat * 10000.0;
  int32_t lon = lon * 10000.0;
  int16_t altitude = alt * 100.0;
  int8_t hdopGPS = hdop / 10.0;


  coords[0] = 0x00; // channel
  coords[1] = 0x88; // see https://developers.mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload
  coords[2] = lat;
  coords[3] = lat >> 8;
  coords[4] = lat >> 16;

  coords[5] = lon;
  coords[6] = lon >> 8;
  coords[7] = lon >> 16;

  coords[8] = altitude;
  coords[9] = altitude >> 8;

  coords[10] = hdopGPS;

  // result = lora_fixed.transferPacket("Hello World!", 5);
  result = lora_fixed.transferPacket(coords, sizeof(coords), 5);
*/
  if (result == true) {
    Serial.println("Sent");
  } else {
    Serial.println("Error");
  }
}

void fetchData(){
  // toggle = !toggle;
  // unit_4relay->relayWrite(0, toggle);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  light = dlight->getLUX();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // press = qmp6988->calcPressure() / 100.0; // hPa
  press = bme->readPressure()*0.01;
  vTaskDelay(100 / portTICK_PERIOD_MS);

  water_temp = kmeter->getTemperature();
  log("water temp="+String(water_temp, 1));

  if (sht30->get() == 0) {
    temp = sht30->cTemp;
    temp_valid = true;
    humid = sht30->humidity;
  } else {
    temp = 0.0;
    temp_valid = false;
    humid = -1;
  }
  
  // hub.openChannel(5); // ph
  //ads->setOSMode(OSMODE_SINGLE);
  double ph_voltage = (double)ads->Measure_Differential() * ((double)ADC_VCC) / ((double)ADC_FACTOR) / (-(double)ADC_MIN);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  log("ph_v = " + String(ph_voltage));
  ph = compute_ph(ph_voltage);
  // hub.closeChannel(5); // ph

  /*
  hub.openChannel(4); // water
  double water_voltage = (double)ads2->Measure_Differential() * ((double)ADC_VCC) / ((double)ADC_FACTOR) / (-(double)ADC_MIN);
  log("water_v = " + String(water_voltage));
  hub.closeChannel(4);
  */

  distance = sonic->getDistance(); // in mm
  log("distance = " + String(distance,1));

  lon = (float)gps.location.lng();
  lat = (float)gps.location.lat();
  alt = (float)gps.altitude.value();
  hdop = (float)gps.hdop.value();

  log("lon = " + String(lon));
  log("lat = " + String(lat));
  log("dt = " + String(gps.date.value()));

  tm timeinfo;
  timeinfo.tm_year = gps.date.year() - 1900;
  timeinfo.tm_mon = gps.date.month() - 1;
  timeinfo.tm_mday = gps.date.day();
  timeinfo.tm_hour = gps.time.hour();
  timeinfo.tm_min = gps.time.minute();
  timeinfo.tm_sec = gps.time.second();
  
  unixtime = mktime ( &timeinfo );

  log("unixtime:"+String(unixtime));

  if (modem.isNetworkConnected()) {
    log("gsm datetime=" + modem.getGSMDateTime(TinyGSMDateTimeFormat::DATE_FULL));
  }
  if (gps.time.isValid()) {
    log("gps time=" + String(gps.time.value()));
  }

  Serial.print("sending Modbus request...\n");
  modbus.readHoldingRegisters(0x02, 0x1e, 3); // npk
  vTaskDelay(100 / portTICK_PERIOD_MS);
  modbus.readHoldingRegisters(0x01, 0x00, 2); // ec
  vTaskDelay(100 / portTICK_PERIOD_MS);
  sendobject();

  if (hasCard) {
    File myFile = SD.open("/data.csv", FILE_APPEND);
    if (myFile) {
      String s;
      if (unixtime > 943920000L) {
        s += String(unixtime);
      } else {
        s += String(millis() / 1000);
      }
      s += ","+String(temp, 1)+","+String(humid, 1) + "," + String(press, 1) + "," + String(light) + "," + String(ph,1) + "," + String(ec,1) + "," + String(nitrogen) + "," + String(phosphorus) + "," + String(potassium);
      myFile.println(s);
      myFile.close();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

unsigned long last_refresh = 0;
#define REFRESH_MS 5000

void refreshDisplay(){
  if (millis() - last_refresh < REFRESH_MS) {
    return;
  }
  last_refresh = millis();
  M5.Lcd.fillScreen(BLACK);

  switch(mode){
    case 0:

      // first menu
      M5.Lcd.setTextSize(3);

      M5.Lcd.drawString("Temperature: ", 0, 0);
      M5.Lcd.setCursor(210,0);
      M5.Lcd.printf("%.1f", temp);

      M5.Lcd.drawString("Light: ", 0, 25);
      M5.Lcd.setCursor(210,25);
      M5.Lcd.printf("%i", light);

      M5.Lcd.drawString("Humidity: ", 0, 50);
      M5.Lcd.setCursor(210,50);
      M5.Lcd.printf("%.1f", humid);

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

      M5.Lcd.drawString("pH:", 0, 175);
      M5.Lcd.setCursor(210,175);
      M5.Lcd.printf("%.1f", ph);

      M5.Lcd.drawString("EC:", 0, 200);
      M5.Lcd.setCursor(210,200);
      M5.Lcd.printf("%.1f", ec);

      //M5.Lcd.drawString("pos:", 0, 200);
      //M5.Lcd.setCursor(210,200);
      //M5.Lcd.printf("%.1f %.1f", lon, lat);

      break;
    case 1:
      

      break;
    case 2:
      
      //third menu


      break;
  }
}

void modemReset() {
  iotBaseInit();
  unsigned long start = millis();
    log("Initializing modem...");
    while (!modem.init()) {
        log("waiting...." + String((millis() - start) / 1000) + "s");
        if ((millis() - start) / 1000 > 120) {
          return;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    };
    vTaskDelay(500 / portTICK_PERIOD_MS);

    start = millis();
    log("Waiting for network...");
    while (!modem.waitForNetwork()) {
        log("waiting...." + String((millis() - start) / 1000) + "s");
        if ((millis() - start) / 1000 > 120) {
          return;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);

    start = millis();
    log("Waiting for GPRS connect...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        log("waiting...." + String((millis() - start) / 1000) + "s");
        if ((millis() - start) / 1000 > 120) {
          return;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    log("gprs success");
}

// connect to the network and keep connected. also, send data if available.
void nbConnect(void * parameter) {
    for(;;) {
      if(!modem.isGprsConnected()) {
        log("gprs not connected, reset modem");
        modemReset();
      }
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      
      //if(!modem.isGprsConnected()) {
      //  vTaskDelay(50000 / portTICK_PERIOD_MS);
      //  continue;
      //}
      if (tcpclient.connect(tcpaddr, tcpport)) {
        String body = "";
        body += "environment,device=" + String(DEVICE_ID) + " temperature=" + String(temp, 1) + ",humidity=" + String(humid, 1) + ",pressure=" + String(press, 1) + ",light=" + String((float)light, 1);
        if(unixtime > 943920000L) {
          body += " " + String(unixtime) + "000000000";
        }
        body += "\n";
        tcpclient.print(body);
        body = "water,device=" + String(DEVICE_ID) + " ph=" + String(ph, 1) + ",ec=" + String(ec, 1) + ",nitrogen=" + String((float)nitrogen, 1) + ",phosphorus=" + String((float)phosphorus, 1) + ",potassium=" + String((float)potassium, 1) + ",distance=" + String(distance, 1) + ",temperature=" + String(water_temp,1);
        if(unixtime > 943920000L) {
          body += " " + String(unixtime) + "000000000";
        }
        body += "\n";
        tcpclient.print(body);
        if (lon != 0 && lat != 0) {
          body = "position,device=" + String(DEVICE_ID) + " longitude=" + String(lon, 6) + ",latitude=" + String(lat, 6) + ",altitude=" + String(alt, 1) + ",hdop=" + String(hdop, 1);
          if(unixtime > 943920000L) {
            body += " " + String(unixtime) + "000000000";
          }
          body += "\n";
          tcpclient.print(body);
        }
        
        //int status = http.responseStatusCode();
        //SerialMon.print(F("Response status code: "));

        //http.stop();
        tcpclient.stop();
      }
      vTaskDelay(60 * 60000 / portTICK_PERIOD_MS);
    }
}

#define SCREENSAVE_MS 10000
unsigned long last_activity = 0;
bool blank = false;

void button_pressed(uint8_t no) {
  last_activity = millis();
  if (blank) {
    M5.Lcd.wakeup();
  }
  blank = false;
  switch (no) {

  }
}

// update the button states - for some reason this needs to run on core 1 (just like the main loop)
void m5update(void * parameter) {
  for(;;) {
    M5.update();
    if (M5.BtnA.wasReleased() || M5.BtnA.pressedFor(1000, 200)) {
      button_pressed(0);
    }
    if (M5.BtnB.wasReleased() || M5.BtnB.pressedFor(1000, 200)) {
      button_pressed(1);
    }
    if (M5.BtnC.wasReleased() || M5.BtnC.pressedFor(1000, 200)) {
      button_pressed(2);
    }
    unsigned long a = millis() - last_activity;
    if (!blank) {
      if (a > SCREENSAVE_MS) {
        blank = true;
        M5.Lcd.clearDisplay();
        M5.Lcd.sleep();
      } else {
        refreshDisplay();
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
  } while (millis() - start < ms);
}

void loop() {
  fetchData();
  smartDelay(10000);
}
