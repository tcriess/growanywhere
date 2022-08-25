/*
Growanywhere main unit
======================

Hardware:
M5-Stack Core (ESP32 (BT, WiFi, ...), SD card, TFT display, 3 buttons)
IoT-Base (RS485, ext. power, CatM)
LoRaWan module
2x GRBL motor driver modules

I2C units:
PaHUB (I2C hub)
ENVII (temperature, humidity, barometric pressure)
DLight (ambient light)
Sonic (ultrasonic distance)
ADC (analog-digital-converter for the pH-sensor)
4RELAY (relays for switching the pumps on / off)
*/
#include <M5Stack.h>
#include <algorithm>
#include <SoftwareSerial.h>

// EDIT THE FOLLOWING TWO HEADER FILES
#include "settings.h"
#include "secrets.h"

#ifdef USE_I2C_HUB
#include <TCA9548A.h>
#endif

#ifdef USE_I2C_DLIGHT
#include <M5_DLight.h>
#endif

#ifdef USE_I2C_ENVIII
#include <M5_ENV.h>
#endif

#ifdef USE_I2C_ENVII
#include <SHT3X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <bmm150.h>
#include <bmm150_defs.h>
#endif

#ifdef USE_I2C_KMETER
#include <M5_KMeter.h>
#endif

#ifdef USE_I2C_ADC
#include <M5_ADS1100.h>
#endif

#ifdef USE_I2C_GRBL
#include <MODULE_GRBL13.2.h>
#endif

#ifdef USE_I2C_SONIC
#include <Unit_Sonic.h>
#endif

#ifdef USE_MODBUS
#include <esp32ModbusRTU.h>
#endif

#ifdef USE_IOT_BASE
#include <IoT_BASE_SIM7080.h>
#define TINY_GSM_DEBUG SerialMon
#include <TinyGsmClient.h> // after IoT_BASE_SIM7080.h (the modem is defined there)
#endif

#ifdef USE_LORAWAN
#include <LoRaWanFixed.h> // from: https://github.com/Bjoerns-TB/LoRaWAN-M5Stack (renamed, such that it can coexist with the original LoRaWan code)
#include <CayenneLPP.h>
#endif

#ifdef USE_GPS
#include <TinyGPSPlus.h>
#endif

// includes for wifi (smartconfig)
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Preferences.h>

// THIS NEEDS TO COME LAST!
#ifdef USE_I2C_4RELAY
#include <UNIT_4RELAY.h> // the globally defined "addr" in the header clashes with the aduinojson lib used in cayennelpp... (or somewhere else)...
#endif

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

// WiFi / smartconfig
WiFiClient wificlient;
Preferences preferences;

// flag: sd card is available
bool hasCard = false;

// these i2c modules are directly part of the stack, we don't have to wait for the hub
#ifdef USE_I2C_GRBL
#ifdef I2C_ADDR_GRBL_LOWER
GRBL _GRBL_LOWER = GRBL(I2C_ADDR_GRBL_LOWER);
#endif
#ifdef I2C_ADDR_GRBL_UPPER
GRBL _GRBL_UPPER = GRBL(I2C_ADDR_GRBL_UPPER);
#endif
#endif

#ifdef USE_LORAWAN
SoftwareSerial loraSerial(LORAWAN_RX, LORAWAN_TX);

char lorabuffer[256];
char *devEui = DEVICE_EUI;
char *appEui = APP_EUI;
char *appKey = APP_KEY;
#endif

#ifdef USE_GPS
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
#endif

#ifdef USE_IOT_BASE
TinyGsm modem(SerialAT);
TinyGsmClient tcpclient(modem);

const char apn[]      = GPRS_APN;
const char gprsUser[] = GPRS_USER;
const char gprsPass[] = GPRS_PASSWORD;

void nbConnect(void *);
#endif

#ifdef USE_I2C_DLIGHT
M5_DLight *dlight;
#endif

#ifdef USE_I2C_ENVII
SHT3X *sht30;
Adafruit_BMP280 *bme;
#endif

// ENVIII would need
// QMP6988 *qmp6988; // this will not work if using an unmodified hub, because it has the same address (0x70) as the tca9548 hub and the motor driver :(

#ifdef USE_I2C_ADC
ADS1100 *ads;
#endif

#ifdef USE_I2C_KMETER
M5_KMeter *kmeter;
#endif

#ifdef USE_I2C_4RELAY
UNIT_4RELAY *unit_4relay;
#endif

#ifdef USE_I2C_SONIC
SONIC_I2C *sonic;
#endif

#ifdef USE_I2C_ADC
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

// compute the pH value based on the voltage
double compute_ph(double voltage) {
  return voltage > 0 ? PH_FIT_A * voltage * voltage + PH_FIT_B * voltage + PH_FIT_C : 0;
}
#endif

#ifdef USE_I2C_HUB
TCA9548A hub(I2C_ADDR_HUB);
#endif

#ifdef USE_MODBUS
esp32ModbusRTU modbus(&Serial2);
#endif

#ifdef USE_I2C_CAM
uint8_t img[10240];
#define BLOCKSIZE 128
HTTPClient http;
#endif

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

void m5update(void *);

void log(String info) {
    Serial.println(info);
}

void init_sd() {
  if (!SD.begin()) {  // Initialize the SD card.
      M5.Lcd.println(
          "Card failed, or not present");  // Print a message if the SD card
                                            // initialization fails or if the
                                            // SD card does not exist
      delay(2000);
  } else {
    hasCard = true;
  }
}

void init_wifi() {
  WiFi.mode(WIFI_AP_STA);
  wifi_config_t conf;
  esp_wifi_get_config(WIFI_IF_STA, &conf);
  String rssiSSID = String(reinterpret_cast<const char*>(conf.sta.ssid));
  String password = String(reinterpret_cast<const char*>(conf.sta.password));
  preferences.begin("wifi", false);
  String PrefSSID =  preferences.getString("ssid", "none");      //NVS key ssid
  String PrefPassword = preferences.getString("password", "none");  //NVS key password
  preferences.end();
  if ((!rssiSSID.equals(PrefSSID)) || (!password.equals(PrefPassword))) {
    M5.Lcd.setTextSize(5);
    M5.Lcd.drawString("ESP smart config ...", 0, 0);
    M5.Lcd.drawString("Use the app", 0, 40);
    M5.Lcd.drawString("to set up WiFi", 0, 80);

    WiFi.beginSmartConfig();
    int waitfor = 0;
    while ((!WiFi.smartConfigDone()) && (waitfor < 240)) {
      delay(500);
      Serial.print(".");
      waitfor++;
    }
    if (WiFi.smartConfigDone()) {
      Serial.println("smart config done, connect");
      while( WiFi.status() != WL_CONNECTED )    	// check till connected
      { 
        delay(2000);
        Serial.print(".");
      }
      preferences.begin("wifi", false);      // put it in storage
      preferences.putString( "ssid", WiFi.SSID());
      preferences.putString( "password", WiFi.psk());
      preferences.end();
      ESP.restart();
    }
    Serial.println("could not connect, abort smarconfig");
    WiFi.stopSmartConfig();
  }
  if (rssiSSID.equals(PrefSSID) && password.equals(PrefPassword)) {
    WiFi.begin( PrefSSID.c_str() , PrefPassword.c_str() );
  }
}

void init_lcd() {
  M5.Lcd.setBrightness(DEFAULT_BRIGHTNESS);
  M5.Lcd.fillScreen(BLACK);
  delay(500);
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
}

#ifdef USE_MODBUS
void init_modbus() {
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
}
#endif

void init_i2c() {
#ifdef USE_I2C_GRBL
  // the motor drivers are directly connected to the i2c bus, thus we do not need to wait for the hub
#ifdef I2C_ADDR_GRBL_LOWER
  _GRBL_LOWER.Init(&Wire);
  _GRBL_LOWER.setMode("absolute");
#endif
#ifdef I2C_ADDR_GRBL_UPPER
  _GRBL_UPPER.Init(&Wire);
  _GRBL_UPPER.setMode("absolute");
#endif
#endif

#ifdef USE_I2C_HUB
  // Open the i2c hub
  hub.begin();
  hub.openAll(); // the sensors (currently) have no conflicting addresses, so we can have all channels activated
  // hub.closeChannel(4); // ADC - water
  // hub.closeChannel(5); // ADC - pH sensor
  // it is important to initialize the sensors *after* the hub channels are open, otherwise the devices are not found!
#endif

#ifdef USE_I2C_DLIGHT
  dlight = new M5_DLight();
  dlight->begin();
  dlight->setMode(CONTINUOUSLY_H_RESOLUTION_MODE);
#endif

  // this would be for the ENVIII
  //qmp6988 = new QMP6988();
  //qmp6988->init();

#ifdef USE_I2C_KMETER
  kmeter = new M5_KMeter();
  kmeter->begin();
#endif

#ifdef USE_I2C_ENVII
  sht30 = new SHT3X();
  bme = new Adafruit_BMP280();
  if (!bme->begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      // while (1);
  }
  Serial.print("\n\rCalibrate done..");
#endif

#ifdef USE_I2C_ADC
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
#endif

#ifdef USE_I2C_4RELAY
  unit_4relay = new UNIT_4RELAY();
  unit_4relay->Init(1); // mode 0: async, 1: sync

  // unit_4relay->relayWrite(0, true);
#endif

#ifdef USE_I2C_SONIC
  sonic = new SONIC_I2C();
  sonic->begin();
#endif
}

#ifdef USE_LORAWAN
CayenneLPP lpp(160);

void init_lorawan() {
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
}
#endif

void setup() {
  Serial.write("Start.");
  disableCore0WDT(); // disable the watchdog on core 0, this only causes issues (it is disabled on core 1 by default)

  Wire.setPins(SDA, SCL);
  M5.begin(true, true, true, false); // do not start I2C here
  M5.Power.begin();

  init_sd();

  init_wifi();

  init_lcd();

#ifdef USE_MODBUS
  init_modbus();
#endif
  
  init_i2c();

#ifdef USE_GPS
  gpsSerial.begin(GPS_BAUD, SWSERIAL_8N1, GPS_RX, GPS_TX);
#endif
  
#ifdef USE_LORAWAN
  init_lorawan();
#endif
  
#ifdef USE_IOT_BASE
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
#endif

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
        log("data.csv doesn't exist.");
        File myFile = SD.open("/data.csv",
                          FILE_WRITE);  // Create a new file "/hello.txt".
        if (myFile) {
          myFile.println("unixtime,temperature,humidity,pressure,light,ph,ec,nitrogen,phosphorus,potassium,distance,wtemperature");
          myFile.close();
        }
    }
  }
}

#ifdef USE_I2C_CAM
void save_image(bool upload=false) {
  uint8_t lenbuf[2];
  lenbuf[0] = 0;
  lenbuf[1] = 0;
  Serial.println("about to read camera image...");
  Wire.flush();
  Wire.requestFrom(I2C_ADDR_CAM, 2, 1); // request size
  // sleep(1);
  bool errored = false;
  Serial.printf("available: %d\n", Wire.available());
  for (int i=0; i<2; i++) {
    if (!Wire.available()) {
      errored = true;
      break;
    }
    lenbuf[i] = (uint8_t)Wire.read();
  }
  int16_t length = (uint16_t) (lenbuf[0]);
  length = (length << 8) + lenbuf[1];
  if (errored || length > 10240) {
    Serial.printf("could not read length, abort");
    return;
  }
  Serial.printf("len: %d", length);
  
  int rbytes = 0;
  bool notdone = true;
  errored = false;
  int blockno = 0;
  int remaining = length;
  // read the image 128b-block-by-block
  while(rbytes < length && (!errored)) {
    //Serial.printf("reading block %d", blockno);
    remaining = length - rbytes;
    if (remaining > BLOCKSIZE) {
      Wire.requestFrom(I2C_ADDR_CAM, BLOCKSIZE, 0);
    } else {
      Wire.requestFrom(I2C_ADDR_CAM, remaining, 1);
    }
    for (int i=0; i<BLOCKSIZE && rbytes<length; i++,rbytes++) {
      if (!Wire.available()) {
        errored = true;
        break;
      }
      img[rbytes] = Wire.read();
      // TODO: write this part to sdcard
    }
    blockno++;
  }
  while(Wire.available()) Wire.read();
  if (errored) {
    Serial.println("errored is set!");
  }
  Serial.printf("read %d bytes", rbytes);
  if (length > 0 && rbytes == length) {
    if (hasCard) {
      File myFile = SD.open("/img.jpg", FILE_WRITE);
      if (myFile) {
        myFile.write(img, length);
      }
      myFile.close();
    }
    if(upload && WiFi.status() == WL_CONNECTED) {
      http.setConnectTimeout(5000);
      http.setTimeout(10000);
      bool success = false;
      for(int i=0;i<10&&(!success);i++) {
        if (http.begin(wificlient, CAM_UPLOAD_URL)) {
          log("about to post image");
          http.addHeader("Authorization", String("Basic ") + String(UPLOAD_BASIC));
          int code = http.POST(img, length);
          http.end();
          log("code:" + String(code));
          if (code >= 200 && code < 400) success = true;
        }
        if (!success) sleep(2);
      }
    }
  }
}
#endif

String createBody() {
  String body = "";
  body += "environment,device=" + String(DEVICE_ID) + " temperature=" + String(temp, 1) + ",humidity=" + String(humid, 1) + ",pressure=" + String(press, 1) + ",light=" + String((float)light, 1);
  if(unixtime > 943920000L) {
    body += " " + String(unixtime) + "000000000";
  }
  body += "\n";
  body = "water,device=" + String(DEVICE_ID) + " ph=" + String(ph, 1) + ",ec=" + String(ec, 1) + ",nitrogen=" + String((float)nitrogen, 1) + ",phosphorus=" + String((float)phosphorus, 1) + ",potassium=" + String((float)potassium, 1) + ",distance=" + String(distance, 1) + ",temperature=" + String(water_temp,1);
  if(unixtime > 943920000L) {
    body += " " + String(unixtime) + "000000000";
  }
  body += "\n";
  if (lon != 0 && lat != 0) {
    body = "position,device=" + String(DEVICE_ID) + " longitude=" + String(lon, 6) + ",latitude=" + String(lat, 6) + ",altitude=" + String(alt, 1) + ",hdop=" + String(hdop, 1);
    if(unixtime > 943920000L) {
      body += " " + String(unixtime) + "000000000";
    }
    body += "\n";
  }
  return body;
}

unsigned long lastSent = 0;

void sendobject() {
  if(WiFi.status() == WL_CONNECTED) {
    if (!wificlient.connect(TELEGRAF_TCP_ADDR, TELEGRAF_TCP_PORT, 10000)) {
      log("could not connect");
    } else {
      log("send data to influx");
      wificlient.print(createBody());
    }
  }
  if (millis() - lastSent < 15 * 60000) {
    save_image();
    return;
  }
  save_image(true);
  lastSent = millis();
  bool result = false;

#ifdef USE_LORAWAN
  Serial.println("Sending via LoRaWan...");
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

  if (result == true) {
    Serial.println("Sent");
  } else {
    Serial.println("Error");
  }
#endif
}

void fetchData(){
#ifdef USE_I2C_DLIGHT
  vTaskDelay(100 / portTICK_PERIOD_MS);
  light = dlight->getLUX();
#endif

#ifdef USE_I2C_ENVII
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // press = qmp6988->calcPressure() / 100.0; // hPa this would be for ENVIII
  press = bme->readPressure()*0.01;

  if (sht30->get() == 0) {
    temp = sht30->cTemp;
    temp_valid = true;
    humid = sht30->humidity;
  } else {
    temp = 0.0;
    temp_valid = false;
    humid = -1;
  }
#endif

#ifdef USE_I2C_KMETER
  vTaskDelay(100 / portTICK_PERIOD_MS);
  water_temp = kmeter->getTemperature();
  log("water temp="+String(water_temp, 1));
#endif

#ifdef USE_I2C_ADC
  double ph_voltage = (double)ads->Measure_Differential() * ((double)ADC_VCC) / ((double)ADC_FACTOR) / (-(double)ADC_MIN);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  log("ph_v = " + String(ph_voltage));
  ph = compute_ph(ph_voltage);
#endif

#ifdef USE_I2C_SONIC
  distance = sonic->getDistance(); // in mm
  log("distance = " + String(distance,1));
#endif

#ifdef USE_GPS
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
  if (gps.time.isValid()) {
    log("gps time=" + String(gps.time.value()));
  }
#endif

#ifdef USE_IOT_BASE
  if (modem.isNetworkConnected()) {
    log("gsm datetime=" + modem.getGSMDateTime(TinyGSMDateTimeFormat::DATE_FULL));
  }
#endif

#ifdef USE_MODBUS
  Serial.print("sending Modbus request...\n");
  modbus.readHoldingRegisters(0x02, 0x1e, 3); // npk
  vTaskDelay(100 / portTICK_PERIOD_MS);
  modbus.readHoldingRegisters(0x01, 0x00, 2); // ec
  vTaskDelay(100 / portTICK_PERIOD_MS);
#endif

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

#ifdef USE_IOT_BASE
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

    bool success = false;
    for(int i=0; i<10&&(!success); i++) {
      if (tcpclient.connect(TELEGRAF_TCP_ADDR, TELEGRAF_TCP_PORT)) {
        success = true;
        String body = createBody();
        tcpclient.print(body);
        tcpclient.stop();
      }
      sleep(1);
    }
    vTaskDelay(60 * 60000 / portTICK_PERIOD_MS); // 1 hour
  }
}
#endif

// display mode
// 0: measurements
// 1: cam image
// 2: QR code
// 3: pump control
uint8_t mode = 0;

// submode
// measurements:
// 0: water (NPK, EC)
// 1: water (temp, pH, EC)
// 2: env (temp, hum, light)
// 3: env (press, water level)
//
// pump control:
// 0: pH+
// 1: pH-
// 2: nutrition+
// 3: nutrition-
uint8_t submode = 0;

unsigned long last_refresh = 0;
#define REFRESH_MS 5000

// LCD size 320x240

void refreshDisplay() {
  if (millis() - last_refresh < REFRESH_MS) {
    return;
  }
  last_refresh = millis();
  M5.Lcd.fillScreen(BLACK);

  switch(mode) {
    case 0:
      switch(submode) {
        case 0:
          // water: NPK, EC
          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("EC", 0, 0);
          M5.Lcd.drawString("mS/cm", 0, 25);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(ec / 1000.0f, 1), 10, 65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Nitrogen", 160, 0);
          M5.Lcd.drawString("mg/l", 160, 25);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(nitrogen), 170, 65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Phosphor", 0, 120);
          M5.Lcd.drawString("mg/l", 0, 145);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(phosphorus), 10, 120+65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Potassium", 160, 120);
          M5.Lcd.drawString("mg/l", 160, 145);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(potassium), 160+10, 120+65);

          // 320 / 3 ~107 arrow length ~40 -> 33 + 40 + 33
          M5.Lcd.drawLine(33, 235, 33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 231, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 239, TFT_BLUE);

          M5.Lcd.drawLine(107+33, 235, 107+33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 231, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 239, TFT_BLUE);

          M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);

          break;

        case 1:
          // water: EC, temperature, pH
          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("pH", 0, 0);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(ph, 1), 10, 65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Temp.", 160, 0);
          M5.Lcd.drawString("degC", 160, 25);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(water_temp, 1), 170, 65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("EC", 0, 120);
          M5.Lcd.drawString("uS/cm", 0, 145);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(ec), 10, 120+65);

          // 320 / 3 ~107 arrow length ~40 -> 33 + 40 + 33
          M5.Lcd.drawLine(33, 235, 33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 231, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 239, TFT_BLUE);

          M5.Lcd.drawLine(107+33, 235, 107+33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 231, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 239, TFT_BLUE);

          M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);

          break;

        case 2:
          // env: temperature, rel.hum., light
          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Temperature", 0, 0);
          M5.Lcd.drawString("degC", 0, 25);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(temp, 1), 10, 65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Rel.hum.", 160, 0);
          M5.Lcd.drawString("%", 160, 25);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(humid), 170, 65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("amb.light", 0, 120);
          M5.Lcd.drawString("lux", 0, 145);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(light), 10, 120+65);

          // 320 / 3 ~107 arrow length ~40 -> 33 + 40 + 33
          M5.Lcd.drawLine(33, 235, 33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 231, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 239, TFT_BLUE);

          M5.Lcd.drawLine(107+33, 235, 107+33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 231, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 239, TFT_BLUE);

          M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);

          break;

        case 3:
          // env: pressure, water level
          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Pressure", 0, 0);
          M5.Lcd.drawString("mbar", 0, 25);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(press, 1), 10, 65);

          M5.Lcd.setTextSize(3);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("Water level", 0, 120);
          M5.Lcd.drawString("mm", 0, 145);
          M5.Lcd.setTextSize(6);
          M5.Lcd.setTextColor(TFT_GREEN);
          M5.Lcd.drawString(String(BOX_HEIGHT-distance), 10, 120+65);

          // 320 / 3 ~107 arrow length ~40 -> 33 + 40 + 33
          M5.Lcd.drawLine(33, 235, 33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 231, TFT_BLUE);
          M5.Lcd.drawLine(33, 235, 33+2, 239, TFT_BLUE);

          M5.Lcd.drawLine(107+33, 235, 107+33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 231, TFT_BLUE);
          M5.Lcd.drawLine(107+33+40, 235, 107+33+40-2, 239, TFT_BLUE);

          M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);

          break;
      }
      break;

    case 1:
      M5.Lcd.drawJpgFile(SD, "/img.jpg");
      M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);
      break;

    case 2:
      M5.Lcd.qrcode(QR_URL);
      M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);
      break;

    case 3:
      switch(submode) {
        case 0:
          // npk+
          M5.Lcd.setTextSize(7);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("NPK+", 100, 120);

          M5.Lcd.drawLine(33, 235, 33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(33+40, 235, 33+40-2, 231, TFT_BLUE);
          M5.Lcd.drawLine(33+40, 235, 33+40-2, 239, TFT_BLUE);

          M5.Lcd.fillCircle(159, 214, 25, TFT_GREEN);

          M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);
          break;

        case 1:
          // ph+
          M5.Lcd.setTextSize(7);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("pH+", 100, 120);

          M5.Lcd.drawLine(33, 235, 33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(33+40, 235, 33+40-2, 231, TFT_BLUE);
          M5.Lcd.drawLine(33+40, 235, 33+40-2, 239, TFT_BLUE);

          M5.Lcd.fillCircle(159, 214, 25, TFT_GREEN);

          M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);
          break;

        case 2:
          // ph-
          M5.Lcd.setTextSize(7);
          M5.Lcd.setTextColor(TFT_WHITE);
          M5.Lcd.drawString("pH-", 100, 120);

          M5.Lcd.drawLine(33, 235, 33+40, 235, TFT_BLUE);
          M5.Lcd.drawLine(33+40, 235, 33+40-2, 231, TFT_BLUE);
          M5.Lcd.drawLine(33+40, 235, 33+40-2, 239, TFT_BLUE);

          M5.Lcd.fillCircle(159, 214, 25, TFT_GREEN);

          M5.Lcd.fillCircle(107*2 + 107/2, 235, 4, TFT_RED);
          break;
      }
      break;
  }
}

unsigned long last_activity = 0;
bool blank = false;

void wakeup_lcd() {
  M5.Lcd.wakeup();
  M5.Lcd.setBrightness(DEFAULT_BRIGHTNESS);
  blank = false;
}

void button_pressed(uint8_t no) {
  last_activity = millis();
  if (blank) {
    wakeup_lcd();
    return;
  }
  switch(mode) {
    case 0:
      switch (no) {
        case 0:
          if (submode > 0) submode--;
          else submode = 3;
          break;
        case 1:
          if (submode < 3) submode++;
          else submode = 0;
          break;
        case 2:
          mode++;
          submode = 0;
          break;
      }
      break;

    case 1:
      switch (no) {
        case 2:
          mode++;
          break;
        default:
          if (WiFi.status() != WL_CONNECTED) {
            WiFi.beginSmartConfig();
            int waitfor = 0;
            while ((!WiFi.smartConfigDone()) && (waitfor < 100)) {
              delay(500);
              Serial.print(".");
              waitfor++;
            }
            if (WiFi.smartConfigDone()) {
              Serial.println("smart config done, connect");
              while( WiFi.status() != WL_CONNECTED )    	// check till connected
              { 
                delay(2000);
                Serial.print(".");
              }
              preferences.begin("wifi", false);      // put it in storage
              preferences.putString( "ssid", WiFi.SSID());
              preferences.putString( "password", WiFi.psk());
              preferences.end();
              ESP.restart();
            }
            Serial.println("could not connect, abort");
            WiFi.stopSmartConfig();
          }
          break;
      }
      break;

    case 2:
      switch (no) {
        case 2:
          mode++;
          break;
        default:
          if (WiFi.status() != WL_CONNECTED) {
            WiFi.beginSmartConfig();
            int waitfor = 0;
            while ((!WiFi.smartConfigDone()) && (waitfor < 100)) {
              delay(500);
              Serial.print(".");
              waitfor++;
            }
            if (WiFi.smartConfigDone()) {
              Serial.println("smart config done, connect");
              while( WiFi.status() != WL_CONNECTED )    	// check till connected
              { 
                delay(2000);
                Serial.print(".");
              }
              preferences.begin("wifi", false);      // put it in storage
              preferences.putString( "ssid", WiFi.SSID());
              preferences.putString( "password", WiFi.psk());
              preferences.end();
              ESP.restart();
            }
            Serial.println("could not connect, abort");
            WiFi.stopSmartConfig();
          }
          break;
      }
      break;

    case 3:
      switch (no) {
        case 0:
          if (submode < 2) submode++;
          else submode = 0;
          break;
        case 1:
          switch(submode) {
            case 0:
              // npk+
              _GRBL_LOWER.setMotor(5,0,0,200);
              _GRBL_LOWER.setMotor(0,0,0,200);
              break;

            case 1:
              // ph+
              _GRBL_LOWER.setMotor(0,5,0,200);
              _GRBL_LOWER.setMotor(0,0,0,200);
              break;

            case 2:
              // ph-
              _GRBL_LOWER.setMotor(0,0,5,200);
              _GRBL_LOWER.setMotor(0,0,0,200);
              break;
          }
          break;
        case 2:
          mode = 0;
          submode = 0;
          break;
      }
      break;
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
    if (a > DEMO_MODE_MS) {
      if(blank) {
        wakeup_lcd();
      }
      refreshDisplay();
    } else {
      if (!blank) {
        if (a > SCREENSAVE_MS) {
          blank = true;
          M5.Lcd.clearDisplay();
          M5.Lcd.sleep();
          M5.Lcd.setBrightness(0);
        } else {
          refreshDisplay();
        }
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
#ifdef USE_GPS
    while (gpsSerial.available()) {
      byte b = gpsSerial.read();
      gps.encode(b);
    }
#endif
    vTaskDelay(100 / portTICK_PERIOD_MS);
  } while (millis() - start < ms);
}

void loop() {
  fetchData();
  sendobject();
  smartDelay(10000);
}
