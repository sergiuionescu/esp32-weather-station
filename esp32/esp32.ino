#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include "DHT12.h"
#include "Wire.h"
#include "SPI.h"
#include "Adafruit_BMP280.h"
#include "MPU9250.h"
#include "MAX44009.h"


const char* ssid     = "ESP32-weather-station";
const char* password = "987654321";

String wifi_ssid;
String wifi_password;

float temperature;
float humidity;
float heat_index;
float dew_point;

float bmp_temperature;
float pressure;
float altitude;

float accel_x;
float accel_y;
float accel_z;
float gyro_x;
float gyro_y;
float gyro_z;
float mag_x;
float mag_y;
float mag_z;

float lux;

IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

AsyncWebServer server(80);

DHT12 dht12;
Adafruit_BMP280 bmp;
MPU9250 MPU(Wire,0x68);
MAX44009 max44009;

int timeSinceLastRead = 0;

void setup() {
  Serial.begin(115200);

  dht12.begin();
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); 

  if (!MPU.begin()) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while(1) {}
  }

  if(max44009.begin()) {
    Serial.println("Could not find a valid MAX44009 sensor, check wiring!");
    while(1);
  }

  // setting the accelerometer full scale range to +/-8G 
  MPU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  MPU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  MPU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  MPU.setSrd(19);

  connectOrAp();

  server.on("/data", handleData);
  server.on("/login", handleLogin);
  server.on("/scan", handleScan);
  server.on("/connect", handleConnect);
  
  server.begin();

  Serial.println("Setup done");
}

void loop() {
  readMPU();
  readBmp();
  readMax44009();
  readDhp12();
}

void handleData(AsyncWebServerRequest *request) {
  String responseText;

  AsyncResponseStream *response = request->beginResponseStream("application/json");
  
  const size_t capacity = JSON_OBJECT_SIZE(17);
  DynamicJsonDocument doc(capacity);
  
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["heat_index"] = heat_index;
  doc["dew_point"] = dew_point;
  doc["bmp_temperature"] = bmp_temperature;
  doc["pressure"] = pressure;
  doc["altitude"] = altitude;
  doc["accel_x"] = accel_x;
  doc["accel_y"] = accel_y;
  doc["accel_z"] = accel_z;
  doc["gyro_x"] = gyro_x;
  doc["gyro_y"] = gyro_y;
  doc["gyro_z"] = gyro_z;
  doc["mag_x"] = mag_x;
  doc["mag_y"] = mag_y;
  doc["mag_z"] = mag_z;
  doc["lux"] = lux;

  serializeJson(doc, responseText);
  response->print(responseText);
  
  request->send(response);
}

void handleConnect(AsyncWebServerRequest *request) {
  int params = request->params();
  for(int i=0;i<params;i++){
    AsyncWebParameter* param = request->getParam(i);
    if(param->name() == "ssid") {
      wifi_ssid = param->value();
    }
    if(param->name() == "password") {
      wifi_password = param->value();
    }
  }

  Serial.println(wifi_ssid.c_str());

  WiFi.disconnect();
  delay(100);
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
}

void handleLogin(AsyncWebServerRequest *request) {
  String responseText = "";
  request->send(200, "text/html", "<html>  <head>    <style>        div.container {        max-width: 200px;        max-height: 200px;        width: 100%;        height: 100%;        top: 0;        bottom: 0;        left: 0;        right: 0;        margin: auto;        }        img {        width: 100%;        height: 100%;        top: 0;        bottom: 0;        left: 0;        right: 0;        margin: auto;        }          form {    border: 3px solid #f1f1f1;    }    input[type=text], input[type=password] {    width: 100%;    padding: 12px 20px;    margin: 8px 0;    display: inline-block;    border: 1px solid #ccc;    box-sizing: border-box;    }    button {    background-color: #4CAF50;    color: white;    padding: 14px 20px;    margin: 8px 0;    border: none;    cursor: pointer;    width: 100%;    }    button:hover {    opacity: 0.8;    }    </style>  </head>  <body>        <div class='container'>        <img id='loader' style='display: none;' src='data:image/gif;base64,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' />        <form method='POST' action='/connect' id='form'>            <label for='ssid'>SSID</label>            <select name='ssid' id='ssid'></select>            <label for='ssid'>Password</label>            <input type='password' name='password' id='password'/>            <input type='submit' />            <input type='button' value='Refresh' onclick='scan();' />        </form>    </div>    <script>        function scan(){            var xhttp = new XMLHttpRequest();            xhttp.onreadystatechange = function() {                if (this.readyState == 4 && this.status == 200) {                    var response = JSON.parse(this.responseText.trim());                    console.log(this.responseText);                    var select = document.getElementById('ssid');                    select.innerHTML = '';                    for (i = 0; i < response.length; i++) {                        var option = document.createElement('option');                        if(response[i]['ssid'] && response[i]['rssi']) {                            option.text = response[i]['ssid'] + '(' + response[i]['rssi'] + ' %)';                            option.value = response[i]['ssid'];                            select.add(option);                        }                    }                    document.getElementById('loader').style.display = 'none';                    document.getElementById('form').style.display = 'inline';                }            };            xhttp.onerror = function(e) {                setTimeout(scan, 1000);            };            document.getElementById('loader').style.display = 'inline';            document.getElementById('form').style.display = 'none';            xhttp.open('GET', '/scan', true);            xhttp.send();      }       scan();    </script>  </body></html>\n");
}

void handleScan(AsyncWebServerRequest *request) {
  String responseText;
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  int n = WiFi.scanNetworks();

  const size_t capacity = JSON_ARRAY_SIZE(n) + n * JSON_OBJECT_SIZE(3);
  DynamicJsonDocument doc(capacity);
  
  for (int i = 0; i < n; ++i) {
    JsonObject wifi = doc.createNestedObject();
    wifi["ssid"] = WiFi.SSID(i);
    wifi["rssi"] = 100 + WiFi.RSSI(i);
    wifi["isOpen"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "1" : "0";
  }
  
  serializeJson(doc, responseText);
  response->print(responseText);
  
  request->send(response);
}

void connectOrAp() {
  WiFi.begin();
  for(int i=0;i<10;i++){
     if(WiFi.status() == WL_CONNECTED) {
      delay(100);
      break;
    }
    delay(500);
    Serial.println("Connecting to previous WiFi..");
  }

  if(WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.disconnect();
    delay(100);
    
    Serial.println("Setting AP (Access Point)…");
  
    WiFi.softAP(ssid, password);
    Serial.println("Wait 100 ms for AP_START...");
    delay(100);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);
  } else {
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.gatewayIP().toString());
  }
}

void readDhp12() {
  bool dht12Read = true;
  if(timeSinceLastRead > 2000) {
    float chk_temperature = dht12.readTemperature();
    float chk_humidity = dht12.readHumidity();
   
    if (isnan(chk_temperature) || isnan(chk_humidity)) {
      dht12Read = false;
      Serial.println("Failed to read from DHT12 sensor!");
    }
     
    if (dht12Read){
      temperature = chk_temperature;
      humidity = chk_humidity;
      heat_index = dht12.computeHeatIndex(temperature, humidity, false);
      dew_point = dht12.dewPoint(temperature, humidity, false);
       
      Serial.print("DHT12=> Humidity: ");
      Serial.print(humidity);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" *C ");
      Serial.print(" Heat index: ");
      Serial.print(heat_index);
      Serial.print(" *C ");
      Serial.print(" Dew point: ");
      Serial.print(dew_point);
      Serial.print(" *C ");
      Serial.print("\n");
    }
      timeSinceLastRead = 0;
    }
  delay(100);
  timeSinceLastRead += 100;
  
}

void readBmp() {
  if(timeSinceLastRead > 2000) {
    bmp_temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1013.25);
  
    Serial.print(F("Temperature = "));
    Serial.print(bmp_temperature);
    Serial.print(" *C");

    Serial.print(F(" Pressure = "));
    Serial.print(pressure);
    Serial.print(" Pa");

    Serial.print(F(" Approx altitude = "));
    Serial.print(altitude); 
    Serial.print(" m");
    Serial.print("\n");

    delay(100);
  }
}

void readMPU() {
  // read the sensor
  MPU.readSensor();

  accel_x = MPU.getAccelX_mss();
  accel_y = MPU.getAccelY_mss();
  accel_z = MPU.getAccelZ_mss();
  gyro_x = MPU.getGyroX_rads();
  gyro_y = MPU.getGyroY_rads();
  gyro_z = MPU.getGyroZ_rads();
  mag_x = MPU.getMagX_uT();
  mag_y = MPU.getMagY_uT();
  mag_z = MPU.getMagZ_uT();

  // display the data
  Serial.print(MPU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(MPU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(MPU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(MPU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(MPU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(MPU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(MPU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(MPU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(MPU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(MPU.getTemperature_C(),6);
  delay(20);
}

void readMax44009() {
  lux = max44009.get_lux();
  Serial.print("Light (lux):    ");
  Serial.println(lux);
}
