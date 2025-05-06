///////// ESP32-C3 SuperMINI inverted Pendulum ////////
/////////////// Practice3 IMU pcocessing //////////////
////////////////// 2025/04/28 by hir. /////////////////

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Wire.h>

// LED_PIN
#define LED1_PIN 1
#define LED2_PIN 8

//MPU6050 I2C PIN
#define SDA 8 // on board LED
#define SCL 9 // GPIO9を0にして電源ONにするとDownloadBootモードで起動

////////// MPU6050 処理関連 //////////
#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_GYRO_LSB     65.5
#define MPU6050_ACCEL_LSB    8192.0

////////// mpu6050 計算用係数 //////////
float y_rad, y_last;
double offsetX = 0, offsetY = 0, offsetZ = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;
bool dmpReady = false;

////////// BLE 処理関連 //////////
#define DEVICENAME "ESP32C3_name"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
boolean isrequested = false;
bool led_bool = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    //Serial.println("** device connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    //Serial.println("** device disconnected");
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      // //Display on serial monitor for debug
      // Serial.println("*********");
      // Serial.print("Received Value: ");
      // //rxValue.trim();
      // Serial.println(rxValue.c_str());
      // Serial.println("*********");
      // //Reply as is
      pTxCharacteristic->setValue(rxValue.c_str());
      pTxCharacteristic->notify();
      delay(10);

      ///////////////////////////////////////////////////////////
      //////////////////////// status 管理 //////////////////////
      if (rxValue.find("start") == 0) {       // BLE start
        isrequested = true;
      } else if (rxValue.find("quit") == 0) { // BLE finished
        isrequested = false;
      }
    }
  }
};

////////// timer intrrupt 関連 //////////
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
u_int32_t isrCounter = 0; // timer内処理完結なので vola要りません
volatile bool first_timer = false, second_timer = false, third_timer = false;
bool timer_enable = false; // mpu6050 の初期化待ち flag

////////// Timer 実行処理 //////////
void ARDUINO_ISR_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  // Depending on the count, you can safely set a flag here.
  if (isrCounter %    5 == 0){ // 5msc timer
    first_timer = true;
  }
  if (isrCounter %  200 == 0){ // 200msc timer
    second_timer = true;
  }
  if (isrCounter % 1000 == 0){ // 1000msc timer
    third_timer = true;
  }
}

////////// LED 処理 //////////
void led_blink(u8_t pin){
  digitalWrite(pin, !digitalRead(pin));
}

////////// I2C 処理 //////////
// i2c write
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// i2C read
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}

////////// MPU6050 初期設定 //////////
void setup_mpu6050(){
  Wire.begin(SDA, SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(50);

  ///// set Register
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x01); // 1kHz/(1+(1))
  writeMPU6050(MPU6050_CONFIG,     0x01); // DLPF No1
  uint8_t data = readMPU6050(MPU6050_GYRO_CONFIG);
  data = ( data & 0b11100000 );
  data = ( data | 0b00001000 );           // 500deg/s (5ms loop => 2.5deg/loop)
  writeMPU6050(MPU6050_GYRO_CONFIG, data);
  data = readMPU6050(MPU6050_ACCEL_CONFIG);
  data = ( data & 0b11100000 );
  data = ( data | 0b00001000 );           // 4g
  writeMPU6050(MPU6050_ACCEL_CONFIG, data);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x00);

  ///// set Calibration
  //Serial.print("Calculate Calibration");
  for(int i = 0; i < 3000; i++){
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);
  
    raw_acc_x = Wire.read() << 8 | Wire.read();
    raw_acc_y = Wire.read() << 8 | Wire.read();
    raw_acc_z = Wire.read() << 8 | Wire.read();
    raw_t = Wire.read() << 8 | Wire.read();
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();
    dpsX = ((float)raw_gyro_x) / MPU6050_GYRO_LSB;
    dpsY = ((float)raw_gyro_y) / MPU6050_GYRO_LSB;
    dpsZ = ((float)raw_gyro_z) / MPU6050_GYRO_LSB;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;
    delay(1);
 
  }
  offsetX /= 3000;
  offsetY /= 3000;
  offsetZ /= 3000;
  
  dmpReady = true;
}

////////// MPU6050 data読み込み, 相補ﾌｨﾙﾀｰ //////////
void calcRotation(){
  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  //レジスタアドレス0x3Bから、計14バイト分のデータを出力するようMPU6050へ指示
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  
  //出力されたデータを読み込み、ビットシフト演算
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
  
  //単位Gへ変換
  acc_x = ((float)raw_acc_x) / MPU6050_ACCEL_LSB;
  acc_y = ((float)raw_acc_y) / MPU6050_ACCEL_LSB;
  acc_z = ((float)raw_acc_z) / MPU6050_ACCEL_LSB;
  
  //加速度センサーから角度を算出
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 /  2.0 / PI;

  //単位をd/sに変換
  dpsX = ((float)raw_gyro_x) / MPU6050_GYRO_LSB;
  dpsY = ((float)raw_gyro_y) / MPU6050_GYRO_LSB;
  dpsZ = ((float)raw_gyro_z) / MPU6050_GYRO_LSB;
  
  //前回計算した時から今までの経過時間を算出
  interval = micros() - preInterval; // millis => micros
  preInterval = micros();
  
  //数値積分
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.000001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.000001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.000001);
  
  ////////////////////////////////////////////////////////
  
  ///// 相補フィルター k=0.996, 0.004
  angleX = (0.98 * gyro_angle_x) + (0.02 * acc_angle_x);
  angleY = (0.98 * gyro_angle_y) + (0.02 * acc_angle_y);
  angleZ = gyro_angle_z;

  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;

  // y軸角度の更新
  y_last = y_rad;
  y_rad = angleY;
}

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  // MPU 6050 connect & initialize
  setup_mpu6050();

  // Create the BLE Device
  BLEDevice::init(DEVICENAME); //BLE Device Name scaned and found by clients

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY );
                    
  // pTxCharacteristic に BLE2902 を追加。BLE Client の Notify (Indicate) を有効化
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_RX,
                    BLECharacteristic::PROPERTY_WRITE );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  //Serial.println("start advertising");
  //Serial.println("Waiting a client connection to notify...");

  //The ESP32S3 bluetoth 5.0 requires security settings.
  //Without it, an error will occur when trying to pair with other devices.
  //Using a 6-digit PIN as the authentication method seems to work.
  //This PIN allows the device to be paired with an Client device.
  //Client device users will be prompted to key in a 6-digit PIN, '123456'.
  BLESecurity *pSecurity = new BLESecurity();
  //pSecurity->setStaticPIN(123456);
  //Setting ESP_LE_AUTH_REQ_SC_ONLY instead of the PIN setting eliminates the need for PIN input during pairing.
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);

  ////////// timer count setting //////////
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true);    // prescaler (1usec, increment => 80)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true); // 1000usec timer
  delay(50);
  //timerAlarmEnable(timer);            // timer start!! ==> mpu の初期化後に起動
}

void loop() {
  // If Timer has fired, do the ballance moving!
  if(timer_enable == false){
    if(dmpReady == true){
      // MPU6050 が稼働開始してから、ﾀｲﾏｰ起動
      timerAlarmEnable(timer);
      timer_enable = true;
    }
  }

  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    portEXIT_CRITICAL(&timerMux);
  }

  ////////////////////////////////////////////////////
  if ( first_timer == true ){
    calcRotation();  // y軸角度の更新 相補ﾌｨﾙﾀｰ
    
    first_timer = false;
  }
  ////////////////////////////////////////////////////
  if ( second_timer == true ){
    if (deviceConnected) {
      if (isrequested) {
        char string0[64]; // up to 256
        sprintf(string0, "angleY= %.2f\r\n", angleY); // Y軸の角度出力
        pTxCharacteristic->setValue(string0);
        pTxCharacteristic->notify();
        //pTxCharacteristic->indicate();
      }
    } else {
      isrequested = false;
    }

    second_timer = false;
  }
  ////////////////////////////////////////////////////
  if ( third_timer == true ){
    led_blink(LED1_PIN);
    third_timer = false;
  }
}