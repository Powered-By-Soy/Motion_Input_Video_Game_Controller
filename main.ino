#include <BleConnectionStatus.h>
#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>
#include "Adafruit_VL53L1X.h" // VL53L1X TOF
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h>
#include <Wire.h>

// JOYSTICK SETUP
/////////////////////////////////////////////////////////////////////
// GPIO pin assign
int VRX_PIN_move = 32 ; //analog input for X (12bit ADC 0-4095)
int VRY_PIN_move = 33 ; //analog input for Y
int press_move = 25; //analog input for press the joystick
int VRX_PIN_aim = 4;
int VRY_PIN_aim = 0;
int press_aim = 2;
//variable declearion
int valueX_move = 0;
int valueY_move = 0;
int push_move = 0;
int valueX_aim = 0;
int valueY_aim = 0;
int push_aim = 0;
int valueX_move_zero = 0; //joystick value at zero position
int valueY_move_zero = 0;
int valueX_aim_zero = 0;
int valueY_aim_zero = 0;
/////////////////////////////////////////////////////////////////////

//PB setup
/////////////////////////////////////////////////////////////////////
// GPIO pin assign
const int shootInput = 17;
const int reloadInput = 5;
const int shootLEDOutput = 19;
const int recoilOutput = 27; // recoil
const int reloadLEDOutput = 12;
const int jumpInput = 13;
const int changeWeaponInput = 16;
const int crouchInput = 23;
//variable declearion
int shootState = 0;
int reloadState = 0;
int jumpState = 0;
int changeWeaponState = 0;
int crouchState = 0;
/////////////////////////////////////////////////////////////////////


//ToF setup
/////////////////////////////////////////////////////////////////////
//GPIO pin assign
//const int scopeInput = 16;
const int scopeLEDOutput = 18;
#define SDA_PIN 21 //SDA
#define SCL_PIN 22 //SCL
//variable declearion
bool isScopeOn = false; // flag for opening scope
//int scopeState = 0; //same as above?
Adafruit_VL53L1X VL53L1X = Adafruit_VL53L1X(SDA_PIN, SCL_PIN);
/////////////////////////////////////////////////////////////////////

//MPU6050 setup
/////////////////////////////////////////////////////////////////////
float AG_ratio = 0.95; // %AG_ratio of accelerometer, 1-%AG_ratio of gyroscope
Adafruit_MPU6050 mpu;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float pitchA, rollA, yawA, pitchG, rollG, yawG;
float pitch, roll, yaw;
float current_time, previous_time, delta_time;
float gyroX_offset, gyroY_offset, gyroZ_offset;
float lastPitch, lastRoll, lastYaw, threshold = 1; // degree, usually detect 1-2 degree
float absolutePitch=0.0, absoluteRoll=0.0, absoluteYaw=0.0;

// def MPU6050 return data type 
struct SensorData{
  float pitch;
  float roll;
  float yaw;
};

const int MPURESET = 15; //gpio 15 connect to PB to detect reset MPU6050
int MPU_reset = 0;
/////////////////////////////////////////////////////////////////////

// int ledPin = 5; // LED connected to digital pin 13
XboxGamepadDevice *gamepad;
BleCompositeHID compositeHID("ESP32XC", "Mystfit", 8);
// void OnVibrateEvent(XboxGamepadOutputReportData data)
// {
//     if(data.weakMotorMagnitude > 0 || data.strongMotorMagnitude > 0){
//         digitalWrite(ledPin, LOW);
//     } else {
//         digitalWrite(ledPin, HIGH);
//     }
//     Serial.println("Vibration event. Weak motor: " + String(data.weakMotorMagnitude) + " Strong motor: " + String(data.strongMotorMagnitude));
// }

void setup()
{
    Serial.begin(115200);
    // pinMode(ledPin, OUTPUT); // sets the digital pin as output

    // JOYSTICK
    /////////////////////////////////////////////////////////////////////
    pinMode(VRX_PIN_move, INPUT);
    pinMode(VRY_PIN_move, INPUT);
    pinMode(press_move, INPUT_PULLUP); //use pull up resistor for button funcationality
    pinMode(VRX_PIN_aim, INPUT);
    pinMode(VRY_PIN_aim, INPUT);
    pinMode(press_aim, INPUT_PULLUP); 
    /////////////////////////////////////////////////////////////////////

    // PB
    /////////////////////////////////////////////////////////////////////
    //pinMode(scopeInput, INPUT_PULLUP); //replace by TOF
    pinMode(shootInput, INPUT_PULLUP); // use pull up resistor, when not press PB, read LOW, when press PB read HIGH
    pinMode(reloadInput, INPUT_PULLUP);
    pinMode(scopeLEDOutput, OUTPUT); 
    pinMode(shootLEDOutput, OUTPUT);
    pinMode(recoilOutput, OUTPUT);
    pinMode(reloadLEDOutput, OUTPUT);
    pinMode(jumpInput, INPUT_PULLUP);
    pinMode(changeWeaponInput, INPUT_PULLUP);
    pinMode(crouchInput, INPUT_PULLUP);
    /////////////////////////////////////////////////////////////////////

    // MPU6050
    /////////////////////////////////////////////////////////////////////
    MPUsetup();
    pinMode(MPURESET, INPUT_PULLUP);
    calibrateGyro();
    /////////////////////////////////////////////////////////////////////

    // Uncomment one of the following two config types depending on which controller version you want to use
    // The XBox series X controller only works on linux kernels >= 6.5
    
    //XboxOneSControllerDeviceConfiguration* config = new XboxOneSControllerDeviceConfiguration();
    XboxSeriesXControllerDeviceConfiguration* config = new XboxSeriesXControllerDeviceConfiguration();

    // The composite HID device pretends to be a valid Xbox controller via vendor and product IDs (VID/PID).
    // Platforms like windows/linux need this in order to pick an XInput driver over the generic BLE GATT HID driver. 
    BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
    Serial.println("Using VID source: " + String(hostConfig.getVidSource(), HEX));
    Serial.println("Using VID: " + String(hostConfig.getVid(), HEX));
    Serial.println("Using PID: " + String(hostConfig.getPid(), HEX));
    Serial.println("Using GUID version: " + String(hostConfig.getGuidVersion(), HEX));
    Serial.println("Using serial number: " + String(hostConfig.getSerialNumber()));
    
    // Set up gamepad
    gamepad = new XboxGamepadDevice(config);

    // Set up vibration event handler
    // FunctionSlot<XboxGamepadOutputReportData> vibrationSlot(OnVibrateEvent);
    // gamepad->onVibrate.attach(vibrationSlot);

    // Add all child devices to the top-level composite HID device to manage them
    compositeHID.addDevice(gamepad);

    // Start the composite HID device to broadcast HID reports
    Serial.println("Starting composite HID device...");
    compositeHID.begin(hostConfig);

    //joystick reset (read zero)
    joystickReset();

    //TOF
    VL53L1X_init(42); // 24Hz -> 42ms period

}

void loop()
{
  // JOYSTICK
  /////////////////////////////////////////////////////////////////////
  // Read X and Y analog values
  valueX_move = analogRead(VRX_PIN_move); //valueX_move = analogRead(VRX_PIN_move); // original value
  valueY_move = analogRead(VRY_PIN_move); //valueY_move = analogRead(VRY_PIN_move); // original value
  push_move = digitalRead(press_move);
  valueX_aim = analogRead(VRX_PIN_aim);
  valueY_aim = analogRead(VRY_PIN_aim);
  push_aim = digitalRead(press_aim);
  /////////////////////////////////////////////////////////////////////

  //PB
  /////////////////////////////////////////////////////////////////////
  //scopeState = digitalRead(scopeInput); // replace by TOF
  isScopeOn = VL53L1X_measure(250);// max -> upper limit distance (mm) to open scope NEED TO BE CHANGE IN ASSEMBLY
  shootState = digitalRead(shootInput);
  reloadState = digitalRead(reloadInput);
  jumpState = digitalRead(jumpInput);
  changeWeaponState = digitalRead(changeWeaponInput);
  crouchState = digitalRead(crouchInput);
  /////////////////////////////////////////////////////////////////////

  // MPU 6050
  /////////////////////////////////////////////////////////////////////
  MPU_reset = digitalRead(MPURESET);
  if(MPU_reset == LOW){
    MPU_reposition();
  }
  //MPUreading();
  /////////////////////////////////////////////////////////////////////

  // When Bluetooth is connect, loop the folowing function, interupt by PB/Joystick event
  if(compositeHID.isConnected()){
      testButtons();
      //testPads();
      testTriggers();
      testThumbsticks();
  } 
  // delay(8);
  delay(8);

  // delay(200);
}

void joystickReset(){
    long valueX_move_sum = 0;
    long valueY_move_sum = 0;
    long valueX_aim_sum = 0;
    long valueY_aim_sum = 0;
    int count = 20; // average of #
    delay(50);
    for(int i=0; i<count; i++){
      valueX_move_sum += analogRead(VRX_PIN_move);
      valueY_move_sum += analogRead(VRY_PIN_move);
      valueX_aim_sum += analogRead(VRX_PIN_aim);
      valueY_aim_sum += analogRead(VRY_PIN_aim);
      delay(2);
    }
    
    valueX_move_zero = valueX_move_sum / count;
    valueY_move_zero = valueY_move_sum / count;
    valueX_aim_zero = valueX_aim_sum / count;
    valueY_aim_zero = valueY_aim_sum / count;

}

void fastflash(int input1, int input2, int time){
  digitalWrite(input1, HIGH);
  digitalWrite(input2, HIGH);
  delay(time);
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
  delay(time);
}

void testButtons(){
  // XBOX button X -> reaload
  if(reloadState == HIGH){ //new value - reloadState == HIGH, old value - reloadState == LOW(active low)
    digitalWrite(reloadLEDOutput, HIGH);
    gamepad->press(XBOX_BUTTON_X);
    gamepad->sendGamepadReport();
    delay(50);
    gamepad->release(XBOX_BUTTON_X);
    gamepad->sendGamepadReport();
    
  }
  else{
    digitalWrite(reloadLEDOutput, LOW);
    //gamepad->release(XBOX_BUTTON_X);
    //gamepad->sendGamepadReport();

  }
  // XBOX button Left Stick (LS) -> run
  if(push_move == LOW){ //active LOW
    gamepad->press(XBOX_BUTTON_LS);
    gamepad->sendGamepadReport();
  }
  else{
    gamepad->release(XBOX_BUTTON_LS);
    gamepad->sendGamepadReport();
  }
  // XBOX button Right Stick (RS) -> mark
  if(push_aim == LOW){ //active LOW
    gamepad->press(XBOX_BUTTON_RS);
    gamepad->sendGamepadReport();
  }
  else{
    gamepad->release(XBOX_BUTTON_RS);
    gamepad->sendGamepadReport();
  }

  // XBOX button A -> jump
  if(jumpState == LOW){
    gamepad->press(XBOX_BUTTON_A);
    gamepad->sendGamepadReport();
  }
  else{
    gamepad->release(XBOX_BUTTON_A);
    gamepad->sendGamepadReport();
  }

  // XBOX button Y -> change weapon
  if(changeWeaponState == LOW){
    gamepad->press(XBOX_BUTTON_Y);
    gamepad->sendGamepadReport();
  }
  else{
    gamepad->release(XBOX_BUTTON_Y);
    gamepad->sendGamepadReport();
  }

  // XBOX button B -> crouch
  if(crouchState == LOW){
    gamepad->press(XBOX_BUTTON_B);
    gamepad->sendGamepadReport();
  }
  else{
    gamepad->release(XBOX_BUTTON_B);
    gamepad->sendGamepadReport();
  }
}


void testTriggers(){
  int16_t on = 32760;
  int16_t off_zero = 0;
  // XBOX left trigger (LT) -> open scope, NEED TO REPLACE WITH TOF SENSOR LATER
  if(isScopeOn == true){ // TOF
    gamepad->setLeftTrigger(on);
    gamepad->sendGamepadReport();
    digitalWrite(scopeLEDOutput, HIGH);
  }
  else{
    gamepad->setLeftTrigger(off_zero);
    gamepad->sendGamepadReport();
    digitalWrite(scopeLEDOutput, LOW);
  }
  
  // XBOX right trigger (RT) -> shoot
  if(shootState == LOW){
    gamepad->setRightTrigger(on);
    gamepad->sendGamepadReport();
    fastflash(shootLEDOutput, recoilOutput, 20);
  }
  else{
    gamepad->setRightTrigger(off_zero);
    gamepad->sendGamepadReport();
    digitalWrite(shootLEDOutput, LOW);
    digitalWrite(recoilOutput, LOW);
  }
}

int16_t XMoveValueNorimalizeJS(int input){
  if (input >= valueX_move_zero){
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(4095 - valueX_move_zero) * static_cast<int16_t>(input - valueX_move_zero));
  }
  else{
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(valueX_move_zero) * static_cast<int16_t>(input - valueX_move_zero));
  }
}

int16_t YMoveValueNorimalizeJS(int input){
  if (input >= valueY_move_zero){
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(4095 - valueY_move_zero) * static_cast<int16_t>(input - valueY_move_zero));
  }
  else{
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(valueY_move_zero) * static_cast<int16_t>(input - valueY_move_zero));
  }
}

int16_t XAimValueNorimalizeJS(int input){
  if (input >= valueX_aim_zero){
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(4095 - valueX_aim_zero) * static_cast<int16_t>(input - valueX_aim_zero));
  }
  else{
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(valueX_aim_zero) * static_cast<int16_t>(input - valueX_aim_zero));
  }
}


int16_t YAimValueNorimalizeJS(int input){
  if (input >= valueY_aim_zero){
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(4095 - valueY_aim_zero) * static_cast<int16_t>(input - valueY_aim_zero));
  }
  else{
    return int16_t (static_cast<int16_t>(32760) / static_cast<int16_t>(valueY_aim_zero) * static_cast<int16_t>(input - valueY_aim_zero));
  }
}

// inputBoundary usually +- 0.5, less -> more sensitive, 
//inputDZ-if input less than this value, don't care, outputDZ-JS do not move until it reach approx 14000
int16_t XAimValueNorimalizeMPU(float input, float inputBoundary, float inputDZ, int outputDZ, float scalefactor){ 
  // change outputDZ ->[output lower, output upper]
  if(input > inputBoundary){
    return static_cast<int16_t> (-(32600)*scalefactor);
  }
  if(input < (-1.0)*inputBoundary){
    return static_cast<int16_t> (32600*scalefactor);
  }

  if(input > inputDZ){
    return static_cast<int16_t> (-((input - inputDZ) * (18760.0)/(inputBoundary - inputDZ) + outputDZ));
  }
  else if(input < (-1.0)*inputDZ){
    return static_cast<int16_t> (-((input + inputDZ) * (18760.0)/(inputBoundary - inputDZ) - outputDZ));
  }
  else{
    return 0;
  }
}

// inputBoundary usually +- 0.5, less -> more sensitive, 
//inputDZ-if input less than this value, don't care, outputDZ-JS do not move until it reach approx 14000
int16_t YAimValueNorimalizeMPU(float input, float inputBoundary, float inputDZ, int outputDZ, float scalefactor){
  if(input > inputBoundary){
    return static_cast<int16_t> (-(32600)*scalefactor);
  }
  if(input < (-1.0)*inputBoundary){
    return static_cast<int16_t> (32600*scalefactor);
  }

  if(input > inputDZ){
    return static_cast<int16_t> (-((input - inputDZ) * (18760.0)/(inputBoundary - inputDZ) + outputDZ));
  }
  else if(input < (-1.0)*inputDZ){
    return static_cast<int16_t> (-((input + inputDZ) * (18760.0)/(inputBoundary - inputDZ) - outputDZ));
  }
  else{
    return 0;
  }
}


void testThumbsticks(){
    // Joystick input
    int16_t x_move_JS = XMoveValueNorimalizeJS(valueY_move);
    int16_t y_move_JS = (-1) * YMoveValueNorimalizeJS(valueX_move);
    int16_t x_aim_JS = (0.8) * (-1) * XAimValueNorimalizeJS(valueX_aim); //reduce the sensitivity
    int16_t y_aim_JS = (-1) * YAimValueNorimalizeJS(valueY_aim);

    // MPU6050 input, deadzone +- 0.5 (base on test)
    SensorData sensorData = MPUreading();
    // inputBoundary usually +- 0.5, less -> more sensitive, 
    //inputDZ-if input less than this value, don't care, outputDZ-JS do not move until it reach approx 14000
    //float input, float inputBoundary, float inputDZ, int outputDZ, float scalefactor
    int16_t x_aim_MPU = YAimValueNorimalizeMPU(sensorData.yaw, 0.2, 0.01, 8000, 1);
    int16_t y_aim_MPU = YAimValueNorimalizeMPU(sensorData.pitch*(-1.0), 0.2, 0.01, 8000, 1);

    int16_t x_aim_final = x_aim_MPU;
    int16_t y_aim_final = (-1)*y_aim_MPU;
    int16_t z = YAimValueNorimalizeMPU(sensorData.roll, 0.5, 0.01, 0, 1);
    gamepad->setLeftThumb(x_move_JS, y_move_JS);
    int16_t sumXaim = x_aim_JS; // + x_aim_MPU;
    int16_t sumYaim = y_aim_JS; // + y_aim_MPU;

    if (sumXaim < -32760){
      x_aim_final = -32000;
    }

    else if (sumXaim > 32760){
      x_aim_final = 32000;
    }

    else {
      x_aim_final = sumXaim;
    }
  
    if (sumYaim < -32760){
      y_aim_final = -32000;
    }

    else if (sumYaim > 32760){
      y_aim_final = 32000;
    }

    else {
      y_aim_final = sumYaim;
    }

    // Serial.print("  x js: ");
    // Serial.print(x_aim_JS);
    // Serial.print("  x aim: ");
    // Serial.print(x_aim_MPU);
    // Serial.print("  x sum: ");
    // Serial.print(sumXaim);
    // Serial.print("  y js: ");
    // Serial.print(y_aim_JS);
    // Serial.print("  y aim: ");
    // Serial.print(y_aim_MPU);
    // Serial.print("  y sum: ");
    // Serial.print(sumYaim);
    // Serial.print("  pitch:  ");
    // Serial.print(sensorData.pitch);
    // Serial.println("");

    gamepad->setRightThumb(x_aim_final , y_aim_final);
    gamepad->sendGamepadReport();
}

void VL53L1X_init(int period_ms){
  Wire.begin();
  VL53L1X.begin(0x29, &Wire); // 0x29 -> I2C default address
  VL53L1X.startRanging();
  VL53L1X.setTimingBudget(period_ms); // sample period (ms)
}

bool VL53L1X_measure(int max){
  int16_t distance;
  if (VL53L1X.dataReady()) {
    distance = VL53L1X.distance(); // new measurement for the taking!
    if (distance == -1) { // something went wrong!
      return false;
    }
    if(distance <= max){
      return true;
    }
    // data is read out, time for another reading!
    VL53L1X.clearInterrupt();
    return false;
  }
}

void MPUsetup(){
  float start = millis();
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  float end = millis();
  float setuptime = end-start;
  Serial.println(" setuptime = ");
  Serial.println(setuptime);
  Serial.println(" ");
  //delay(100);
}

//set all variable to ZERO
void MPU_reposition(){
  pitchG = 0.0;
  rollG = 0.0;
  yawG = 0.0;
  pitch = 0.0;
  roll = 0.0;
  yaw = 0.0;
}

void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int n = 500; 

  Serial.println("Calibrating Gyro...");
  for (int i = 0; i < n; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(2);
  }

  gyroX_offset = sumX / n;
  gyroY_offset = sumY / n;
  gyroZ_offset = sumZ / n;

  Serial.println("Calibration Done!");
}

SensorData MPUreading(){
    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  AccX = a.acceleration.x;
  AccY = a.acceleration.y;
  AccZ = a.acceleration.z;

  pitchA = atan (AccY / sqrt(AccX * AccX + AccZ * AccZ) );  
  rollA = atan( (-1.0)*AccX / sqrt(AccY * AccY + AccZ * AccZ));

  previous_time = current_time;
  current_time = millis();
  delta_time = (current_time - previous_time) / 1000.0;

  GyroX = g.gyro.x- gyroX_offset;
  GyroY = g.gyro.y- gyroY_offset;
  GyroZ = g.gyro.z- gyroZ_offset;

  pitchG = GyroX*delta_time ;
  rollG = GyroY*delta_time ;
  yawG = GyroZ*delta_time ;

  pitch = (AG_ratio * pitchG + (1.0-AG_ratio)*pitchA);
  roll = (AG_ratio * rollG + (1.0-AG_ratio)*rollA);
  yaw = yawG;
  SensorData data = {pitch, roll, yaw};
  return data;
}
