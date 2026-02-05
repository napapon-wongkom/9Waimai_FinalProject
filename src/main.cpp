#include <ps5Controller.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Ticker.h>
/// ######################################################################################
/// Variable
/// ######################################################################################

/// #############################################
/// Define Value
#define RXD2 16 //
#define TXD2 17 //
#define PWML_F 2
#define PWML_B 15
#define PWMR_F 18 // 4 -> 18
#define PWMR_B 19 // 16 -> 19
#define ENCODER_LEFT_A 14
#define ENCODER_LEFT_B 12
#define ENCODER_RIGHT_A 26
#define ENCODER_RIGHT_B 27
#define I2C_SDA1 21
#define I2C_SCL1 22
#define BNO08X_INT 4 // 17 -> 4
#define BNO08X_RST -1
#define BNO08X_ADDR 0x4B

/// ##############################################

hw_timer_t *timer1 = NULL;
Adafruit_BNO08x bno085 = Adafruit_BNO08x();
unsigned long lastTimeStamp = 0;
int analog_ly = 0;
int analog_rx = 0;
volatile float PWM_left;
volatile float PWM_right;
float analog_left;
float analog_right;
float angular_velocity;
float linear_velocity;
float left_wheel_velocity; // Left wheel velocity (m/s)
float right_wheel_velocity; // Right wheel velocity (m/s)
volatile float current_PWM_left = 0;
volatile float current_PWM_right = 0;
volatile float step_size;
float timer1_interval = 10; // Timer for accelerate ramping (ms)
volatile bool ramping_tick = false; // To tick ramping function

/// ##############################################
/// Physical Variables
float distance = 0.0; // Distance traveled (m)
float rpm_left = 0.0; // Left wheel RPM
float rpm_right = 0.0; // Right wheel RPM
float velocity_mps_left = 0.0; // Left wheel velocity (m/s)
float velocity_mps_right = 0.0; // Right wheel velocity (m/s)
/// ##############################################

/// ##############################################
/// Virtual Variables'
float x_pos = 0.0;
float y_pos = 0.0;
/// ##############################################

/// ##############################################
/// Differential Drive Variables
const float wheel_distance = 0.475; //Distance between wheels in m
const float max_speed = 0.5; //Max linear speed in m/s
const float max_angular_speed = 45.0; //Max angular speed in deg/s
const float wheel_diameter = 0.2; //Wheel diameter in m
const float gear_ratio = 70.0; // Motor to wheel gear ratio
const int motor_RPM = 6000; // Motor RPM
const float accelerate_time = 3.0; // Time to reach speed in seconds
/// ##############################################


/// ##############################################
/// Encoder Variables 
static int lastPos_left = 0;
static int lastPos_right = 0;
unsigned long previousMillis = 0;
const long encoder_interval = 100; // calculate every 100 ms
long oldPos_left = 0;
long oldPos_right = 0;
const float MOTOR_PPR = 600.0;
const float ENCODER_MULTIPILIER = 2.0; // Use FOUR3 latch mode
const float GEAR_RATIO = 70.0/40.0; // Gear ratio from moter to encoder
const float COUNTS_PER_REVOLUTION = MOTOR_PPR * GEAR_RATIO * ENCODER_MULTIPILIER;
const float WHEEL_CIRCUMFERENCE = PI * wheel_diameter;
/// ###############################################

/// ###############################################
/// IMU Variables
/// Quaternion
float imu_r;
float imu_i;
float imu_j;
float imu_k;

float gyro_z = 0;
uint32_t last_imu_time = 0;

float angular_accel_z;
/// ###############################################

/// ###########################################
/// Encode Setup 
RotaryEncoder *encoder_left = nullptr;
RotaryEncoder *encoder_right = nullptr;

IRAM_ATTR void checkPositon() {
  encoder_left->tick();
  encoder_right->tick();
}
/// ############################################

/// ############################################
/// UART Setup
Ticker uartTimer;
const int ARRAY_SIZE = 4;
float incomingData[ARRAY_SIZE];
volatile float outgoingData[8] = {0,0,0,0,0,0,0,0};
/// ############################################


/// ######################################################################################
/// Function
/// ######################################################################################
void notify()
{
  char messageString[200];
  sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  ps5.LStickX(),
  ps5.LStickY(),
  ps5.RStickX(),
  ps5.RStickY(),
  ps5.Left(),
  ps5.Down(),
  ps5.Right(),
  ps5.Up(),
  ps5.Square(),
  ps5.Cross(),
  ps5.Circle(),
  ps5.Triangle(),
  ps5.L1(),
  ps5.R1(),
  ps5.L2(),
  ps5.R2(),  
  ps5.Share(),
  ps5.Options(),
  ps5.PSButton(),
  ps5.Touchpad(),
  ps5.Charging(),
  ps5.Audio(),
  ps5.Mic(),
  ps5.Battery());

  //Only needed to print the message properly on serial monitor. Else we dont need it.
  if (millis() - lastTimeStamp > 50)
  {
    Serial.println(messageString);
    lastTimeStamp = millis();
  }
}

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

int deadzone(int value) {
  /*
    Function to handle deadzone of joystick controller.
  */
  const int DEAD_ZONE_THRESHOLD = 8; 
  if (abs(value) < DEAD_ZONE_THRESHOLD) {
    return 0; // Returns 0 if centered
  } else {
    return value; // Returns the actual value
  }
}

void zero_handle(){
  if (analog_right == 0) angular_velocity = 0;
  if (analog_left == 0) linear_velocity = 0;
}

float deg2rad(float deg){
  /*
    Function to convert degree to radian.
  */
  return (deg *(PI/180));
}

float rad2deg(float rad){
  /*
    Function to convert radian to degree.
  */
  return (rad * (180/PI));
}

int speed2pwm(float speed){
  /*
    Function to convert desired speed (m/s) to PWM value (0-255).
  */
  float wheel_circumference = PI * wheel_diameter;
  float RPS = speed / wheel_circumference;
  float RPM = RPS * 60;
  float motor_speed = RPM * gear_ratio;
  float duty_cycle = motor_speed / motor_RPM;
  float PWM = duty_cycle * 255.0;
  if (PWM > 255){
    PWM = 255;
  }
  return PWM;
}

float step_size_calculate(float accelerate_time){
  /*
    Function to calculate step size for PWM ramping based on desired accelerate time.
  */
  float Total_step = (accelerate_time * 1000) / timer1_interval;
  float step_size = 255 / Total_step;
  step_size = max(1.0f, step_size);
  return step_size;
}

void pwm_command(int pwml1, int pwml2, int pwmr1, int pwmr2){
  /*
    Function to send PWM values to motor driver.
  */
  analogWrite(PWML_F,pwml1); // left wheel backward
  analogWrite(PWML_B,pwml2); //left wheel forward
  analogWrite(PWMR_F,pwmr1); // right wheel forward
  analogWrite(PWMR_B,pwmr2); // right wheel backward
}

void diffdrive_equation(float v, float w, float L, float &vl, float &vr){
  /*
    Differential drive kinematic equations to calculate left and right wheel velocities.
    v: linear velocity (m/s)
    w: angular velocity (rad/s)
    L: distance between wheels (m)
    vl: left wheel velocity (m/s)
    vr: right wheel velocity (m/s)
  */
  vr = v + ((w * L) / 2);
  vl = v - ((w * L) / 2);
}

void PWM_ramping(){
  /*
    Function to ramp PWM values for smooth acceleration/deceleration.
  */
  float delta_L = abs(PWM_left - current_PWM_left);
  float delta_R = abs(PWM_right - current_PWM_right);
  float step_size_L;
  float step_size_R;

  if (delta_L > delta_R){
    step_size_L = step_size;
    step_size_R = step_size * (delta_R / delta_L);
  }
  else if (delta_R > delta_L){
    step_size_R = step_size;
    step_size_L = step_size * (delta_L / delta_R);
  }
  else{
    step_size_L = step_size;
    step_size_R = step_size;
  }

  // Left Wheel Ramping
  if (current_PWM_left < PWM_left){
    current_PWM_left += step_size_L;
    if (current_PWM_left > PWM_left) current_PWM_left = PWM_left;
  }
  else if (current_PWM_left > PWM_left){
    current_PWM_left -= step_size_L;
    if (current_PWM_left < PWM_left) current_PWM_left = PWM_left;
  }

  // Right Wheel Ramping
  if (current_PWM_right < PWM_right){
    current_PWM_right += step_size_R;
    if (current_PWM_right > PWM_right) current_PWM_right = PWM_right;
  }
  else if (current_PWM_right > PWM_right){
    current_PWM_right -= step_size_R;
    if (current_PWM_right < PWM_right) current_PWM_right = PWM_right;
  }
}

void IRAM_ATTR onTimer(){
  /*
    Timer interrupt to tick ramping function.
  */
  ramping_tick = true;
}

void control(int PWM_l, int PWM_r, int debug_mode){
  /*
    Function to control motor PWM based on desired left and right PWM values.
  */
  if ((PWM_l > 0) && (PWM_r > 0) && (PWM_l == PWM_r)){
    pwm_command(0,abs(PWM_l),abs(PWM_r),0);
    if (debug_mode == 1){
      Serial.print("Moving Foward | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l < 0) && (PWM_r < 0) && (PWM_l == PWM_r)){
    pwm_command(abs(PWM_l),0,0,abs(PWM_r));
    if (debug_mode == 1){
      Serial.print("Moving Backward | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l < 0) && (PWM_r > 0) && (abs(PWM_l) == abs(PWM_r))){
    pwm_command(abs(PWM_l),0,abs(PWM_r),0);
    if (debug_mode == 1){
      Serial.print("Rotating Counter Clockwise | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l > 0) && (PWM_r < 0) && (abs(PWM_l) == abs(PWM_r))){
    pwm_command(0,abs(PWM_l),0,abs(PWM_r));
    if (debug_mode == 1){
      Serial.print("Rotating Clockwise | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l > 0) && (PWM_r > 0) && (PWM_l < PWM_r)){
    pwm_command(0,abs(PWM_l),abs(PWM_r),0);
    if (debug_mode == 1){
      Serial.print("Turning Left Foward | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l > 0) && (PWM_r > 0) && (PWM_l > PWM_r)){
    pwm_command(0,abs(PWM_l),abs(PWM_r),0);
    if (debug_mode == 1){
      Serial.print("Turning Right Foward | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l < 0) && (PWM_r < 0) && (abs(PWM_l) < abs(PWM_r))){
    pwm_command(abs(PWM_l),0,0,abs(PWM_r));
    if (debug_mode == 1){
      Serial.print("Turning Left Backward | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l < 0) && (PWM_r < 0) && (abs(PWM_l) > abs(PWM_r))){
    pwm_command(abs(PWM_l),0,0,abs(PWM_r));
    if (debug_mode == 1){
      Serial.print("Turning Right Backward | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else if((PWM_l == 0) && (PWM_r == 0)){
    pwm_command(0,0,0,0);
    if (debug_mode == 1){
      Serial.print("Stoping | ");
      Serial.print("Left : ");
      Serial.print(PWM_l);
      Serial.print(" | Right : ");
      Serial.println(PWM_r);
    }
    
  }else{
    pwm_command(0,0,0,0);
    if (debug_mode == 1){
      Serial.println("System Error!!!");
    }
    
  }
}

void encoder_read(int pos_left, int pos_right){
  /*
    Function to read encoder positions
  */
  int newPos_left = encoder_left->getPosition();
  int newPos_right = encoder_right->getPosition();
  if ((pos_left != newPos_left) || (pos_right != newPos_right)) {
    Serial.print("pos_left:");
    Serial.print(newPos_left);
    Serial.print(" dir_left:");
    Serial.print((int)(encoder_left->getDirection()));
    Serial.print(" | pos_right:");
    Serial.print(newPos_right);
    Serial.print(" dir_right:");
    Serial.println((int)(encoder_right->getDirection()));
    pos_left = newPos_left;
    pos_right = newPos_right;
  }
}

void encoder_calculate(){
  /*
    Function to calculate encoder-based metrics
  */
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= encoder_interval) {
    previousMillis = currentMillis;

    /// Get encoder tick position.
    long currentPos_left = encoder_left->getPosition();
    long currentPos_right = encoder_right->getPosition();

    /// Find delta tick positin.
    long deltaPos_left = currentPos_left - oldPos_left;
    long deltaPos_right = currentPos_right - oldPos_right;

    /// Calculate RPS.
    float revolution_left = (float)currentPos_left / COUNTS_PER_REVOLUTION;
    float revolution_right = (float)currentPos_right / COUNTS_PER_REVOLUTION;

    /// Calculate Distance (m) that wheels have traveled.
    float distance_left = revolution_left * WHEEL_CIRCUMFERENCE;
    float distance_right = revolution_right * WHEEL_CIRCUMFERENCE;

    /// Calculate average distance (m) to get robot distance traveled.
    distance = (distance_left + distance_right) / 2.0;
    
    /// Calculate velocity in ticks per second.
    float velocity_tps_left = deltaPos_left * (1000.0 / encoder_interval); // convert to counts per second
    float velocity_tps_right = deltaPos_right * (1000.0 / encoder_interval);

    /// Calculate RPM by converting ticks per second.
    rpm_left = (velocity_tps_left / COUNTS_PER_REVOLUTION) * 60.0;
    rpm_right = (velocity_tps_right / COUNTS_PER_REVOLUTION) * 60.0;

    /// Calculate linear velocity (m/s) by converting ticks per second.
    velocity_mps_left = (velocity_tps_left / COUNTS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
    velocity_mps_right = (velocity_tps_right / COUNTS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;

    /// Set old position for next calculation.
    oldPos_left = currentPos_left;
    oldPos_right = currentPos_right;
  }
}

void IMU_Quaternion(){
  sh2_SensorValue_t sensorValue;
  if (bno085.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      imu_r = sensorValue.un.gameRotationVector.real;
      imu_i = sensorValue.un.gameRotationVector.i;
      imu_j = sensorValue.un.gameRotationVector.j;
      imu_k = sensorValue.un.gameRotationVector.k;

      // Serial.print("Quaternion: ");
      // Serial.print(r);
      // Serial.print(", ");
      // Serial.print(i);
      // Serial.print(", ");
      // Serial.print(j);
      // Serial.print(", ");
      // Serial.println(k);
      // Process quaternion data (quat[0], quat[1], quat[2], quat[3])
    }
  }
}

void UART_send_data(){
  outgoingData[0] = velocity_mps_left;
  outgoingData[1] = velocity_mps_right;
  outgoingData[2] = imu_i;
  outgoingData[3] = imu_j;
  outgoingData[4] = imu_k;
  outgoingData[5] = imu_r;
  outgoingData[6] = angular_accel_z;
  Serial2.write((uint8_t*)outgoingData, sizeof(outgoingData));
}

void encoder_debug(){
  Serial.print("RPM l/r : ");
  Serial.print(rpm_left);
  Serial.print(" | ");
  Serial.print(rpm_right);
  Serial.print(" - Velocity l/r (m/s) : ");
  Serial.print(velocity_mps_left);
  Serial.print(" | ");
  Serial.print(velocity_mps_right);
  Serial.print(" - Distance (m): ");
  Serial.println(distance);
}

void debug(){
  Serial.print(analog_ly);
  Serial.print(" | ");
  Serial.print(analog_rx);
  Serial.print(" | ");
  Serial.print(deadzone(analog_ly));
  Serial.print(" | ");
  Serial.println(deadzone(analog_rx));
}

void speed_debug(){
  Serial.print("V (m/s): ");
  Serial.print(linear_velocity);
  Serial.print(" | ");
  Serial.print("W (deg/s): ");
  Serial.print(angular_velocity);
  Serial.print(" | ");
  Serial.print("W (rad/s): ");
  Serial.println(deg2rad(angular_velocity));
}

void PWM_debug(){
  Serial.print("L Wheel V: ");
  Serial.print(left_wheel_velocity);
  Serial.print(" | R Wheel V: ");
  Serial.print(right_wheel_velocity);
  Serial.print(" | PWM left: ");
  Serial.print(PWM_left);
  Serial.print(" | PWM right: ");
  Serial.println(PWM_right);
}

/// ######################################################################################
/// Main
/// ######################################################################################
void setup() 
{
  Serial.begin(115200); /// Set Serial Baud rate.
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  step_size = step_size_calculate(accelerate_time);

  pinMode(PWML_F,OUTPUT);
  pinMode(PWML_B,OUTPUT);
  pinMode(PWMR_F,OUTPUT);
  pinMode(PWMR_B,OUTPUT);

  /// ##################################
  /// Timer Setup 
  timer1 = timerBegin(0, 80, true); /// 80 MHz / 80 = 1 MHz (1 tick = 1 us).
  timerAttachInterrupt(timer1, &onTimer, true);
  timerAlarmWrite(timer1, timer1_interval * 1000, true);
  timerAlarmEnable(timer1);
  /// ##################################
  
  /// ##################################
  /// PS5 Controller Setup
  //ps5.attach(notify);
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("90:B6:85:3C:00:79");  /// your PS5 controller mac address.
  while (ps5.isConnected() == false) 
  { 
    Serial.println("PS5 controller not found");
    delay(300);
  }
  Serial.println("Ready.");
  /// ##################################

  /// ##################################
  /// Encdoer Setup 
  encoder_left = new RotaryEncoder(ENCODER_LEFT_A, ENCODER_LEFT_B, RotaryEncoder::LatchMode::TWO03);
  encoder_right = new RotaryEncoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), checkPositon, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), checkPositon, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), checkPositon, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), checkPositon, CHANGE);
  /// ##################################

  /// ##################################
  /// BNO085 Setup
  bno085.begin_I2C();
  bno085.enableReport(SH2_GAME_ROTATION_VECTOR);
  bno085.enableReport(SH2_GYROSCOPE_CALIBRATED);
  /// ##################################

  /// ##################################
  /// Ticker Initialize
  uartTimer.attach(0.01, UART_send_data);
  /// ##################################
}

void loop() 
{
  
  /// Read PS5 Controller Joystick values and calculate desired velocities.
  analog_ly = ps5.LStickY();
  analog_rx = ps5.RStickX();
  analog_right = deadzone(analog_rx);
  analog_left = deadzone(analog_ly);
  angular_velocity = map(analog_right,-128,127,-max_angular_speed,max_angular_speed);
  linear_velocity = map(analog_left,-128,127,-(max_speed * 1000),(max_speed * 1000)) / 1000.0;
  zero_handle(); /// Handle zero deadzone case.

  /// Convert angular velocity for negative velocity case.
  if (linear_velocity < 0){
    angular_velocity = -angular_velocity;
  }

  /// Use differential drive equations to calculate velocities and convert to PWM values.
  diffdrive_equation(linear_velocity,deg2rad(-angular_velocity),wheel_distance, left_wheel_velocity, right_wheel_velocity);
  PWM_left = speed2pwm(left_wheel_velocity);
  PWM_right = speed2pwm(right_wheel_velocity);

  /// Handle ramping acceleration/deceleration.
  if (ramping_tick){
    ramping_tick = false;
    PWM_ramping();
    control((int)round(current_PWM_left),(int)round(current_PWM_right), 0); // Main control function.
  }

  static int pos_left = 0;
  static int pos_right = 0;

  encoder_left->tick(); // just call tick() to check the state.
  encoder_right->tick();

  // encoder_read(pos_left, pos_right);
  encoder_calculate();
  // encoder_debug();
  IMU_Quaternion();
  /// ################################################
  /// UART Zone
  if (Serial2.available() >= (ARRAY_SIZE * sizeof(float))) {
    
    Serial2.readBytes((char*)incomingData, (ARRAY_SIZE * sizeof(float)));
    Serial.printf("RX Float: &.2f\n", incomingData[0]);
  }
  /// ################################################
}