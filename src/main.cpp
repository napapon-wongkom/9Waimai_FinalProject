#include <ps5Controller.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Ticker.h>
#include <PID_v1.h>
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
#define SERVO 23
#define HALL_SENSOR 5
#define IR_SENSOR 34
/// ##############################################



hw_timer_t *timer1 = NULL;
Adafruit_BNO08x bno085 = Adafruit_BNO08x();
unsigned long lastTimeStamp = 0;
int analog_ly = 0;
int analog_rx = 0;
volatile float PWM_left; // Command PWM left
volatile float PWM_right; // Command PWM right
float analog_left;
float analog_right;
float angular_velocity; // Command angular velocity
float linear_velocity; // Command linear velocity
float left_wheel_velocity; // Left wheel command velocity (m/s)
float right_wheel_velocity; // Right wheel command velocity (m/s)
float timer1_interval = 10; // Timer for accelerate ramping (ms)
volatile bool timer_tick = false; // To tick ramping function
volatile bool serial_handle = false;
const double IMU_noise = 0.03;


/// ##############################################
/// Robot Spec
const float wheel_distance = 0.475; //Distance between wheels in m
const float wheel_diameter = 0.2032; //Wheel diameter in m
const float WHEEL_RADIUS = wheel_diameter/2;
const float GEAR_RATIO = 51.0; // Motor to wheel gear ratio
const int motor_RPM = 6000; // Motor RPM
const float ENCODE_PPR = 600.0;
const float MOTOR2ENCODE_RATIO = 1; //70.0/40.0; // Gear ratio from moter to encoder
const float WHEEL_CIRCUMFERENCE = PI * wheel_diameter; // 2*PI*R
/// ##############################################


/// ##############################################
/// Physical Variables
float distance = 0.0; // Reality distance traveled (m)
float RPM_LEFT = 0.0; // Reality left wheel RPM
float RPM_RIGHT = 0.0; // Reality right wheel RPM
float velocity_mps_left = 0.0; // Reality left wheel velocity (m/s)
float velocity_mps_right = 0.0; // Reality right wheel velocity (m/s)
/// ##############################################


/// ##############################################
/// Differential Drive Variables
const float max_speed = 0.3; //Max linear speed in m/s
const float max_angular_speed = 30.0; //Max angular speed in deg/s
/// ##############################################


/// ##############################################
/// Encoder Variables 
static int lastPos_left = 0;
static int lastPos_right = 0;
unsigned long previousMillis = 0;
const long encoder_interval = 20; // calculate every 20 ms
long oldPos_left = 0;
long oldPos_right = 0;
const float ENCODER_MULTIPILIER = 2.0; // Use TWO03 latch mode
float velocity_mps_left_filtered = 0.0;
float velocity_mps_right_filtered = 0.0;
const float filter_alpha = 0.05; // Lower is smoother (0.05 - 0.2 is typical)
unsigned long last_encoder_time = 0;
const float COUNTS_PER_REVOLUTION = 1600; //ENCODE_PPR * MOTOR2ENCODE_RATIO * ENCODER_MULTIPILIER;
long currentPos_left = 0;
long currentPos_right = 0;
/// ###############################################


/// ###############################################
/// IMU Variables
/// Quaternion
float imu_r; 
float imu_i; 
float imu_j; 
float imu_k; 

float yaw = 0;

uint32_t last_imu_time = 0;
double angular_vel_z = 0.0; // IMU angular velocity
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
Ticker controlTimer;
Ticker SerialTimer;
const int ARRAY_SIZE = 2;
float incomingData[ARRAY_SIZE];
volatile float outgoingData[13] = {0};
/// ############################################


/// ############################################
/// PID Variable
double PIDsetpoint; // Desire value for PID left
double PIDinput; // Reality current value for PID left
double PIDoutput; // Output value for PID left
/// PID Setup
double Kp=200, Ki=5, Kd=1;
unsigned long straightStartTime = 0;
const float FADE_DURATION = 500.0; // Half a second fade-in

PID ControlPID(&PIDinput, &PIDoutput, &PIDsetpoint, Kp, Ki, Kd, DIRECT);
/// ############################################


/// ##################################
///Wheel profile variables
double startWheelL = 0, targetWheelL = 0, currentProfiledWheelL = 0; // Output velocity from S-Curve
double startWheelR = 0, targetWheelR = 0, currentProfiledWheelR = 0; // Output velocity from S-Curve
unsigned long moveStartTime = 0;
const double RAMP_DURATION = 0.5 * 1000.0; // Duration of accelerate from initial to desire value
/// ##################################

/// ##############################################
/// Global State for Change Detection
float last_joystick_linear = 0.0;
float last_joystick_angular = 0.0;
const float JOYSTICK_THRESHOLD = 0.02; // 2% change triggers new ramp
/// ##############################################

/// ##############################################
/// Command Variable
float command_angular_speed = 0; // Command Angular speed rad/s
float L_PWM_out = 0;
float R_PWM_out = 0;
/// ##############################################

/// ##############################################
/// Robot State
bool rotating;
/// ##############################################


/// ##############################################
/// Safety Variable
const int SAFETY_PWM = 177; // To prevent PWM from exceed
/// ##############################################


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

double velocity2RPM(double linear_vel){
  return (linear_vel / (2.0 * PI * WHEEL_RADIUS)) * 60.0 * GEAR_RATIO;
}

double RPM2velocity(double RPM){
  return ((2.0 * PI * WHEEL_RADIUS) * RPM) / (GEAR_RATIO * 60.0);
}

int speed2pwm(float speed){
  /*
    Function to convert desired speed (m/s) to PWM value (0-255).
  */ 
  float wheel_circumference = PI * wheel_diameter; 
  float RPS = speed / wheel_circumference;
  float RPM = RPS * 60;
  float motor_speed = RPM * GEAR_RATIO;
  float duty_cycle = motor_speed / motor_RPM;
  float PWM = duty_cycle * 255.0;
  if (PWM > 255){
    PWM = 255;
  }
  return PWM;
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

void IRAM_ATTR onTimer(){
  /*
    Timer interrupt to tick function.
  */
  timer_tick = true;
}

void SerialTick() {
  serial_handle = true;
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

void encoder_calculate() {
  /*
    Function to calculate value from encoder to use in PID
  */
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= encoder_interval) {
    float dt = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;

    currentPos_left = encoder_left->getPosition();
    currentPos_right = encoder_right->getPosition();
    
    // 1. Calculate Delta Distance (m) directly
    // (Delta Ticks / Ticks per Rev) * Circumference
    float delta_dist_L = ((float)(currentPos_left - oldPos_left) / COUNTS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
    float delta_dist_R = ((float)(currentPos_right - oldPos_right) / COUNTS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;

    // 2. Raw Velocity = Distance / Time
    float raw_vel_L = delta_dist_L / dt;
    float raw_vel_R = delta_dist_R / dt;


    // velocity_mps_left = raw_vel_L;
    // velocity_mps_right = raw_vel_R;

    // 3. Apply Filter
    velocity_mps_left = (filter_alpha * raw_vel_L) + ((1.0 - filter_alpha) * velocity_mps_left);
    velocity_mps_right = (filter_alpha * raw_vel_R) + ((1.0 - filter_alpha) * velocity_mps_right);

    // Update variables for Odom/Debug
    RPM_LEFT = (raw_vel_L / WHEEL_CIRCUMFERENCE) * 60.0;
    RPM_RIGHT = (raw_vel_R / WHEEL_CIRCUMFERENCE) * 60.0;


    // Calculate distance
    distance = ((float)currentPos_left + (float)currentPos_right) / (2.0 * COUNTS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;

    oldPos_left = currentPos_left;
    oldPos_right = currentPos_right;
  }
}

double SCurve(double t, double start, double delta_vel, double duration){
  /*
    Functuin to calculate S-Curve.
  */
  if (t >= duration) return start + delta_vel;
  t /= duration / 2;
  if (t < 1) return delta_vel / 2 * t * t * t + start;
  t -= 2;
  return delta_vel / 2 * (t * t * t + 2) + start;
}

void IMU_Quaternion(){
  sh2_SensorValue_t sensorValue;
  if (bno085.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      imu_i = sensorValue.un.gameRotationVector.i; // x
      imu_j = sensorValue.un.gameRotationVector.j; // y
      imu_k = sensorValue.un.gameRotationVector.k; // z
      imu_r = sensorValue.un.gameRotationVector.real; // w

      yaw = atan2(2.0 * (imu_r * imu_k + imu_i * imu_j), 1.0 - 2.0 * (imu_j * imu_j + imu_k * imu_k));
    }
  }
}

void IMU_Angular(){
  sh2_SensorValue_t Angular_Value;
  if (bno085.getSensorEvent(&Angular_Value)){
    if (Angular_Value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      angular_vel_z = Angular_Value.un.gyroscope.z;

      // Serial.print("Angular Velocity :");
      // Serial.print(angular_velocity);
      // Serial.print(" | ");
      // Serial.println(angular_vel_z);
    }
  }
}

void UART_send_data(){
  outgoingData[0] = (float)millis();
  outgoingData[1] = currentPos_left;
  outgoingData[2] = currentPos_right;
  outgoingData[3] = yaw;
  outgoingData[4] = 0;
  outgoingData[5] = 0;
  outgoingData[6] = 0;
  outgoingData[7] = angular_accel_z;
  outgoingData[8] = PIDsetpoint;
  outgoingData[9] = PIDinput;
  outgoingData[10] = PIDoutput;
  outgoingData[11] = L_PWM_out;
  outgoingData[12] = R_PWM_out;

  uint8_t header = 0xAA;
  Serial2.write(header);

  Serial2.write((uint8_t*)outgoingData, sizeof(outgoingData));
}

void encoder_debug(){

  Serial.print("Output PWM : ");
  Serial.print(PWM_left);
  Serial.print(" | ");
  Serial.print(PWM_right);
  Serial.print("  RPM l/r : ");
  Serial.print(RPM_LEFT);
  Serial.print(" | ");
  Serial.print(RPM_RIGHT);
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

void PID_debug(){
  Serial.print("Setpoint :");
  Serial.print(PIDsetpoint);
  Serial.print(" | Feedback:");
  Serial.print(PIDinput);
  Serial.print(" | error:");
  Serial.print(PIDsetpoint-PIDinput);
  Serial.print(" | PIDoutput :");
  Serial.print(PIDoutput);
  Serial.print(" | PWM_L_out :");
  Serial.print(L_PWM_out);
  Serial.print(" | PWM_R_out :");
  Serial.println(R_PWM_out);
}

void updateControl(){
  /// S-Curve calculate
  unsigned long elapsed = millis() - moveStartTime;
  currentProfiledWheelL = SCurve((double)elapsed, startWheelL, targetWheelL - startWheelL, RAMP_DURATION);
  currentProfiledWheelR = SCurve((double)elapsed, startWheelR, targetWheelR - startWheelR, RAMP_DURATION);

  PWM_left = speed2pwm(currentProfiledWheelL);
  PWM_right = speed2pwm(currentProfiledWheelR);

  if (PWM_right > SAFETY_PWM){
    PWM_right = SAFETY_PWM;
  }
  
  if (PWM_left > SAFETY_PWM){
    PWM_left = SAFETY_PWM;
  }

  if (PWM_right < -SAFETY_PWM){
    PWM_right = -SAFETY_PWM;
  }

  if (PWM_left < -SAFETY_PWM){
    PWM_left = -SAFETY_PWM;
  }
  encoder_calculate();

  bool isMoving = (abs(targetWheelL) > 0.01 || abs(targetWheelR) > 0.01);
  
  if (rotating == false){
    if (ControlPID.GetMode() == MANUAL){
      PIDoutput = 0;
      ControlPID.SetMode(AUTOMATIC);
      straightStartTime = millis(); // Mark the moment we started going straight
    }

    PIDsetpoint = 0;
    PIDinput =  velocity_mps_right - velocity_mps_left;


    if (isMoving){
      ControlPID.Compute();

      /// PID Fade-in logic
      float timeSinceStraight = millis() - straightStartTime;
      float pidScalar = timeSinceStraight / FADE_DURATION;

      if (pidScalar > 1.0) pidScalar = 1.0;
      if (pidScalar < 0) pidScalar = 0;

      L_PWM_out = PWM_left - (PIDoutput * pidScalar);
      R_PWM_out = PWM_right + (PIDoutput * pidScalar);
    } else{
      PIDoutput = 0;
      L_PWM_out = PWM_left;
      R_PWM_out = PWM_right;
    }    

  } else if (rotating == true){
    ControlPID.SetMode(MANUAL);
    PIDoutput = 0;
    L_PWM_out = PWM_left;
    R_PWM_out = PWM_right;
  }
  

  // if (abs(command_angular_speed - angular_vel_z) < IMU_noise){
  //   PIDinput = command_angular_speed;
  // } else{
  //   PIDinput = angular_vel_z;
  // }
  
  

  
  control((int)round(L_PWM_out),(int)round(R_PWM_out), 0);
  // control((int)round(PWM_left),(int)round(PWM_right), 0); // Main control function.
}


/// ######################################################################################
/// Main
/// ######################################################################################
void setup() 
{
  Serial.begin(115200); /// Set Serial Baud rate.
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(PWML_F,OUTPUT);
  pinMode(PWML_B,OUTPUT);
  pinMode(PWMR_F,OUTPUT);
  pinMode(PWMR_B,OUTPUT);

  pinMode(SERVO, OUTPUT);
  pinMode(HALL_SENSOR, INPUT_PULLUP);
  pinMode(IR_SENSOR,INPUT);

  digitalWrite(SERVO,0);


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
  /// PID safety code !!!
  ControlPID.SetOutputLimits(-80,80);

  ControlPID.SetSampleTime(20);

  ControlPID.SetMode(AUTOMATIC);
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
  controlTimer.attach(0.02, updateControl);
  SerialTimer.attach(0.01, SerialTick);
  /// ##################################
}

void loop() 
{
  
  /// Read PS5 Controller Joystick values and calculate desired velocities.
  analog_ly = ps5.LStickY();
  analog_rx = ps5.RStickX();
  analog_right = deadzone(analog_rx);
  analog_left = deadzone(analog_ly);
  angular_velocity = -map(analog_right,-128,127,-max_angular_speed,max_angular_speed); // deg
  linear_velocity = map(analog_left,-128,127,-(max_speed * 1000),(max_speed * 1000)) / 1000.0;
  zero_handle(); /// Handle zero deadzone case.


  if (angular_velocity != 0) {
    rotating = true;
  } else{
    rotating = false;
  }

  /// Convert angular velocity for negative velocity case.
  if (linear_velocity < 0){
    angular_velocity = -angular_velocity;
  }

  command_angular_speed = deg2rad(angular_velocity);

  /// Use differential drive equations to calculate velocities and convert to PWM values.
  diffdrive_equation(linear_velocity,deg2rad(angular_velocity),wheel_distance, left_wheel_velocity, right_wheel_velocity);

  if (abs(left_wheel_velocity - targetWheelL) > 0.01 || abs(right_wheel_velocity - targetWheelR) > 0.01) {
    startWheelL = currentProfiledWheelL;
    startWheelR = currentProfiledWheelR;
    targetWheelL = left_wheel_velocity;
    targetWheelR = right_wheel_velocity;
    moveStartTime = millis();
  }
  
  
  /// Handle ramping acceleration/deceleration.
  if (timer_tick){
    timer_tick = false;
    //encoder_debug();
    IMU_Angular();
    IMU_Quaternion();
    
    // PID_debug();
    // Serial.print(velocity_mps_left);
    // Serial.print(" ");
    // Serial.print(velocity_mps_right);
    // Serial.print(" ");
    // Serial.println(velocity_mps_left - velocity_mps_right);
  }
  
  
  
  /// ################################################
  /// UART Zone
  if (serial_handle){
    serial_handle = false;

    if (Serial2.available() >= (ARRAY_SIZE * sizeof(float))) {
    
      Serial2.readBytes((char*)incomingData, (ARRAY_SIZE * sizeof(float)));
      Serial.print(incomingData[0]);
      Serial.print(" | ");
      Serial.println(incomingData[1]);
      // Serial.printf("RX Float: &.2f\n", incomingData[0]);
    }
  }
  
  /// ################################################
}