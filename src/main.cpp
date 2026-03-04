#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <Ticker.h>

/// ######################################################################################
/// Variable
/// ######################################################################################

/// #############################################
/// Define Value
#define RXD1 18
#define TXD1 19
#define SERVO 12
#define HALL_SENSOR 15
#define IR_SENSOR 34
#define DOWN_STEP 13
#define DOWN_DIR 25
#define UP_STEP 32
#define UP_DIR 33
#define motorInterfaceType 1 
/// ##############################################


/// ##############################################
/// Hardware Initialize
Servo EndEffector;
AccelStepper down_stepper(motorInterfaceType, DOWN_STEP, DOWN_DIR);
AccelStepper up_stepper(motorInterfaceType, UP_STEP, UP_DIR);
/// ##############################################


/// ##############################################
/// UART Variable
const int ARRAY_SIZE = 6;
float incomingData[ARRAY_SIZE];
volatile float outgoingData[3] = {0};
/// ##############################################


/// ##############################################
/// Status Variable
enum State {
  IDLE,
  HOME,
  MOVING
};

State currentState = IDLE;

bool Home_pose = false;
float Home_status = 0;
int hall_sensor = 1;
int ir_sensor = 0;
float up_stepper_pos = 0;
float down_stepper_pos = 0;
/// ##############################################


/// ##############################################
/// Arm Initialize
int up_step_max_speed = 4000;
int up_step_max_acc = up_step_max_speed * 2;
int down_step_max_speed = 4000;
int down_step_max_acc = down_step_max_speed * 2;
float timer1_interval = 1000000 / up_step_max_speed; // 1 second / max speed = time for one step in microseconds
/// ##############################################


/// ##############################################
/// Arm Command
int arm_right_cmd = 0;
int arm_left_cmd = 0;
int arm_up_cmd = 0;
int arm_down_cmd = 0;
int arm_press_cmd = 0;
int arm_unpress_cmd = 0;
/// ##############################################




/// ##############################################
/// Timer Setup
// hw_timer_t *timer1 = NULL;
Ticker ControlFlow;
Ticker uartTimer;
Ticker SerialTimer;
volatile bool Timer_Flag = false;
/// ##############################################


/// ######################################################################################
/// Function
/// ######################################################################################

void UART_Handle() {
  outgoingData[0] = up_stepper_pos;
  outgoingData[1] = down_stepper_pos;
  outgoingData[2] = Home_status;

  uint8_t header = 0x55;
  Serial1.write(header);

  Serial1.write((uint8_t*)outgoingData, sizeof(outgoingData));

  while (Serial1.available() > 0 && Serial1.peek() != 0x55) {
    Serial1.read();
  }

  if (Serial1.available() >= (1 + ARRAY_SIZE * sizeof(float))) {
    uint8_t header = Serial1.read();

    if (header == 0x55) {
      Serial1.readBytes((char*)incomingData, sizeof(incomingData));
      arm_up_cmd = incomingData[0];
      arm_down_cmd = incomingData[1];
      arm_left_cmd = incomingData[2];
      arm_right_cmd = incomingData[3];
      arm_press_cmd = incomingData[4];
      arm_unpress_cmd = incomingData[5];
    }
  }
}


void String_Handle() {
  Timer_Flag = true;
}


void Set_Home(){
  if (Home_pose == false){
    hall_sensor = digitalRead(HALL_SENSOR);
    ir_sensor = digitalRead(IR_SENSOR);
    EndEffector.write(10);
    if (hall_sensor != 0){
      up_stepper.setSpeed(up_step_max_speed);
    }else {
      up_stepper.setCurrentPosition(0);
      up_stepper.setSpeed(0);
    }
    
    if (ir_sensor != 1){
      down_stepper.setSpeed(-down_step_max_speed);
    }else{
      down_stepper.setSpeed(0);
      down_stepper.setCurrentPosition(0);
    }

    if (hall_sensor == 0 && ir_sensor == 1){
      Home_pose = true;
    }
  }
}

void stepper_control(){  
  hall_sensor = digitalRead(HALL_SENSOR);
  ir_sensor = digitalRead(IR_SENSOR);

  /// UP STEPPER #######################################
  if (hall_sensor == 1){
    if (arm_up_cmd == 1 && up_stepper_pos >= -54000.0){
      up_stepper.setSpeed(-up_step_max_speed);
      if( up_stepper_pos < -54000.0){
        up_stepper.setSpeed(up_step_max_speed);
      }
    }else if (arm_down_cmd == 1) {
      up_stepper.setSpeed(up_step_max_speed);
    } else{
      up_stepper.setSpeed(0);
    }
  }else if (hall_sensor == 0){
    if (up_stepper_pos != 0){
      up_stepper.setCurrentPosition(0);
    }
    if (arm_up_cmd == 1) {
        up_stepper.setSpeed(-up_step_max_speed);
    } else {
        up_stepper.setSpeed(0);
    }
    // up_stepper.setSpeed(-up_step_max_speed);
  }

  /// DOWN STEPPER #######################################
  if (ir_sensor == 0){
    if (arm_right_cmd == 1 && down_stepper_pos < 26000.0){
      down_stepper.setSpeed(down_step_max_speed);
      if (down_stepper_pos >= 26000.0){
        down_stepper.setSpeed(-down_step_max_speed);
      }
    } else if (arm_left_cmd == 1) {
      down_stepper.setSpeed(-down_step_max_speed);
    } else {
      down_stepper.setSpeed(0);
    }
  }else if (ir_sensor == 1){
    if (down_stepper_pos != 0){
      down_stepper.setCurrentPosition(0);
    }
    if (arm_right_cmd == 1) {
        down_stepper.setSpeed(down_step_max_speed);
    } else {
        down_stepper.setSpeed(0);
    }
    // down_stepper.setSpeed(down_step_max_speed);
  }

  if (arm_press_cmd == 1){
    EndEffector.write(80);
  }else if (arm_unpress_cmd == 1) {
    EndEffector.write(20);
  }
  
}


void setup() {

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  
  
  /// ##################################
  /// Pin Setup
  pinMode(HALL_SENSOR, INPUT_PULLUP);
  pinMode(IR_SENSOR,INPUT);
  /// ##################################

  uartTimer.attach(0.01, UART_Handle);
  // SerialTimer.attach(0.01, String_Handle);
  
  
  
  up_stepper.setMaxSpeed(up_step_max_speed);
  up_stepper.setAcceleration(up_step_max_acc);
  down_stepper.setMaxSpeed(down_step_max_speed);
  down_stepper.setAcceleration(down_step_max_acc);


  /// ##################################
  /// Timer Setup
  // timer1 = timerBegin(0, 80, true); /// 80 MHz / 80 1 MHz (1 tick = 1 us)
  // timerAttachInterrupt(timer1, &stepper_handle, true);
  // timerAlarmWrite(timer1, timer1_interval, true);
  // timerAlarmEnable(timer1);
  /// ##################################


  EndEffector.attach(SERVO);
  EndEffector.write(10);

}

void loop() {

  if (Timer_Flag == true){
    Timer_Flag = false;

    if (Home_pose == false){
      Set_Home();
    }else if (Home_pose == true){ 
      stepper_control();
      Home_status = 1;
    }

    if (arm_unpress_cmd == -1) {
      Home_pose = false;
      Home_status = 0;
    }
    // Serial.print((int)arm_up_cmd);
    // Serial.print(" | ");
    // Serial.print(arm_down_cmd);
    // Serial.print(" | ");
    // Serial.print(arm_right_cmd);
    // Serial.print(" | ");
    // Serial.print(arm_left_cmd);
    // Serial.print(" | ");
    // Serial.print(arm_press_cmd);
    // Serial.print(" | ");
    // Serial.println(arm_unpress_cmd);
    // Serial.print(up_stepper_pos);
    // Serial.print(" | ");
    // Serial.println(down_stepper_pos);
    
  }

up_stepper.runSpeed();
down_stepper.runSpeed();
up_stepper_pos = up_stepper.currentPosition();
down_stepper_pos = down_stepper.currentPosition();

}