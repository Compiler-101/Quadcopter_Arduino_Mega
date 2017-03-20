#include <PID_v1.h>
#include <VarSpeedServo.h>
#define BUFF_SIZE 40
char buffer[BUFF_SIZE];
byte buffer_throttle[4];

int i = 0;
int j = 0;

char yaw[7] = {' ', ' ', ' ', ' ', ' ', ' ', ' '};
char pitch[7] = {' ', ' ', ' ', ' ', ' ', ' ', ' '};
char roll[7] = {' ', ' ', ' ', ' ', ' ', ' ', ' '};

VarSpeedServo FR;
VarSpeedServo BR;
VarSpeedServo FL;
VarSpeedServo BL;

boolean start = false;

double throttle = 1100;
double minimum = 1050;
double maximum = 1900;

char data;

int esc_FR;
int esc_BR;
int esc_FL;
int esc_BL;

double p_gain_roll_pitch = 0.06;
double i_gain_roll_pitch = 0.006;
double d_gain_roll_pitch = 0.4;

double roll_angle = 0;
double roll_pid_output = 0;
double roll_setpoint = 0;

double pitch_angle = 0;
double pitch_pid_output = 0;
double pitch_setpoint = 1;

double yaw_angle = 0;
double yaw_pid_output = 0;
double yaw_setpoint = 0;

PID roll_pid(&roll_angle, &roll_pid_output, &roll_setpoint, p_gain_roll_pitch, i_gain_roll_pitch, d_gain_roll_pitch, DIRECT);
PID pitch_pid(&pitch_angle, &pitch_pid_output, &pitch_setpoint, p_gain_roll_pitch, i_gain_roll_pitch, d_gain_roll_pitch, DIRECT);
//PID yaw_pid(&yaw_angle, &yaw_pid_output, &yaw_setpoint, 3, 0.02, 0, DIRECT);


void setup() {
  Serial.begin(250000);
  Serial1.begin(9600);
  Serial3.begin(250000);
  Serial3.println("#o0");

  FR.attach(5);
  BR.attach(6);
  FL.attach(7);
  BL.attach(8);

  FR.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
  FL.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);

  roll_pid.SetOutputLimits(-600, 600);
  roll_pid.SetMode(AUTOMATIC);
  roll_pid.SetSampleTime(100);
  pitch_pid.SetOutputLimits(-600, 600);
  pitch_pid.SetMode(AUTOMATIC);
  pitch_pid.SetSampleTime(100);
  //  yaw_pid.SetOutputLimits(-400, 400);
  // yaw_pid.SetMode(AUTOMATIC);

  serialFlush();
}

void loop() {
  while (!start) {
    Serial3.println("#o0");
    if (Serial1.available()) {
      Serial1.readBytes(buffer_throttle, 4);
      if (buffer_throttle[3] == 16)
        start = true;
    }
  }
  read_sensor();

  roll_angle = atoi(roll);
  pitch_angle = atoi(pitch);
  yaw_angle = atoi(yaw);

  roll_pid.Compute();
  pitch_pid.Compute();
  //yaw_pid.Compute();

  esc_FR = throttle - (int)roll_pid_output +  (int)pitch_pid_output + (int)yaw_pid_output;
  esc_BR = throttle - (int)roll_pid_output - (int)pitch_pid_output - (int)yaw_pid_output ;
  esc_FL = throttle + (int)roll_pid_output + (int)pitch_pid_output - (int)yaw_pid_output;
  esc_BL = throttle + (int)roll_pid_output - (int)pitch_pid_output + (int)yaw_pid_output;

  if (esc_FR <= minimum) esc_FR = minimum;
  if (esc_BR <= minimum) esc_BR = minimum;
  if (esc_FL <= minimum) esc_FL = minimum;
  if (esc_BL <= minimum) esc_BL = minimum;

  if (esc_FR >= maximum) esc_FR = maximum;
  if (esc_BR >= maximum) esc_BR = maximum;
  if (esc_FL >= maximum) esc_FL = maximum;
  if (esc_BL >= maximum) esc_BL = maximum;

  FR.writeMicroseconds(esc_FR);
  BR.writeMicroseconds(esc_BR);
  FL.writeMicroseconds(esc_FL);
  BL.writeMicroseconds(esc_BL);

  //Serial.println(roll_pid_output);
  while (Serial1.available()) {
    Serial1.readBytes(buffer_throttle, 4);
    throttle = map(buffer_throttle[1], 0, 255, 1100, 1650);
    if (buffer_throttle[3] == 0) {
      start = false;
      FR.writeMicroseconds(1000);
      BR.writeMicroseconds(1000);
      FL.writeMicroseconds(1000);
      BL.writeMicroseconds(1000);
    }
    //Serial.println(throttle);
  }
}


void serialFlush() {
  while (Serial3.available() > 0) {
    char t = Serial3.read();
  }
}

void read_sensor() {
  i = 0;
  j = 0;
  Serial3.println("#f");
  while (Serial3.available()) {
    buffer[i] = Serial3.read();
    while (1) {
      if (buffer[i] != '\n') {
        yaw[j] = buffer[i];
        i++;
        j++;
      } else {
        break;
      }
      while (!Serial3.available());
      buffer[i] = Serial3.read();
    }

    j = 0;
    while (1) {
      while (!Serial3.available());
      buffer[i] = Serial3.read();
      if (buffer[i] != '\n') {
        pitch[j] = buffer[i];
        i++;
        j++;
      } else {
        break;
      }
    }

    j = 0;
    while (1) {
      while (!Serial3.available());
      buffer[i] = Serial3.read();
      if (buffer[i] != '\n') {
        roll[j] = buffer[i];
        i++;
        j++;
      } else {
        break;
      }
    }
  }
}

void clear_data() {
  for (int i = 0; i < 7 ; i++) {
    yaw[i] = ' ';
    pitch[i] = ' ';
    roll[i] = ' ';
  }
}
