#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
SoftwareSerial HC06(9, 10); //HC06-TX Pin 10, HC06-RX to Arduino Pin 11

//____________________________________________________________________
//----------------------- Hardware parameters  -----------------------
const float X0 = 0;
const float Y0 = 0;
const float Omega_traj = 0.5; //  meter
const float R_traj = 0.075;   //  meter
const float R_wheel = 0.031;  //  wheel radius  meter
const float L_wheel = 0.135;   //  wheels distance  meter
//--------------------- END Hardware parameters  ---------------------
//____________________________________________________________________
//----------------------  SPEED CONTROLLER VARS ----------------------
volatile double pulses_R, pulses_L;
double Ts;
double speed_R, speed_ref_R;
double speed_L, speed_ref_L;
double E_R,E1_R,E2_R;
double E_L,E1_L,E2_L;
double alfa_R, beta_R, gamma_R, Kp_speed_R, Kd_speed_R, Ki_speed_R;
double alfa_L, beta_L, gamma_L, Kp_speed_L, Kd_speed_L, Ki_speed_L;
double PWM_R, lastPWM_R;
double PWM_L, lastPWM_L;

//--------------------  POSITION CONTROLLER VARS ----------------------
float state_ref[2];
float state_real[3];

float state_C[2];
float state_E[3];
float state_dot[3];
float state_M;

Matrix<3,1> state_enc;

//------------------------------ MCU6050 ------------------------------
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0, Degree = 0, VirYaw = 0, Vir1Yaw = 0, i = 0;

//--------------------------------------------------------------------
float ep1=0, ep2=0, ep1_old=0, ep2_old=0;
float Kp_pose, Kd_pose;

float time_enc = 0, time_enc_old = 0;
float sample_time_enc = 0;    //  = time_enc - time_enc_old

float time = 0, time_old = 0;
float sample_time = 0;    //  = time - time_old

//_____________________________________________________________________
//-------------------------  SPEED CONTROLLER -------------------------
void counter_R()
{
    pulses_R++;
}

void counter_L()
{
    pulses_L++;
}

void speed_cal()
{
  noInterrupts();
 
  speed_R = (pulses_R/334)*(1/Ts)*60;            //  RPM
  pulses_R = 0;
  speed_L = (pulses_L/334)*(1/Ts)*60;            //  RPM
  pulses_L = 0;

  interrupts();
}

float Speed_converter(float RPM)
{
  //  convert wheel speed from RPM to m/S
  return RPM * 2*PI*R_wheel / 60;
}
void speed_controller()
{ 
  alfa_L = 2*Ts*Kp_speed_L+Ki_speed_L*Ts*Ts+2*Kd_speed_L;
  beta_L = Ts*Ts*Ki_speed_L-4*Kd_speed_L-2*Ts*Kp_speed_L;
  gamma_L = 2*Kd_speed_L;

  alfa_R = 2*Ts*Kp_speed_R+Ki_speed_R*Ts*Ts+2*Kd_speed_R;
  beta_R = Ts*Ts*Ki_speed_R-4*Kd_speed_R-2*Ts*Kp_speed_R;
  gamma_R = 2*Kd_speed_R;
  //------------------------- SPEED RIGHT -------------------------
  E_R = speed_ref_R - speed_R;
  PWM_R = (alfa_R*E_R+beta_R*E1_R+gamma_R*E2_R+2*Ts*lastPWM_R)/(2*Ts);
  lastPWM_R = PWM_R;
  E2_R= E1_R;
  E1_R= E_R;
  if (PWM_R > 255)
    PWM_R  = 255;
  if (PWM_R < 0)
    PWM_R  = 0;
  //__________________  pulse PWM ________________
  if (PWM_R > 0)
  {
    analogWrite(5, PWM_R);
   digitalWrite(8, LOW);
  }
  else
  {
    analogWrite(5, 0);
    digitalWrite(8, LOW);
  }
  //------------------------- SPEED LEFT -------------------------
  E_L = speed_ref_L - speed_L;
  PWM_L = (alfa_L*E_L+beta_L*E1_L+gamma_L*E2_L+2*Ts*lastPWM_L)/(2*Ts);
  lastPWM_L = PWM_L;
  E2_L= E1_L;
  E1_L= E_L;
  if (PWM_L > 255)
    PWM_L  = 255;
  if (PWM_L < 0)
    PWM_L  = 0;
  //__________________  pulse PWM ________________
  if (PWM_L > 0)
  {
    analogWrite(6, PWM_L);
    digitalWrite(7, LOW);
    
  }
  else
  {
    analogWrite(6, 0);
    digitalWrite(7, LOW);
    
  }
}
//-----------------------  END SPEED CONTROLLER -----------------------
//_____________________________________________________________________
//------------------------  SENSOR CALCULATION  -----------------------
void state_encoder()
{
  //  use speed_R, speed_L and theta to calculate position of robot
  time_enc = millis();
  float speed_R_ms = Speed_converter(speed_R);
  float speed_L_ms = Speed_converter(speed_L);

  float theta = state_enc(2);
  sample_time_enc = (time_enc - time_enc_old)/1000;

  float x_dot     = 0.5*R_wheel*cos(theta)  *(speed_R_ms + speed_L_ms);
  float y_dot     = 0.5*R_wheel*sin(theta)  *(speed_R_ms + speed_L_ms);
  float theta_dot = R_wheel/L_wheel         *(speed_R_ms - speed_L_ms);
  
  state_enc(0) += x_dot * sample_time_enc;
  state_enc(1) += y_dot * sample_time_enc;
  state_enc(2) += theta_dot * sample_time_enc;

  // state_E[0] += x_dot * sample_time_enc;
  // state_E[1] += y_dot * sample_time_enc;
  // state_E[2] += theta_dot * sample_time_enc;
  
  time_enc_old = time_enc;
}
void state_camera()
{
  // if(HC06.available() > 0) //When HC06 receive something
  // {
  //   char receive = HC06.read(); //Read from Serial Communication
  //   Serial.println(receive);
  // }
  if(HC06.available()>0) 
  {
    String X, Y;
    X = HC06.readStringUntil('X');
    state_C[0] = X.toFloat();
    Serial.print("x:");
    Serial.print(state_C[0]);
    Serial.print("y:");
    Y = HC06.readStringUntil('Y');
    state_C[1] = X.toFloat();
    Serial.println(state_C[1]);
    //delay (500);
  }
}
//----------------------  END SENSOR CALCULATION  ---------------------
//_____________________________________________________________________
void setup() 
{
  pinMode(2,INPUT_PULLUP);    //chan encoder R
  pinMode(3,INPUT_PULLUP);    //chan encoder L
  pinMode(5,OUTPUT);          //chan PWM R
  pinMode(6,OUTPUT);          //chan PWM L
  pinMode(7,OUTPUT);          //chan DIR1 
  pinMode(8,OUTPUT);         //chan DIR2
 
  Serial.begin(9600);
  HC06.begin(9600);           //Baudrate 9600 , Choose your own baudrate 
  attachInterrupt(0, counter_R, FALLING);
  attachInterrupt(1, counter_L, FALLING);
  Timer1.initialize(10000);     //  microseconds
  Timer1.attachInterrupt(speed_cal);
  
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // end the transmission
  calculate_IMU_error();
  delay(20);
  //----------------------------------------------------
  E_R = 0;E1_R = 0;E2_R = 0;
  E_L = 0;E1_L = 0;E2_L = 0;
  PWM_R = 0;lastPWM_R = 0;
  PWM_L = 0;lastPWM_L = 0;
  //thong so PID
  Ts = 0.01;//thoi gian lay mau
  Kp_speed_R = 0.0138;      Kd_speed_R = 0.00004;     Ki_speed_R = 0.2983;   // Bộ 3 => Good for motor 2!
  Kp_speed_L = 0.0138;      Kd_speed_L = 0.00004;     Ki_speed_L = 0.2983;   // Bộ 3 => Good for motor 2!
  //Kp = 1;Kd = 0;Ki = 0;
  speed_ref_R = 3000;   speed_R = 0;
  speed_ref_L = 3000;   speed_L = 0;

  state_E[0] = 0; state_E[1] = 0; state_E[2] = 0;
  state_enc.Fill(0);
  //--------------------------------------------------
  //Kp_pose = 10; Kd_pose = 0;
}

void loop()
{  
  //time_enc = millis();
  speed_controller();
  //state_encoder();
  //state_camera();
  //state_MCU();

  //Serial.print(speed_R);                  Serial.print(" ");
  Serial.print(PWM_R);                    Serial.print(" ");
  //Serial.print(speed_L);                  Serial.print(" ");
  Serial.println(PWM_L);
 
}
