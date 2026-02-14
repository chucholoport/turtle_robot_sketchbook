/*
 * turtle_robot.ino
 *
 * Differential drive robot
 * Compatible with:
 *   - Arduino UNO
 *   - Arduino Mega 2560
 *
 * ROS Noetic + rosserial
 *
 * Encoders use:
 *   Left  -> Pin 2 (INT0)
 *   Right -> Pin 3 (INT1)
 *
 * ROS:
 *   Sub:  /cmd_vel     (geometry_msgs/Twist)
 *   Pub:  /wheel_rpm   (Float32MultiArray [left, right])
 *
 * Motor Driver (L298N example):
 *
 *   LEFT MOTOR
 *     ENA -> 5 (PWM)
 *     IN1 -> 6
 *     IN2 -> 7
 *
 *   RIGHT MOTOR
 *     ENB -> 9 (PWM)
 *     IN3 -> 10
 *     IN4 -> 11
 *
 * All grounds must be common.
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

// ===================== Pin Definitions =====================
#define ENA 5
#define IN1 6
#define IN2 7

#define ENB 9
#define IN3 10
#define IN4 11

#define ENC_L_A 2   // INT0
#define ENC_L_B 4

#define ENC_R_A 3   // INT1
#define ENC_R_B 12

// ===================== Robot Constants =====================
#define PULSES_PER_REV 330.0f
#define WHEEL_BASE     0.12f
#define SAMPLE_TIME_MS 1000UL

// ===================== Variables =====================
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

unsigned long last_time = 0;

// ===================== ROS =====================
ros::NodeHandle nh;

std_msgs::Float32MultiArray rpm_msg;
ros::Publisher rpm_pub("wheel_rpm", &rpm_msg);

void cmdVelCallback(const geometry_msgs::Twist &msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmdVelCallback);

// ===================== Encoder ISRs =====================
void leftEncoderISR()
{
  if (digitalRead(ENC_L_B) == HIGH)
    left_ticks++;
  else
    left_ticks--;
}

void rightEncoderISR()
{
  if (digitalRead(ENC_R_B) == HIGH)
    right_ticks++;
  else
    right_ticks--;
}

// ===================== Motor Control =====================
void setMotor(uint8_t en, uint8_t in1, uint8_t in2, float speed)
{
  int pwm = constrain((int)(fabs(speed) * 255.0f), 0, 255);

  if (speed > 0.0f)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (speed < 0.0f)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(en, pwm);
}

// ===================== ROS Callback =====================
void cmdVelCallback(const geometry_msgs::Twist &msg)
{
  float linear  = msg.linear.x;
  float angular = msg.angular.z;

  // Differential drive kinematics
  float left_speed  = linear - (angular * WHEEL_BASE * 0.5f);
  float right_speed = linear + (angular * WHEEL_BASE * 0.5f);

  setMotor(ENA, IN1, IN2, left_speed);
  setMotor(ENB, IN3, IN4, right_speed);
}

// ===================== Setup =====================
void setup()
{
  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // Hardware interrupts (same mapping for UNO and Mega)
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);

  // ROS
  nh.initNode();
  nh.advertise(rpm_pub);
  nh.subscribe(cmd_sub);

  rpm_msg.data_length = 2;
  rpm_msg.data = (float*)malloc(sizeof(float) * 2);

  last_time = millis();
}

// ===================== Loop =====================
void loop()
{
  nh.spinOnce();

  unsigned long now = millis();

  if (now - last_time >= SAMPLE_TIME_MS)
  {
    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    left_ticks  = 0;
    right_ticks = 0;
    interrupts();

    float left_rpm  = (lt / PULSES_PER_REV) * 60.0f;
    float right_rpm = (rt / PULSES_PER_REV) * 60.0f;

    rpm_msg.data[0] = left_rpm;
    rpm_msg.data[1] = right_rpm;

    rpm_pub.publish(&rpm_msg);

    last_time = now;
  }

  delay(5);
}
