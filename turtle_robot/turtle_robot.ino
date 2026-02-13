/*
 * turtle_robot.ino
 *
 * Differential drive robot
 * Arduino UNO + ROS Noetic (rosserial)
 *
 * Encoders connected to hardware interrupts:
 *   INT0 -> pin 2
 *   INT1 -> pin 3
 *
 * Topics:
 *   Sub:  /cmd_vel      (geometry_msgs/Twist)
 *   Pub:  /wheel_rpm    (Float32MultiArray [left, right])
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

// ================= PIN CONFIG =================
#define ENA 5
#define IN1 6
#define IN2 7

#define ENB 9
#define IN3 10
#define IN4 11

#define ENC_L_A 2   // INT0
#define ENC_R_A 3   // INT1

// ================= CONSTANTS =================
#define PULSES_PER_REV 330.0f
#define WHEEL_BASE     0.12f

// ================= VARIABLES =================
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

unsigned long last_time = 0;

ros::NodeHandle nh;
std_msgs::Float32MultiArray rpm_msg;
ros::Publisher rpm_pub("wheel_rpm", &rpm_msg);

// ================= ENCODER ISR =================
void leftEncoderISR()  { left_ticks++; }
void rightEncoderISR() { right_ticks++; }

// ================= MOTOR CONTROL =================
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

// ================= CMD_VEL CALLBACK =================
void cmdVelCallback(const geometry_msgs::Twist &msg)
{
  float linear  = msg.linear.x;
  float angular = msg.angular.z;

  float left_speed  = linear - (angular * WHEEL_BASE * 0.5f);
  float right_speed = linear + (angular * WHEEL_BASE * 0.5f);

  setMotor(ENA, IN1, IN2, left_speed);
  setMotor(ENB, IN3, IN4, right_speed);
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmdVelCallback);

// ================= SETUP =================
void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);

  attachInterrupt(0, leftEncoderISR, RISING);
  attachInterrupt(1, rightEncoderISR, RISING);

  nh.initNode();
  nh.advertise(rpm_pub);
  nh.subscribe(cmd_sub);

  rpm_msg.data_length = 2;
  rpm_msg.data = (float*)malloc(sizeof(float) * 2);

  last_time = millis();
}

// ================= LOOP =================
void loop()
{
  nh.spinOnce();

  unsigned long now = millis();

  if (now - last_time >= 1000UL)
  {
    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    left_ticks = 0;
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
