/*
 * test_motor.ino
 *
 * Single DC motor test for ROS Noetic + Arduino UNO.
 * Uses L298N motor driver and one quadrature encoder channel.
 *
 * ROS:
 *   Sub:  /cmd_vel    (geometry_msgs/Twist)  -> uses linear.x
 *   Pub:  /wheel_rpm  (std_msgs/Float32)
 *
 * Hardware (Arduino UNO):
 *
 *   L298N ENA  -> Pin 5  (PWM)
 *   L298N IN1  -> Pin 8
 *   L298N IN2  -> Pin 9
 *
 *   Encoder A  -> Pin 2  (INT0)
 *   Encoder B  -> Pin 4  (optional, not used in this test)
 *
 * Important:
 *   All grounds (Arduino, L298N, encoder power) must be common.
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

// ===================== Pin Definitions =====================
#define ENA_PIN   5
#define IN1_PIN   8
#define IN2_PIN   9
#define ENC_A_PIN 2   // INT0
#define ENC_B_PIN 4   // optional

// ===================== Constants =====================
#define PULSES_PER_REV 11.0f   // Adjust to your encoder
#define SAMPLE_TIME_MS 1000UL  // RPM calculation window

// ===================== Global Variables =====================
volatile long encoder_ticks = 0;

unsigned long last_time = 0;
float current_rpm = 0.0f;

// ===================== ROS =====================
ros::NodeHandle nh;

std_msgs::Float32 rpm_msg;
ros::Publisher rpm_pub("wheel_rpm", &rpm_msg);

void cmdVelCallback(const geometry_msgs::Twist &msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmdVelCallback);

// ===================== Encoder ISR =====================
void encoderISR()
{
  encoder_ticks++;
}

// ===================== Motor Control =====================
void setMotorSpeed(float pwm_value)
{
  // Clamp PWM range
  if (pwm_value > 255.0f)  pwm_value = 255.0f;
  if (pwm_value < -255.0f) pwm_value = -255.0f;

  if (pwm_value > 0.0f)
  {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, (int)pwm_value);
  }
  else if (pwm_value < 0.0f)
  {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, (int)(-pwm_value));
  }
  else
  {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);
  }
}

// ===================== ROS Callback =====================
void cmdVelCallback(const geometry_msgs::Twist &msg)
{
  /*
   * For this test:
   * linear.x range expected between -1.0 and 1.0
   * Scale directly to PWM range.
   */
  float pwm_command = msg.linear.x * 255.0f;
  setMotorSpeed(pwm_command);
}

// ===================== Setup =====================
void setup()
{
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);

  // Arduino UNO:
  // INT0 = interrupt 0 = pin 2
  attachInterrupt(0, encoderISR, RISING);

  nh.initNode();
  nh.advertise(rpm_pub);
  nh.subscribe(cmd_sub);

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
    long ticks = encoder_ticks;
    encoder_ticks = 0;
    interrupts();

    current_rpm = (ticks / PULSES_PER_REV) * 60.0f;

    rpm_msg.data = current_rpm;
    rpm_pub.publish(&rpm_msg);

    last_time = now;
  }

  delay(5);
}
