#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

#define EncoderA PB1
#define EncoderB PB0
#define DIR PB4
#define PWM PB5

#define Kp 0.1
#define Ki 0.0001
#define Kd 0.1

float speed_pwm = 0;
float speed_encoder = 0;
int ts_encoder = 0;
float error = 0;
float error_sum = 0;
float error_initial = 0;

ros::NodeHandle nh;

// Class for motors with encoders for MD10C drivers
class integrated_encoder_and_motor
{

  private:

  // Motor and Encoder Pins
  uint8_t direction, pwm, encoderA, encoderB;

  // Pulses required to complete a rotation
  int pulsesPerRotation;

  void stop()
  {
    analogWrite(pwm, 0);
  }

  void rotateAntiClockwise(int speed)
  {
    digitalWrite(direction, LOW);
    analogWrite(pwm, speed);
  }

  void rotateClockwise(int speed)
  {
    digitalWrite(direction, HIGH);
    analogWrite(pwm, speed);
  }

  public:

  // Pulses of the encoder
  long pulseCount;

  // Constructor for this class
  integrated_encoder_and_motor(int direction, int pwm, int encoderA, int encoderB, int pulsesPerRotation)
  {
    this->direction = direction;
    this->pwm = pwm;
    this->encoderA = encoderA;
    this->encoderB = encoderB;
    this->pulsesPerRotation = pulsesPerRotation;
    this->pulseCount = 0;
  }

  // Function to rotate the motor
  void rotate(int speed)
  {
    if (speed > 0 && speed <= 255)
    {
      rotateAntiClockwise(speed);
    } else if (speed < 0 && speed >= -255)
    {
      rotateClockwise((-1) * speed);
    } else
    {
      analogWrite(pwm, 0);
    }
  }

  // Functions for changing the pulse count

  void changePulseCount0()
  {
    if (digitalRead(EncoderB) == LOW)
    {
      pulseCount++;
    } else
    {
      pulseCount--;
    }
  }

  void changePulseCount1()
  {
    if (digitalRead(EncoderA) == LOW)
    {
      pulseCount--;
    } else
    {
      pulseCount++;
    }
  }

  float getSpeed()
  {
    long pulseCountI = this->pulseCount;
    delay(10);
    long pulseCountF = this->pulseCount;
    return (pulseCountF - pulseCountI) * 100;
  }

};

integrated_encoder_and_motor Motor(DIR, PWM, EncoderA, EncoderB, 7605);

void changePulseCount_0()
{
  Motor.changePulseCount0();
}

void changePulseCount_1()
{
  Motor.changePulseCount1();
}

void setSpeed(const std_msgs::Int16 &msg)
{
  ts_encoder = msg.data;
  error = 0;
  error_sum = 0;
  error_initial = 0;
}

std_msgs::Float64 msg;

ros::Publisher motor_speed_publisher("motor_speed", &msg);
ros::Subscriber<std_msgs::Int16> motor_speed_subscriber("set_motor_speed", &setSpeed);

void setup()
{

  // put your setup code here, to run once:

  // Defining the pinmodes for all the pins
  pinMode(EncoderA, INPUT_PULLUP);
  pinMode(EncoderB, INPUT_PULLUP);
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);

  // Interrupts to be attached
  attachInterrupt(EncoderA, changePulseCount_0, RISING);
  attachInterrupt(EncoderB, changePulseCount_1, RISING);

  // Initializing the nodes
  nh.initNode();
  nh.advertise(motor_speed_publisher);
  nh.subscribe(motor_speed_subscriber);

}

void loop()
{

  // put your main code here, to run repeatedly:

  if (speed_encoder < ts_encoder)
  {
    error_initial = error;
    error = (ts_encoder - speed_encoder) / 80;
    error_sum += error;
    speed_pwm += Kp * error + Ki * error_sum + Kd * (error - error_initial);
    if (speed_pwm > 255)
    {
      speed_pwm = 255;
    }
  } else if (speed_encoder > ts_encoder)
  {
    error_initial = error;
    error = (ts_encoder - speed_encoder) / 80;
    error_sum += error;
    speed_pwm += Kp * error + Ki * error_sum + Kd * (error - error_initial);
    if (speed_pwm < -255)
    {
      speed_pwm = -255;
    }
  }

  Motor.rotate(speed_pwm);
  delay(10);
  speed_encoder = Motor.getSpeed();
  msg.data = speed_encoder;
  motor_speed_publisher.publish(&msg);

  nh.spinOnce();
  delay(10);

}