// Basic demo for accelerometer readings from Adafruit MPU6050
#define VOL_MEASURE_PIN A2
#define ECHO_PIN A3
#define TRIG_PIN 11
#define RECV_PIN 9
#define NUMPIXELS 4
#define RGB_PIN 3
#define AIN1 7
#define PWMA_LEFT 5
#define BIN1 12
#define PWMB_RIGHT 6
#define STBY_PIN 8
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN A7
#define ENCODER_RIGHT_A_PIN 4
#define ENCODER_RIGHT_B_PIN A6
#define IR_SEND_PIN 9
#define LEFT_RECEIVE_PIN A0
#define RIGHT_RECEIVE_PIN A1
#define KEY_MODE 10
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Encoder.h>

#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN A7
#define ENCODER_RIGHT_A_PIN 4
#define ENCODER_RIGHT_B_PIN A6

Encoder left_encoder( ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN );
Encoder right_encoder( ENCODER_RIGHT_B_PIN, ENCODER_RIGHT_A_PIN );

long left_angle  = -999;
long right_angle  = -999;


volatile unsigned long int left_encoder_a_count = 0;
volatile unsigned long int left_encoder_b_count = 0;
volatile unsigned long int right_encoder_a_count = 0;
volatile unsigned long int right_encoder_b_count = 0;
Adafruit_MPU6050 mpu;
float last_gxx = 0;
float last_body_angle = 0;
void motor_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  //left_encoder.write(0);
}
void motor_left_command( int speed )
{
    if ( speed >= 0 )
    {
      digitalWrite( AIN1, 1 );
      analogWrite( PWMA_LEFT, speed );
    }
  else
    {
      digitalWrite( AIN1, 0 );
      analogWrite( PWMA_LEFT, -speed );
    }
}

// reverses the sign of "speed"
void motor_right_command( int speed )
{
  if ( speed >= 0 )
    {
      digitalWrite( BIN1, 1 );
      analogWrite( PWMB_RIGHT, speed );
    }
  else
    {
      digitalWrite( BIN1, 0 );
      analogWrite( PWMB_RIGHT, -speed );
    }
}
void setup(void) {


  Serial.begin(115200);
  motor_init();
  motor_left_command(70);

 // Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //Serial.print("Accelerometer range set to: ");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //Serial.print("Gyro range set to: ");
  

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.print("Filter bandwidth set to: ");
  

  //Serial.println("");
  delay(100);
}

void loop() {
 //gzz and axx
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */

  Serial.print(a.acceleration.x);
  Serial.print(" ");



  Serial.print(g.gyro.z);
  float gscale = 0.025;
  float Kf_body = 0.02
  
  ;
  float gxx = g.gyro.z;
  float ayy = a.acceleration.x;
  float body_angle = last_body_angle + gscale*(gxx + last_gxx);
      // measurement update
  body_angle = body_angle - Kf_body*(body_angle - ayy);
  last_body_angle = body_angle;
  last_gxx = gxx;
  Serial.print(" ");
  Serial.print(body_angle);
  Serial.println(" ");

  delay(2);
}
