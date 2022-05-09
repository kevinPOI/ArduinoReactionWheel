/**********************************************************************
Run a balancing servo off a clock.
Get input from IMU:
 * Need to have "I2Cdev" folder in Arduino libraries
 * Need to have "MPU6050" folder in Arduino libraries
Estimate body angle (in accelerometer Y units)
/**********************************************************************/

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Pins.h"

#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN A7
#define ENCODER_RIGHT_A_PIN 4
#define ENCODER_RIGHT_B_PIN A6
#include <Encoder.h>
//Encoder left_encoder( ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN );
//Encoder right_encoder( ENCODER_RIGHT_B_PIN, ENCODER_RIGHT_A_PIN );
/**********************************************************************/

#define MAX_ULONG 0xFFFFFFFF

// I had to increase this to 3ms to accomodate reading the IMU
#define SERVO_INTERVAL 3000 // microseconds

// start data collection after N milliseconds after starting servo
#define START_COLLECT  2000 // milliseconds
// start balancer N milliseconds after START_COLLECT
#define START_BALANCE   100 // milliseconds

#define DEBUG_PRINT_INTERVAL 500 // milliseconds, 

#define MAX_COMMAND 220

// #define PI 3.141592653589793
#define ENCODER_TO_RADIANS 0.004027682889218

/**********************************************************************/
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN A7
#define ENCODER_RIGHT_A_PIN 4
#define ENCODER_RIGHT_B_PIN A6

volatile unsigned long int left_encoder_a_count = 0;
volatile unsigned long int left_encoder_b_count = 0;
volatile unsigned long int right_encoder_a_count = 0;
volatile unsigned long int right_encoder_b_count = 0;
bool go = false;  /* motor enable */
bool direction_cw = true; //keep track of direction, so that when switch direction we can pulse
bool jump = false; //jump-up mode
int jump_counter = 0;
int avg_movement = 0;
#define JUMP_COUNTER_MAX 150 //if full-powered for x cycles without angle change, enable jump-up. Must be multiple of 5 
// Encoder readings: these can be positive or negative.
long left_angle = 0;
long right_angle = 0;

// velocity filter stuff
float left_diff[10];
float right_diff[10];
float past_left_angle_radians;
float past_right_angle_radians;
float past_left_velocity[10];
float past_right_velocity[10];

MPU6050 accelgyro;

// Current accelerometer and gyro zero values.
int16_t ax, ay, az;
int16_t gx, gy, gz;
int state = 0;
int ax0 = 0;
int ay0 = 1000;
// adjustment of az is not useful.
int gx0 = 325;
int gy0 = 0;
int gz0 = 0;

int last_body_angle = 0;
int last_gxx = 0; // last X gyro reading
int last_gzz = 0; // last X gyro reading



// Keep track of late control cycles. 
unsigned long servo_late = 0; // How many times am I late?
unsigned long max_servo_late = 0; // What was the worst one?

/**********************************************************************/
/**********************************************************************/
void cw(int p){
  analogWrite(LPWM,0);
  analogWrite(RPWM,p);
}
void ccw(int p){
  analogWrite(RPWM,0);
  analogWrite(LPWM,p);
}
void sm(){
  analogWrite(RPWM,0);
  analogWrite(LPWM,0);
}
void motor_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
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

void motor_stop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
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

/**********************************************************************/

void setup()
{

  Wire.begin();  // I2C (TWI) uses Wire library
  Serial.begin( 1000000 );  // run Serial fast to print out data
  Wire.setClock( 1000000UL ); // run I2C as fast as possible
  analogReference(INTERNAL1V1); // was INTERNAL on 328P, voltage

  while( !Serial ); // Wait for serial port to become ready
  Serial.println( "balance1 version 1" ); // print out what code is running
  delay(200); // Delay to make sure above message prints out.

  motor_init(); // Initialize motors
  Serial.println( "motor_init done." );
  delay(200); // Delay to make sure above message prints out.

  //Encoder_init( &left_angle, &right_angle ); // Initialize (zero) encoders
  Serial.println( "Initialized encoders" );
  delay(200); // Delay to make sure above message prints out.

  accelgyro.initialize(); // Initialize IMU
  Serial.println( "IMU initialized" );
  delay(200); // Delay to make sure above message prints out.

  Serial.println( "Robot should be standing up with training wheels."  );
  Serial.println( "Type g <return> to run test, s <return> to stop."  );
  Serial.println( "Typing window is at the top of the Arduino serial monitor window." );
  Serial.println( "Type into the main window of a Putty serial monitor window." );

  // code starts disabled, so every time Arduino powers up nothing happens
  go = false; 
}

/******************************************************************/

// Take user input
void ProcessCommand()
{

  if ( Serial.available() <= 0 )
    return;

  int c = Serial.read();
  switch (c)
    {
      case 'S': case 's':
        Serial.println( "Stop!" );
        go = false;
        break;
      case 'G': case 'g':
      Serial.println( "Go!" );
        go = true;
        break;
      default:
        break;
    }
}

/**********************************************************************

/**********************************************************************/

void run_balancer( float body_angle_gain, float body_angular_velocity_gain,
     		   float some_kind_of_integral_gain, // to be determined
                   unsigned long run_time_limit, int collect_data )
{
  // Clocks and time management
  // used to keep track of time intervals in servo
  unsigned long last_micros = micros();
  unsigned long last_millis = millis();
  int servo_time = 0; // Servo clock: needs to be signed, can be int
  // how long have we been running? Safety feature
  unsigned long run_time = 0; // milliseconds
  // Clocks for printing and other stuff
  unsigned long debug_print_time = 0; // milliseconds

  run_time_limit += START_COLLECT; // account for startup time

  // state
  int started = 0;

  // integrators
  long integrator_state = 0;

  // keep track of voltage
  int voltage_raw = analogRead(VOL_MEASURE_PIN); //Read voltage value
  double voltage = (voltage_raw*1.1/1024)*((10+1.5)/1.5);
  if ( collect_data )
    {
      Serial.print( "Current voltage " );
      Serial.print( voltage );
      Serial.print( " " );
      Serial.println( voltage_raw );
    }

  // print out the angle_gain
  if ( collect_data )
    {
      Serial.print( "Gains " );
      Serial.print( body_angle_gain );
      Serial.print( " " );
      Serial.println( body_angular_velocity_gain );
    }

  // print out IMU biases
  if ( collect_data )
    {
      Serial.print( "IMU biases " );
      Serial.print( ax0 );
      Serial.print( " " );
      Serial.print( ay0 );
      Serial.print( " " );
      Serial.print( gx0 );
      Serial.print( " " );
      Serial.print( gy0 );
      Serial.print( " " );
      Serial.println( gz0 );
    }

  // servo loop
  for( ; ; )
    {

/*****************************************
Timekeeping part */

      unsigned long current_micros = micros();
      if ( current_micros > last_micros )
        servo_time += current_micros - last_micros;
      else // rollover
        servo_time += (MAX_ULONG - last_micros) + current_micros;
      last_micros = current_micros;

      // It isn't time yet ...
      if ( servo_time < SERVO_INTERVAL )
        continue;

      // It is time! Reset the servo clock.
      servo_time -= SERVO_INTERVAL;

      // Keep track of maximum lateness
      if ( max_servo_late < servo_time )
        max_servo_late = servo_time;

      // Let's have some slop in counting late cycles.
      if ( servo_time > 50 )
        servo_late += 1;
    
      // handle the millisecond clocks
      unsigned long current_millis = millis();
      int millis_increment;
      if ( current_millis > last_millis )
        millis_increment = current_millis - last_millis;
      else // rollover
        millis_increment = (MAX_ULONG - last_millis) + current_millis;
      last_millis = current_millis;
  
      run_time += millis_increment;
      debug_print_time += millis_increment;

/*****************************************
Administrative part */

      // Turn off after a while to keep from running forever
      if ( ( run_time > run_time_limit ) || ( !go ) )
        {
          motor_stop();
          Serial.println( "Motor stopped." );
          Serial.println( "Wheels should be off the ground."  );
          Serial.println( "Type g <return> to run test, s <return> to stop." );
          return;
        }

/*****************************************
State estimation part */


      // Read the sensors
      //Read_encoders( &left_angle, &right_angle );

      // Current accelerometer and gyro zero values.
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      //##################################################################gzz and axx#################################################
      // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      accelgyro.getAcceleration( &ax, &ay, &az );
      gz = accelgyro.getRotationZ();
      // subtract zeros
      
      int axx = ax - ax0-1800;
      int ayy = ay - ay0;
      // adjustment of az is not useful.
      int azz = az;
      int gzz = gz;
      // int gyy = gy - gy0;
      // int gzz = gz - gz0;

      float gscale = 0.03;
      float Kf_body = 0.025;

      // prediction step
      int body_angle = last_body_angle + gscale*(gzz + last_gzz);
      // measurement update
      body_angle = body_angle - Kf_body*(body_angle - axx);
      avg_movement = 0.8*avg_movement + 0.2*(body_angle - last_body_angle); // track recent changes in position, discounted by time
      last_gzz = gzz;
      last_body_angle = body_angle; //moved this after command calculation for ease of jump up part
      int body_angular_velocity = gzz;
      Serial.print("axx is");
      Serial.print(axx);
      Serial.print("angle is:");
      //long dist = left_encoder.read();
      Serial.print(body_angle);
      //Serial.print(" dist is:");
      //Serial.println(dist);

/*****************************************

//    if ( run_time >= move_time )

/*
      float left_error = left_angle_radians - angle_desired;
      float right_error = right_angle_radians - angle_desired;
*/  

      long command_long = 0;
     
      if ( run_time < START_COLLECT + START_BALANCE )
        { // Do nothing;
      	}
      else // Let's balance!
        {
          
          command_long = - body_angle_gain*body_angle
	                 - body_angular_velocity_gain*body_angular_velocity;
    	                 // + angle_error_integral_left;
          command_long = command_long >> 8;
          
        }
      float clf = command_long;
      if(clf > 0){
        clf = pow(clf, 0.85);
      }else{
        clf = -pow(-clf, 0.85);
      }
      int command = (int) clf;

      if ( command > MAX_COMMAND )
        command = MAX_COMMAND;
      if ( command < -MAX_COMMAND )
        command = -MAX_COMMAND;
      //Jump up part//////////////////////////////////////////////////////////////////////////////////////
      if(command == abs(MAX_COMMAND) && avg_movement < 300){//if full power yet not moving
        jump_counter ++; 
        if( jump_counter > JUMP_COUNTER_MAX){
          jump_counter = 30;//reverse-rev continue for 30 ticks
          jump = true;
        }
      }
      if(jump == true){//reverse-rev motor before jumpup to generate greater change angular momentum
        jump_counter -= 1;
        if(command > 0)command = -75;
        else command = 75;
        if(jump_counter <0) jump == false;
      }
      Serial.print("command");
      Serial.println(command);
      //command part//////////////////////////////////////////////////////////////////////////////////////
      if (direction_cw && command > 0){
        cw(command);
      }else if(!direction_cw && command > 0){//delay to fully turnoff hbridge before reversing to avoid kboom
        direction_cw = true;
        sm();
        delay(10);
        cw(command);
      }else if(direction_cw && command < 0){
        direction_cw = false;
        sm();
        delay(10);
        ccw(-command);
      }else if(!direction_cw && command < 0){
        ccw(-command);
      }else{
        sm();
      }
      

/*****************************************
Data printing part */

      // print out debugging info before data collection
      if ( ( debug_print_time > DEBUG_PRINT_INTERVAL )
           && ( ( run_time < START_COLLECT )
	        || ( run_time > run_time_limit ) ) )
        {

          ProcessCommand();
          debug_print_time -= DEBUG_PRINT_INTERVAL;
        }

      // print out data
      if ( ( run_time >= START_COLLECT ) && ( run_time <= run_time_limit )
           && collect_data )
        {
          if ( !started )
            {
              servo_late = 0;
              max_servo_late = 0;
              started = 1;
              Serial.println( "Data" );
            }
          
        }
    }
}

/**********************************************************************/

void loop()
{
  if ( !go )
    {
      motor_stop();
      delay(500);
      ProcessCommand();
      return;
    }


  Serial.println( "Please type the X accelerometer zero: " );
  while(Serial.available() == 0){}
  ax0 = Serial.parseInt();
  Serial.print( "You typed: " );
  Serial.println( ax0 );
  if ( ax0 < -4000 || ax0 > 4000 )
    {
      Serial.println( "Bad value: Start over" );
      return;
    }


  Serial.println( "Please type the desired body angle wrt vertical gain: " );
  while(Serial.available() == 0){}
  float body_angle_gain = Serial.parseInt();
  body_angle_gain /= 100;
  Serial.print( "You typed: " );
  Serial.println( body_angle_gain );
  if ( body_angle_gain < 0 || body_angle_gain > 10000 )
    {
      Serial.println( "Bad value: Start over" );
      return;
    }

  Serial.println( "Please type the desired body angular velocity gain: " );
  while(Serial.available() == 0){}
  float body_angular_velocity_gain = Serial.parseInt();
  body_angular_velocity_gain /= 100.0;
  Serial.print( "You typed: " );
  Serial.println( body_angular_velocity_gain );
  if ( body_angular_velocity_gain < -100 || body_angular_velocity_gain > 100 )
   {
      Serial.println( "Bad value: Start over" );
      return;
    }

  run_balancer( body_angle_gain, body_angular_velocity_gain, 0.0, 20000, 1 );
  sm();
  Serial.println( "Stop!" );
  go = false;
}
  
/**********************************************************************/
