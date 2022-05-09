#include <Encoder.h>

Encoder left_encoder( ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN );
Encoder right_encoder( ENCODER_RIGHT_B_PIN, ENCODER_RIGHT_A_PIN );

void Encoder_init(  long *left_angle,  long *right_angle )
{
  left_encoder.write(0);
  right_encoder.write(0);
  *left_angle = 0;
  *right_angle = 0;
}

void Read_encoders(  long *left_angle, long *right_angle )
{
  *left_angle = left_encoder.read();
  *right_angle = right_encoder.read();
}
