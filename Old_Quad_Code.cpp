
#include <Servo.h>
#include <Wire.h>

// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-9150 Register Map and Descriptions Revision 4.0",

#define MPU9150_SELF_TEST_X        0x0D   // R/W
#define MPU9150_SELF_TEST_Y        0x0E   // R/W
#define MPU9150_SELF_TEST_X        0x0F   // R/W
#define MPU9150_SELF_TEST_A        0x10   // R/W
#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_FF_THR             0x1D   // R/W
#define MPU9150_FF_DUR             0x1E   // R/W
#define MPU9150_MOT_THR            0x1F   // R/W
#define MPU9150_MOT_DUR            0x20   // R/W
#define MPU9150_ZRMOT_THR          0x21   // R/W
#define MPU9150_ZRMOT_DUR          0x22   // R/W
#define MPU9150_FIFO_EN            0x23   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9150_I2C_SLV0_REG       0x26   // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9150_I2C_SLV1_REG       0x29   // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9150_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU9150_I2C_SLV2_REG       0x2C   // R/W
#define MPU9150_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU9150_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU9150_I2C_SLV3_REG       0x2F   // R/W
#define MPU9150_I2C_SLV3_CTRL      0x30   // R/W
#define MPU9150_I2C_SLV4_ADDR      0x31   // R/W
#define MPU9150_I2C_SLV4_REG       0x32   // R/W
#define MPU9150_I2C_SLV4_DO        0x33   // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9150_I2C_SLV4_DI        0x35   // R
#define MPU9150_I2C_MST_STATUS     0x36   // R
#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_INT_ENABLE         0x38   // R/W
#define MPU9150_INT_STATUS         0x3A   // R
#define MPU9150_ACCEL_XOUT_H       0x3B   // R
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  pda
#define MPU9150_ACCEL_YOUT_H       0x3D   // R
#define MPU9150_ACCEL_YOUT_L       0x3E   // R
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R
#define MPU9150_ACCEL_ZOUT_L       0x40   // R
#define MPU9150_TEMP_OUT_H         0x41   // R
#define MPU9150_TEMP_OUT_L         0x42   // R
#define MPU9150_GYRO_XOUT_H        0x43   // R
#define MPU9150_GYRO_XOUT_L        0x44   // R
#define MPU9150_GYRO_YOUT_H        0x45   // R
#define MPU9150_GYRO_YOUT_L        0x46   // R
#define MPU9150_GYRO_ZOUT_H        0x47   // R
#define MPU9150_GYRO_ZOUT_L        0x48   // R
#define MPU9150_EXT_SENS_DATA_00   0x49   // R
#define MPU9150_EXT_SENS_DATA_01   0x4A   // R
#define MPU9150_EXT_SENS_DATA_02   0x4B   // R
#define MPU9150_EXT_SENS_DATA_03   0x4C   // R
#define MPU9150_EXT_SENS_DATA_04   0x4D   // R
#define MPU9150_EXT_SENS_DATA_05   0x4E   // R
#define MPU9150_EXT_SENS_DATA_06   0x4F   // R
#define MPU9150_EXT_SENS_DATA_07   0x50   // R
#define MPU9150_EXT_SENS_DATA_08   0x51   // R
#define MPU9150_EXT_SENS_DATA_09   0x52   // R
#define MPU9150_EXT_SENS_DATA_10   0x53   // R
#define MPU9150_EXT_SENS_DATA_11   0x54   // R
#define MPU9150_EXT_SENS_DATA_12   0x55   // R
#define MPU9150_EXT_SENS_DATA_13   0x56   // R
#define MPU9150_EXT_SENS_DATA_14   0x57   // R
#define MPU9150_EXT_SENS_DATA_15   0x58   // R
#define MPU9150_EXT_SENS_DATA_16   0x59   // R
#define MPU9150_EXT_SENS_DATA_17   0x5A   // R
#define MPU9150_EXT_SENS_DATA_18   0x5B   // R
#define MPU9150_EXT_SENS_DATA_19   0x5C   // R
#define MPU9150_EXT_SENS_DATA_20   0x5D   // R
#define MPU9150_EXT_SENS_DATA_21   0x5E   // R
#define MPU9150_EXT_SENS_DATA_22   0x5F   // R
#define MPU9150_EXT_SENS_DATA_23   0x60   // R
#define MPU9150_MOT_DETECT_STATUS  0x61   // R
#define MPU9150_I2C_SLV0_DO        0x63   // R/W
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_SLV2_DO        0x65   // R/W
#define MPU9150_I2C_SLV3_DO        0x66   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9150_MOT_DETECT_CTRL    0x68   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_FIFO_COUNTH        0x72   // R/W
#define MPU9150_FIFO_COUNTL        0x73   // R/W
#define MPU9150_FIFO_R_W           0x74   // R/W
#define MPU9150_WHO_AM_I           0x75   // R

//MPU9150 Compass
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R


int MPU9150_I2C_ADDRESS = 0x68;
int16_t zeros[9] = { -4000, -4000, -5000, -1582, 1760, -2115, 0, 0, 0};

/*
  int gyroXzero=-1583;
  int gyroYzero=1764;
  int gyroZzero=2111;
  int accelXzero=-6929;
  int accelYzero=-7902;
  int accelZzero=-18935;
*/

//Clock and cycletime vars
long cycleTime = 0;
long currentTime;
long oldTime;
//Clock and cycletime vars

int Acceltodeg = 2048;
int Gyrotodegs = 32.8;
float biasVal = .95;
int maxAngle = 180;

int num_motors = 6;
int motor_command[num_motors]=0; //Int Vals for motor command
float motor_com_float[num_motors]=0; //Float vals for motor command
Servo motor[num_motors];

float KP = 1;
float KI = 1;
float KD = .6;
float KY = 1.25;

int MAX_RATE = 1700;
float angle_pitchA = 0;
float angle_rollA = 0;
float angle_pitchG = 0;
float angle_rollG = 0;
float angle_pitch = 0;
float angle_roll = 0;
float rate_yawG=0;
//unsigned int cycletime;
int rate_rollG = 0;
int pitch_error = 0;
int roll_error = 0;
int angle_pitchGRate = 0;
int16_t imudata[9];
int angles[3];
float pitch_error_integral=0;
float roll_error_integral=0;
float INT_SAT=100;
byte pitch_command = 127;
byte roll_command = 127;
byte yaw_command = 127;



int rate_pitch = 0;
int rate_roll = 0;
int rate_yaw = 0;
int throttle_command_int = 0;
byte throttle_command = 90;

int throttle_u_bound=1800;  //Throttle Upper Bound (microseconds)
int throttle_l_bound=1200;  //Throttle Lower Bound (microseconds)



void setup()

{
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  Wire.begin();

  // Clear the 'sleep' bit to start the sensor.
  MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);

  // MPU9150_setupCompass();
  motor[0].attach(10);
  motor[1].attach(11);
  motor[2].attach(12);
  motor[3].attach(13);
  motor[4].attach(8);
  motor[5].attach(9);
  delay(5000)
  ACCELGYROSETUP();
  motorstart();

}

void loop()
{


  //motor1.writeMicroseconds(2000);
  zero_motors() //sets motor commands to zero should be overwritten if code is running correctly

  //This line takes requests information from the IMU and sums it with the zeroing values stored in zero[] format is 16 bit signed int.
  imudata[0] = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H) + zeros[0];
  imudata[1] = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H) + zeros[1];
  imudata[2] = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H) + zeros[2];
  imudata[3] = MPU9150_readSensor(MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H) + zeros[3];
  imudata[4] = MPU9150_readSensor(MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H) + zeros[4];
  imudata[5] = MPU9150_readSensor(MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H) + zeros[5];
  imudata[6] = MPU9150_readSensor(MPU9150_CMPS_XOUT_L, MPU9150_CMPS_XOUT_H) + zeros[6];
  imudata[7] = MPU9150_readSensor(MPU9150_CMPS_YOUT_L, MPU9150_CMPS_YOUT_H) + zeros[7];
  imudata[8] = MPU9150_readSensor(MPU9150_CMPS_ZOUT_L, MPU9150_CMPS_ZOUT_H) + zeros[8];


  oldTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - oldTime;
  //orientation angles computed from accel
  rate_pitch = imudata[3] / Gyrotodegs;
  rate_roll = imudata[4] / Gyrotodegs;
  rate_yaw =  imudata[5] / Gyrotodegs;

  angle_pitchA = (atan2(imudata[1], imudata[2]) * 57.3); //atan2 gives an angle in radians from accel data 57.3 converts to deg (This outputs in an INT so no dec resolution)
  angle_rollA = -1* (atan2(imudata[0], imudata[2]) * 57.3);
  //orientation angles computed from accel

  //orientation angles computed from gyro
  angle_pitchG = angle_pitch + rate_pitch * (micros_to_seconds(cycleTime)); //old fully resolved pitch angle + what the gyro thinks * conversion to deg * cycletime
  angle_rollG = angle_roll + rate_roll * (micros_to_seconds(cycleTime));
  //orientation angles computed from gyro

  //Weighted average used to derive pitch and roll angles
  angle_pitch = angle_pitchA * (1 - biasVal) + angle_pitchG * (biasVal);
  angle_roll = angle_rollA * (1 - biasVal) + angle_rollG * (biasVal);
  //Weighted average used to derive pitch and roll angles

  pitch_error = (float) (pitch_command - 127) / 127.0 * maxAngle - angle_pitch;
  pitch_error_integral += pitch_error * (micros_to_seconds(cycleTime));

  roll_error = (float) (roll_command - 127) / 127.0 * maxAngle - angle_roll;
  roll_error_integral += roll_error * (micros_to_seconds(cycleTime));

  boundsaturation();    // prevents I spooling

  motor_con_float[0] += (-pitch_error + roll_error) * KP;  //updated 2016
  motor_con_float[0] += (-pitch_error_integral + roll_error_integral) * KI;
  motor_con_float[0] += (-rate_pitch + rate_roll) * KD;  //UPdated 2016
  motor_con_float[0] -= ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;

  motor_con_float[1] += (-pitch_error - roll_error) * KP;
  motor_con_float[1] += (-pitch_error_integral - roll_error_integral) * KI;
  motor_con_float[1] += (-rate_pitch - rate_roll) * KD;
  motor_con_float[1] += ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;

  motor_con_float[2] += (pitch_error - roll_error) * KP;
  motor_con_float[2] += (pitch_error_integral - roll_error_integral) * KI;
  motor_con_float[2] += (rate_pitch - rate_roll) * KD;
  motor_con_float[2] -= ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;

  //MOTOR 4 = FRONT LEFT
  motor_con_float[3] += (pitch_error + roll_error) * KP;
  motor_con_float[3] += (pitch_error_integral + roll_error_integral) * KI;
  motor_con_float[3] += (rate_pitch + rate_roll) * KD;
  motor_con_float[3] += ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;

  throttle_command_int = (int)throttle_command * 3.906 + 1000; //Converts throttle command from a byte to an int
  //  throttle_command_int += biasval;
  //PITCH AND ROLL THROTTLE BIAS

  for(i=0; i<= sizeof(motor_command);i++){
      motor_command[i] = (int) motor_con_float[i] + throttle_command_int;
  }

  //OVERFLOW PROTECTION FOR COMMANDS
   boundthrottles();
  //OVERFLOW PROTECTION FOR COMMANDS

   for(i=0; i<= sizeof(motor);i++){
  motor[i].writeMicroseconds(motor_command[i]);
}


void ACCELGYROSETUP()
{

  MPU9150_writeSensor(0X1C, 0XE8); //Treiggers Self Test Accel, Sets Range to 4G
  MPU9150_writeSensor(0X1B, 0XF0); //Triggers Self Test GYRO, Sets Range to 1000 DEG/SEC (1110000)
  MPU9150_writeSensor(0X1A, 0X06); //Sets Low Pass Filter to 5 Hz Bandwidth (19 mSec delay)
  //MPU9150_writeSensor(0X1A, 0X00); //Sets Low Pass Filter to OFF (0 mSec delay)

  SerialUSB.print("Accel Settings: ");
  SerialUSB.println(MPU9150_readSensor(0X1C));
  SerialUSB.print("Gyro Settings: ");
  SerialUSB.println(MPU9150_readSensor(0X1B));

  SerialUSB.print("X Axis Self Test Settings: ");
  SerialUSB.println(MPU9150_readSensor(0X0D));
  SerialUSB.print("Y Axis Self Test Settings: ");
  SerialUSB.println(MPU9150_readSensor(0X0E));
  SerialUSB.print("Z Axis Self Test Settings: ");
  SerialUSB.println(MPU9150_readSensor(0X0F));

}

void MPU9150_setupCompass() {
  MPU9150_I2C_ADDRESS = 0x0C;      //change Address to Compass

  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
  MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

  MPU9150_I2C_ADDRESS = 0x68;      //change Address to MPU

  MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
  MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
  MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
  MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
  MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
  MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
  MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
  MPU9150_writeSensor(0x64, 0x01); //overvride register
  MPU9150_writeSensor(0x67, 0x03); //set delay rate
  MPU9150_writeSensor(0x01, 0x80);

  MPU9150_writeSensor(0x34, 0x04); //set i2c slv4 delay
  MPU9150_writeSensor(0x64, 0x00); //override register
  MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
  MPU9150_writeSensor(0x64, 0x01); //override register
  MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
  MPU9150_writeSensor(0x34, 0x13); //disable slv4
}

void updateMotors()
{



}

void test_scripting(){
//This runs the quad through a set of commands which test control
    if (currentTime>=25000000 && currentTime<= 35000000)
    {
      pitch_command=110;
    }
    if (currentTime>=35000000 && currentTime<= 45000000)
    {
      pitch_command=140;
    }
    if (currentTime>=55000000)
    {
      pitch_command=127;
    }
}


int MPU9150_readSensor(int addrL, int addrH) {
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();

  return (int16_t)((H << 8) + L);
}

int MPU9150_readSensor(int addr) {
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  return Wire.read();
}

int MPU9150_writeSensor(int addr, int data) {
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}
void boundthrottles()
//This function prevents over or underspeeding of the motors
{
    for(i=0;i < sizeof(motor_command);i++){
        if (motor_command[i]>throttle_u_bound)){
            motor_command[i]=throttle_u_bound
        }
        else if (motor_command[i]<throttle_l_bound)){
            motor_command[i]=throttle_l_bound
        }
    }
}

void boundsaturation()
{
    if (pitch_error_integral >= INT_SAT)  //Bounding to prevent I spooling.
    {
      pitch_error_integral = INT_SAT;
    }
    if (pitch_error_integral <= -INT_SAT) //Bounding to prevent underruns.
    {
      pitch_error_integral = -INT_SAT;
    }

    if (roll_error_integral >= INT_SAT) {
      roll_error_integral = INT_SAT;
    }
    if (roll_error_integral <= -INT_SAT) {
      roll_error_integral = -INT_SAT;
    }



}

void zero_motors()
{
//Function sets all motors to minimum
    for(i=0; i<=sizeof(motor_command); i++){
        motor_command[i]=0;
    }
}
int micros_to_seconds(int inputmicros){

    return(inputmicros *.00001)
}

void motorstart()
//This function sets the motors to idle and low speed at startup
{
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(5000);
  motor1.writeMicroseconds(1250);
  motor2.writeMicroseconds(1250);
  motor3.writeMicroseconds(1250);
  motor4.writeMicroseconds(1250);
}
