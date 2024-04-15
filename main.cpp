/* ICM20948 Basic Example Code
 by: Kris Winer
 modified by Eric Nativel MBEB_OS6 port 
 date: 29, MArch 2021
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016
 Demonstrate basic ICM20948 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Pimoroni icm20948 and stm32L432kc nucleo board
 */

#include "mbed.h"
#include "ahrs.h"
#include "icm20948.h"
#include <cstdio>
#include <stdint.h>
#include "uLCD.hpp"
#include <iostream>

Mutex lcdMutex;
using namespace std::chrono;
Timer t1;
typedef unsigned char byte;
float selft[6];
static BufferedSerial pc(USBTX, USBRX);
uLCD uLCD(p9,p10,p11,uLCD::BAUD_1500000); // serial tx, serial rx, reset pin;;
BusOut urdl(p24,p23,p22,p21);
PwmOut speaker(p26);
char msg[255];
class Nav_Switch
{
public:
    Nav_Switch(PinName up,PinName down,PinName left,PinName right,PinName fire);
    int read();
//boolean functions to test each switch
    bool up();
    bool down();
    bool left();
    bool right();
    bool fire();
//automatic read on RHS
    operator int ();
//index to any switch array style
    bool operator[](int index) {
        return _pins[index];
    };
private:
    BusIn _pins;

};
Nav_Switch::Nav_Switch (PinName up,PinName down,PinName left,PinName right,PinName fire):
    _pins(up, down, left, right, fire)
{
    _pins.mode(PullUp); //needed if pullups not on board or a bare nav switch is used - delete otherwise
    thread_sleep_for(1); //delays just a bit for pullups to pull inputs high
}
inline bool Nav_Switch::up()
{
    return !(_pins[0]);
}
inline bool Nav_Switch::down()
{
    return !(_pins[1]);
}
inline bool Nav_Switch::left()
{
    return !(_pins[2]);
}
inline bool Nav_Switch::right()
{
    return !(_pins[3]);
}
inline bool Nav_Switch::fire()
{
    return !(_pins[4]);
}
inline int Nav_Switch::read()
{
    return _pins.read();
}
inline Nav_Switch::operator int ()
{
    return _pins.read();
}
void lightConvert(int direc) {
    if (direc == 3) // down
    {
        urdl = 0b0100;
        speaker.period(1.0/250.0);
        speaker = 0.5;
        thread_sleep_for(40);
        speaker = 0;
    }
    if (direc == 1) // up
    {
        urdl = 0b0001;
        speaker.period(1.0/1000.0);
        speaker = 0.5;
        thread_sleep_for(40);
        speaker = 0;
    }
    if (direc == 2) // right
    {
        urdl = 0b0010;
        speaker.period(1.0/500.0);
        speaker = 0.5;
        thread_sleep_for(40);
        speaker = 0;
    }
    if (direc == 4) // left
    {
        urdl = 0b1000;
        speaker.period(1.0/750.0);
        speaker = 0.5;
        thread_sleep_for(40);
        speaker = 0;
    }
    if (direc == 0)
    {
        urdl = 0b0000;
    }
}
Nav_Switch myNav(p17, p19, p18, p20, p16);
void setup()
{
     //Set up I2C
    
    pc.set_baud(9600);
    pc.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
  // Reset ICM20948
  begin();
 
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAGS);
  thread_sleep_for(100);
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  thread_sleep_for(100);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(ICM20948_ADDRESS, WHO_AM_I_ICM20948);
  sprintf(msg,"ICM20948 I AM 0x %x I should be 0x %x",c,0xEA);
  pc.write(msg, strlen(msg));
  if (c == 0xEA) // WHO_AM_I should always be 0x71
  {
    sprintf(msg,"ICM20948 is online...\n");
    pc.write(msg, strlen(msg));
   // writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
    // Start by performing self test and reporting values
    ICM20948SelfTest(selft);
    sprintf(msg,"x-axis self test: acceleration trim within : %f of factory value\n",selft[0]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"y-axis self test: acceleration trim within : %f of factory value\n",selft[1]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"z-axis self test: acceleration trim within : %f  of factory value\n",selft[2]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"x-axis self test: gyration trim within : %f of factory value\n",selft[3]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"y-axis self test: gyration trim within : %f of factory value\n",selft[4]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"z-axis self test: gyration trim within : %f of factory value\n",selft[5]);
    pc.write(msg, strlen(msg));
        // Calibrate gyro and accelerometers, load biases in bias registers
    calibrateICM20948(gyroBias, accelBias);

    initICM20948();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    sprintf(msg,"ICM20948 initialized for active data mode....\n");
    pc.write(msg, strlen(msg));
        // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    tempCount =readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
    temperature = ((float) tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
    sprintf(msg,"Temperature is %f degrees C\n",temperature);
    pc.write(msg, strlen(msg));
    byte d = readByte(AK09916_ADDRESS<<1, WHO_AM_I_AK09916);
    sprintf(msg,"AK8963 I AM 0x %x  I should be 0x %d\n",d,0x09);
    pc.write(msg, strlen(msg));

    // if (d != 0x09)
    // {
    //   // Communication failed, stop here
    // sprintf(msg,"Communication with magnetometer failed, abort!\n");
    // pc.write(msg, strlen(msg));
    // exit(0);
    // }

    // Get magnetometer calibration from AK8963 ROM
    //initAK09916();
    // Initialize device for active mode read of magnetometer
    //sprintf(msg,"AK09916 initialized for active data mode....\n");
    //pc.write(msg, strlen(msg));
   
  

    // Get sensor resolutions, only need to do this once
    getAres();
    getGres();
    //getMres();
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
//     magCalICM20948(magBias, magScale);
//     sprintf(msg,"AK09916 mag biases (mG)\n %f\n%f\n%f\n",magBias[0],magBias[1],magBias[2]);
//     pc.write(msg, strlen(msg));
//    sprintf(msg,"AK09916 mag scale (mG)\n %f\n%f\n%f\n",magScale[0],magScale[1],magScale[2]);
//     pc.write(msg, strlen(msg));
//     thread_sleep_for(2000); // Add delay to see results before pc spew of data
  } // if (c == 0x71)
  else
  {
    sprintf(msg,"Could not connect to ICM20948: 0x%x",c);
    pc.write(msg, strlen(msg));
    // Communication failed, stop here
    sprintf(msg," Communication failed, abort!\n");
    pc.write(msg, strlen(msg));
    exit(0);
  }
 }

void sadSong()
{
     speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1567.598);
    speaker = 0.5;
    thread_sleep_for(160);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1567.598);
    speaker = 0.5;
    thread_sleep_for(160);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1567.598);
    speaker = 0.5;
    thread_sleep_for(160);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1318.51);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1174.66);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/1046.50);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/987.77);
    speaker = 0.5;
    thread_sleep_for(160);
    speaker = 0;
    thread_sleep_for(10);

    speaker.period(1.0/783.99);
    speaker = 0.5;
    thread_sleep_for(80);
    speaker = 0;
    thread_sleep_for(10);
}

void megalovania()
{
    speaker.period(1.0/146.83);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/146.83);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/293.66);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/220.0);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/207.65);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/146.83);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/130.81);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/130.81);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/293.66);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/220.0);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/207.65);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/146.83);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/123.47);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/123.47);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/293.66);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/220.0);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/207.65);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/146.83);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/116.54);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/116.54);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/293.66);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/220.0);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/207.65);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/0);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(25.0);
speaker.period(1.0/146.83);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/174.61);
speaker = 0.5;
thread_sleep_for(13);
speaker.period(1.0/196.0);
speaker = 0.5;
thread_sleep_for(13);
speaker = 0;
}

void drawArrow(char direction)
{
    if(direction == 'd')
    {
        uLCD.drawCircleFilled(64, 64, 50, 0xFFFF);
        uLCD.drawTriangle(64, 64+33, 64-33, 64-17, 64+30, 64-17, 0x0400); //down
    }

    if(direction == 'u')
    {
        uLCD.drawCircleFilled(64, 64, 50, 0xFFFF);
        uLCD.drawTriangle(64, 64-33, 64-30, 64+17, 64+30, 64+17, 0x0400); //up
    }

    if(direction == 'r')
    {
    uLCD.drawCircleFilled(64, 64, 50, 0xFFFF);
    uLCD.drawTriangle(64-17, 64+30, 64-17, 64-30, 64+33, 64, 0x0400); //right
    }
    if (direction == 'l')
    {
        uLCD.drawCircleFilled(64, 64, 50, 0xFFFF);
        uLCD.drawTriangle(64+17, 64+30, 64+17, 64-30, 64-33, 64, 0x0400); //left
    }

}
int main(void)
{
    srand(time(NULL));
    uLCD.setFontSize(2,2);
    uLCD.locate(2,2);
    uLCD.printf("Simon\nSays");
    uLCD.setFontSize(1,1);
    uLCD.locate(4,10);
    uLCD.printf("Loading...");

    setup();
    int rs1[10] = {3, 2, 1, 1, 4, 2, 1, 3, 3, 4};
    int rs2[10] = {2, 4, 3, 2, 2, 1, 3, 2, 1, 4};
    int rs3[10] = {4, 3, 3, 2, 3, 1, 4, 2, 3, 4};
    int rs4[10] = {3, 2, 4, 1, 4, 2, 3, 3, 3, 1};

    megalovania();
    uLCD.setFontSize(1,1);
    uLCD.cls();

int sequence[10];
int counter = 0;
bool Lose = 0;
int inputcount = 0;
bool neutral = 0;
bool start = 0;

while(true)
{
counter = 0;
Lose = 0;
inputcount = 0;
neutral = 0;
start = 0;

uLCD.printf("Select a level\n");
uLCD.printf("level1\n");
uLCD.printf("level2\n");
uLCD.printf("level3\n");
uLCD.printf("level4\n");
while (!start)
{
    if (myNav.left()) {
        if (neutral) {
            if (counter > 0) {
                counter--;
            }
            neutral = 0;
        }
    }
    else if (myNav.right()) {
        if (neutral) {
            if (counter < 3) {
                counter++;
            }
            neutral = 0;
        }
    }
    else {
        neutral = 1;
    }
    if (counter == 0) {
        for (int i = 0; i < 10; i++) {
            sequence[i] = rs1[i];
        }
    }
    if (counter == 1) {
        for (int i = 0; i < 10; i++) {
            sequence[i] = rs2[i];
        }
    }
    if (counter == 2) {
        for (int i = 0; i < 10; i++) {
            sequence[i] = rs3[i];
        }
    }
    if (counter == 3) {
        for (int i = 0; i < 10; i++) {
            sequence[i] = rs4[i];
        }
    }
    uLCD.drawRectangleFilled(50, 9, 130, 100, 0x0000);
    uLCD.drawCircle(55, (8*(counter+1))+4, 3, 0xFFFF);
    if (myNav.fire()) {
        start = 1;
        uLCD.cls();
    }
}

counter = 0;
neutral = 0;

uLCD.setFontSize(3,3);
uLCD.printf("3");
speaker.period(1.0/100.0);
speaker = 0.5;
thread_sleep_for(30);
speaker = 0;
thread_sleep_for(20);

uLCD.printf("\n2");
speaker.period(1.0/100.0);
speaker = 0.5;
thread_sleep_for(30);
speaker = 0;
thread_sleep_for(20);

uLCD.printf("\n1");
speaker.period(1.0/100.0);
speaker = 0.5;
thread_sleep_for(30);
speaker = 0;
thread_sleep_for(20);

uLCD.printf("\nStart");
speaker.period(1.0/200.0);
speaker = 0.5;
thread_sleep_for(30);

speaker = 0;
thread_sleep_for(50);
uLCD.cls();
thread_sleep_for(50);


while (!Lose) {
counter++;

for(int i = 0; i < counter; i++)
    {
        lightConvert(sequence[i]);
        thread_sleep_for(20);
        lightConvert(0);
        thread_sleep_for(10);
    }
    
inputcount = 0;

while(!Lose && inputcount < counter) {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (readByte(ICM20948_ADDRESS, INT_STATUS_1) & 0x01)
{
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    ax = (float)accelCount[0] * aRes; // - accelBias[0];
    ay = (float)accelCount[1] * aRes; // - accelBias[1];
    az = (float)accelCount[2] * aRes; // - accelBias[2];
    sprintf(msg,"X-acceleration: %f mg\n",1000*ax);
    pc.write(msg, strlen(msg));
    sprintf(msg,"Y-acceleration: %f mg\n",1000*ay);
    pc.write(msg, strlen(msg));
    sprintf(msg,"Z-acceleration: %f mg\n",1000*az);
    pc.write(msg, strlen(msg));
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    gx = (float)gyroCount[0] * gRes;
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;
sprintf(msg,"x -gyroscope: %f and bias %f deg/s\n",gx,gyroBias[0]);
        pc.write(msg, strlen(msg));
   //readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    // mx = (float)magCount[0] * mRes - magBias[0];
    // my = (float)magCount[1] * mRes - magBias[1];
    // mz = (float)magCount[2] * mRes - magBias[2];
   // if (readByte(ICM20948_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // ICM20948, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(ax, ay, az, gx * DEG_TO_RAD,
                         gy * DEG_TO_RAD, gz * DEG_TO_RAD, my,
                         mx, mz, deltat);

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
//       yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
//                     * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
//                     * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
//                     * *(getQ()+3));
//       pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
//                     * *(getQ()+2)));
//       roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
//                     * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
//                     * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
//                     * *(getQ()+3));
//       pitch *= RAD_TO_DEG;
//       yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      //    1° 46' E 2021-03-27
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
    //   yaw  -= 1.7666;
    //   roll *= RAD_TO_DEG;

     
        // sprintf(msg,"Yaw %f, Pitch %f, Roll %f\n ",yaw,pitch,roll);
        pc.write(msg, strlen(msg));
    sumCount = 0;
    sum = 0;
    

    if (ax >= 0.8 && neutral)
     {
         drawArrow('d');
         if (sequence[inputcount] != 3) {
             Lose = 1;
         }
         else {
            inputcount++;
            speaker.period(1.0/250.0);
            speaker = 0.5;
            thread_sleep_for(40);
            speaker = 0;
            thread_sleep_for(60);
         }
         neutral = 0;
         //urdl = 0b0100;
     }
     if (ax <= -0.8)
     {
         drawArrow('u');
         if (sequence[inputcount] != 1) {
             Lose = 1;
         }
         else {
            inputcount++;
            speaker.period(1.0/1000.0);
            speaker = 0.5;
            thread_sleep_for(40);
            speaker = 0;
            thread_sleep_for(60);
         }
         neutral = 0;
         //urdl = 0b0001;
     }
     if (ay >= 0.8)
     {
        drawArrow('r');
        if (sequence[inputcount] != 2) {
             Lose = 1;
        }
        else {
        inputcount++;
        speaker.period(1.0/500.0);
        speaker = 0.5;
        thread_sleep_for(40);
        speaker = 0;
        thread_sleep_for(60);
        }
        neutral = 0;
        // urdl = 0b0010;
     }
     if (ay <= -0.8)
     {
         drawArrow('l');
         if (sequence[inputcount] != 4) {
             Lose = 1;
         }
         else {
            inputcount++;
            speaker.period(1.0/750.0);
            speaker = 0.5;
            thread_sleep_for(40);
            speaker = 0;
            thread_sleep_for(60);
         }
         neutral = 0;
         //urdl = 0b1000;
     }
     if (ax < 0.3 && ax > -0.3 && ay < 0.3 && ay > -0.3) {
         neutral = 1;
     }
     thread_sleep_for(20);
     uLCD.cls();
}
}
if (Lose) {
    uLCD.setFontSize(4,4);
    uLCD.printf("You\nLost!");
    sadSong();
    thread_sleep_for(1000); 
}
if (inputcount == 10){
{
    uLCD.setFontSize(4,4);
    uLCD.printf("You\nWin!");
    Lose = 1;
    thread_sleep_for(500);
    uLCD.cls();
    uLCD.setFontSize(1,1);
}
}
thread_sleep_for(100);
uLCD.cls();
}
}
return 0;
}