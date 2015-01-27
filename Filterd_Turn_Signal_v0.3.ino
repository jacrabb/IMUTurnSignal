// Filtered_Turn_Signal_v0.2.ino
/*
Essentially original version:
Binary sketch size: 13520 bytes (of a 32256 byte maximum, 41.91 percent).
Estimated memory use: 752 bytes (of a  2048 byte maximum, 36.72 percent).

After moving ahrs fusionGetOrientation() function and eliminating 9DOF.h:
Binary sketch size: 13434 bytes (of a 32256 byte maximum, 41.65 percent).
Estimated memory use: 751 bytes (of a  2048 byte maximum, 36.67 percent).

After removing Sensor.h
Binary sketch size: 11946 bytes (of a 32256 byte maximum, 37.03 percent).	4.88%
Estimated memory use: 551 bytes (of a  2048 byte maximum, 26.90 percent).	9.82%

After adding autoranging back into Mag
Binary sketch size: 12258 bytes (of a 32256 byte maximum, 38.00 percent).
Estimated memory use: 551 bytes (of a  2048 byte maximum, 26.90 percent).

After F()ing a string and removing redundant gyro variables 
(also removed autoranging)
Binary sketch size: 11750 bytes (of a 32256 byte maximum, 36.43 percent).
Estimated memory use: 497 bytes (of a  2048 byte maximum, 24.27 percent).

With serial disabled
Binary sketch size:  7654 bytes (of a 32256 byte maximum, 23.73 percent).
Estimated memory use: 297 bytes (of a  2048 byte maximum, 14.50 percent).
*/
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <aac_LSM303_U.h>
#include <aac_L3GD20.h>
//#include <Adafruit_9DOF.h>
/** struct sensors_vec_s is used to return a vector in a common format. */
typedef struct {
    union {
        //float v[3];
        //struct {
        //    float x;
        //    float y;
        //    float z;
        //};
        /* Orientation sensors */
        struct {
            float roll;    /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90°<=roll<=90° */
            float pitch;   /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180°<=pitch<=180°) */
            float heading; /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359° */
        };
    };
    int8_t status;
    uint8_t reserved[3];
} sensors_vec_t;


/* Assign a unique ID to the sensors */
//Adafruit_9DOF					dof   = Adafruit_9DOF();
aac_LSM303_Accel	accel;// = aac_LSM303_Accel_Unified(30301);
aac_LSM303_Mag		mag;//   = aac_LSM303_Mag_Unified(30302);
//Adafruit_L3GD20_Unified		gyro  = Adafruit_L3GD20_Unified(30303);
aac_L3GD20			gyro;

//double gyroX, gyroY, gyroZ;

sensors_vec_t   comp_orientation;
sensors_vec_t   gyro_orientation;
  
uint32_t timer;

#define serialdebug
void setup() {
	analogReference(INTERNAL);


  pinMode(3,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(13,OUTPUT);

  #ifdef serialdebug
   Serial.begin(115200);
  #endif
    /* Initialise the sensors */
  if(!accel.begin() || !mag.begin() || !gyro.begin(gyro.L3DS20_RANGE_250DPS))
  {
  	#ifdef serialdebug
	 Serial.println(F("IMU Error"));
	#endif
    while(1){	    digitalWrite(3,!digitalRead(3));  
	    digitalWrite(9,!digitalRead(9));
	    delay(100);  
	}
  }

  
  timer = micros();

}

void loop() {
	double dt = (double)(micros() - timer) / 1000000;//1000.0; // Calculate delta time
  	timer = micros();
  gyro.read();

  //sensors_event_t accel_event;
  //sensors_event_t mag_event;
  //sensors_event_t gyro_event;
  sensors_vec_t   orientation;
//  sensors_vec_t   gyro_orientation;

  /* Read the accelerometer and magnetometer */
  //accel.getEvent(&accel_event);
  //mag.getEvent(&mag_event);
  accel.read();
  mag.read();
  //gyro.getEvent(&gyro_event);
  //gyroX = gyro.data.x;
  //gyroY = gyro.data.y;
  //gyroZ = gyro.data.z;

///////////////////////////////////////////////////////////
// 
//  what if we 'normalize' the accel data to 1g before we
//    enter the rotational matrix?
//  Whats the best way to filter out linear acceleration?
//  How much does this matter for this fusion method?
//
//  Just reject pases when accel magnitude is > 2
// 
///////////////////////////////////////////////////////////


/* Make sure the input is valid, not null, etc. */
  float const PI_F = 3.14159265F;

  /* roll: Rotation around the X-axis. -180 <= roll <= 180                                          */
  /* a positive roll angle is defined to be a clockwise rotation about the positive X-axis          */
  /*                                                                                                */
  /*                    y                                                                           */
  /*      roll = atan2(---)                                                                         */
  /*                    z                                                                           */
  /*                                                                                                */
  /* where:  y, z are returned value from accelerometer sensor                                      */
  orientation.roll = (float)atan2(accel._accelData.y, accel._accelData.z);

  /* pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         */
  /* a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         */
  /*                                                                                                */
  /*                                 -x                                                             */
  /*      pitch = atan(-------------------------------)                                             */
  /*                    y * sin(roll) + z * cos(roll)                                               */
  /*                                                                                                */
  /* where:  x, y, z are returned value from accelerometer sensor                                   */
  if (accel._accelData.y * sin(orientation.roll) + accel._accelData.z * cos(orientation.roll) == 0)
    orientation.pitch = accel._accelData.x > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    orientation.pitch = (float)atan(-accel._accelData.x / (accel._accelData.y * sin(orientation.roll) + \
                                                                     accel._accelData.z * cos(orientation.roll)));

  /* heading: Rotation around the Z-axis. -180 <= roll <= 180                                       */
  /* a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       */
  /*                                                                                                */
  /*                                       z * sin(roll) - y * cos(roll)                            */
  /*   heading = atan2(--------------------------------------------------------------------------)  */
  /*                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   */
  /*                                                                                                */
  /* where:  x, y, z are returned value from magnetometer sensor                                    */
  orientation.heading = (float)atan2(mag._magData.z * sin(orientation.roll) - mag._magData.y * cos(orientation.roll), \
                                      mag._magData.x * cos(orientation.pitch) + \
                                      mag._magData.y * sin(orientation.pitch) * sin(orientation.roll) + \
                                      mag._magData.z * sin(orientation.pitch) * cos(orientation.roll));


  /* Convert angular data to degree */
  orientation.roll = orientation.roll * 180 / PI_F;
  orientation.pitch = orientation.pitch * 180 / PI_F;
  orientation.heading = orientation.heading * 180 / PI_F;

 /* Use the new fusionGetOrientation function to merge accel/mag._magData */  
  //if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  //if (dof.accelGetOrientation(&accel_event, &orientation))
  //if(true)// (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
	//comp_orientation.pitch = 0.93 * (comp_orientation.pitch + gyro_even * dt) + 0.07 * orientation.pitch;
  // -- what if we scale (sliding averages?) the offset percentage based on how close normalized accel is to 1G
  // -- also, we may trust Z less than X & Y because of the physics of our system (road noise is primarily on Z)
  #define compA 0.99
  #define compB 0.01
  comp_orientation.roll = compA * (comp_orientation.roll + (gyro.data.x * dt)-0.006) + compB * orientation.roll;
  comp_orientation.pitch = compA * (comp_orientation.pitch + (gyro.data.y * dt)-0.006) + compB * orientation.pitch;
  comp_orientation.heading = compA * (comp_orientation.heading + (gyro.data.z * dt)+0.0428) + compB * orientation.heading;


  gyro_orientation.roll += (gyro.data.x * dt)-0.006;
  gyro_orientation.pitch += (gyro.data.y * dt)-0.006;
  gyro_orientation.heading += (gyro.data.z * dt)+0.0428;

  // -- it would be interesting to get deceleration force.
  // -- some of that vector will be gravity
  // -- we 'know' that we'll never have more than a 30% grade 
  // -- (certin lean angle which translates into a certain accel reading)
  // -- we also are interested in deceleration truely along that board axis
  // -- this is to say we don't need deceleration along a mixed orientation
  // -- this should mean we only need to offset this accel reading by the 
  // -- gravitational component.
  // -- 

#ifdef serialdebug
	/* 'orientation' should have valid .roll and .pitch fields */
	if(timer % 2 == 0)
	{
	    Serial.print(F("|| "));

	    Serial.print(orientation.roll);	    Serial.print(F(" "));
	    Serial.print(orientation.pitch);	    Serial.print(F(" "));
	    Serial.print(orientation.heading);	    Serial.print(F(" "));

	    Serial.print(comp_orientation.roll);	    Serial.print(F(" "));
	    Serial.print(comp_orientation.pitch);	    Serial.print(F(" "));
	    Serial.print(comp_orientation.heading);	    Serial.print(F(" "));

	    Serial.print(gyro_orientation.roll);	    Serial.print(F(" "));
	    Serial.print(gyro_orientation.pitch);	    Serial.print(F(" "));
	    Serial.print(gyro_orientation.heading);	    Serial.print(F(" "));

	    Serial.print((gyro.data.x * dt)-0.006);	    Serial.print(F(" "));
	    Serial.print((gyro.data.y * dt)-0.006);	    Serial.print(F(" "));
	    Serial.print((gyro.data.z * dt)+0.0428);	    Serial.print(F(" "));

	    Serial.print(accel._accelData.x);	    Serial.print(F(" "));
	    Serial.print(accel._accelData.y);	    Serial.print(F(" "));
	    Serial.print(accel._accelData.z);	    Serial.print(F(" "));

	    Serial.print(dt, DEC);

	    Serial.println(F(""));
	}
  #endif

	if(comp_orientation.roll < -10){
	    analogWrite(9,HIGH);  
	}
	else if (comp_orientation.roll > 10){
	    analogWrite(3,HIGH);  
	}else{
	    analogWrite(3,LOW);  
	    analogWrite(9,LOW);  
	}  
  //delay(100);
}




