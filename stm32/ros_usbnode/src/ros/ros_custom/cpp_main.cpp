/**
  ******************************************************************************
  * @file    cpp_main.c
  * @author  Georg Swoboda <cn@warp.at>
  * @date    21/09/2022
  * @version 1.0.0
  * @brief   ROS Node main C++ routines
  ******************************************************************************
  * Main ROS routines
  * Publish/Subscribe to Topics
  * Provide Services
  * Odometry (for DR)
  ******************************************************************************
  */


#include "board.h"
#include "main.h"

#include <cpp_main.h>
#include "panel.h"
#include "emergency.h"
#include "drivemotor.h"
#include "blademotor.h"
#include "ultrasonic_sensor.h"
#include "spiflash.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "ros/time.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int16MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

// IMU
#include "imu/wt901.h"
#include "imu/imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Temperature.h"
#include "mowgli/magnetometer.h"


// Flash Configuration Services
#include "mowgli/SetCfg.h"
#include "mowgli/GetCfg.h"
#include "mowgli/Led.h"

// Status message
#include "mowgli/status.h"
#include "mowgli/WheelTick.h"



#define MAX_MPS	  	0.5		 	// Allow maximum speed of 0.5 m/s 
#define PWM_PER_MPS 300.0		// PWM value of 300 means 1 m/s bot speed

#define TICKS_PER_M 300.0		// Motor Encoder ticks per meter

//#define WHEEL_BASE  0.325		// The distance between the center of the wheels in meters
#define WHEEL_BASE  0.285		// The distance between the center of the wheels in meters
#define WHEEL_DIAMETER 0.2 	// The diameter of the wheels in meters

#define ODOM_NBT_TIME_MS   100	
#define IMU_NBT_TIME_MS    20  
#define MOTORS_NBT_TIME_MS 20
#define STATUS_NBT_TIME_MS 250

uint8_t RxBuffer[RxBufferSize];
struct ringbuffer rb;

ros::Time last_cmd_vel(0, 0);
ros::Time last_cmd_blade(0, 0);
uint32_t last_cmd_vel_age;		// age of last velocity command
uint32_t last_cmd_blade_age;		// age of last blade command

// drive motor control
static uint8_t left_speed=0;
static uint8_t right_speed=0;
static uint8_t left_dir=0;
static uint8_t right_dir=0;

// blade motor control
static uint8_t blade_on_off=0;

static uint8_t svcCfgDataBuffer[256];

ros::NodeHandle nh;


float imu_onboard_temperature; // cached temp value, so we dont poll I2C constantly

std_msgs::Int16MultiArray buttonstate_msg;


/* ultrasonic sensors */
sensor_msgs::Range ultrasonic_left_msg;
sensor_msgs::Range ultrasonic_right_msg;

/* bumper sensors */
sensor_msgs::Range bumper_left_msg;
sensor_msgs::Range bumper_right_msg;


// IMU
// external IMU (i2c)
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField imu_mag_msg;
// onboard IMU (accelerometer and temp)
sensor_msgs::Imu imu_onboard_msg;
//sensor_msgs::Temperature imu_onboard_temp_msg;

//sensor_msgs::MagneticField imu_mag_calibration_msg;
mowgli::magnetometer imu_mag_calibration_msg;

// mowgli status message
mowgli::status status_msg;
xbot_msgs::WheelTick wheel_ticks_msg;
/*
 * PUBLISHERS
 */
ros::Publisher pubButtonState("buttonstate", &buttonstate_msg);
ros::Publisher pubStatus("mowgli/status", &status_msg);
ros::Publisher pubWheelTicks("mowgli/wheelticks", &wheel_ticks_msg);


// IMU external
ros::Publisher pubIMU("imu/data_raw", &imu_msg);
ros::Publisher pubIMUMag("imu/mag", &imu_mag_msg);
ros::Publisher pubIMUMagCalibration("imu/mag_calibration", &imu_mag_calibration_msg);

ros::Publisher pubLeftUltrasonic("ultrasonic/left", &ultrasonic_left_msg);
ros::Publisher pubRightUltrasonic("ultrasonic/right", &ultrasonic_left_msg);

ros::Publisher pubLeftBumper("bumper/left", &bumper_left_msg);
ros::Publisher pubRightBumper("bumper/right", &bumper_left_msg);

/*
 * SUBSCRIBERS
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> subCommandVelocity("cmd_vel", CommandVelocityMessageCb);

// SERVICES
void cbSetCfg(const mowgli::SetCfgRequest &req, mowgli::SetCfgResponse &res);
void cbGetCfg(const mowgli::GetCfgRequest &req, mowgli::GetCfgResponse &res);
void cbEnableMowerMotor(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void cbResetEmergency(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void cbReboot(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void cbSetLed(const mowgli::LedRequest &req, mowgli::LedResponse &res);
void cbClrLed(const mowgli::LedRequest &req, mowgli::LedResponse &res);

ros::ServiceServer<mowgli::SetCfgRequest, mowgli::SetCfgResponse> svcSetCfg("mowgli/SetCfg", cbSetCfg);
ros::ServiceServer<mowgli::GetCfgRequest, mowgli::GetCfgResponse> svcGetCfg("mowgli/GetCfg", cbGetCfg);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> svcEnableMowerMotor("mowgli/EnableMowerMotor", cbEnableMowerMotor);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> svcResetEmergency("mowgli/ResetEmergency", cbResetEmergency);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> svcReboot("mowgli/Reboot", cbReboot);
ros::ServiceServer<mowgli::LedRequest, mowgli::LedResponse> svcSetLed("mowgli/SetLed", cbSetLed);
ros::ServiceServer<mowgli::LedRequest, mowgli::LedResponse> svcClrLed("mowgli/ClrLed", cbClrLed);

/*
 * NON BLOCKING TIMERS
 */
static nbt_t ros_nbt;
static nbt_t publish_nbt;
static nbt_t motors_nbt;
static nbt_t panel_nbt;
static nbt_t imu_nbt;
static nbt_t status_nbt;

/*
 * reboot flag, if true we reboot after next publish_nbt
 */
static bool reboot_flag = false;

/*
 * receive and parse cmd_vel messages
 * actual driving (updating drivemotors) is done in the drivemotors_nbt
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg)
{
		last_cmd_vel = nh.now();

		//	debug_printf("x: %f  z: %f\r\n", msg.linear.x, msg.angular.z);

		// calculate twist speeds to add/substract 
		float left_twist_mps = -1.0 * msg.angular.z * WHEEL_BASE * 0.5;
		float right_twist_mps =  msg.angular.z * WHEEL_BASE * 0.5;
    

		// add them to the linear speed 
		float left_mps = msg.linear.x + left_twist_mps;
        float right_mps = msg.linear.x + right_twist_mps;

		// cap left motor speed to MAX_MPS
		if (left_mps > MAX_MPS)
		{
			left_mps = MAX_MPS;
		}
		else if (left_mps < -1.*MAX_MPS)
		{
			left_mps = -1.*MAX_MPS;
		}
		// cap right motor speed to MAX_MPS
		if (right_mps > MAX_MPS)
		{
			right_mps = MAX_MPS;
		}
		else if (right_mps < -1.*MAX_MPS)
		{
			right_mps = -1.*MAX_MPS;
		}

		// set directions		
		left_dir = (left_mps >= 0)?1:0;
		right_dir = (right_mps >= 0)?1:0;

		// set drivemotors PWM values
		left_speed = abs(left_mps * PWM_PER_MPS);
		right_speed = abs(right_mps * PWM_PER_MPS);		

	//	debug_printf("left_mps: %f (%c)  right_mps: %f (%c)\r\n", left_mps, left_dir?'F':'R', right_mps, right_dir?'F':'R');
}

uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len){

	ringbuffer_put(&rb,Buf,len);
	return CDC_RX_DATA_HANDLED;
}





/*
 * Update various chatters topics
 */
extern "C" void chatter_handler()
{
	  if (NBT_handler(&publish_nbt))
	  {


#ifdef IMU_ONBOARD_TEMP
		  imu_onboard_temperature = IMU_TempRaw();
	      DB_TRACE("temp : %2.2f \r\n", imu_onboard_temperature);

#else
		  imu_onboard_temperature = IMU_Onboard_ReadTemp();
#endif
/*
		  imu_onboard_temp_msg.variance = 0.5;		// 0.5°C resolution
		  imu_onboard_temp_msg.header.frame_id = base_link;
		  pubIMUOnboardTemp.publish(&imu_onboard_temp_msg);
*/

		  // reboot if set via cbReboot (mowgli/Reboot)
		  if (reboot_flag)
		  {
			nh.spinOnce();
			NVIC_SystemReset();
			// we never get here ...
		  }
	  }
}

/*
 *  Drive Motors handler
 *  Blade Motor handler
 */
extern "C" void motors_handler()
{
	if (NBT_handler(&motors_nbt))
	{
		if (Emergency_State())
		{			
			DRIVEMOTOR_SetSpeed(0,0,0,0);
			BLADEMOTOR_Set(0);
		}
		else {
			// if the last blade cmd is older than 25sec we stop the motor
			last_cmd_blade_age = nh.now().sec - last_cmd_blade.sec;
			if (last_cmd_blade_age > 25) {
				blade_on_off = 0;				
			}			
			BLADEMOTOR_Set(blade_on_off);	

			// if the last velocity cmd is older than 1sec we stop the drive motors
			last_cmd_vel_age = nh.now().sec - last_cmd_vel.sec;			
			if (last_cmd_vel_age > 1) {
				DRIVEMOTOR_SetSpeed(0, 0, left_dir, right_dir);
			}
			else {
				DRIVEMOTOR_SetSpeed(left_speed, right_speed, left_dir, right_dir);
			}

		
		}
	}
}

/*
 *  Keyboard/LED Panel handler
 */
extern "C" void panel_handler()
{
	  if (NBT_handler(&panel_nbt))
	  {			  
		PANEL_Tick();
		if (buttonupdated == 1)
		{
			debug_printf("ROS: panel_nbt() - buttonstate changed\r\n");
			buttonstate_msg.data = (int16_t*) malloc(sizeof(int16_t) * PANEL_BUTTON_BYTES);
			buttonstate_msg.data_length = PANEL_BUTTON_BYTES;
			memcpy(buttonstate_msg.data,buttonstate,sizeof(int16_t) * PANEL_BUTTON_BYTES);
			pubButtonState.publish(&buttonstate_msg);		
			free(buttonstate_msg.data);
			buttonupdated=0;
		}
	  }
}

extern "C" void ultrasonic_handler(void)
{
	ultrasonic_left_msg.header.stamp = nh.now();
	ultrasonic_left_msg.header.frame_id = "ultrasonic_left_link";
	ultrasonic_right_msg.header.stamp = nh.now();
	ultrasonic_right_msg.header.frame_id = "ultrasonic_right_link";

	ultrasonic_left_msg.radiation_type = 0;
	ultrasonic_left_msg.field_of_view  = 0.5; /* 30°*/
	ultrasonic_left_msg.min_range = 0.30;
	ultrasonic_left_msg.max_range  = 2.0;
	ultrasonic_left_msg.range = (float)(ULTRASONICSENSOR_u32GetLeftDistance())/10000;

	ultrasonic_right_msg.radiation_type = 0;
	ultrasonic_right_msg.field_of_view  = 0.5; /* 30°*/
	ultrasonic_right_msg.min_range = 0.30;
	ultrasonic_right_msg.max_range  = 2.0;
	ultrasonic_right_msg.range = (float)(ULTRASONICSENSOR_u32GetRightDistance())/10000;

	pubLeftUltrasonic.publish(&ultrasonic_left_msg);
	pubRightUltrasonic.publish(&ultrasonic_right_msg);

}

/* \fn wheelTicks_handler
* \brief Send wheelt tick to openmower by rosserial
* is called when receiving the motors unit answer (every 20ms)
*/
extern "C" void wheelTicks_handler(int8_t p_u8LeftDirection,int8_t p_u8RightDirection, uint32_t p_u16LeftTicks, uint32_t p_u16RightTicks, int16_t p_s16LeftSpeed, int16_t p_s16RightSpeed){

	wheel_ticks_msg.stamp = nh.now();
	wheel_ticks_msg.wheel_tick_factor = TICKS_PER_M;
	wheel_ticks_msg.valid_wheels = 0x0C;
	wheel_ticks_msg.wheel_direction_fl = 0;
	wheel_ticks_msg.wheel_ticks_fl = (int32_t)p_s16LeftSpeed;
	wheel_ticks_msg.wheel_direction_fr = 0;
	wheel_ticks_msg.wheel_ticks_fr = (int32_t)p_s16RightSpeed;
	wheel_ticks_msg.wheel_direction_rl = (p_u8LeftDirection == -1)? 1 : 0;
	wheel_ticks_msg.wheel_ticks_rl = p_u16LeftTicks;
	wheel_ticks_msg.wheel_direction_rr = (p_u8RightDirection == -1)? 1 : 0;;
	wheel_ticks_msg.wheel_ticks_rr = p_u16RightTicks;

	pubWheelTicks.publish(&wheel_ticks_msg);
}

extern "C" void broadcast_handler()
{
	if (NBT_handler(&imu_nbt))
	{
	////////////////////////////////////////
	// IMU Messages
	////////////////////////////////////////		
	imu_msg.header.frame_id = "imu";
	
	// No Orientation in IMU message
	imu_msg.orientation.x = 0;
	imu_msg.orientation.y = 0;
	imu_msg.orientation.z = 0;
	imu_msg.orientation.w = 0;
	imu_msg.orientation_covariance[0] = -1;

	/**********************************/
	/* Exernal Accelerometer 		  */
	/**********************************/
#ifdef IMU_ACCELERATION
	// Linear acceleration		
	IMU_ReadAccelerometer(&imu_msg.linear_acceleration.x, &imu_msg.linear_acceleration.y, &imu_msg.linear_acceleration.z);		
	IMU_AccelerometerSetCovariance(imu_msg.linear_acceleration_covariance);	
#else
	imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.z = 0;		
	imu_msg.linear_acceleration_covariance[0] = -1;
#endif
	/**********************************/
	/* Exernal Gyro					  */
	/**********************************/
#ifdef IMU_ANGULAR
	// Angular velocity
	IMU_ReadGyro(&imu_msg.angular_velocity.x, &imu_msg.angular_velocity.y, &imu_msg.angular_velocity.z);
	IMU_GyroSetCovariance(imu_msg.angular_velocity_covariance);	
#else
	imu_msg.angular_velocity.x = imu_msg.angular_velocity.y = imu_msg.angular_velocity.z = 0;		
	imu_msg.angular_velocity_covariance[0] = -1;
#endif		
	imu_msg.header.stamp = nh.now();
	pubIMU.publish(&imu_msg);

	/**********************************/
	/* Exernal Magnetometer Corrected */
	/**********************************/
	double x,y,z;	

	// Orientation (Magnetometer)
	imu_mag_msg.header.frame_id = "imu";								
	IMU_ReadMagnetometer(&x, &y, &z);
	imu_mag_msg.magnetic_field.x = x;
	imu_mag_msg.magnetic_field.y = y;
	imu_mag_msg.magnetic_field.z = z;

	// covariance is fixed for now
	imu_mag_msg.magnetic_field_covariance[0] = 1e-3;
	imu_mag_msg.magnetic_field_covariance[4] = 1e-3;
	imu_mag_msg.magnetic_field_covariance[8] = 1e-3;
	imu_mag_msg.header.stamp = nh.now();
	//pubIMUMag.publish(&imu_mag_msg);

	/******************************************/
	/* Exernal Magnetometer RAW (Calibration) */
	/******************************************/
	IMU_ReadMagnetometerRaw(&x, &y, &z);
	imu_mag_calibration_msg.x = x;
	imu_mag_calibration_msg.y = y;
	imu_mag_calibration_msg.z = z;

	imu_mag_msg.header.stamp = nh.now();
	//pubIMUMagCalibration.publish(&imu_mag_calibration_msg);

	} // if (NBT_handler(&imu_nbt))

	if (NBT_handler(&status_nbt))
	{
	////////////////////////////////////////
	// mowgli/status Message
	////////////////////////////////////////		
	status_msg.stamp = nh.now();
	status_msg.rain_detected = RAIN_Sense();
	status_msg.emergency_status = Emergency_State();
	status_msg.emergency_left_stop = HALLSTOP_Left_Sense();
	status_msg.emergency_right_stop = HALLSTOP_Right_Sense();
	status_msg.emergency_tilt_mech_triggered = Emergency_Tilt();
	status_msg.emergency_tilt_accel_triggered = Emergency_LowZAccelerometer();
	status_msg.emergency_left_wheel_lifted = Emergency_WheelLiftBlue();
	status_msg.emergency_right_wheel_lifted = Emergency_WheelLiftRed();
	status_msg.emergency_stopbutton_triggered = Emergency_StopButtonYellow() || Emergency_StopButtonWhite();
	/* not used anymore*/
	status_msg.left_encoder_ticks = DRIVEMOTOR_u32ErrorCnt;
	status_msg.right_encoder_ticks = 0;
	status_msg.v_charge = charge_voltage;
	status_msg.i_charge = charge_current;
	status_msg.v_battery = battery_voltage;
	status_msg.charge_pwm = chargecontrol_pwm_val;
	status_msg.is_charging = chargecontrol_is_charging;
	status_msg.imu_temp = imu_onboard_temperature;
	status_msg.blade_motor_ctrl_enabled = blade_on_off;	
	status_msg.drive_motor_ctrl_enabled = true; // hardcoded for now
	status_msg.blade_motor_enabled = BLADEMOTOR_bActivated;	// set by feedback from blademotor	
	status_msg.left_power = left_power;	
	status_msg.right_power = right_power; 
	status_msg.blade_power = BLADEMOTOR_u16Power;
	status_msg.blade_RPM = BLADEMOTOR_u16RPM;
	status_msg.blade_temperature = blade_temperature;
	status_msg.sw_ver_maj = MOWGLI_SW_VERSION_MAJOR;
	status_msg.sw_ver_bra = MOWGLI_SW_VERSION_BRANCH;
	status_msg.sw_ver_min = MOWGLI_SW_VERSION_MINOR;
	pubStatus.publish(&status_msg);		
	} // if (NBT_handler(&status_nbt))
}

/*
 *  callback for mowgli/GetCfg Service
 */
void cbGetCfg(const mowgli::GetCfgRequest &req, mowgli::GetCfgResponse &res) 
{	
    debug_printf("cbGetCfg:\r\n");	
	debug_printf(" name: %s\r\n", req.name);

	res.data_length = SPIFLASH_ReadCfgValue(req.name, &res.type, svcCfgDataBuffer);

	if (res.data_length > 0)
	{		
		res.data = (uint8_t*)&svcCfgDataBuffer;	
		res.status = 1;
	}
	else
	{
		res.status = 0;
	}
}

/// @brief Set Led and optionally reset all other Leds (0x40) + Chirp (0x80)
/// @param req req.led the LED number and any option flags
/// @param res 
void cbSetLed(const mowgli::LedRequest &req, mowgli::LedResponse &res)
{	
 //  debug_printf("cbSetLed:\r\n");
 //  debug_printf(" led: %d\r\n", req.led);
   uint8_t v=req.led;
   if ( (req.led & 0x40) == 0x40)	// clear all Leds
   {
		for (uint8_t i=0;i<LED_STATE_SIZE;i++)
		{
			PANEL_Set_LED(i, PANEL_LED_OFF);
		}
   }  
   if ( (req.led & 0x80) == 0x80)
   {
     do_chirp = 1;
   }

   // remove flag bits, turn on led
   v &= ~(1UL<<7);
   v &= ~(1UL<<6);
   PANEL_Set_LED(v, PANEL_LED_ON);
}

/// @brief Clear Led and optionally reset all other Leds (0x40) + Chirp (0x80)
/// @param req req.led the LED number and any option flags
/// @param res 
void cbClrLed(const mowgli::LedRequest &req, mowgli::LedResponse &res)
{	
 //  debug_printf("cbClrLed:\r\n");
 //  debug_printf(" led: %d\r\n", req.led);
   uint8_t v=req.led;
   if ( (req.led & 0x40) == 0x40)	// clear all Leds
   {
		for (uint8_t i=0;i<LED_STATE_SIZE;i++)
		{
			PANEL_Set_LED(i, PANEL_LED_OFF);
		}
   }  
   if ( (req.led & 0x80) == 0x80)
   {
     do_chirp = 1;
   }
 
   // remove flag bits, turn of led
   v &= ~(1UL<<7);
   v &= ~(1UL<<6);
   PANEL_Set_LED(v, PANEL_LED_OFF);
}


/*
 *  callback for mowgli/SetCfg Service
 */
void cbSetCfg(const mowgli::SetCfgRequest &req, mowgli::SetCfgResponse &res) {
	union {
		float f;
		uint8_t b[4];
	} float_val;

	union {
		double d;
		uint8_t b[8];
	} double_val;
	uint8_t i;

    debug_printf("cbSetCfg:\r\n");
	debug_printf(" type: %d\r\n", req.type);
	debug_printf(" len: %d\r\n", req.data_length);
	debug_printf(" name: %s\r\n", req.name);

	if (req.type == 0) // TYPE_INT32 (0)
	{
		int32_t int32_val = (req.data[0]) + (req.data[1]<<8) + (req.data[2]<<16) + (req.data[3]<<24);
		debug_printf("(int32) data: %d\r\n", int32_val);			
	}
	if (req.type == 1) // TYPE_UINT32 (1)
	{
		uint32_t uint32_val = (req.data[0]) + (req.data[1]<<8) + (req.data[2]<<16) + (req.data[3]<<24);
		debug_printf("(uint32) data: %d\r\n", uint32_val);			
	}
	if (req.type == 2) // TYPE_FLOAT (2)
	{		
		for (i=0;i<4;i++)
		{
			float_val.b[i] = req.data[i];
		}		
		debug_printf("(float) data: %f\r\n", float_val.f);					
	}
	if (req.type == 3)  // TYPE_DOUBLE (3)
	{		
		for (i=0;i<8;i++)
		{
			double_val.b[i] = req.data[i];
		}
		debug_printf("(double) data: %Lf\r\n", double_val.d);			
	}
	if (req.type == 4)	// TYPE_STRING (4)
	{	
		debug_printf("(string) data: '");				
		for (i=0;i<req.data_length;i++)
		{
			debug_printf("%c", req.data[i]);
		}
		debug_printf("'\r\n");	
	}
	if (req.type == 5)	// TYPE_BARRAY (5)
	{	
		debug_printf("(byte array) data: '");				
		for (i=0;i<req.data_length;i++)
		{
			debug_printf("0x%02x ", req.data[i]);
		}
		debug_printf("'\r\n");	
	}

	// debug print data[] array
	for (i=0;i<req.data_length;i++)
	{
		debug_printf(" data[%d]: %d\r\n", i, req.data[i]);	
	}		

	SPIFLASH_WriteCfgValue(req.name, req.type, req.data_length, req.data);
	res.status = 1;
}

/*
 *  callback for mowgli/EnableMowerMotor Service
 */
void cbEnableMowerMotor(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{	
	last_cmd_blade = nh.now();	// if the last blade cmd is older than 25sec the motor will be stopped !
	debug_printf("ROS: cbEnableMowerMotor(from %d to %d)", blade_on_off, req.data);	
	blade_on_off = req.data;	
    if (req.data) {        		
        res.success = true;
        res.message = "Blade Motor ON";
    }
    else {
        res.success = true;
        res.message = "Blade Motor OFF";
    }    
	debug_printf("[DONE]\r\n");
}


/*
 * callback for reset emergency state by remote 
 */
void cbResetEmergency(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if(req.data){
		Emergency_ResetState();
		res.success = true;
		res.message = "Emergency reset";
	}
	else{
		res.success = false;
		res.message = "Set to true to reset";
	}
}

/*
 *  callback for mowgli/Reboot Service
 */
void cbReboot(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	//debug_printf("cbReboot:\r\n");
	reboot_flag = true;	
}

/*
 * ROS housekeeping
 */
extern "C" void spinOnce()
{
	  if (NBT_handler(&ros_nbt))
	  {
		nh.spinOnce();

		bumper_left_msg.header.stamp = nh.now();
		bumper_left_msg.header.frame_id = "bumper_left_link";
		bumper_right_msg.header.stamp = nh.now();
		bumper_right_msg.header.frame_id = "bumper_right_link";

		bumper_left_msg.radiation_type = 0;
		bumper_left_msg.field_of_view  = 1.64; /* 90°*/
		bumper_left_msg.min_range = 0.0;
		bumper_left_msg.max_range  = 0.20;
		bumper_left_msg.range = HALLSTOP_Left_Sense() * 0.05;

		bumper_right_msg.radiation_type = 0;
		bumper_right_msg.field_of_view  = 1.64; /* 90°*/
		bumper_right_msg.min_range = 0.0;
		bumper_right_msg.max_range  = 0.20;
		bumper_right_msg.range = HALLSTOP_Right_Sense() * 0.05;

		pubLeftBumper.publish(&bumper_left_msg);
		pubRightBumper.publish(&bumper_right_msg);
	  }
}

/* 
 *  Initialize rosserial
 */
extern "C" void init_ROS()
{
	ringbuffer_init(&rb, RxBuffer, RxBufferSize);

	// Initialize ROS
	nh.initNode();
	/*set max time to 10ms to give some time to the other functions*/
	//nh.setSpinTimeout(10);



	nh.advertise(pubLeftUltrasonic);
	nh.advertise(pubRightUltrasonic);

	nh.advertise(pubLeftBumper);
	nh.advertise(pubRightBumper);

	nh.advertise(pubButtonState);
	nh.advertise(pubIMU);
	nh.advertise(pubStatus);
	nh.advertise(pubWheelTicks);
	
	// Initialize Subscribers
	nh.subscribe(subCommandVelocity);

	// Initialize Services	
	nh.advertiseService(svcSetCfg);	  
	nh.advertiseService(svcGetCfg);	  
    nh.advertiseService(svcEnableMowerMotor);
	nh.advertiseService(svcResetEmergency);
	nh.advertiseService(svcReboot);
    nh.advertiseService(svcSetLed);
	nh.advertiseService(svcClrLed);
	
	// Initialize Timers
	NBT_init(&publish_nbt, 1000);
	NBT_init(&panel_nbt, 100);	
	NBT_init(&status_nbt, STATUS_NBT_TIME_MS);
	NBT_init(&imu_nbt, IMU_NBT_TIME_MS);
	NBT_init(&motors_nbt, MOTORS_NBT_TIME_MS);
	NBT_init(&ros_nbt, 10);	
	
}