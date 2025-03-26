/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//SBUS MESSAGE DEFINES
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

//ACTUATOR DEFINES
#define true 1
#define false 0

#define THROTTLE_ACTUATOR 1
#define BRAKE_ACTUATOR 2
#define GEAR_ACTUATOR 3

#define INPUT_THROTTLE_MAX 1712 //1712
#define INPUT_THROTTLE_MIN 992 //272
#define INPUT_THROTTLE_MAX_REV 272 //1712
#define INPUT_THROTTLE_MIN_REV 992 //272


#define FEEDBACK_THROTTLE_HIGHRPM_MAX 255 //260 //520
#define FEEDBACK_THROTTLE_MEDRPM_MAX 240
#define FEEDBACK_THROTTLE_LOWRPM_MAX 225 //240
#define FEEDBACK_THROTTLE_MIN  140 //120 //0 //165

#define FEEDBACK_BRAKE_MAX 1508//1508
#define FEEDBACK_BRAKE_MIN 0
#define INPUT_BRAKE_MIN 1050
#define INPUT_BRAKE_MAX 1712

#define INPUT_STEERING_MAX 1712
#define INPUT_STEERING_MIN 272
#define PWM_THRESHOLD_BRAKE 30
#define PWM_THRESHOLD_THROTTLE 30
#define ERROR_MARGIN_PWM 5
#define BRAKE_RETRACTED 1
#define BRAKE_EXTENDED 0

//ERROR CODES
#define HARD_RESET 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

//Test Variables
uint8_t can_ERROR=0;

//RC_Variables
int throttle_joystick=0, gear_knob = 0, brake=0, steering=0, mode_rpm=0, mode_sw=0, slide=0,
		jetson_toggle=0, init_pos_finetune = 0, throttle_limit_calib = 0, arm_trigger = 0, arm_trigger_msg = 0, emergency_brake = 0,
		lock_calib_change = 0;
int prev_lock_calib_change;
int input_rpm = 0;
int init_rpm = 0;
int init_velocity = 0;
int max_velocity = 20;
int highest_rpm_low = 225, highest_rpm_med = 240, highest_rpm_high = 255;
int throttle_input = 0,throttle_input_rev=0, brake_input = 0, gear_input = 0;
bool emg_cutoff = false;
bool write_to_flash = false;
uint32_t init_rpm_flash = 0;
uint32_t highest_rpm_low_flash = 0, highest_rpm_med_flash = 0, highest_rpm_high_flash = 0;
bool flash_read_once = false;
uint32_t uartCounter=0;
uint8_t current_gear_value, mileage;
int vehicle_speed;
int vehicle_speed_other;
uint32_t hcan1_error, hcan2_error;
HAL_CAN_StateTypeDef can1_state, can2_state;

//Input from RC
uint16_t failsafe_status;
uint16_t SBUS_footer;
uint8_t buf[25];
uint16_t CH[18];

uint8_t pre_catch_buf[1];
int pre_init_counter = 0;
bool start_counting = false;
bool catch_signal = false;

//CAN Variables
CAN_RxHeaderTypeDef Rxheader;
CAN_RxHeaderTypeDef Rxheader2;
uint8_t Rxmsg[8]={0};  // CAN frame 8 bytes
uint8_t Rxmsg1[8]={0};
CAN_TxHeaderTypeDef SteeringMsg;
CAN_TxHeaderTypeDef Throttle_Brake_PC_Msg;
CAN_TxHeaderTypeDef Steering_PC_Msg;
CAN_TxHeaderTypeDef Speed_PC_Msg;
uint8_t SteeringData[8] = {0}; // Initialize data to send
uint8_t Throttle_Brake_PC_Data[8] = {0};
uint8_t Steering_PC_Data[8] = {0};
uint8_t Speed_PC_Data[8] = {0};
uint32_t TxMailbox;

//0x18F Steering Data Received
uint8_t ecuStatus =0, ecuTemperature=0, faultEnumeration=0;// 8 bits
int16_t currentAngle=0, angularVelocity=0; // 16 bits
int8_t motorCurrent=0; // 8 bits (-128A to +127A)
int gpioc_value;
int can_enter = 0;
//0x349
float accelerationx = 0, accelerationy = 0, accelerationz = 0;
//int velocity1 = 0, velocity2 = 0, velocity3 = 0;
float roll = 0, pitch = 0, yaw = 0;

//0x18F Error Messages
uint8_t wireControlMode=0,ecuFault=0,medianCalibrated=0;

//0x314 Steering Sending Data
int steeringAngle=0 ,steeringVelocity=0;
char write_data[50];
char read_data[50];

//Actuator Control
int throttle_feedback, brake_feedback, gear_feedback;
int desired_throttle_feedback = 0, desired_brake_feedback = 0, desired_gear_feedback = 0;
int throttle_duty_cycle_val = 0;
int brake_duty_cycle_val = 0;
int gear_duty_cycle_val = 0;
uint8_t dirThrottle = 0;
uint8_t dirBrake = 0;
uint8_t dirGear = 0;
int testVar = 0;
int rpm_finetune = 0;
int throttle_current_diff;
int throttle_past_diff;
int throttle_unchanged_count = 0;

//#if 1
//uint32_t sector = 0;
//static FLASH_EraseInitTypeDef EraseInitStruct;
//uint32_t SECTORError;
//int sofar=0;
//uint32_t StartSector, EndSectorAddress, EndSector, StartSectorAddress1, result;
//uint32_t result1;
//#endif
uint32_t result_temp[4];
//int dbga, dbgb, dbgc, dbgd;
uint32_t init_flash_rpm[4];
uint32_t changed_flash_rpm[4];
//int can_enter = 0;

typedef struct pidParams {
	float ki;
	float kp;
	float kd;

	float error;
	float prev_error;
	float error_sum;

	float P, I, D;
	float current_time, prev_time, ts;
	float learning_rate;

	//	float
} pidParams;
int output;
pidParams throttle_PID = {0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1};
int desired_speed = 0 ;
float h1, h2, h3;
float w1 = 0.5f, w2 = 0.3f, w3 = 0.2f;
float w4 = 0.4f, w5 = 0.3f, w6 = 0.2f;
float w7 = 0.6f, w8 = 0.4f, w9 = 0.2f;
int input_velocity = 0;
int velocity_joystick = 0, velocity_lock_value = 0, gear_switch = 0, steering_input_from_RC = 0, brake_input_from_RC = 0;

typedef enum{
	NEUTRAL,
	REVERSE,
	LOW
} gear_position;

typedef enum{
	THROTTLE,
	BRAKE,
	GEAR
} linear_actuator;

typedef enum{
	CAN1_NOTIF_ERROR,
	CAN2_NOTIF_ERROR,
	CAN1_START_ERROR,
	CAN2_START_ERROR
} error_codes;

//Lookup Table
int throttle_lookup_table[633] = {180, 180, 180, 180, 180, 181, 181, 181, 181, 181, 181, 181, 181, 181, 182, 182, 182, 182, 182, 182, 182, 182, 182,
		183, 183, 183, 183, 183, 183, 183, 183, 183, 184, 184, 184, 184, 184, 184, 184, 184, 184, 185, 185, 185, 185, 185, 185, 185, 185,
		185, 186, 186, 186, 186, 186, 186, 186, 186, 186, 187, 187, 187, 187, 187, 187, 187, 187, 187, 188, 188, 188, 188, 188, 188, 188, 188,
		188, 189, 189, 189, 189, 189, 189, 189, 189, 189, 190, 190, 190, 190, 190, 190, 190, 190, 190, 191, 191, 191, 191, 191, 191, 191, 191,
		191, 192, 192, 192, 192, 192, 192, 192, 192, 192, 193, 193, 193, 193, 193, 193, 193, 193, 193, 194, 194, 194, 194, 194, 194, 194, 194,
		194, 195, 195, 195, 195, 195, 195, 195, 195, 195, 196, 196, 196, 196, 196, 196, 196, 196, 196, 197, 197, 197, 197, 197, 197, 197, 197,
		197, 198, 198, 198, 198, 198, 198, 198, 198, 198, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 200, 200, 200, 200, 200, 200, 200,
		200, 200, 201, 201, 201, 201, 201, 201, 201, 201, 201, 202, 202, 202, 202, 202, 202, 202, 202, 202, 203, 203, 203, 203, 203, 203, 203,
		203, 203, 204, 204, 204, 204, 204, 204, 204, 204, 204, 205, 205, 205, 205, 205, 205, 205, 205, 205, 206, 206, 206, 206, 206, 206, 206,
		206, 206, 207, 207, 207, 207, 207, 207, 207, 207, 207, 208, 208, 208, 208, 208, 208, 208, 208, 208, 209, 209, 209, 209, 209, 209, 209,
		209, 209, 210, 210, 210, 210, 210, 210, 210, 210, 210, 211, 211, 211, 211, 211, 211, 211, 211, 211, 212, 212, 212, 212, 212, 212, 212,
		212, 212, 213, 213, 213, 213, 213, 213, 213, 213, 213, 214, 214, 214, 214, 214, 214, 214, 214, 214, 215, 215, 215, 215, 215, 215, 215,
		215, 215, 216, 216, 216, 216, 216, 216, 216, 216, 216, 217, 217, 217, 217, 217, 217, 217, 217, 217, 218, 218, 218, 218, 218, 218, 218,
		218, 218, 219, 219, 219, 219, 219, 219, 219, 219, 219, 220, 220, 220, 220, 220, 220, 220, 220, 220, 221, 221, 221, 221, 221, 221, 221,
		221, 221, 222, 222, 222, 222, 222, 222, 222, 222, 222, 223, 223, 223, 223, 223, 223, 223, 223, 223, 224, 224, 224, 224, 224, 224, 224,
		224, 224, 225, 225, 225, 225, 225, 225, 225, 225, 225, 226, 226, 226, 226, 226, 226, 226, 226, 226, 227, 227, 227, 227, 227, 227, 227,
		227, 227, 228, 228, 228, 228, 228, 228, 228, 228, 228, 229, 229, 229, 229, 229, 229, 229, 229, 229, 230, 230, 230, 230, 230, 230, 230,
		230, 230, 231, 231, 231, 231, 231, 231, 231, 231, 231, 232, 232, 232, 232, 232, 232, 232, 232, 232, 232, 233, 233, 233, 233, 233, 233,
		233, 233, 233, 234, 234, 234, 234, 234, 234, 234, 234, 234, 235, 235, 235, 235, 235, 235, 235, 235, 235, 236, 236, 236, 236, 236, 236,
		236, 236, 236, 237, 237, 237, 237, 237, 237, 237, 237, 237, 238, 238, 238, 238, 238, 238, 238, 238, 238, 239, 239, 239, 239, 239, 239,
		239, 239, 239, 240, 240, 240, 240, 240, 240, 240, 240, 240, 241, 241, 241, 241, 241, 241, 241, 241, 241, 242, 242, 242, 242, 242, 242,
		242, 242, 242, 243, 243, 243, 243, 243, 243, 243, 243, 243, 244, 244, 244, 244, 244, 244, 244, 244, 244, 245, 245, 245, 245, 245, 245,
		245, 245, 245, 246, 246, 246, 246, 246, 246, 246, 246, 246, 247, 247, 247, 247, 247, 247, 247, 247, 247, 248, 248, 248, 248, 248, 248,
		248, 248, 248, 249, 249, 249, 249, 249, 249, 249, 249, 249, 250, 250, 250, 250, 250 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */
void SteeingsMsg_Init(void);
void PC_CANMsg_Init(void);
void CAN1_Send_SteeringData(void);
void Map_RCtoCAN(void);
void ActuatorsControl();
void TIMResetCounter2(void);
void TIMResetCounter8(void);
void TIMResetCounter4(void);
void actuatorApply(int, int);
void actuatorRelease(int, int, int);
void actuatorStop(int);
void test_CAN(void);
uint16_t map(int, int, int, int, int, int);
void RC_Data(uint8_t *);
void Rcv_RC_Data();
void CAN1_Send_ThrottleDataToPC(int, int);
void CAN1_Send_SteeringDataToPC(void);
void CAN1_Send_Speed_PC_Msg(void);
void Encoder_Freeze_Handler(int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float sigmoid(float x) {
	return 1.0f / (1.0f + expf(-x));
}

void actuatorApply(int actuator, int val) {
	switch(actuator){
#if 1
	case THROTTLE_ACTUATOR:
		desired_throttle_feedback = val;
		//Changes the PWM and DIR PIN OF THE THROTTLE ACTUATOR
		if((throttle_feedback - desired_throttle_feedback) > ERROR_MARGIN_PWM){
			dirThrottle = 1;
			if((throttle_feedback - desired_throttle_feedback) > PWM_THRESHOLD_THROTTLE){
				throttle_duty_cycle_val = 100;
				TIM3->CCR1 = throttle_duty_cycle_val;
				throttle_duty_cycle_val = 99;
				TIM3->CCR1 = throttle_duty_cycle_val;
				//				Encoder_Freeze_Handler(throttle_feedback - desired_throttle_feedback);
			}
			else if((throttle_feedback - desired_throttle_feedback) < PWM_THRESHOLD_THROTTLE){
				throttle_duty_cycle_val = ((abs(throttle_feedback - desired_throttle_feedback))/PWM_THRESHOLD_THROTTLE * 100);
				if(throttle_duty_cycle_val < 30){
					throttle_duty_cycle_val += 30;
				}
				TIM3->CCR1 = throttle_duty_cycle_val;
				//				Encoder_Freeze_Handler(throttle_feedback - desired_throttle_feedback);
			}

		}

		else if((throttle_feedback - desired_throttle_feedback) < -ERROR_MARGIN_PWM){
			dirThrottle = 0;
			if(abs(throttle_feedback - desired_throttle_feedback) > PWM_THRESHOLD_THROTTLE){
				throttle_duty_cycle_val = 100;
				TIM3->CCR1 = throttle_duty_cycle_val;
				throttle_duty_cycle_val = 99;
				TIM3->CCR1 = throttle_duty_cycle_val;
				//				Encoder_Freeze_Handler(abs(throttle_feedback - desired_throttle_feedback));
			}
			if(abs(throttle_feedback - desired_throttle_feedback) < PWM_THRESHOLD_THROTTLE){
				throttle_duty_cycle_val = ((abs(throttle_feedback - desired_throttle_feedback)));
				if(throttle_duty_cycle_val < 30){
					throttle_duty_cycle_val += 30;

				}
				TIM3->CCR1 = throttle_duty_cycle_val;
				//				Encoder_Freeze_Handler(abs(throttle_feedback - desired_throttle_feedback));
			}
			//			Encoder_Freeze_Handler(throttle_feedback - desired_throttle_feedback);
		}
		else {
			TIM3->CCR1 = 0;
		}
		HAL_GPIO_WritePin(DIR2_THROTTLE_GPIO_Port, DIR2_THROTTLE_Pin, dirThrottle);
		break;
#endif

#if 0
	case THROTTLE_ACTUATOR:
		can_rcv_cmd_vel(&throttle_PID, input_velocity - vehicle_speed);
		break;
#endif


	case BRAKE_ACTUATOR:
		desired_brake_feedback = val;
		//Changes the PWM and DIR PIN OF THE BRAKE ACTUATOR
		if((brake_feedback - desired_brake_feedback) > ERROR_MARGIN_PWM){
			dirBrake = 1;
			HAL_GPIO_WritePin(DIR1_BRAKE_GPIO_Port, DIR1_BRAKE_Pin, dirBrake);
			if((brake_feedback - desired_brake_feedback) > PWM_THRESHOLD_BRAKE){
				brake_duty_cycle_val = 95;
				TIM1->CCR1 = brake_duty_cycle_val;
			}
			if((brake_feedback - desired_brake_feedback) < PWM_THRESHOLD_BRAKE){
				brake_duty_cycle_val = ((abs(brake_feedback - desired_brake_feedback))/PWM_THRESHOLD_BRAKE * 100);
				if(brake_duty_cycle_val < 55){
					brake_duty_cycle_val += 55;
				}
				TIM1->CCR1 = brake_duty_cycle_val;
			}
		}

		else if((brake_feedback - desired_brake_feedback) < -ERROR_MARGIN_PWM){
			dirBrake = 0;
			HAL_GPIO_WritePin(DIR1_BRAKE_GPIO_Port, DIR1_BRAKE_Pin, dirBrake);
			if(abs(brake_feedback - desired_brake_feedback) > PWM_THRESHOLD_BRAKE){
				brake_duty_cycle_val = 95;
				TIM1->CCR1 = brake_duty_cycle_val;
			}
			if(abs(brake_feedback - desired_brake_feedback) < PWM_THRESHOLD_BRAKE){
				brake_duty_cycle_val = ((abs(brake_feedback - desired_brake_feedback))/PWM_THRESHOLD_BRAKE * 100);
				if(brake_duty_cycle_val < 55){
					brake_duty_cycle_val += 55;
				}
				TIM1->CCR1 = brake_duty_cycle_val;
			}
		}
		else {
			TIM1->CCR1 = 0;
		}
		break;

	case GEAR_ACTUATOR:

		desired_gear_feedback = val;
		//Changes the PWM and DIR PIN OF THE GEAR ACTUATOR
		if((gear_feedback - desired_gear_feedback) > ERROR_MARGIN_PWM){
			dirGear = 0;
			HAL_GPIO_WritePin(DIR3_GEAR_GPIO_Port, DIR3_GEAR_Pin, dirGear);
			if(abs(gear_feedback - desired_gear_feedback) > 100){
				gear_duty_cycle_val = 100;
				TIM3->CCR2 = gear_duty_cycle_val;
				gear_duty_cycle_val = 99;
				TIM3->CCR2 = gear_duty_cycle_val;
			}
			else if(abs(gear_feedback - desired_gear_feedback) < 100){
				gear_duty_cycle_val = ((abs(gear_feedback - desired_gear_feedback))/100 * 100);
				if(gear_duty_cycle_val < 20){
					gear_duty_cycle_val += 20;
				}
				TIM3->CCR2 = gear_duty_cycle_val;
			}
		}

		else if((gear_feedback - desired_gear_feedback) < -ERROR_MARGIN_PWM){
			dirGear = 1;
			HAL_GPIO_WritePin(DIR3_GEAR_GPIO_Port, DIR3_GEAR_Pin, dirGear);
			if(abs(gear_feedback - desired_gear_feedback) > 100){
				//				can_enter = 101;
				gear_duty_cycle_val = 100;
				TIM3->CCR2 = gear_duty_cycle_val;
				gear_duty_cycle_val = 99;
				TIM3->CCR2 = gear_duty_cycle_val;
			}
			else {
				//				can_enter = 90;
				gear_duty_cycle_val = ((abs(gear_feedback - desired_gear_feedback))/200 * 100);
				if(gear_duty_cycle_val < 20){
					gear_duty_cycle_val += 20;

				}
				TIM3->CCR2 = gear_duty_cycle_val;
			}
		}
		else{
			TIM3->CCR2 = 0;
		}
		break;

	}
}

void actuatorRelease(int actuator, int val, int brake_value){
	switch(actuator){
	case THROTTLE_ACTUATOR:
		HAL_GPIO_WritePin(DIR2_THROTTLE_GPIO_Port, DIR2_THROTTLE_Pin, 1);
		htim3.Instance->CCR1 = val;
		break;
	case BRAKE_ACTUATOR:
		HAL_GPIO_WritePin(DIR1_BRAKE_GPIO_Port, DIR1_BRAKE_Pin, brake_value);
		htim1.Instance->CCR1 = val;
		break;
		break;
	case GEAR_ACTUATOR:
		HAL_GPIO_WritePin(DIR3_GEAR_GPIO_Port, DIR3_GEAR_Pin, 0);
		htim3.Instance->CCR2 = val;
		break;
	}
}

void actuatorStop(int actuator){
	switch(actuator){
	case THROTTLE_ACTUATOR:
		TIM3->CCR1 = 0;
		break;
	case BRAKE_ACTUATOR:
		TIM1->CCR1 = 0;
		break;
	case GEAR_ACTUATOR:
		TIM3->CCR2 = 0;
		break;
	}
}

uint16_t map(int x, int in_min, int in_max, int out_min,
		int out_max, int invertedControl) {
	testVar = invertedControl ? (x - in_min) * (out_min - out_max) / (in_max - in_min) + out_max : (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	testVar = (testVar < out_min) ? out_min : testVar;
	return testVar;
}

void ActuatorsControl(){
	if(mode_sw == 992){
		throttle_input = map(throttle_joystick, INPUT_THROTTLE_MIN, INPUT_THROTTLE_MAX, init_rpm, input_rpm, 0);
		throttle_input_rev = map(throttle_joystick, 992, INPUT_THROTTLE_MAX_REV, init_rpm, input_rpm, 0);
		if(throttle_joystick >= INPUT_THROTTLE_MIN){
			gear_input = 230;
			actuatorApply(GEAR_ACTUATOR, gear_input);
					actuatorApply(THROTTLE_ACTUATOR, throttle_input);
				}
				if(throttle_joystick < 992) {
					gear_input = 0;
					actuatorApply(GEAR_ACTUATOR, gear_input);
					actuatorApply(THROTTLE_ACTUATOR, throttle_input_rev);
				}

#if 0
		throttle_input = throttle_lookup_table[throttle_joystick-1080];
#endif

		brake_input = map(brake_input_from_RC, 992, INPUT_BRAKE_MAX, FEEDBACK_BRAKE_MIN, FEEDBACK_BRAKE_MAX, 1);

		if (brake_input_from_RC > 1050){
			actuatorApply(BRAKE_ACTUATOR, brake_input);
		}
		if(brake_input_from_RC <= 1050){
			actuatorApply(BRAKE_ACTUATOR, FEEDBACK_BRAKE_MAX);
			brake_input = FEEDBACK_BRAKE_MAX;
		}

//		if(gear_switch == 272){
//			gear_input = 0;
//		}
//		if(gear_switch == 992){
//			gear_input = 230;
//		}
//		if(gear_switch == 1712)
//			gear_input = 2400;
//		actuatorApply(GEAR_ACTUATOR, gear_input);
	}
}
//Data received from RC
void Rcv_RC_Data(){

	//	velocity_joystick = CH[2];     //Velocity from Joystick
	throttle_joystick = CH[2]; //change mapping if joystick inputs rpm
	velocity_lock_value = CH[7];   //Lock velocity
	gear_switch = CH[5]; 				 //Changes gear position to L, N and R
	steering_input_from_RC = CH[3];	     //Steering
	brake_input_from_RC = CH[1];		//Braking
	mode_sw = 	CH[4];		      		//Changes mode - manual, autonomous,
	mode_rpm = 	CH[13];
	emergency_brake = CH[12];  //To trigger emergency brake in STM
	if (emergency_brake == 1712 ) emg_cutoff = true;  //Check value again

	if(emg_cutoff == false){
		if(mode_sw == 272){
			//Manual Mode
			actuatorRelease(BRAKE_ACTUATOR, 95,BRAKE_EXTENDED);
			actuatorRelease(THROTTLE_ACTUATOR, 95,BRAKE_EXTENDED);
			//			actuatorApply(GEAR_ACTUATOR, 730);
			throttle_unchanged_count = 0;
			TIMResetCounter2();
			TIMResetCounter4();
			TIMResetCounter8();
		}
		if(mode_sw == 992){
			switch(mode_rpm){
			case 272:
				input_rpm = highest_rpm_low;
				break;
			case 992:
				input_rpm = highest_rpm_med;
				break;
			case 1712:
				input_rpm = highest_rpm_high;
				break;
			}
			ActuatorsControl();
			Map_RCtoCAN();
		}

		CAN1_Send_SteeringDataToPC();
		CAN1_Send_ThrottleDataToPC(throttle_input, brake_input);
		CAN1_Send_Speed_PC_Msg();
	}

	else {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		actuatorRelease(BRAKE_ACTUATOR, 95,BRAKE_RETRACTED);
		actuatorRelease(THROTTLE_ACTUATOR, 95,BRAKE_RETRACTED);
		actuatorApply(GEAR_ACTUATOR, 730);
		TIMResetCounter2();
		TIMResetCounter8();
		TIMResetCounter4();
		HAL_TIM_Base_Stop_IT(&htim12);
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim8);
		HAL_TIM_Base_Stop_IT(&htim4);
		HAL_CAN_Stop(&hcan1);
		//		HAL_CAN_Stop(&hcan2);
		//Stop PWM Timers
		throttle_unchanged_count =  0;
	}
}

#if 1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (pre_catch_buf[0] == 0x0f) start_counting = true;

	if (pre_init_counter < 24 && start_counting == true)
	{
		pre_init_counter++;
		return;
	}
	else if (catch_signal == false)
	{
		catch_signal = true;
		HAL_UART_AbortReceive(&huart1);
		HAL_UART_Receive_DMA(&huart1, buf, 25);
		return;
	}

	if (buf[0] == 0x0f)
	{
		CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF; //J1
		CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF; //J2
		CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF; //J3
		CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF; //J4
		CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF; //SA
		CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF; //SE
		CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF; //SF
		CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF; //S1
		CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF; //LD2
		CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF; //LD1
		CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF; //RD1
		CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF; //SC
		CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF; //S2
		CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF; //SB
		CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF; //SD
		CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF; //RD2

		if (buf[23] & (1 << 0))
		{
			CH[16] = 1;
		}
		else
		{
			CH[16] = 0;
		}

		if (buf[23] & (1 << 1))
		{
			CH[17] = 1;
		}
		else
		{
			CH[17] = 0;
		}

		// Failsafe
		failsafe_status = SBUS_SIGNAL_OK;
		if (buf[23] & (1 << 2))
		{
			failsafe_status = SBUS_SIGNAL_LOST;
		}

		if (buf[23] & (1 << 3))
		{
			failsafe_status = SBUS_SIGNAL_FAILSAFE;
		}

		SBUS_footer = buf[24];
		Rcv_RC_Data();
	}
	else
	{
		catch_signal = false;
		start_counting = false;
		catch_signal = false;
		pre_init_counter = 0;
		HAL_UART_AbortReceive(&huart1);
		HAL_UART_Receive_DMA(&huart1, pre_catch_buf, 1);
		return;
	}
	uartCounter++;
	HAL_IWDG_Refresh(&hiwdg);
}
#endif

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {  //RC Receiver
		if (pre_catch_buf[0] == 0x0f) start_counting = true;

		if (pre_init_counter < 24 && start_counting == true)
		{
			pre_init_counter++;
			return;
		}
		else if (catch_signal == false)
		{
			catch_signal = true;
			HAL_UART_AbortReceive(&huart1);
			HAL_UART_Receive_IT(&huart1, buf, 25);
			return;
		}

		if (buf[0] == 0x0f)
		{
			CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF;
			CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF;
			CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF;
			CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF;
			CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF;
			CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF;
			CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF;
			CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF;
			CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF;
			CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF;
			CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF;
			CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF;
			CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF;
			CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF;
			CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF;
			CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF;

			if (buf[23] & (1 << 0))
			{
				CH[16] = 1;
			}
			else
			{
				CH[16] = 0;
			}

			if (buf[23] & (1 << 1))
			{
				CH[17] = 1;
			}
			else
			{
				CH[17] = 0;
			}

			// Failsafe
			failsafe_status = SBUS_SIGNAL_OK;
			if (buf[23] & (1 << 2))
			{
				failsafe_status = SBUS_SIGNAL_LOST;
			}

			if (buf[23] & (1 << 3))
			{
				failsafe_status = SBUS_SIGNAL_FAILSAFE;
			}

			SBUS_footer = buf[24];
			Rcv_RC_Data();
			HAL_UART_Receive_IT(&huart1, buf, 25);
		}
		else
		{
			catch_signal = false;
			start_counting = false;
			catch_signal = false;
			pre_init_counter = 0;
			HAL_UART_AbortReceive(&huart1);
			HAL_UART_Receive_IT(&huart1, pre_catch_buf, 1);
			return;
		}
		uartCounter++;
	}
	HAL_IWDG_Refresh(&hiwdg);
}
#endif


//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//	//can_ERROR = 90;
//	if(hcan == &hcan1 ){
//		//		printf("CAN Callback Triggered");
//		//		can_enter = 20;
//		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxheader, Rxmsg) != HAL_OK) {
//			// Reception Error
//			//			Error_Handler();
//			can_ERROR = 4;
//		}
//
//		switch(Rxheader.ExtId)
//		{
//		//		case 0x18F0C8F6h:
//		//
//		//			break;
//		//		case 0x18F0C1F6h:
//		//
//		//			break;
//		case 0x18F005F6:
//			if(Rxmsg[3] == 0x4C)
//				current_gear_value = 1; //L gear
//			else if(Rxmsg[3] == 0x48)
//				current_gear_value = 2; //H gear
//			else if(Rxmsg[3] == 0x4E)
//				current_gear_value = 3; //N gear
//			else if(Rxmsg[3] == 0X52)
//				current_gear_value = 4; //R gear
//			else if(Rxmsg[3] == 0x50)
//				current_gear_value = 5; //P gear
//			break;
//			//		case 0x18f006f6h:
//			//
//			//			break;
//		case 0x18FEC1F6:
//			mileage = (Rxmsg[0] << 24 ) | (Rxmsg[1] << 16) | (Rxmsg[2] << 8) | Rxmsg[3];
//			break;
//		case 0x18FEF1F6:
//			if(Rxmsg[1] == 29)
//				vehicle_speed = 1;
//			if(Rxmsg[1] == 58)
//				vehicle_speed = 2;
//			if(Rxmsg[1] == 87)
//				vehicle_speed = 3;
//			if(Rxmsg[1] == 116)
//				vehicle_speed = 4;
//			if(Rxmsg[1] == 145)
//				vehicle_speed = 5;
//			if(Rxmsg[1] == 174)
//				vehicle_speed = 6;
//			if(Rxmsg[1] == 203)
//				vehicle_speed = 7;
//			if(Rxmsg[1] == 232)
//				vehicle_speed = 8;
//			if(Rxmsg[1] == 5)
//				vehicle_speed = 9;
//			if(Rxmsg[1] == 34)
//				vehicle_speed = 10;
//			if(Rxmsg[1] == 63)
//				vehicle_speed = 11;
//			if(Rxmsg[1] == 92)
//				vehicle_speed = 12;
//			if(Rxmsg[1] == 121)
//				vehicle_speed = 13;
//			if(Rxmsg[1] == 150)
//				vehicle_speed = 14;
//			if(Rxmsg[1] == 179)
//				vehicle_speed = 15;
//			if(Rxmsg[1] == 208)
//				vehicle_speed = 16;
//			if(Rxmsg[1] == 237)
//				vehicle_speed = 17;
//			if(Rxmsg[1] == 10)
//				vehicle_speed = 18;
//			if(Rxmsg[1] == 39)
//				vehicle_speed = 19;
//			//			vehicle_speed = Rxmsg[1] & 0xFF;
//			break;
//		}
//
//		//ADD THE CAN RECEIVER FOR IMU DATA
//
//	}
//}
////}
//
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
//	if(hcan == &hcan2){
//		//		can_enter = 90;
//		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Rxheader, Rxmsg) != HAL_OK) {
//			// Reception Error
//			//			Error_Handler();
//			can_ERROR = 4;
//		}
//
//		switch(Rxheader.StdId)
//		{
//		case 0x18F:
//			ecuStatus = Rxmsg[0]; // 8 bits
//			currentAngle = (Rxmsg[1] << 8 ) | Rxmsg[2]; // 16 bits
//			angularVelocity = (Rxmsg[3] << 8) | Rxmsg[4]; // 16 bits
//			motorCurrent = (int8_t)Rxmsg[5]; // 8 bits
//			ecuTemperature = Rxmsg[6]; // 8 bits
//			faultEnumeration = Rxmsg[7]; // 8 bits
//
//			// Decode ECU Status bits
//			wireControlMode = ecuStatus & 0x01;   // Bit 0
//			ecuFault = (ecuStatus >> 1) & 0x01;   // Bit 1
//			medianCalibrated = (ecuStatus >> 2) & 0x01; // Bit 2
//
//			// Print the decoded ECU Status
//			//			printf("Received CAN message with ID: 0x18F\n");
//			//			printf("ECU Status: %d\n", ecuStatus);
//			//			printf("  Wire Control Mode: %s\n", wireControlMode ? "Manual Mode" : "Automatic Mode");
//			//			printf("  ECU Fault Detected: %s\n", ecuFault ? "Fault Detected" : "No Fault Detected");
//			//			printf("  Median Calibrated: %s\n", medianCalibrated ? "Not Calibrated" : "Calibrated");
//			//
//			//			// Print the parsed data
//			//			printf("Current Angle: %d\n", currentAngle);
//			//			printf("Angular Velocity: %d\n", angularVelocity);
//			//			printf("Motor Current: %d A\n", motorCurrent);
//			//			printf("ECU Temperature: %d °C\n", ecuTemperature);
//			//			printf("Fault Enumeration: %d\n", faultEnumeration);
//
//			//			CAN1_Send_SteeringDataToPC();
//			break;
//
//			//		default:
//			//						printf("No Data Received\n");
//
//		case 0x311:
//			//						int accx_sign = (Rxmsg[0] >> 7) & 0x01;
//			//						accelerationx = (float)(Rxmsg[0] & 0x7F) + (((float)Rxmsg[1] & 0x7F) / 100.0);
//			//						if (accx_sign == 1) accelerationx = -accelerationx;
//			//
//			//						int accy_sign = (Rxmsg[2] >> 7) & 0x01;
//			//						accelerationy = (float)(Rxmsg[2] & 0x7F) + (((float)Rxmsg[3] & 0x7F) / 100.0);
//			//						if (accy_sign == 1) accelerationy = -accelerationy;
//			//
//			//						int accz_sign = (Rxmsg[4] > 7) & 0x01;
//			//						accelerationz = (float)(Rxmsg[4] & 0x7F) + (((float)Rxmsg[5] & 0x7F) / 100.0);
//			//						if (accz_sign == 1) accelerationz = -accelerationz;
//			//
//			//						accelerationx = (Rxmsg[1] << 8 ) | Rxmsg[0];
//			//						accelerationy = (Rxmsg[3] << 8 ) | Rxmsg[2];
//			//						accelerationz = (Rxmsg[5] << 8 ) | Rxmsg[6];
//			desired_speed = Rxmsg[0];
//
//			break;
//
//			//		case 0x312:
//			//			velocity1 = (Rxmsg[1] << 8 ) | Rxmsg[0];
//			//			velocity2 = (Rxmsg[3] << 8 ) | Rxmsg[2];
//			//			velocity3 = (Rxmsg[5] << 8 ) | Rxmsg[6];
//			//			break;
//
//		case 0x313:
//			int roll_sign = (Rxmsg[0] >> 7) & 0x01;
//			roll = (float)((Rxmsg[0] & 0x7F) | ((Rxmsg[1] >> 7) << 7)) + ((float)(Rxmsg[1] & 0x7F) / 100.0);
//			if (roll_sign == 1) roll = -roll;
//
//			int pitch_sign = (Rxmsg[2] >> 7) & 0x01;
//			pitch = (float)((Rxmsg[2] & 0x7F) | ((Rxmsg[3] >> 7) << 7)) + ((float)(Rxmsg[3] & 0x7F) / 100.0);
//			if (pitch_sign == 1) pitch = -pitch;
//
//			int yaw_sign = (Rxmsg[4] >> 7) & 0x01;
//			yaw = (float)((Rxmsg[4] & 0x7F) | ((Rxmsg[5] >> 7) << 7)) + ((float)(Rxmsg[5] & 0x7F) / 100.0);
//			if (yaw_sign == 1) yaw = -yaw;
//
//			//			roll = (Rxmsg[1] << 8 ) | Rxmsg[0];
//			//			pitch = (Rxmsg[3] << 8 ) | Rxmsg[2];
//			//			yaw = (Rxmsg[5] << 8 ) | Rxmsg[6];
//			break;
//		}
//
//		//		switch(Rxheader.ExtId){
//		//		case 0x1806E536:
//		//			break;
//		//		case 0x1806E544:
//		//			break;
//		//		}
//	}
//}

#if 1
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if(hcan == &hcan1 ){
		//		can_enter = 20;
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxheader, Rxmsg) != HAL_OK) {

			// Reception Error
			//			Error_Handler();
		}
		switch(Rxheader.StdId){
		case 0x18F:
			ecuStatus = Rxmsg[0]; // 8 bits
			currentAngle = (Rxmsg[1] << 8 ) | Rxmsg[2]; // 16 bits
			angularVelocity = (Rxmsg[3] << 8) | Rxmsg[4]; // 16 bits
			motorCurrent = (int8_t)Rxmsg[5]; // 8 bits
			ecuTemperature = Rxmsg[6]; // 8 bits
			faultEnumeration = Rxmsg[7]; // 8 bits

			// Decode ECU Status bits
			wireControlMode = ecuStatus & 0x01;   // Bit 0
			ecuFault = (ecuStatus >> 1) & 0x01;   // Bit 1
			medianCalibrated = (ecuStatus >> 2) & 0x01; // Bit 2

			// Print the decoded ECU Status
			//			printf("Received CAN message with ID: 0x18F\n");
			//			printf("ECU Status: %d\n", ecuStatus);
			//			printf("  Wire Control Mode: %s\n", wireControlMode ? "Manual Mode" : "Automatic Mode");
			//			printf("  ECU Fault Detected: %s\n", ecuFault ? "Fault Detected" : "No Fault Detected");
			//			printf("  Median Calibrated: %s\n", medianCalibrated ? "Not Calibrated" : "Calibrated");
			//
			//			// Print the parsed data
			//			printf("Current Angle: %d\n", currentAngle);
			//			printf("Angular Velocity: %d\n", angularVelocity);
			//			printf("Motor Current: %d A\n", motorCurrent);
			//			printf("ECU Temperature: %d °C\n", ecuTemperature);
			//			printf("Fault Enumeration: %d\n", faultEnumeration);

			//			CAN1_Send_SteeringDataToPC();
			break;

			//		default:
			//						printf("No Data Received\n");
		}

		//		switch(Rxheader.ExtId){
		//		case 0x1806E536:
		//			break;
		//		case 0x1806E544:
		//			break;
		//		}
	}
#if 0
	if (hcan == &hcan2) {
		// Process messages from CAN 2
		//Sends data to pc
		can_enter = 40;
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxheader2, Rxmsg1) != HAL_OK) {
			//			Error_Handler();
		}
		//	}
		switch(Rxheader2.ExtId){
		case 0x18F005F6:
			if(Rxmsg1[3] == 0x4C)
				current_gear_value = 1; //L gear
			else if(Rxmsg1[3] == 0x48)
				current_gear_value = 2; //H gear
			else if(Rxmsg1[3] == 0x4E)
				current_gear_value = 3; //N gear
			else if(Rxmsg1[3] == 0X52)
				current_gear_value = 4; //R gear
			else if(Rxmsg1[3] == 0x50)
				current_gear_value = 5; //P gear
			break;
			//			//		case 0x18f006f6h:
			//			//
			//			//			break;
			//		case 0x18FEC1F6:
			//			mileage = (Rxmsg[0] << 24 ) | (Rxmsg[1] << 16) | (Rxmsg[2] << 8) | Rxmsg[3];
			//			break;
		case 0x18FEF1F6:
			if(Rxmsg[1] == 29)
				vehicle_speed = 1;
			if(Rxmsg[1] == 58)
				vehicle_speed = 2;
			if(Rxmsg[1] == 87)
				vehicle_speed = 3;
			if(Rxmsg[1] == 116)
				vehicle_speed = 4;
			if(Rxmsg[1] == 145)
				vehicle_speed = 5;
			if(Rxmsg[1] == 174)
				vehicle_speed = 6;
			if(Rxmsg[1] == 203)
				vehicle_speed = 7;
			if(Rxmsg[1] == 232)
				vehicle_speed = 8;
			if(Rxmsg[1] == 5)
				vehicle_speed = 9;
			if(Rxmsg[1] == 34)
				vehicle_speed = 10;
			if(Rxmsg[1] == 63)
				vehicle_speed = 11;
			if(Rxmsg[1] == 92)
				vehicle_speed = 12;
			if(Rxmsg[1] == 121)
				vehicle_speed = 13;
			if(Rxmsg[1] == 150)
				vehicle_speed = 14;
			if(Rxmsg[1] == 179)
				vehicle_speed = 15;
			if(Rxmsg[1] == 208)
				vehicle_speed = 16;
			if(Rxmsg[1] == 237)
				vehicle_speed = 17;
			if(Rxmsg[1] == 10)
				vehicle_speed = 18;
			if(Rxmsg[1] == 39)
				vehicle_speed = 19;
			//			//			vehicle_speed = Rxmsg[1] & 0xFF;
			break;
			//		}
		}
	}
#endif
}
#endif

#if 1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if (hcan == &hcan2) {
		//		// Process messages from CAN 2
		//		//Sends data to pc
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Rxheader2, Rxmsg1) != HAL_OK) {
			can_enter = 40;
			//			Error_Handler();
		}
		//	}
		switch(Rxheader2.ExtId){
		case 0x18F005F6:
			if(Rxmsg1[3] == 0x4C)
				current_gear_value = 1; //L gear
			else if(Rxmsg1[3] == 0x48)
				current_gear_value = 2; //H gear
			else if(Rxmsg1[3] == 0x4E)
				current_gear_value = 3; //N gear
			else if(Rxmsg1[3] == 0X52)
				current_gear_value = 4; //R gear
			else if(Rxmsg1[3] == 0x50)
				current_gear_value = 5; //P gear
			break;
			//			//		case 0x18f006f6h:
			//			//
			//			//			break;
			//		case 0x18FEC1F6:
			//			mileage = (Rxmsg[0] << 24 ) | (Rxmsg[1] << 16) | (Rxmsg[2] << 8) | Rxmsg[3];
			//			break;
		case 0x18FEF1F6:
			if(Rxmsg[1] == 29)
				vehicle_speed = 1;
			if(Rxmsg[1] == 58)
				vehicle_speed = 2;
			if(Rxmsg[1] == 87)
				vehicle_speed = 3;
			if(Rxmsg[1] == 116)
				vehicle_speed = 4;
			if(Rxmsg[1] == 145)
				vehicle_speed = 5;
			if(Rxmsg[1] == 174)
				vehicle_speed = 6;
			if(Rxmsg[1] == 203)
				vehicle_speed = 7;
			if(Rxmsg[1] == 232)
				vehicle_speed = 8;
			if(Rxmsg[1] == 5)
				vehicle_speed = 9;
			if(Rxmsg[1] == 34)
				vehicle_speed = 10;
			if(Rxmsg[1] == 63)
				vehicle_speed = 11;
			if(Rxmsg[1] == 92)
				vehicle_speed = 12;
			if(Rxmsg[1] == 121)
				vehicle_speed = 13;
			if(Rxmsg[1] == 150)
				vehicle_speed = 14;
			if(Rxmsg[1] == 179)
				vehicle_speed = 15;
			if(Rxmsg[1] == 208)
				vehicle_speed = 16;
			if(Rxmsg[1] == 237)
				vehicle_speed = 17;
			if(Rxmsg[1] == 10)
				vehicle_speed = 18;
			if(Rxmsg[1] == 39)
				vehicle_speed = 19;
			//			//			vehicle_speed = Rxmsg[1] & 0xFF;
			break;
			//		}
		}
	}
}
#endif

void can_rcv_cmd_vel(pidParams *pidValue, float error) {
    // Update timing values
    pidValue->current_time = HAL_GetTick();
    pidValue->ts = (pidValue->current_time - pidValue->prev_time) / 1000.0f;
    if (pidValue->ts < 1e-6) pidValue->ts = 1e-6;  // Prevent division by zero
    pidValue->prev_time = pidValue->current_time;

    // Update error values
    pidValue->prev_error = pidValue->error;
    pidValue->error = error;
    pidValue->error_sum += pidValue->error * pidValue->ts;

    // Compute P, I, D components
    pidValue->P = pidValue->kp * pidValue->error;

    pidValue->I = pidValue->ki * pidValue->error_sum;
    pidValue->I = fminf(fmaxf(pidValue->I, -1000.0f), 1000.0f);

    pidValue->D = pidValue->kd * (pidValue->error - pidValue->prev_error) / pidValue->ts;

    // Compute PID output
    output = pidValue->P + pidValue->I + pidValue->D;
    output = fminf(fmaxf(output, 20.0f), 100.0f);

    // Neural Network Adjustment
    h1 = sigmoid(w1 * pidValue->error + w2 * ((pidValue->prev_error - pidValue->error) / pidValue->ts)  + w3 * pidValue->error_sum);
    h2 = sigmoid(w4 * pidValue->error + w5 * ((pidValue->prev_error - pidValue->error) / pidValue->ts)  + w6 * pidValue->error_sum);
    h3 = sigmoid(w7 * pidValue->error + w8 * ((pidValue->prev_error - pidValue->error) / pidValue->ts)  + w9 * pidValue->error_sum);

    // Update PID gains dynamically
    pidValue->kp = 2.0f * h1;
    pidValue->ki = 0.05f * h2;
    pidValue->kd = 0.4f * h3;

    // Recalculate PID with updated gains
    pidValue->P = pidValue->kp * pidValue->error;
    pidValue->I = pidValue->ki * pidValue->error_sum;
    pidValue->I = fminf(fmaxf(pidValue->I, -1000.0f), 1000.0f);
    pidValue->D = pidValue->kd * (pidValue->error - pidValue->prev_error) / pidValue->ts;

    output = pidValue->P + pidValue->I + pidValue->D;
    output = fminf(fmaxf(output, 20.0f), 100.0f);

    // Update NN Weights using Gradient Descent (adjusted)
    float learning_rate = pidValue->learning_rate;

    w1 += learning_rate * (error * h1 * (1 - h1));
    w2 += learning_rate * ((pidValue->prev_error - pidValue->error) * h1 * (1 - h1));
    w3 += learning_rate * (pidValue->error_sum * h1 * (1 - h1));

    w4 += learning_rate * (error * h2 * (1 - h2));
    w5 += learning_rate * ((pidValue->prev_error - pidValue->error) * h2 * (1 - h2));
    w6 += learning_rate * (pidValue->error_sum * h2 * (1 - h2));

    w7 += learning_rate * (error * h3 * (1 - h3));
    w8 += learning_rate * ((pidValue->prev_error - pidValue->error) * h3 * (1 - h3));
    w9 += learning_rate * (pidValue->error_sum * h3 * (1 - h3));

    // Set Motor Direction and PWM Output
    if (error > 0) {
        HAL_GPIO_WritePin(DIR2_THROTTLE_GPIO_Port, DIR2_THROTTLE_Pin, GPIO_PIN_RESET);
    }
    else if(error==0){
    	output = 0;
    }


   else {
        HAL_GPIO_WritePin(DIR2_THROTTLE_GPIO_Port, DIR2_THROTTLE_Pin, GPIO_PIN_SET);
    }

    TIM3->CCR1 = abs(output);
}

void SteeingsMsg_Init(void)
{
	SteeringMsg.DLC = 8;
	SteeringMsg.IDE = CAN_ID_STD;
	SteeringMsg.RTR = CAN_RTR_DATA;
	SteeringMsg.StdId = 0x314;
	SteeringMsg.ExtId = 0;
	SteeringMsg.TransmitGlobalTime = DISABLE ;
}

void PC_CANMsg_Init(void)
{
	Throttle_Brake_PC_Msg.DLC = 8; //Data
	Throttle_Brake_PC_Msg.IDE = CAN_ID_STD;
	Throttle_Brake_PC_Msg.RTR = CAN_RTR_DATA;
	Throttle_Brake_PC_Msg.StdId = 0x318;
	Throttle_Brake_PC_Msg.ExtId = 0;
	Throttle_Brake_PC_Msg.TransmitGlobalTime = DISABLE;

	Steering_PC_Msg.DLC = 8; //Data
	Steering_PC_Msg.IDE = CAN_ID_STD;
	Steering_PC_Msg.RTR = CAN_RTR_DATA;
	Steering_PC_Msg.StdId = 0x319;
	Steering_PC_Msg.ExtId = 0;
	Steering_PC_Msg.TransmitGlobalTime = DISABLE;

	Speed_PC_Msg.DLC = 8; //Data
	Speed_PC_Msg.IDE = CAN_ID_STD;
	Speed_PC_Msg.RTR = CAN_RTR_DATA;
	Speed_PC_Msg.StdId = 0x320;
	Speed_PC_Msg.ExtId = 0;
	Speed_PC_Msg.TransmitGlobalTime = DISABLE;
}

void CAN1_Send_SteeringData(void) {
	// Send CAN message
	if (HAL_CAN_AddTxMessage(&hcan1, &SteeringMsg, SteeringData, &TxMailbox) != HAL_OK) {
		//		can_ERROR=5;
	}
}

void CAN1_Send_ThrottleDataToPC(int throttle_input, int brake_input){
	//Send CAN Data to PC
	//Throttle & Brake
	Throttle_Brake_PC_Data[0] = (throttle_feedback & 0xFF); //0-7 bits of throttle feedback
	Throttle_Brake_PC_Data[1] = (throttle_feedback >> 8) & 0xFF; //8-15 bits of throttle feedback
	Throttle_Brake_PC_Data[2] = throttle_input  & 0xFF;; //0-7 of throttle input
	Throttle_Brake_PC_Data[3] = (throttle_input >> 8) & 0xFF; //8-15 of throttle input
	Throttle_Brake_PC_Data[4] = brake_feedback  & 0xFF; //0-7 bit values of break feedback
	Throttle_Brake_PC_Data[5] = (brake_feedback >> 8) & 0xFF; //8-15 bits values of throttle feedback
	Throttle_Brake_PC_Data[6] = brake_input & 0xFF;
	Throttle_Brake_PC_Data[7] = (brake_input >> 8) & 0xFF;

	if (HAL_CAN_AddTxMessage(&hcan1, &Throttle_Brake_PC_Msg, Throttle_Brake_PC_Data, &TxMailbox) != HAL_OK) {
		//		can_ERROR=3;
	}
}

void CAN1_Send_SteeringDataToPC(){
	//Send CAN Data to PC
	//Mode, Steering and Velocity
	if(mode_sw==1712)  //TODO: CHANGE
		Steering_PC_Data[0] = 1;
	else if(mode_sw==992)
		Steering_PC_Data[0] = 2;
	else if(mode_sw == 272)
		Steering_PC_Data[0] = 3;
	else
		Steering_PC_Data[0] = 0;
	Steering_PC_Data[1] = currentAngle & 0xFF; //0-7
	Steering_PC_Data[2] = (currentAngle >> 8) & 0xFF; //Steering input 2
	Steering_PC_Data[3] = steeringAngle & 0xFF;
	Steering_PC_Data[4] = (steeringAngle >> 8) & 0xFF; // from input
	Steering_PC_Data[5] = angularVelocity & 0xFF; //steering velocity
	Steering_PC_Data[6] = (angularVelocity >> 8) & 0xFF;
	Steering_PC_Data[7] = 0x00;

	//	Modified to send data through can1.
	if (HAL_CAN_AddTxMessage(&hcan1, &Steering_PC_Msg, Steering_PC_Data, &TxMailbox) != HAL_OK) {
		//		can_ERROR=6;
	}
}

void CAN1_Send_Speed_PC_Msg(){
	Speed_PC_Data[0] = vehicle_speed;
	Speed_PC_Data[1] = current_gear_value;
	Speed_PC_Data[2] = 0x00;
	Speed_PC_Data[3] = 0x00;
	Speed_PC_Data[4] = 0x00;
	Speed_PC_Data[5] = 0x00;
	Speed_PC_Data[6] = 0x00;
	Speed_PC_Data[7] = 0x00;
	if (HAL_CAN_AddTxMessage(&hcan1, &Speed_PC_Msg, Speed_PC_Data, &TxMailbox) != HAL_OK) {
		can_ERROR=7;
	}
}

void Map_RCtoCAN(void){

	SteeringData[0] = 0x01; // 1-Wire control mode; 0-Manual mode
	//	SteeringData[2] = 0x01; // Set the current position to "middle"

	// Map rc_channel_data (0 to 1000) to CAN data fields as needed
	// Example: Mapping channel data to angle and velocity
	//	steeringAngle = map(steering,0,1000,600,-600); // Maps 0-1000 to -650° to +650°
	//	steeringAngle= -650 + ((1000-(int)steering) * 1300 / 1000)+50;
	//	steeringAngle= -400 + ((1000-(int)steering) * 1600 / 1000);
	//	l458 r-388
	steeringAngle = 458 + ((int)(steering_input_from_RC-230)*(-388-458)/1520) -50;
	//TODO: Centre steering angle
	//	steeringAngle = 458 +
	steeringVelocity = 520;//340;   // Should map 0-1000 to 50°/s to 520°/s

	// Store steeeringAngle in Txmsg (data length 16 bytes)
	SteeringData[1] = (steeringAngle >> 8) & 0xFF;
	SteeringData[2] = steeringAngle & 0xFF;
	SteeringData[3] = (steeringVelocity >> 8) & 0xFF;
	SteeringData[4] = steeringVelocity & 0xFF;
}

void TIMResetCounter2(void){
	HAL_TIM_Base_Stop_IT(&htim2);

	TIM2->CNT = 0;
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;

	throttle_feedback = 0;
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
}

void TIMResetCounter8(void){
	HAL_TIM_Base_Stop_IT(&htim8);

	TIM8->CNT = 0;
	TIM8->CCR1 = 0;
	TIM8->CCR2 = 0;

	brake_feedback = 0;
	HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);
}

void TIMResetCounter4(){
	HAL_TIM_Base_Stop_IT(&htim4);

	TIM4->CNT = 0;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;

	gear_feedback = 0;
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
}

//TIM4 changed to TIM12
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM12) { //20ms trigger
		if(mode_sw==992){
			CAN1_Send_SteeringData();
		}
	}
}

void Encoder_Freeze_Handler(int current_throttle_diff){
	//WHEN THE ACTUATOR FEEDBACK DOESN'T INCREASE
	throttle_current_diff = current_throttle_diff;
	if(throttle_current_diff == throttle_past_diff && throttle_duty_cycle_val > PWM_THRESHOLD_THROTTLE && throttle_past_diff> ERROR_MARGIN_PWM && throttle_current_diff> ERROR_MARGIN_PWM)
	{
		throttle_unchanged_count ++;
		if(throttle_unchanged_count == 10){
			//			emg_cutoff = true;
		}
	}
	else {
		throttle_unchanged_count =  0;
		throttle_past_diff = throttle_current_diff;
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2){
		brake_feedback = __HAL_TIM_GET_COUNTER(htim);
	}

	if(htim->Instance == TIM8){
		throttle_feedback =  __HAL_TIM_GET_COUNTER(htim);
	}

	if(htim->Instance == TIM4){
		gear_feedback = __HAL_TIM_GET_COUNTER(htim);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	SteeingsMsg_Init();
	PC_CANMsg_Init();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN1_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	//MX_IWDG_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	MX_USART3_UART_Init();
	MX_TIM12_Init();
	MX_TIM4_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_CAN2_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);


	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {// Notification Error
		can_ERROR=1;
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		can_ERROR=2;
	}

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { //Notification Error
		can_ERROR=1;
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) { // Notification Error
		can_ERROR=199;
	}

	if (HAL_CAN_Start(&hcan2) != HAL_OK) { // Start Error
		can_ERROR=200;
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) { //	 Notification Error
		can_ERROR=199;
	}
	//	HAL_NVIC_GetPriority(IRQn, PriorityGroup, pPreemptPriority, pSubPriority)
	//	HAL_CAN_GetRxMessage(hcan, RxFifo, pHeader, aData)

	//tim12 replaces tim4 for callback of can sending data to steering wheel
	if (HAL_TIM_Base_Start_IT(&htim12) != HAL_OK) { //Starting error
		Error_Handler();
	}

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	//These timers start PWM output
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	//These timers are needed for feedback
	HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

		//actuatorRelease(BRAKE_ACTUATOR, 100, BRAKE_EXTENDED);
		actuatorRelease(THROTTLE_ACTUATOR, 100, BRAKE_RETRACTED);
	actuatorRelease(GEAR_ACTUATOR, 100, BRAKE_RETRACTED);
		//HAL_Delay(12000);
	TIMResetCounter4();
	TIMResetCounter2();
	TIMResetCounter8();
	//	HAL_Delay(4000);
	//	actuatorApply(GEAR_ACTUATOR, 760); //Set gear to neutral
	//	HAL_Delay(2000);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	//	TIMResetCounter4();


	//	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	//	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	//
	//	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
	//	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	//	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
	//	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	//	HAL_Delay(5000);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//	HAL_Delay(100);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//	HAL_Delay(100);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//	HAL_Delay(100);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	//	TIMResetCounter2();
	//	TIMResetCounter8();
	//	TIMResetCounter4();
	HAL_UART_Receive_DMA(&huart1, pre_catch_buf, 1);
	//	HAL_Delay(100);

	//MX_IWDG_Init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	//		printf("Current values are 1: %d 2: %d 3: %d 4: %d", throttle_input, throttle, brake_input, brake);
	//	printf("Current values are %d, %d, %d, %d, %d, %d", brake, brake_input, brake_feedback, throttle, throttle_input, throttle_feedback);
	// 	printf("Current values are brake_input: %d, break: %d, brake_feedback: %d, throttle_joystick: %d, throttle_knob: %d, input_rpm: %d, init_rpm: %d, throttle_input: %d, init_pos_finetune: %d, mode_sw: %d ", brake, brake_input, brake_feedback, throttle_joystick, throttle_knob, input_rpm, init_rpm, throttle_input, init_pos_finetune, mode_sw);

	/*

	TIM1->CCR1 = 100;
	TIM9->CCR1 = 100;
	TIM9->CCR2 = 100;
	HAL_GPIO_WritePin(DIR1_BRAKE_GPIO_Port, DIR1_BRAKE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR2_THROTTLE_GPIO_Port, DIR2_THROTTLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR3_GEAR_GPIO_Port, DIR3_GEAR_Pin, GPIO_PIN_SET);
	 */

	//	can1_error = HAL_CAN_GetError(&hcan1);
	//	can2_error = HAL_CAN_GetError(&hcan2);
	//	can1_state = HAL_CAN_GetState(&hcan1);
	//	can2_state = HAL_CAN_GetState(&hcan2);

	while (1)
	{
		/* USER CODE END WHILE */
		hcan1_error = HAL_CAN_GetError(&hcan1);
		hcan2_error = HAL_CAN_GetError(&hcan2);
		can1_state = HAL_CAN_GetState(&hcan1);
		can2_state = HAL_CAN_GetState(&hcan2);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canFilterConfig;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.FilterBank = 0;
	//	canFilterConfig.SlaveStartFilterBank = 0;
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK) {
		// Filter configuration Error
		//	  Error_Handler();

	}

	/* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 12;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_6TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = ENABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */
	CAN_FilterTypeDef canFilterConfig;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.FilterBank = 15;
	//		canFilterConfig.SlaveStartFilterBank = 14;
	canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	if (HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig) != HAL_OK) {
		// Filter configuration Error
		//	  Error_Handler();

	}
	/* USER CODE END CAN2_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Reload = 300;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 84-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 100-1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 65535;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void)
{

	/* USER CODE BEGIN TIM12_Init 0 */

	/* USER CODE END TIM12_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = {0};

	/* USER CODE BEGIN TIM12_Init 1 */

	/* USER CODE END TIM12_Init 1 */
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 0;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = 65535;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim12, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM12_Init 2 */

	/* USER CODE END TIM12_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 100000;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_2;
	huart1.Init.Parity = UART_PARITY_EVEN;
	huart1.Init.Mode = UART_MODE_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, FRONT_MID_DFRNTL_GPIO_Pin|DIR2_THROTTLE_Pin|DIR1_BRAKE_Pin|RC_MODE_CHANGE_LED_Pin
			|JETSON_ONOFF_GPIO_Pin|PARKING_LIGHT_GPIO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, JETSON_REC_GPIO_Pin|LD2_Pin|RC_DISCONNECT_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DIR3_GEAR_Pin|HEADLIGHT_CONTROL_GPIO_Pin|ERROR_INDICATOR_LED_Pin|INDICATORS_GPIO_Pin
			|REAR_DFRNTL_GPIO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(HORN_CONTROL_GPIO_GPIO_Port, HORN_CONTROL_GPIO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Blue_PushButton_Pin */
	GPIO_InitStruct.Pin = Blue_PushButton_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Blue_PushButton_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : FRONT_MID_DFRNTL_GPIO_Pin DIR2_THROTTLE_Pin RC_MODE_CHANGE_LED_Pin JETSON_ONOFF_GPIO_Pin
                           PARKING_LIGHT_GPIO_Pin */
	GPIO_InitStruct.Pin = FRONT_MID_DFRNTL_GPIO_Pin|DIR2_THROTTLE_Pin|RC_MODE_CHANGE_LED_Pin|JETSON_ONOFF_GPIO_Pin
			|PARKING_LIGHT_GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : JETSON_REC_GPIO_Pin LD2_Pin RC_DISCONNECT_LED_Pin */
	GPIO_InitStruct.Pin = JETSON_REC_GPIO_Pin|LD2_Pin|RC_DISCONNECT_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : DIR1_BRAKE_Pin */
	GPIO_InitStruct.Pin = DIR1_BRAKE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DIR1_BRAKE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR3_GEAR_Pin HEADLIGHT_CONTROL_GPIO_Pin ERROR_INDICATOR_LED_Pin INDICATORS_GPIO_Pin
                           REAR_DFRNTL_GPIO_Pin */
	GPIO_InitStruct.Pin = DIR3_GEAR_Pin|HEADLIGHT_CONTROL_GPIO_Pin|ERROR_INDICATOR_LED_Pin|INDICATORS_GPIO_Pin
			|REAR_DFRNTL_GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : HORN_CONTROL_GPIO_Pin */
	GPIO_InitStruct.Pin = HORN_CONTROL_GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HORN_CONTROL_GPIO_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	//	while (1)
	//	{
	//	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
