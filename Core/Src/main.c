/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t user_button_value = 1; // store the current state of the user button. 1 = unpressed, 0 = pressed.

// state control variables
#define NO_CONTROL 0
#define INITIAL_PUSH 1
#define ENERGY_SHAPING 2
#define PID_CONTROL 3
uint8_t control_mode = NO_CONTROL;
uint8_t previous_control_mode = NO_CONTROL;

float velocity = 0.0; // velocity (output of the PID controller)
float low_pass_velocity = 0.0; // velocity after filtering to remove sudden spikes
float velocity_low_pass_constant = 0.0; // constant used for filtering velocity (0 means no filtering, approaching 1 means aggressive filtering)

uint32_t delay_since_last_cycle;
float cycle_state = 0; // where the BLDC motor is in its cycle (updated from velocity)

// variables for tracking encoder position
uint8_t encoder_one_value = 0; // value of channel one of the encoder signal
uint8_t encoder_two_value = 0; // value of channel two of the encoder signal
int position = 1200; // position of the encoder
int setpoint = 0; // target position for the PID controller

// timing for PID and energy shaping control
float time_change;
float sample_time = 0.001; // how frequently the PID & energy shaping controllers should act

float max_speed = 2.2; // maximum speed permitted

// sine lookup table for calculating duty cycles for the BLDC motor
const uint16_t sine_lookup_table[1200] = {
32768, 32939, 33111, 33282, 33454, 33625, 33797, 33968, 34140, 34311, 34482, 34654, 34825, 34996, 35167, 35338, 35509, 35680, 35851, 36022, 36193, 36363, 36534, 36704, 36874, 37045, 37215, 37384, 37554, 37724, 37893, 38063, 38232, 38401, 38570, 38739, 38908, 39076, 39244, 39412, 39580, 39748, 39916, 40083, 40250, 40417, 40584, 40750, 40916, 41083, 41248, 41414, 41579, 41744, 41909, 42074, 42238, 42402, 42566, 42730, 42893, 43056, 43219, 43381, 43544, 43706, 43867, 44028, 44189, 44350, 44510, 44670, 44830, 44989, 45148, 45307, 45465, 45623, 45781, 45938, 46095, 46252, 46408, 46564, 46719, 46874, 47029, 47183, 47337, 47491, 47644, 47796, 47949, 48100, 48252, 48403, 48553, 48703, 48853, 49002,
49151, 49300, 49448, 49595, 49742, 49888, 50035, 50180, 50325, 50470, 50614, 50758, 50901, 51043, 51186, 51327, 51468, 51609, 51749, 51889, 52028, 52166, 52304, 52442, 52579, 52715, 52851, 52986, 53121, 53255, 53389, 53522, 53654, 53786, 53918, 54048, 54178, 54308, 54437, 54565, 54693, 54820, 54947, 55073, 55198, 55323, 55447, 55571, 55694, 55816, 55938, 56059, 56179, 56299, 56418, 56536, 56654, 56771, 56888, 57003, 57118, 57233, 57347, 57460, 57572, 57684, 57795, 57906, 58015, 58124, 58233, 58340, 58447, 58553, 58659, 58764, 58868, 58971, 59074, 59176, 59277, 59377, 59477, 59576, 59675, 59772, 59869, 59965, 60060, 60155, 60249, 60342, 60434, 60526, 60616, 60706, 60796, 60884, 60972, 61059,
61145, 61230, 61315, 61399, 61482, 61564, 61646, 61726, 61806, 61885, 61964, 62041, 62118, 62194, 62269, 62343, 62416, 62489, 62561, 62632, 62702, 62771, 62840, 62908, 62975, 63041, 63106, 63170, 63234, 63297, 63359, 63420, 63480, 63539, 63598, 63656, 63712, 63768, 63824, 63878, 63931, 63984, 64036, 64086, 64136, 64186, 64234, 64281, 64328, 64374, 64418, 64462, 64506, 64548, 64589, 64630, 64669, 64708, 64746, 64783, 64819, 64854, 64889, 64922, 64955, 64986, 65017, 65047, 65076, 65104, 65132, 65158, 65183, 65208, 65232, 65255, 65277, 65298, 65318, 65337, 65355, 65373, 65390, 65405, 65420, 65434, 65447, 65459, 65470, 65481, 65490, 65499, 65506, 65513, 65519, 65524, 65528, 65531, 65533, 65535,
65535, 65535, 65533, 65531, 65528, 65524, 65519, 65513, 65506, 65499, 65490, 65481, 65470, 65459, 65447, 65434, 65420, 65405, 65390, 65373, 65355, 65337, 65318, 65298, 65277, 65255, 65232, 65208, 65183, 65158, 65132, 65104, 65076, 65047, 65017, 64986, 64955, 64922, 64889, 64854, 64819, 64783, 64746, 64708, 64669, 64630, 64589, 64548, 64506, 64462, 64418, 64374, 64328, 64281, 64234, 64186, 64136, 64086, 64036, 63984, 63931, 63878, 63824, 63768, 63712, 63656, 63598, 63539, 63480, 63420, 63359, 63297, 63234, 63170, 63106, 63041, 62975, 62908, 62840, 62771, 62702, 62632, 62561, 62489, 62416, 62343, 62269, 62194, 62118, 62041, 61964, 61885, 61806, 61726, 61646, 61564, 61482, 61399, 61315, 61230,
61145, 61059, 60972, 60884, 60796, 60706, 60616, 60526, 60434, 60342, 60249, 60155, 60060, 59965, 59869, 59772, 59675, 59576, 59477, 59377, 59277, 59176, 59074, 58971, 58868, 58764, 58659, 58553, 58447, 58340, 58233, 58124, 58015, 57906, 57795, 57684, 57572, 57460, 57347, 57233, 57118, 57003, 56888, 56771, 56654, 56536, 56418, 56299, 56179, 56059, 55938, 55816, 55694, 55571, 55447, 55323, 55198, 55073, 54947, 54820, 54693, 54565, 54437, 54308, 54178, 54048, 53918, 53786, 53654, 53522, 53389, 53255, 53121, 52986, 52851, 52715, 52579, 52442, 52304, 52166, 52028, 51889, 51749, 51609, 51468, 51327, 51186, 51043, 50901, 50758, 50614, 50470, 50325, 50180, 50035, 49888, 49742, 49595, 49448, 49300,
49151, 49002, 48853, 48703, 48553, 48403, 48252, 48100, 47949, 47796, 47644, 47491, 47337, 47183, 47029, 46874, 46719, 46564, 46408, 46252, 46095, 45938, 45781, 45623, 45465, 45307, 45148, 44989, 44830, 44670, 44510, 44350, 44189, 44028, 43867, 43706, 43544, 43381, 43219, 43056, 42893, 42730, 42566, 42402, 42238, 42074, 41909, 41744, 41579, 41414, 41248, 41083, 40916, 40750, 40584, 40417, 40250, 40083, 39916, 39748, 39580, 39412, 39244, 39076, 38908, 38739, 38570, 38401, 38232, 38063, 37893, 37724, 37554, 37384, 37215, 37045, 36874, 36704, 36534, 36363, 36193, 36022, 35851, 35680, 35509, 35338, 35167, 34996, 34825, 34654, 34482, 34311, 34140, 33968, 33797, 33625, 33454, 33282, 33111, 32939,
32768, 32596, 32424, 32253, 32081, 31910, 31738, 31567, 31395, 31224, 31053, 30881, 30710, 30539, 30368, 30197, 30026, 29855, 29684, 29513, 29342, 29172, 29001, 28831, 28661, 28490, 28320, 28151, 27981, 27811, 27642, 27472, 27303, 27134, 26965, 26796, 26627, 26459, 26291, 26123, 25955, 25787, 25619, 25452, 25285, 25118, 24951, 24785, 24619, 24452, 24287, 24121, 23956, 23791, 23626, 23461, 23297, 23133, 22969, 22805, 22642, 22479, 22316, 22154, 21991, 21829, 21668, 21507, 21346, 21185, 21025, 20865, 20705, 20546, 20387, 20228, 20070, 19912, 19754, 19597, 19440, 19283, 19127, 18971, 18816, 18661, 18506, 18352, 18198, 18044, 17891, 17739, 17586, 17435, 17283, 17132, 16982, 16832, 16682, 16533,
16384, 16235, 16087, 15940, 15793, 15647, 15500, 15355, 15210, 15065, 14921, 14777, 14634, 14492, 14349, 14208, 14067, 13926, 13786, 13646, 13507, 13369, 13231, 13093, 12956, 12820, 12684, 12549, 12414, 12280, 12146, 12013, 11881, 11749, 11617, 11487, 11357, 11227, 11098, 10970, 10842, 10715, 10588, 10462, 10337, 10212, 10088, 9964, 9841, 9719, 9597, 9476, 9356, 9236, 9117, 8999, 8881, 8764, 8647, 8532, 8417, 8302, 8188, 8075, 7963, 7851, 7740, 7629, 7520, 7411, 7302, 7195, 7088, 6982, 6876, 6771, 6667, 6564, 6461, 6359, 6258, 6158, 6058, 5959, 5860, 5763, 5666, 5570, 5475, 5380, 5286, 5193, 5101, 5009, 4919, 4829, 4739, 4651, 4563, 4476,
4390, 4305, 4220, 4136, 4053, 3971, 3889, 3809, 3729, 3650, 3571, 3494, 3417, 3341, 3266, 3192, 3119, 3046, 2974, 2903, 2833, 2764, 2695, 2627, 2560, 2494, 2429, 2365, 2301, 2238, 2176, 2115, 2055, 1996, 1937, 1879, 1823, 1767, 1711, 1657, 1604, 1551, 1499, 1449, 1399, 1349, 1301, 1254, 1207, 1161, 1117, 1073, 1029, 987, 946, 905, 866, 827, 789, 752, 716, 681, 646, 613, 580, 549, 518, 488, 459, 431, 403, 377, 352, 327, 303, 280, 258, 237, 217, 198, 180, 162, 145, 130, 115, 101, 88, 76, 65, 54, 45, 36, 29, 22, 16, 11, 7, 4, 2, 0,
0, 0, 2, 4, 7, 11, 16, 22, 29, 36, 45, 54, 65, 76, 88, 101, 115, 130, 145, 162, 180, 198, 217, 237, 258, 280, 303, 327, 352, 377, 403, 431, 459, 488, 518, 549, 580, 613, 646, 681, 716, 752, 789, 827, 866, 905, 946, 987, 1029, 1073, 1117, 1161, 1207, 1254, 1301, 1349, 1399, 1449, 1499, 1551, 1604, 1657, 1711, 1767, 1823, 1879, 1937, 1996, 2055, 2115, 2176, 2238, 2301, 2365, 2429, 2494, 2560, 2627, 2695, 2764, 2833, 2903, 2974, 3046, 3119, 3192, 3266, 3341, 3417, 3494, 3571, 3650, 3729, 3809, 3889, 3971, 4053, 4136, 4220, 4305,
4390, 4476, 4563, 4651, 4739, 4829, 4919, 5009, 5101, 5193, 5286, 5380, 5475, 5570, 5666, 5763, 5860, 5959, 6058, 6158, 6258, 6359, 6461, 6564, 6667, 6771, 6876, 6982, 7088, 7195, 7302, 7411, 7520, 7629, 7740, 7851, 7963, 8075, 8188, 8302, 8417, 8532, 8647, 8764, 8881, 8999, 9117, 9236, 9356, 9476, 9597, 9719, 9841, 9964, 10088, 10212, 10337, 10462, 10588, 10715, 10842, 10970, 11098, 11227, 11357, 11487, 11617, 11749, 11881, 12013, 12146, 12280, 12414, 12549, 12684, 12820, 12956, 13093, 13231, 13369, 13507, 13646, 13786, 13926, 14067, 14208, 14349, 14492, 14634, 14777, 14921, 15065, 15210, 15355, 15500, 15647, 15793, 15940, 16087, 16235,
16384, 16533, 16682, 16832, 16982, 17132, 17283, 17435, 17586, 17739, 17891, 18044, 18198, 18352, 18506, 18661, 18816, 18971, 19127, 19283, 19440, 19597, 19754, 19912, 20070, 20228, 20387, 20546, 20705, 20865, 21025, 21185, 21346, 21507, 21668, 21829, 21991, 22154, 22316, 22479, 22642, 22805, 22969, 23133, 23297, 23461, 23626, 23791, 23956, 24121, 24287, 24452, 24619, 24785, 24951, 25118, 25285, 25452, 25619, 25787, 25955, 26123, 26291, 26459, 26627, 26796, 26965, 27134, 27303, 27472, 27642, 27811, 27981, 28151, 28320, 28490, 28661, 28831, 29001, 29172, 29342, 29513, 29684, 29855, 30026, 30197, 30368, 30539, 30710, 30881, 31053, 31224, 31395, 31567, 31738, 31910, 32081, 32253, 32424, 32596};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// variables for communication with LEDs
uint8_t spi_data[290];
int data_sent_flag = 1;
#define SPI_DATA_SIZE 290

float value = 0.05; // LED brightness

#define NUM_LEDS 10 // number of LEDs to control
#define DUTY_CYCLE_ONE 0b111100 // SPI data to send a 1 to the LED chip
#define DUTY_CYCLE_ZERO 0b110000 // SPI data to send a 0 to the LED chip

// LED CONTROL
void WS2812_Send(uint32_t colours[])
{
  for (int x = 0; x < NUM_LEDS; x++) {
	  int colour = colours[x];
    for (int i = 0; i < 24; i++)
    {
      if (colour&(1<<(23 - i)))
      {
        spi_data[i + (x * 24)] = DUTY_CYCLE_ONE;
      }
      else spi_data[i + (x * 24)] = DUTY_CYCLE_ZERO;
    }
  }

  // force a gap after the data is finished (from datasheet, min 50 microseconds)
  for (int i = 0; i < 50; i++) {
    spi_data[i + (NUM_LEDS * 24)] = 0;
  }

  // wait for the previous data to be sent
  while (!data_sent_flag){};
	data_sent_flag = 0;

	HAL_SPI_Transmit_DMA(&hspi2, (uint32_t *)spi_data, SPI_DATA_SIZE);
}

// callback for when sending data to the LEDs has finished
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_DMAStop(&hspi2);
	data_sent_flag = 1;
}

// ENCODER READING (via interrupts)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
	if (GPIO_pin == ENCODER_ONE_Pin) {
    encoder_one_value = HAL_GPIO_ReadPin(ENCODER_ONE_GPIO_Port, ENCODER_ONE_Pin);

    if (encoder_two_value == encoder_one_value) { // determine phase of the two signals
      position++;
    } else {
      position--;
    }
	} else if (GPIO_pin == ENCODER_TWO_Pin) {
    encoder_two_value = HAL_GPIO_ReadPin(ENCODER_TWO_GPIO_Port, ENCODER_TWO_Pin);

    if (encoder_one_value == encoder_two_value) { // determine phase of the two signals
      position--;
    } else {
      position++;
    }
  } else {
		UNUSED(GPIO_pin);
	}
}

// variables to track for energy shaping control
float energy_shaping_scalar = 0.9;
float previous_position = 0;
float cosine_angle;
float angular_velocity;
float energy_term;

void ComputeEnergyShaping() {
  int time_change_arbitrary_units = __HAL_TIM_GET_COUNTER(&htim2);
  time_change = time_change_arbitrary_units / 84000000.0;

  if (time_change >= sample_time) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    if (previous_control_mode != ENERGY_SHAPING) { // don't act during the first cycle of energy shaping control
      previous_position = position;
      previous_control_mode = control_mode;
      return;
    }

    int position_inverted = 1200 - position;
    if (position_inverted > 1200) {
      position_inverted -= 2400;
    } else if (position_inverted <= -1200) {
      position_inverted += 2400;
    }

    // work out the equivelant index in the sine lookup table for cosine
    int index = (int) floor(((position_inverted + 600) % 2400) / 2);
    if (index < 0) {
      index += 1200;
    } else if (index >= 1200) {
      index -= 1200;
    }

    cosine_angle = (sine_lookup_table[index] / 65535.0 * 2.0) - 1.0;

    int position_change = (position - previous_position);
    
    if (position_change > 1200) {
      position_change -= 2400;
    } else if (position_change <= -1200) {
      position_change += 2400;
    }

    angular_velocity = position_change / time_change / 2400.0; // angular velocity of the encoder (in full cycles per second, NOT radians per second)

    energy_term = (9.81 * (1 + cosine_angle) - 0.9 * angular_velocity * angular_velocity); // a term in the expression based on how much more energy is required to get the pendulum upright

    velocity += time_change * energy_shaping_scalar * angular_velocity * (cosine_angle * cosine_angle * cosine_angle * cosine_angle) * energy_term;

    velocity *= 0.98; // damp the velocity so that it remains close to zero

    if (velocity > max_speed) velocity = max_speed;
    else if (velocity < -max_speed) velocity = -max_speed;

    previous_position = position;
  }
}

// PID coefficients
float kp = 8.25; // proportional constant
float ki = 151.0; // integral constant
float kd = 0.300; // differential constant

// float kp = 7.2; // proportional constant
// float ki = 109.0; // integral constant
// float kd = 0.238; // differential constant

// Ku ~ 25.0
// Tu ~ 0.109s

float max_integral_term = 1.2; // maximum magnitude of the I term, to prevent integral windup

// variables to track for PID control
float error = 0.0;
float low_pass_error = 0.0;
float error_low_pass_constant = 0.0;

float previous_error = 0.0;

float integral_term = 0.0;
float min_error = 0.020;

// PID algorithm
void ComputePID() {
  int time_change_arbitrary_units = __HAL_TIM_GET_COUNTER(&htim2);
  time_change = time_change_arbitrary_units / 84000000.0;

  if (time_change >= sample_time) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    error = (position - setpoint) / 1200.0;

    if (error > 1.0) error -= 2.0;
    if (error <= -1.0) error += 2.0;

    if (previous_control_mode != PID_CONTROL) { // in the first cycle of PID control, reset the integral
      low_pass_error = error;
      integral_term = 0.0;
    }

		low_pass_error = (error_low_pass_constant * previous_error) + (1 - error_low_pass_constant) * error;

		integral_term += ki * time_change * error;

    if (previous_control_mode != PID_CONTROL) { // don't act during the first cycle of PID control
      previous_error = low_pass_error;
      previous_control_mode = control_mode;
      return;
    }

    float error_magnitude = error;
    if (error_magnitude < 0) {
      error_magnitude = -error_magnitude;
    }

    if (error_magnitude < min_error) integral_term *= 0.92 + error_magnitude * 4.0; // when the error is small, scale down the integral term

		if (integral_term > max_integral_term) integral_term = max_integral_term;
		else if (integral_term < -max_integral_term) integral_term = -max_integral_term;

		float error_change = low_pass_error - previous_error;

		if (error_change >= 1.0) error_change -= 2.0;
		else if (error_change < -1.0) error_change += 2.0;

		float new_velocity = (kp * error) + (integral_term) + (kd * error_change / time_change);

		if (new_velocity > max_speed) new_velocity = max_speed;
		else if (new_velocity < -max_speed) new_velocity = -max_speed;

		velocity = new_velocity;

		previous_error = low_pass_error;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim2); // timer 2 is used for PID & energy shaping controllers
  HAL_TIM_Base_Start(&htim4); // timer 4 is used for timing the while loop below

  // read the initial encoder values
  encoder_one_value = HAL_GPIO_ReadPin(ENCODER_ONE_GPIO_Port, ENCODER_ONE_Pin);
  encoder_two_value = HAL_GPIO_ReadPin(ENCODER_TWO_GPIO_Port, ENCODER_TWO_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t colours[10];

    int distance = position;

    if (distance < 0) {
      distance = -distance;
    }

    distance = distance * 2.125; // scale the distance so that half a rotation causes all of the LEDs to be fully lit

    // calculate the intensity of each LED
    for (int i = 0; i < 10; i++) {
      int intensity = distance - i * 255;
      if (intensity < 0) {
        intensity = 0;
      } else if (intensity > 255) {
        intensity = 255;
      }
      if (position >= 0) {
        colours[i] = ((uint32_t) floor(intensity * value)) * 0x010001;
      } else {
        colours[i] = ((uint32_t) floor(intensity * value)) * 0x010100;
      }
    }

    WS2812_Send(colours); // send data to the LEDs

    user_button_value = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);

    if (position > 1200) { // check if encoder position has rolled over
      position -= 2400;
    } else if (position <= -1200) {
      position += 2400;
    }

    // state logic
    if (!user_button_value) {
      control_mode = INITIAL_PUSH;
    } else if (control_mode == NO_CONTROL) {
      control_mode = NO_CONTROL;
    } else if (position > 60 || position < -60) {
      control_mode = ENERGY_SHAPING;
    } else {
      control_mode = PID_CONTROL;
    }

    // execute logic for the current state
    switch (control_mode) {
      case NO_CONTROL:
        velocity = 0.0;
        previous_control_mode = control_mode;
        break;
      case INITIAL_PUSH:
        velocity = 0.2;
        previous_control_mode = control_mode;
        break;
      case ENERGY_SHAPING:
        ComputeEnergyShaping();
        break;
      case PID_CONTROL:
        ComputePID();
        break;
    }

    delay_since_last_cycle = __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    low_pass_velocity = (velocity_low_pass_constant * low_pass_velocity) + ((1 - velocity_low_pass_constant) * velocity);

    cycle_state += low_pass_velocity * delay_since_last_cycle / 50.0; // update BLDC motor position according to velocity

    if (cycle_state < 0) { // check if BLDC motor position has rolled over
      cycle_state += 1200;
    } else if (cycle_state >= 1200) {
      cycle_state -= 1200;
    }

    float speed = velocity;
    if (speed < 0) {
      speed = -speed;
    }

    // choose a time period for the PWM signal sent to the BLDC motor; at higher speeds, the time period should be shorter (i.e. frequency is proportional to speed)
    int period = 2000;
    if (speed > 0.0) {
      period = (int) floor(2000.0 / speed);
      if (period > 40000) {
        period = 40000;
      } else if (period < 1000) {
        period = 1000;
      }
    }

    float duty_cycle_scalar = 0.1 + (speed / 2.5); // all three duty cycles should be higher at high speeds; this effectively regulates the voltage to the motor (though not really)

    // use the sine lookup table to cal
    uint16_t motor_in1_duty_cycle = sine_lookup_table[(int) floor(cycle_state)];
    uint16_t motor_in2_duty_cycle = sine_lookup_table[((int) floor(cycle_state + 400)) % 1200];
    uint16_t motor_in3_duty_cycle = sine_lookup_table[((int) floor(cycle_state + 800)) % 1200];

    // set PWM duty cycles
    TIM1->CCR1 = (int) floor(((float) motor_in1_duty_cycle) * duty_cycle_scalar * period / 65535.0);
    TIM1->CCR2 = (int) floor(((float) motor_in2_duty_cycle) * duty_cycle_scalar * period / 65535.0);
    TIM1->CCR3 = (int) floor(((float) motor_in3_duty_cycle) * duty_cycle_scalar * period / 65535.0);

    // set PWM period
    TIM1->ARR = period;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 120-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_EN1_Pin|MOTOR_EN3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_EN1_Pin MOTOR_EN3_Pin */
  GPIO_InitStruct.Pin = MOTOR_EN1_Pin|MOTOR_EN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_EN2_Pin */
  GPIO_InitStruct.Pin = MOTOR_EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_EN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_TWO_Pin ENCODER_ONE_Pin */
  GPIO_InitStruct.Pin = ENCODER_TWO_Pin|ENCODER_ONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
