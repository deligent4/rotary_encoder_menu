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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_DIGITS 				6

/*
 * MAXIMUM AND MINIMUM SETTINGS FOR LOAD
 */
#define MAX_CC_VALUE			5.0			// 5A maximum current
#define MIN_CC_VALUE			0.001		// 1mA minimum	current
#define MAX_CV_VALUE			30			// 30V maximum voltage
#define MIN_CV_VALUE			3			// 3V minimum voltage
#define MAX_CR_VALUE			10			// 10Ohm maximum resistance
#define MIN_CR_VALUE			0.1			// 100mOhm minimum resistance
#define MAX_CP_VALUE			99.999		// 99.999W maximum power CP is limited to 99.999Watt
#define MIN_CP_VALUE			1			// 1W minimum power

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t tick;
uint32_t rotary_out, rot_pos, rot_cnt, rot_prev_pos = 0;
volatile uint8_t menu_flag = 0;


// Define menu states
typedef enum {
    HOME_SCREEN,
    MODE_SELECTION,
    PARAMETER_SETTING,
    RETURN_TO_HOME
}Menu_State_e;

typedef struct {
	float voltage;
	float current;
	float power;
	float resistance;
}Param_Mode_t;

// Define global variables
Menu_State_e current_state = HOME_SCREEN;
Param_Mode_t param_mode	= {0.0};

int cursor_position = 0;
int mode_index = -1;  // Store the index of mode setting
int mode_index_last = -1;
int last_cursor_position = -1, new_rot_pos = 0;
float volt = 0.0, curr = 0.0, chg = 0.0, temp = 0.0;
float param_value = 00.0, param_value_limit = 0.0;
int digit_position = 0;
int last_rot_cnt = 0;
uint16_t current_a_cnt = 0, current_b_cnt = 0, current_c_cnt = 0;
bool rot_sw_state = false, a_sw_state = false, b_sw_state = false, c_sw_state = false;
bool adjusting_digit = false; // Flag to check if adjusting digit
volatile uint8_t digit_value = 0;
bool output_on_flag = false;

// 'ON', 140x81px
const unsigned char ON_BITMAP[] = {
		0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xf8, 0xf8, 0x1f, 0x1f, 0x38, 0xf0, 0x0f, 0x0f, 0x38,
		0xe1, 0xc7, 0x07, 0x38, 0xe7, 0xe7, 0x07, 0x38, 0xc7, 0xe3, 0x03, 0x38, 0xc7, 0xe3, 0x23, 0x38,
		0xc7, 0xe3, 0x31, 0x38, 0xc7, 0xe3, 0x31, 0x38, 0xc7, 0xe3, 0x38, 0x38, 0xc7, 0xe7, 0x38, 0x38,
		0xe3, 0xc7, 0x3c, 0x38, 0xe0, 0x0f, 0x3e, 0x38, 0xf8, 0x1f, 0x3e, 0x38, 0xff, 0xff, 0xff, 0xf8,
		0xff, 0xff, 0xff, 0xf8
};

// 'OFF', 29x16px
const unsigned char OFF_BITMAP[] = {
		0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xf8, 0xe0, 0x78, 0x08, 0x08, 0xc0, 0x38, 0x08, 0x08,
		0x87, 0x18, 0xf8, 0xf8, 0x8f, 0x18, 0xf8, 0xf8, 0x8f, 0x88, 0xf8, 0xf8, 0x8f, 0x88, 0x18, 0x18,
		0x8f, 0x88, 0x18, 0x18, 0x8f, 0x98, 0xf8, 0xf8, 0x8f, 0x18, 0xf8, 0xf8, 0x80, 0x18, 0xf8, 0xf8,
		0xc0, 0x38, 0xf8, 0xf8, 0xf0, 0xf9, 0xfd, 0xf8, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xf8
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data);
void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data);
void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data);
void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data);
void Move_Cursor(void);
void Print_Mode(void);

// Function Prototypes
void display_home_screen(bool force_update);
void display_mode_selection(bool force_update);
void display_parameter_setting(bool force_update);
void update_encoder_state();
void handle_button_press();
void update_display();
void update_digit_value(int direction);
void update_parameter_value(int direction); // function to update parameter value
void put_parameter_limit(void);				// put limit on parameter values


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  ssd1306_Init();
  update_display(); // Ensure the initial display is updated

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  tick = HAL_GetTick();

	  HAL_GPIO_TogglePin(LED_BLU_GPIO_Port, LED_BLU_Pin);
//	  HAL_Delay(100);
	  if(HAL_GPIO_ReadPin(ROT_SW_GPIO_Port, ROT_SW_Pin) == 0){
		  HAL_GPIO_WritePin(LED_BLU_GPIO_Port, LED_BLU_Pin, 1);
	  }

//      printf("Hello World\n\r\v");

      update_encoder_state();
      handle_button_press();
      update_display();
      printf("current_state %d\n\r", current_state);
      printf("digit_position %d\n\r", digit_position);
      printf("cursor_position %d\n\r", cursor_position);
      printf("mode_index %d\n\r\v", mode_index);

//	  ssd1306_Fill(White);
//      ssd1306_DrawBitmap(95,45,ON_BITMAP,29,17,White);
//      ssd1306_DrawBitmap(95,45,OFF_BITMAP,29,16,White);
//
//      ssd1306_UpdateScreen();

	  HAL_Delay(100); // Adjust the delay as needed

    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	rotary_out = __HAL_TIM_GET_COUNTER(htim);
	rot_cnt = rotary_out / 4;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){
	case ROT_SW_Pin:
		rot_sw_state = true;
		break;
	case A_SW_Pin:
		a_sw_state = true;
		current_a_cnt++;
		break;
	case B_SW_Pin:
		b_sw_state = true;
		current_b_cnt++;
		break;
	case C_SW_Pin:
		c_sw_state = true;
		current_c_cnt++;
		break;
	default:
//		a_sw_state = b_sw_state = c_sw_state = false;
		break;
	}
}

void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data){

	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(data, Font_7x10, White);
}

void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data){
	char str_data[10];

	sprintf(str_data, "%.3f", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data){
	char str_data[10];

	sprintf(str_data, "%u", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data){
	char str_data[10];

	sprintf(str_data, "%d", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

// Update Display
void update_display() {
    static Menu_State_e last_state = HOME_SCREEN;
    static bool first_update = true;
    bool force_update = (current_state != last_state) || first_update;

    if (force_update) {
        last_state = current_state;
        last_cursor_position = -1; // Force full update on state change
        first_update = false;
    }

    // Handle cursor position updates separately
    bool cursor_changed = (cursor_position != last_cursor_position);

    switch (current_state) {
        case HOME_SCREEN:
            display_home_screen(force_update || cursor_changed);
            break;
        case MODE_SELECTION:
            display_mode_selection(force_update || cursor_changed);
            break;
        case PARAMETER_SETTING:
            display_parameter_setting(force_update || cursor_changed);
            break;
        case RETURN_TO_HOME:
            current_state = HOME_SCREEN;
            break;
        default:
        	break;
    }

    last_cursor_position = cursor_position;
}

// Display Home Screen
void display_home_screen(bool force_update) {
    if (force_update || output_on_flag) {
        ssd1306_Fill(Black);			// Clear the display before printing
        myOLED_char(0, 0, "VT:");
        myOLED_float(21, 0, volt);
        myOLED_char(0, 10, "CU:");
        myOLED_float(21, 10, curr);
        myOLED_char(0, 20, "CH:");
        myOLED_float(21, 20, chg);
        myOLED_char(0, 30, "TP:");
        myOLED_float(21, 30, temp);
        ssd1306_Line(70, 0, 70, 64, White);		// Draw line to separate the values and options
        myOLED_char(90, 0, "<SET>");			// SET MODE
        myOLED_char(90, 10, "<ON>");			// Turn ON LOAD TODO
        myOLED_char(90, 20, "<RST>");			// Reset the LOAD
        myOLED_char(90, 30, "<HLP>");			// TODO

        // Show ON or OFF bitmap on display for LOAD status
        if(output_on_flag){
        	myOLED_char(90, 10, "<OFF>");							// Print OFF in ON position if button is pressed
        	output_on_flag = false;									// Change the flag state
			ssd1306_DrawBitmap(95,45,ON_BITMAP,29,16,White);		//	Draw ON bitmap
        }else{
        	ssd1306_DrawBitmap(95,45,OFF_BITMAP,29,16,White);		// Draw OFF bitmap
        }
    }

    // Update cursor only
    for (int i = 0; i < 4; i++) {
            if (i == cursor_position) {
                myOLED_char(75, i* 10, "->");
            } else {
                myOLED_char(75, i* 10, "  ");
            }
        }

    // Display the param value and mode if it is set
    if(current_state == 0){
    	switch (mode_index){
    	case 0:
    		myOLED_char(0, 50, "CC:");
			myOLED_float(21, 50, param_mode.current);
    		break;
    	case 1:
			myOLED_char(0, 50, "CV:");
			myOLED_float(21, 50, param_mode.voltage);
    		break;
    	case 2:
			myOLED_char(0, 50, "CP:");
			myOLED_float(21, 50, param_mode.power);
    		break;
    	case 3:
			myOLED_char(0, 50, "CR:");
			myOLED_float(21, 50, param_mode.resistance);
    		break;
    	default:
    		break;
    	}
    }


    ssd1306_UpdateScreen();
}

// Display Mode Selection Screen
void display_mode_selection(bool force_update) {
    const char* modes[] = {"CC", "CV", "CP", "CR"};
    if (force_update) {
        ssd1306_Fill(Black); // Clear the screen
        for (int i = 0; i < 4; i++) {
            myOLED_char(15, i * 10, (char*)modes[i]); // Print modes in column
        }
    }

    // Update cursor only
    for (int i = 0; i < 4; i++) {
        if (i == cursor_position) {
            myOLED_char(0, i* 10, "->");
        } else {
            myOLED_char(0, i* 10, "  ");
        }
    }

    myOLED_char(5, 50, "<SELECT THE MODE>");
    ssd1306_UpdateScreen();
}


void display_parameter_setting(bool force_update) {
    if (force_update) {
        // Redraw entire screen if forced
        ssd1306_Fill(Black);
        myOLED_char(0, 0, "Set Value:");
        myOLED_char(20, 40, "RETURN");

        // Check the state and print the mode in parameter setting screen
        if(current_state == 2){
        	switch(mode_index) {
        		case 0:
                	myOLED_char(70, 0, "CC");
                	myOLED_char(70, 20, "Amp");
                	break;
        		case 1:
        			myOLED_char(70, 0, "CV");
        			myOLED_char(70, 20, "Volt");
        			break;
        		case 2:
        			myOLED_char(70, 0, "CP");
        			myOLED_char(70, 20, "Watt");
        			break;
        		case 3:
        			myOLED_char(70, 0, "CR");
        			myOLED_char(70, 20, "Ohm");
                	break;
        		default:
        			break;
        	}
        }
    }

    printf("mode_index %d\n\v\r", mode_index);

    // Fetch the current mode's parameter value
	float display_value;

	switch (mode_index) {
		case 0: display_value = param_mode.current; break;
		case 1: display_value = param_mode.voltage; break;
		case 2: display_value = param_mode.power; break;
		case 3: display_value = param_mode.resistance; break;
		default: display_value = 0.0f; break;
	}

	// Display the value with proper formatting
	if (display_value >= 10.000) {
		myOLED_float(0, 20, display_value);
	} else {
		myOLED_float(7, 20, display_value);
		myOLED_int(0, 20, 0);  // Print "0" at the first location
	}

    // Clear previous cursor position by redrawing the entire line
    ssd1306_SetCursor(0, 30);
    ssd1306_WriteString("        ", Font_7x10, White);  // Assuming 7 characters wide space to clear

    // Draw cursor under the digit
    int cursor_x = digit_position * 7;  // Assuming 7 pixels width per character
    if(digit_position == MAX_DIGITS){
    	myOLED_char(40, 50, "^");  // Draw the cursor under "RETURN" text
    }else{
    	myOLED_char(cursor_x, 30, "^");  	// Draw the cursor
    	myOLED_char(40, 50, " ");			// Clears the cursor under "RETURN" label
    }

    // Refresh the display after updating
    ssd1306_UpdateScreen();
}



// Update Encoder State
void update_encoder_state() {
    int new_rot_pos = rot_cnt;
    static int old_rot_pos = 0;

    if (new_rot_pos > old_rot_pos) {
        if (current_state == PARAMETER_SETTING) {
            if (adjusting_digit) {
                update_parameter_value(1); // Increment digit
            } else {
                digit_position++;
            }
        } else {
            cursor_position++;
        }
    } else if (new_rot_pos < old_rot_pos) {
        if (current_state == PARAMETER_SETTING) {
            if (adjusting_digit) {
                update_parameter_value(-1); // Decrement digit
            } else {
                digit_position--;
            }
        } else {
            cursor_position--;
        }
    }
    old_rot_pos = new_rot_pos;
    put_parameter_limit();		// put limit on parameter values based on mode
//    set_parameter_to_mode();	// set parameter to mode
    // putting limits
    if (cursor_position < 0) cursor_position = 0;
    if (current_state == HOME_SCREEN && cursor_position > 3) cursor_position = 3;
    if (current_state == MODE_SELECTION && cursor_position > 3) cursor_position = 3;
    if (current_state == PARAMETER_SETTING && digit_position > MAX_DIGITS) digit_position = MAX_DIGITS;
    if (digit_position < 0) digit_position = 0;
}

// Update parameter value
void update_parameter_value(int direction) {
    // Convert the whole value to an integer, treating it as 00.000 (in this case, a 5-digit number)
    int full_value = (int)(param_value * 1000);

    // Determine the position and multiplier
    int multiplier = 1;

    switch (digit_position) {
        case 0:
            multiplier = 10000;  // Corresponds to the tens digit of the integer part
            break;
        case 1:
            multiplier = 1000;   // Corresponds to the units digit of the integer part
            break;
        case 2:
            // Do nothing as this is the decimal point
            return;
        case 3:
            multiplier = 100;    // Corresponds to the first digit after the decimal point
            break;
        case 4:
            multiplier = 10;     // Corresponds to the second digit after the decimal point
            break;
        case 5:
            multiplier = 1;      // Corresponds to the third digit after the decimal point
            break;
    }

    // Store the original integer part before updating
    int original_integer_part = full_value / 1000;

    // Update the selected digit
    full_value += direction * multiplier;

    // Wrap the relevant digit only
    if (digit_position <= 1) {  // If modifying the integer part
        if (full_value < 0) {
            full_value += 100000;  // Wrap within the range of 00.000 to 99.999
        } else if (full_value >= 100000) {
            full_value -= 100000;
        }
    } else {  // If modifying the fractional part
        int fractional_int = full_value % 1000;
        if (fractional_int < 0) {
            fractional_int += 1000;  // Wrap within the range of 000 to 999
        } else if (fractional_int >= 1000) {
            fractional_int -= 1000;
        }
        full_value = (original_integer_part * 1000) + fractional_int;
    }

    // Convert back to floating-point value
    param_value = full_value / 1000.0;

    // Debug: Print the updated param_value
    printf("Updated param_value: %05d.%03d\n\r", full_value / 1000, full_value % 1000);
}

// Put limit on parameter value
void put_parameter_limit(){
	// RESET the param_value if mode is changed
	if(mode_index_last != mode_index){
		param_value = 0.0;
		mode_index_last = mode_index;
	}

	// check the MIN and MAX value of each mode to set limits on the value.
	if(current_state == 2){
		switch(mode_index) {
			case 0:
				if(param_value >= MAX_CC_VALUE){
					param_value = MAX_CC_VALUE;
				}else if(param_value <= MIN_CC_VALUE){
					param_value = MIN_CC_VALUE;
				}
				param_mode.current = param_value;
				break;
			case 1:
				if(param_value >= MAX_CV_VALUE){
					param_value = MAX_CV_VALUE;
				}else if(param_value <= MIN_CV_VALUE){
					param_value = MIN_CV_VALUE;
				}
				param_mode.voltage =  param_value;
				break;
			case 2:
				if(param_value >= MAX_CP_VALUE){
					param_value = MAX_CP_VALUE;
				}else if(param_value <= MIN_CP_VALUE){
					param_value = MIN_CP_VALUE;
				}
				param_mode.power = param_value;
				break;
			case 3:
				if(param_value >= MAX_CR_VALUE){
					param_value = MAX_CR_VALUE;
				}else if(param_value <= MIN_CR_VALUE){
					param_value = MIN_CR_VALUE;
				}
				param_mode.resistance = param_value;
				break;
			default:
				break;
		}
	}
}



void handle_button_press() {
    if (rot_sw_state) {
    	switch (current_state) {
			case HOME_SCREEN:
				if (cursor_position == 0) {
					current_state = MODE_SELECTION;			// GoTo MODE SELECTION PAGE
				} else if (cursor_position == 1){
					output_on_flag = true;
					// Handle "TURN ON" functionality TODO
				} else if (cursor_position == 2){			// Reset everything
					current_state = HOME_SCREEN;			// not necessary to reset current_state
					cursor_position = 0;
					mode_index = -1;
					param_value = 0.0;
					output_on_flag = false;
				} else if (cursor_position == 3){
					__NOP();								// TODO
				}
				break;
			case MODE_SELECTION:
				current_state = PARAMETER_SETTING;
				mode_index = cursor_position;
				cursor_position = 0;
				adjusting_digit = false;
				break;
			case PARAMETER_SETTING:
				if (adjusting_digit) {
					adjusting_digit = false; // Stop adjusting the digit
				} else {
					adjusting_digit = true; // Start adjusting the digit
				}
				if (digit_position == MAX_DIGITS) {
					current_state = RETURN_TO_HOME;
					digit_position = 0;
				}
				break;
			default:
				break;
        }
        rot_sw_state = false; // Reset button state
    }
}


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
