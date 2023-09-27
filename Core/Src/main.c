/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include "ili9341.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 3D point structure */
typedef struct {
	float x;
	float y;
	float z;
} Point_xyz;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WIDTH ILI9341_WIDTH
#define HEIGHT ILI9341_HEIGHT

/* Calculated points are multiplied by this value */
#define SCALE_VAL 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Starting points */
const Point_xyz point1 = {-1.0f, -1.0f, 1.0f};
const Point_xyz point2 = {1.0f, -1.0f, 1.0f};
const Point_xyz point3 = {1.0f, 1.0f, 1.0f};
const Point_xyz point4 = {-1.0f, 1.0f, 1.0f};
const Point_xyz point5 = {-1.0f, -1.0f, -1.0f};
const Point_xyz point6 = {1.0f, -1.0f, -1.0f};
const Point_xyz point7 = {1.0f, 1.0f, -1.0f};
const Point_xyz point8 = {-1.0f, 1.0f, -1.0f};

/* Array of base (non rotated) cube points */
const Point_xyz base_cube[8] = {point1, point2, point3, point4, point5, point6, point7, point8};

/* Array of rotated cube points */
Point_xyz rotated_cube[8];

float projection_matrix[3][3] = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 0.0f}
};

/* Displayed image */
uint8_t image[320][240];

/* Value of angle in degrees */
uint16_t angle = 0;

/* Value of angle in radians */
float rad = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void scale_cube(Point_xyz *cube);
void project_cube(Point_xyz *cube);
void rotate_cube_x(float rad, Point_xyz *rotated_cube, Point_xyz const *cube);
void rotate_cube_y(float rad, Point_xyz *rotated_cube, Point_xyz const *cube);
void rotate_cube_z(float rad, Point_xyz *rotated_cube, Point_xyz const *cube);
void draw_line(uint8_t pixel_x, uint8_t pixel_y, uint8_t dest_x, uint8_t dest_y);
void clear_display(void);
void display_cube(void);
inline float to_radians(float degrees);
Point_xyz dot_matrix(float const matrix[3][3], Point_xyz point);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* At first rotated cube should have base cube values */
  memcpy(&rotated_cube, &base_cube, sizeof(base_cube));

  clear_display();

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Display initialize */
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Rotate cube by all angles */
	  rotate_cube_x(rad, rotated_cube, base_cube);
	  rotate_cube_y(rad, rotated_cube, rotated_cube);
	  rotate_cube_z(rad, rotated_cube, rotated_cube);

	  /* Scale cube by scale value */
	  scale_cube(rotated_cube);

	  project_cube(rotated_cube);

	  display_cube();

	  angle++;

	  if (angle > 359) {
		  angle = 0;
	  }

	  /* Convert degrees to radians */
	  rad = to_radians(angle);

	  /* Clear display */
	  clear_display();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Scale points of cube by constant value to proper display
  * @param  cube: Pointer to array of cube's 3D points
  * @retval None
  */
void scale_cube(Point_xyz *cube) {

	for (uint8_t i = 0; i < 8; i++) {
		cube[i].x = cube[i].x * SCALE_VAL + WIDTH / 2;
		cube[i].y = cube[i].y * SCALE_VAL + HEIGHT / 2;
		cube[i].z = cube[i].z * SCALE_VAL + HEIGHT / 2;
	}
}

/**
  * @brief  Prepare cube to display it on 2D plane
  * @param  cube: Pointer to array of cube's 3D points
  * @retval None
  */
void project_cube(Point_xyz *cube) {

	for (uint8_t i = 0; i < 8; i++) {
		cube[i] = dot_matrix(projection_matrix, cube[i]);
	}
}

/**
  * @brief       Rotate cube by angle in the X axis
  * @param[in]   angle: Angle through which the rotation is performed
  * @param[out]  rotated_cube: Pointer to rotated cube
  * @param[in]   cube: Pointer to cube before rotation
  * @retval      None
  */
void rotate_cube_x(float angle, Point_xyz *rotated_cube, Point_xyz const *cube) {

	float const matrix[3][3] = {
			{1.0f, 0.0f, 0.0f},
			{0.0f, cosf(rad), -sinf(rad)},
			{0.0f, sinf(rad) , cosf(rad)}
	};

	for (uint8_t i = 0; i < 8; i++) {
		rotated_cube[i] = dot_matrix(matrix, cube[i]);
	}

}

/**
  * @brief       Rotate cube by angle in the Y axis
  * @param[in]   angle: Angle through which the rotation is performed
  * @param[out]  rotated_cube: Pointer to rotated cube
  * @param[in]   cube: Pointer to cube before rotation
  * @retval      None
  */
void rotate_cube_y(float angle, Point_xyz *rotated_cube, Point_xyz const *cube) {

	float const matrix[3][3] = {
			{cosf(rad), 0.0f, sinf(rad)},
			{0.0f, 1.0f, 0.0f},
			{-sinf(rad), 0.0f, cosf(rad)}
	};

	for (uint8_t i = 0; i < 8; i++) {
		rotated_cube[i] = dot_matrix(matrix, cube[i]);
	}

}

/**
  * @brief  Draw lines on display between two 2D points
  * @param  pixel_x: Start value on the X axis
  * @param  pixel_y: Start value on the Y axis
  * @param  dest_x: End value on the X axis
  * @param  dest_y: End value on the Y axis
  * @retval None
  */
void draw_line(uint8_t pixel_x, uint8_t pixel_y, uint8_t dest_x, uint8_t dest_y) {
	/* Drawing is performed by calculating equation of a line passing through two points */

	float a = 0.0f, b = 0.0f;

	if (pixel_x != dest_x) {

		a = (float)(dest_y - pixel_y) / (float)(dest_x - pixel_x);
		b = (float)dest_y - (float)dest_x * a;

		while (!(pixel_y == dest_y && pixel_x == dest_x)) {

			pixel_y = round(pixel_x * a + b);

			if (image[pixel_x][pixel_y] == 0) {
				ILI9341_DrawPixel(pixel_x, pixel_y, ILI9341_WHITE);
			}

			image[pixel_x][pixel_y] = 1;

			if (pixel_x < dest_x) {
				pixel_x++;
			} else if (pixel_x > dest_x) {
				pixel_x--;
			}

		}

	} else if (pixel_y != dest_y) {

		while (!(pixel_y == dest_y && pixel_x == dest_x)) {

			if (image[pixel_x][pixel_y] == 0) {
				ILI9341_DrawPixel(pixel_x, pixel_y, ILI9341_WHITE);
			}

			image[pixel_x][pixel_y] = 1;

			if (pixel_y < dest_y) {
				pixel_y++;
			} else if (pixel_y > dest_y) {
				pixel_y--;
			}
		}
	}
}

/**
  * @brief       Rotate cube by angle in the Z axis
  * @param[in]   angle: Angle through which the rotation is performed
  * @param[out]  rotated_cube: Pointer to rotated cube
  * @param[in]   cube: Pointer to cube before rotation
  * @retval      None
  */
void rotate_cube_z(float angle, Point_xyz *rotated_cube, Point_xyz const *cube) {

	float const matrix[3][3] = {
			{cosf(rad), -sinf(rad), 0.0f},
			{sinf(rad), cosf(rad), 0.0f},
			{0.0f, 0.0f, 1.0f}
	};

	for (uint8_t i = 0; i < 8; i++) {
		rotated_cube[i] = dot_matrix(matrix, cube[i]);
	}

}

/**
  * @brief  Display cube by drawing line between points
  * @retval None
  */
void display_cube() {

	uint8_t dest_x, dest_y, pixel_x, pixel_y;

	for (uint8_t i = 0; i < 8; i++) {
		pixel_x = (uint8_t)rotated_cube[i].x;
		pixel_y = (uint8_t)rotated_cube[i].y;

		if (i < 4) {
			dest_x = (uint8_t)rotated_cube[i + 4].x;
			dest_y = (uint8_t)rotated_cube[i + 4].y;

			draw_line(pixel_x, pixel_y, dest_x, dest_y);
		}

		if (i == 3 || i == 7) {
			dest_x = (uint8_t)rotated_cube[i - 3].x;
			dest_y = (uint8_t)rotated_cube[i - 3].y;
		} else {
			dest_x = (uint8_t)rotated_cube[i + 1].x;
			dest_y = (uint8_t)rotated_cube[i + 1].y;
		}

		draw_line(pixel_x, pixel_y, dest_x, dest_y);
	}
}

/**
  * @brief  Clear display and image buffer
  * @retval None
  */
void clear_display() {

	for (uint16_t i = 0; i < ILI9341_WIDTH; i++) {
		for (uint8_t j = 0; j < ILI9341_HEIGHT; j++) {
			if (image[i][j] == 1) {
				ILI9341_DrawPixel(i, j, ILI9341_BLACK);
				image[i][j] = 0;
			}
		}
	}
}

/**
  * @brief  Convert angle in degrees to angle in radians
  * @param  degrees: Angle in degrees
  * @retval Angle in radians
  */
float to_radians(float degrees) {
    return degrees * M_PI / 180.0f;
}

/**
  * @brief  Performs matrix multiplication by a point
  * @param  matrix: Matrix that will be multiplied
  * @param  point: Point that will be multiplied
  * @retval Result of matrix multiplication
  */
Point_xyz dot_matrix(float const matrix[3][3], Point_xyz point) {

	Point_xyz new_point = {0.0f, 0.0f, 0.0f};

	float point_array[3] = {point.x, point.y, point.z};

	for (uint8_t i = 0; i < 3; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			switch(i) {
				case 0:
					new_point.x += matrix[i][j] * point_array[j];
					break;
				case 1:
					new_point.y += matrix[i][j] * point_array[j];
					break;
				case 2:
					new_point.z += matrix[i][j] * point_array[j];
					break;
				default:
					break;
			}
		}
	}

	return new_point;
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
