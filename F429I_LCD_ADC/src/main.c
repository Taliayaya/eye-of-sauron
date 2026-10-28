#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "../Components/ili9341/ili9341.h"
#include "devices.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct {
    uint16_t left;
    uint16_t right;
} light_t;

static QueueHandle_t light_queue;

/* ******************************************************
-      Global Context
- ******************************************************/

#define LIGHT_THRESHOLD 150
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

int cycle = 0;
int nbTIM2 = 0;

/* Servo pulse state (in timer ticks / us) */
volatile int ServoON = 1500;  // Default 1.5ms (90 degrees center)
volatile int servo_curr_angle = 90;

volatile uint16_t adc_buffer[2];
ADC_HandleTypeDef hadc1;
TaskHandle_t xDisplayHandle = NULL;

/* ******************************************************
-      Servo Control Helper
- ******************************************************/

void set_servo_angle(int angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    servo_curr_angle = angle;

    ServoON = 500 + ((uint32_t)angle * 2000) / 180;
}

uint16_t read_adc_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&gAdcHandle, &sConfig) != HAL_OK)
    {
        return 0;
    }

    HAL_ADC_Start(&gAdcHandle);

    if (HAL_ADC_PollForConversion(&gAdcHandle, 10) == HAL_OK)
    {
        uint16_t value = (uint16_t)HAL_ADC_GetValue(&gAdcHandle);
        HAL_ADC_Stop(&gAdcHandle);
        return value;
    }

    HAL_ADC_Stop(&gAdcHandle);
    return 0;
}

/* ******************************************************
-      Tasks
- ******************************************************/

void task_servo(void *p)
{
    TickType_t last = xTaskGetTickCount();
    light_t l;

    set_servo_angle(90);

    for (;;)
    {
        if (xQueuePeek(light_queue, &l, 0) == pdTRUE)
        {
            int32_t diff = (int32_t)l.left - (int32_t)l.right;

            if (diff > LIGHT_THRESHOLD)
                set_servo_angle(servo_curr_angle + 2);
            else if (diff < -LIGHT_THRESHOLD)
                set_servo_angle(servo_curr_angle - 2);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

void task_sensor(void *p)
{
    TickType_t last = xTaskGetTickCount();
    light_t l;

    for (;;)
    {
        l.left  = read_adc_channel(ADC_CHANNEL_11);
        l.right = read_adc_channel(ADC_CHANNEL_5);
        xQueueOverwrite(light_queue, &l);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    }
}

void task_display(void *p)
{
    char msg[20] = {0};
    bool toggled = false;
    bool prev_pressed = false;
    uint32_t back_color = LCD_COLOR_LIGHTGRAY;
    uint32_t text_color = LCD_COLOR_BLACK;
    light_t l = {0};

    LCDdisplayInit(COLORS_NORMAL);

    for (;;)
    {
        bool pressed = (BSP_PB_GetState(BUTTON_KEY) != 0);
        if (pressed && !prev_pressed)
        {
            toggled = !toggled;
            BSP_LED_Toggle(LED4);
            LCDdisplayInit(toggled ? COLORS_REVERSE : COLORS_NORMAL);
            back_color = toggled ? LCD_COLOR_DARKGRAY : LCD_COLOR_LIGHTGRAY;
            text_color = toggled ? LCD_COLOR_WHITE : LCD_COLOR_BLACK;
        }
        prev_pressed = pressed;

        xQueuePeek(light_queue, &l, 0);

        BSP_LCD_SetBackColor(back_color);

        snprintf(msg, sizeof(msg), "Right: %4u", l.right);
        LCDdisplayText(5, text_color, msg);

        snprintf(msg, sizeof(msg), "Left:  %4u", l.left);
        LCDdisplayText(8, LCD_COLOR_GREEN, msg);

        snprintf(msg, sizeof(msg), "%3d deg", servo_curr_angle);
        LCDdisplayText(10, LCD_COLOR_BLUE, msg);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


int main(void)
{
    setup();

    light_queue = xQueueCreate(1, sizeof(light_t));
    configASSERT(light_queue != NULL);
    xTaskCreate(task_sensor,  "sensor",  256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(task_servo,   "servo",   256, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(task_display, "display", 512, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (1) {}
}

/* ******************************************************
-      Interrupt Handlers & Callbacks
- ******************************************************/

void TIM2_IRQHandler(void)
{
    // Frame Start (Every 20ms) => Drive PG13 HIGH
    if (__HAL_TIM_GET_IT_SOURCE(&gTimHandle, TIM_IT_UPDATE) != RESET &&
        __HAL_TIM_GET_FLAG(&gTimHandle, TIM_FLAG_UPDATE) != RESET)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&gTimHandle, TIM_CHANNEL_1, ServoON);
        __HAL_TIM_ENABLE_IT(&gTimHandle, TIM_IT_CC1);
        __HAL_TIM_CLEAR_IT(&gTimHandle, TIM_IT_UPDATE);
    }

    // Pulse Duration Complete  => Drive PG13 LOW
    if (__HAL_TIM_GET_IT_SOURCE(&gTimHandle, TIM_IT_CC1) != RESET &&
        __HAL_TIM_GET_FLAG(&gTimHandle, TIM_FLAG_CC1) != RESET)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
        __HAL_TIM_DISABLE_IT(&gTimHandle, TIM_IT_CC1);
        __HAL_TIM_CLEAR_IT(&gTimHandle, TIM_IT_CC1);
    }
}

void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

/* ******************************************************
-      Hardware Init
- ******************************************************/


void setup(void)
{
    HAL_Init();
    SystemClock_Config();
    initLCD();
    initGPIO();
    initTimer();
    initADC();
}
