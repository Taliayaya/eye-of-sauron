#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "../Components/ili9341/ili9341.h"
#include "devices.h"
#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef enum { STATE_STOP, STATE_TRACK_ANIM, STATE_TRACK_DEBUG } state_t;
typedef enum { EVT_BTN, EVT_LIGHT } event_type_t;

typedef struct {
    event_type_t type;
    uint16_t left;
    uint16_t right;
} event_t;

typedef struct {
    state_t state;
    uint16_t left;
    uint16_t right;
    int angle;
} display_t;

QueueHandle_t event_queue;
QueueHandle_t display_queue;

/* ******************************************************
-      Global Context
- ******************************************************/

#define START_THRESHOLD 1000
#define LIGHT_THRESHOLD 100
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180


volatile int ServoON = 1500;  // Default 1.5ms
volatile int servo_curr_angle = 90;

volatile int adc_channel = 0;

/* ******************************************************
-      Servo Control Helper
- ******************************************************/

void set_servo_angle(int angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    servo_curr_angle = angle;

    ServoON = 500 + (angle * 2000) / 180;
}

void start_adc(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = adc_channel ? ADC_CHANNEL_5 : ADC_CHANNEL_11;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    HAL_ADC_ConfigChannel(&gAdcHandle, &sConfig);
    HAL_ADC_Start_IT(&gAdcHandle);
}

/* ******************************************************
-      Tasks
- ******************************************************/

void task_automate(void *p)
{
    state_t state = STATE_STOP;
    int left = 0;
    int right = 0;
    event_t evt;

    for (;;)
    {
        if (xQueueReceive(event_queue, &evt, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (evt.type == EVT_LIGHT)
            {
                left = evt.left;
                right = evt.right;
            }
            else if (state == STATE_TRACK_ANIM)
                state = STATE_TRACK_DEBUG;
            else
                state = STATE_TRACK_ANIM;
        }

        int diff = left - right;

        if (state == STATE_STOP && abs(diff) > START_THRESHOLD)
            state = STATE_TRACK_ANIM;

        if (state != STATE_STOP)
        {
            if (diff > LIGHT_THRESHOLD)
                set_servo_angle(servo_curr_angle + 2);
            else if (diff < -LIGHT_THRESHOLD)
                set_servo_angle(servo_curr_angle - 2);
        }

        display_t d = { state, left, right, servo_curr_angle };
        xQueueOverwrite(display_queue, &d);
    }
}

void task_servo(void *p)
{
    for (;;)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&gTimHandle, TIM_CHANNEL_1, (__HAL_TIM_GET_COUNTER(&gTimHandle) + ServoON) % 20000);
        __HAL_TIM_CLEAR_FLAG(&gTimHandle, TIM_FLAG_CC1);
        __HAL_TIM_ENABLE_IT(&gTimHandle, TIM_IT_CC1);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void task_display(void *p)
{
    char msg[20] = {0};
    display_t d;
    int shown = -1;
    int eye_x = 0;

    for (;;)
    {
        xQueueReceive(display_queue, &d, portMAX_DELAY);

        if (d.state != shown)
        {
            shown = d.state;
            LCDdisplayInit(COLORS_NORMAL);
            BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGRAY);

            if (d.state == STATE_STOP)
                LCDdisplayText(7, LCD_COLOR_BLACK, "zzz...");

            if (d.state == STATE_TRACK_ANIM)
            {
                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                BSP_LCD_DrawEllipse(120, 160, 105, 40);
            }
        }

        if (d.state == STATE_TRACK_ANIM)
        {
            // erase old pupil, draw new one
            BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
            BSP_LCD_FillCircle(eye_x, 160, 15);

            eye_x = 50 + d.angle * 140 / 180;
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            BSP_LCD_FillCircle(eye_x, 160, 15);
        }

        if (d.state == STATE_TRACK_DEBUG)
        {
            snprintf(msg, sizeof(msg), "Right: %4u", d.right);
            LCDdisplayText(5, LCD_COLOR_BLACK, msg);

            snprintf(msg, sizeof(msg), "Left:  %4u", d.left);
            LCDdisplayText(8, LCD_COLOR_GREEN, msg);

            snprintf(msg, sizeof(msg), "%3d deg", d.angle);
            LCDdisplayText(10, LCD_COLOR_BLUE, msg);
        }
    }
}


int main(void)
{
    setup();

    xTaskCreate(task_servo, "servo", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(task_automate, "automate", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(task_display, "display", 512, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (1) {}
}

/* ******************************************************
-      Interrupt Handlers & Callbacks
- ******************************************************/

void TIM2_IRQHandler(void)
{
    // Every 20ms
    if (__HAL_TIM_GET_IT_SOURCE(&gTimHandle, TIM_IT_UPDATE) != RESET &&
        __HAL_TIM_GET_FLAG(&gTimHandle, TIM_FLAG_UPDATE) != RESET)
    {
        start_adc();
        __HAL_TIM_CLEAR_IT(&gTimHandle, TIM_IT_UPDATE);
    }

    // Pulse Duration Complete
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
    static uint16_t light[2];
    uint16_t value = HAL_ADC_GetValue(&gAdcHandle);

    if (abs(value - light[adc_channel]) > LIGHT_THRESHOLD)
    {
        light[adc_channel] = value;
        event_t evt = { EVT_LIGHT, light[0], light[1] };
        xQueueSendFromISR(event_queue, &evt, NULL);
    }

    adc_channel = !adc_channel;
}

void EXTI0_IRQHandler(void)
{
    static TickType_t last_press = 0;
    TickType_t now = xTaskGetTickCountFromISR();

    __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);

    // debounce
    if (now - last_press > pdMS_TO_TICKS(200))
    {
        event_t evt = { EVT_BTN, 0, 0 };
        xQueueSendFromISR(event_queue, &evt, NULL);
    }
    last_press = now;
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
    // important!!
    event_queue = xQueueCreate(8, sizeof(event_t));
    display_queue = xQueueCreate(1, sizeof(display_t));
    initGPIO();
    initADC();
    initTimer();
}
