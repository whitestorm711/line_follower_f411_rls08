/*
 * Cytron MDD10A, sign-magnitude mode:
 *   PA8  = TIM1_CH1 = PWM1 (left speed)     PA10 = DIR1 (left direction)
 *   PA9  = TIM1_CH2 = PWM2 (right speed)    PB15 = DIR2 (right direction)
 */

#include "motordrive.h"
#include "main.h"
#include <stdint.h>

#define LEFT_FWD_LEVEL    1U
#define RIGHT_FWD_LEVEL   1U

#define DIR_LEFT_PIN      10U   /* GPIOA */
#define DIR_RIGHT_PIN     15U   /* GPIOB */

uint16_t LF_GetPwmMax(void)
{
    return (uint16_t)TIM1->ARR;
}

void LF_SetMotorSpeed(uint16_t left, uint16_t right)
{
    const uint16_t top = LF_GetPwmMax();

    if (left  > top) left  = top;
    if (right > top) right = top;

    TIM1->CCR1 = left;    /* PA8: left motor PWM  */
    TIM1->CCR2 = right;   /* PA9: right motor PWM */
}

static void dir_pin_write(GPIO_TypeDef *port, uint8_t pin, uint8_t level)
{
    if (level) port->BSRR = (1U << pin);
    else       port->BSRR = (1U << (pin + 16U));
}

void LF_SetMotorDirection(uint8_t left_forward,
                          uint8_t right_forward)
{
    dir_pin_write(GPIOA, DIR_LEFT_PIN,
                  left_forward  ? LEFT_FWD_LEVEL  : (uint8_t)!LEFT_FWD_LEVEL);

    dir_pin_write(GPIOB, DIR_RIGHT_PIN,
                  right_forward ? RIGHT_FWD_LEVEL : (uint8_t)!RIGHT_FWD_LEVEL);
}
