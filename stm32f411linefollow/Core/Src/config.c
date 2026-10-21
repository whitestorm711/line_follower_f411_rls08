
/*
 * config.c
 *
 * Created on: Oct 20, 2026
 * Author: Arnav Patil
 */

#include "config.h"
#include "main.h"
#include <stdlib.h>
#include <stdint.h>

void initconfig(void)
{
    // =====================================================
    // 1. ENABLE GPIO CLOCKS
    // =====================================================

    // GPIOA clock
    RCC->AHB1ENR |= (1U << 0U);

    // GPIOB clock
    RCC->AHB1ENR |= (1U << 1U);


    // =====================================================
    // 2. RLS-08 SENSOR INPUTS
    // PA1-PA7 and PB0
    // =====================================================

    // PA1 through PA7: input mode (00)
    GPIOA->MODER &= ~(
        (3U << 2U)  |
        (3U << 4U)  |
        (3U << 6U)  |
        (3U << 8U)  |
        (3U << 10U) |
        (3U << 12U) |
        (3U << 14U)
    );

    // PB0: input mode (00)
    GPIOB->MODER &= ~(3U << 0U);


    // No pull-up or pull-down on sensor inputs
    GPIOA->PUPDR &= ~(
        (3U << 2U)  |
        (3U << 4U)  |
        (3U << 6U)  |
        (3U << 8U)  |
        (3U << 10U) |
        (3U << 12U) |
        (3U << 14U)
    );

    GPIOB->PUPDR &= ~(3U << 0U);


    // =====================================================
    // 3. ONBOARD KEY BUTTON: PA0
    // Active LOW, with internal pull-up
    // =====================================================

    // PA0: input mode
    GPIOA->MODER &= ~(3U << 0U);

    // PA0: pull-up (01)
    GPIOA->PUPDR &= ~(3U << 0U);
    GPIOA->PUPDR |=  (1U << 0U);


    // =====================================================
    // 4. MOTOR DIRECTION PINS
    // PA10 = Left direction
    // PB15 = Right direction
    // =====================================================

    // Clear PA10 mode bits, then output mode (01)
    GPIOA->MODER &= ~(3U << 20U);
    GPIOA->MODER |=  (1U << 20U);

    // Clear PB15 mode bits, then output mode (01)
    GPIOB->MODER &= ~(3U << 30U);
    GPIOB->MODER |=  (1U << 30U);

    // Initial direction outputs LOW
    GPIOA->BSRR = (1U << (10U + 16U));
    GPIOB->BSRR = (1U << (15U + 16U));


    // =====================================================
    // 5. MOTOR PWM PINS
    // PA8 = TIM1_CH1 (left)
    // PA9 = TIM1_CH2 (right)
    // Alternate function AF1
    // =====================================================

    // Clear PA8 and PA9 mode bits
    GPIOA->MODER &= ~(
        (3U << 16U) |
        (3U << 18U)
    );

    // PA8 and PA9: alternate function mode (10)
    GPIOA->MODER |= (
        (2U << 16U) |
        (2U << 18U)
    );

    // Clear AF selection for PA8 and PA9
    GPIOA->AFR[1] &= ~(
        (15U << 0U) |
        (15U << 4U)
    );

    // Select AF1: TIM1_CH1 and TIM1_CH2
    GPIOA->AFR[1] |= (
        (1U << 0U) |
        (1U << 4U)
    );


    // =====================================================
    // 6. ENABLE TIM1 CLOCK (APB2)
    // =====================================================

    RCC->APB2ENR |= (1U << 0U);


    // =====================================================
    // 7. CONFIGURE TIM1 PWM
    // =====================================================

    // Stop timer during configuration
    TIM1->CR1 = 0U;

    TIM1->PSC = 3U;
    TIM1->ARR = 1049U;

    // Enable ARR preload
    TIM1->CR1 |= (1U << 7U);

    // CH1 and CH2: PWM mode 1 with compare preload
    TIM1->CCMR1 = 0U;

    TIM1->CCMR1 |=
        (6U << 4U)  |   // CH1 PWM mode 1
        (1U << 3U)  |   // CH1 preload
        (6U << 12U) |   // CH2 PWM mode 1
        (1U << 11U);    // CH2 preload

    // Enable CH1 and CH2 outputs
    TIM1->CCER &= ~(
        (1U << 0U) |
        (1U << 4U)
    );

    TIM1->CCER |= (
        (1U << 0U) |
        (1U << 4U)
    );

    // Initial motor duty cycles: 0%
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;

    // Main output en able (required for TIM1 only)
    TIM1->BDTR |= (1U << 15U);

    // Generate update event to load PSC and ARR
    TIM1->EGR |= TIM_EGR_UG;

    // Start TIM1
    TIM1->CR1 |= (1U << 0U);
}
