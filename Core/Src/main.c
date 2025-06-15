#include "stm32f1xx.h"
#include <stdio.h>

// === CONFIG ===
#define DHT_PIN     9   // PB9
#define FAN_IN1     7   // PB7 -> L298N IN3
#define FAN_IN2     6   // PB6 -> L298N IN4

// === INIT ===
void SystemInit72MHz(void);
void uart_init(void);
void uart_send(char *s);
void timer_init(void);
void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
void fan_on(void);
void fan_off(void);

void gpio_output_pb(uint8_t pin);
void gpio_input_pb(uint8_t pin);
void dht_output_mode(void);
uint8_t dht22_read(float *temp);

// === SYSTEM ===
void SystemInit72MHz(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}

void uart_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (0x0B << 4); // PA9: AF push-pull
    USART1->BRR = 72000000 / 9600;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void uart_send(char *s) {
    while (*s) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *s++;
    }
}

// === TIMER ===
void timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint16_t us) {
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

void delay_ms(uint16_t ms) {
    while (ms--) delay_us(1000);
}

// === GPIO ===
void gpio_output_pb(uint8_t pin) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    if (pin >= 8) {
        GPIOB->CRH &= ~(0xF << ((pin - 8) * 4));
        GPIOB->CRH |= (0x3 << ((pin - 8) * 4));
    } else {
        GPIOB->CRL &= ~(0xF << (pin * 4));
        GPIOB->CRL |= (0x3 << (pin * 4));
    }
}

void gpio_input_pb(uint8_t pin) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    if (pin >= 8) {
        GPIOB->CRH &= ~(0xF << ((pin - 8) * 4));
        GPIOB->CRH |= (0x4 << ((pin - 8) * 4));
    } else {
        GPIOB->CRL &= ~(0xF << (pin * 4));
        GPIOB->CRL |= (0x4 << (pin * 4));
    }
}

void dht_output_mode(void) {
    GPIOB->CRH &= ~(0xF << 4);  // PB9
    GPIOB->CRH |= (0x3 << 4);   // Output push-pull
}

// === FAN ===
void fan_on(void) {
    GPIOB->ODR |= (1 << FAN_IN1);
    GPIOB->ODR &= ~(1 << FAN_IN2);
}

void fan_off(void) {
    GPIOB->ODR &= ~(1 << FAN_IN1);
    GPIOB->ODR &= ~(1 << FAN_IN2);
}

// === DHT22 ===
uint8_t dht22_read(float *temp) {
    uint8_t data[5] = {0};
    uint32_t timeout;

    // Start signal
    dht_output_mode();
    GPIOB->ODR &= ~(1 << DHT_PIN);
    delay_ms(2);  // ≥1ms
    GPIOB->ODR |= (1 << DHT_PIN);
    delay_us(30);
    gpio_input_pb(DHT_PIN);

    // Wait response
    timeout = 10000;
    while (!(GPIOB->IDR & (1 << DHT_PIN)) && timeout--) delay_us(1);
    if (!timeout) return 1;

    timeout = 10000;
    while ((GPIOB->IDR & (1 << DHT_PIN)) && timeout--) delay_us(1);
    if (!timeout) return 1;

    // Read 5 bytes
    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            timeout = 10000;
            while (!(GPIOB->IDR & (1 << DHT_PIN)) && timeout--) delay_us(1);

            delay_us(40);
            data[i] <<= 1;
            if (GPIOB->IDR & (1 << DHT_PIN)) data[i] |= 1;

            timeout = 10000;
            while ((GPIOB->IDR & (1 << DHT_PIN)) && timeout--) delay_us(1);
        }
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return 1;

    uint16_t raw_temp = (data[2] << 8) | data[3];
    *temp = raw_temp / 10.0;

    return 0;
}

// === MAIN ===
int main(void) {
    SystemInit72MHz();
    uart_init();
    timer_init();
    gpio_output_pb(FAN_IN1);
    gpio_output_pb(FAN_IN2);
    gpio_input_pb(DHT_PIN);

    uart_send("Khoi dong OK\n");

    float temp = 0;
    char buff[64];

    while (1) {
        if (dht22_read(&temp) == 0) {
            sprintf(buff, "Nhiet do: %.1f C\n", temp);
            uart_send(buff);

            if (temp >= 34) {
                uart_send("Trang thai quat: BAT\n");
                fan_on();
            } else {
                uart_send("Trang thai quat: TAT\n");
                fan_off();
            }
        } else {
            uart_send("Doc DHT22 that bai!\n");
            fan_off();
        }
        delay_ms(2000);
    }
}
