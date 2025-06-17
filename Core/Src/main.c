#include "stm32f1xx.h"
#include <stdio.h>

// === Định nghĩa chân ===
#define DHT22_PIN   9   // PB9
#define RELAY_PIN   10  // PB10
#define MODE_BTN    8   // PA8
#define FAN_BTN     5   // PB5
#define UART_TX     9   // PA9 (USART1 TX)

// === Prototype ===
void SystemInit72MHz(void);
void uart_init(void);
void uart_send(char *s);
void timer_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void gpio_input_pa(uint8_t pin);
void gpio_input_pb(uint8_t pin);
void gpio_output_pb(uint8_t pin);
void dht22_output_mode(void);
uint8_t dht22_read(float *temperature);

// === Biến toàn cục ===
uint8_t mode = 0;          // 0 = AUTO, 1 = MANUAL
uint8_t fan_manual = 0;    // Trạng thái quạt khi MANUAL

// === UART ===
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

// === Clock 72MHz ===
void SystemInit72MHz(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

// === Delay ===
void timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint32_t us) {
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) delay_us(1000);
}

// === GPIO ===
void gpio_input_pa(uint8_t pin) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    if (pin < 8) {
        GPIOA->CRL &= ~(0xF << (pin * 4));
        GPIOA->CRL |= (0x4 << (pin * 4)); // Floating input
    } else {
        GPIOA->CRH &= ~(0xF << ((pin - 8) * 4));
        GPIOA->CRH |= (0x4 << ((pin - 8) * 4));
    }
}

void gpio_input_pb(uint8_t pin) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    if (pin < 8) {
        GPIOB->CRL &= ~(0xF << (pin * 4));
        GPIOB->CRL |= (0x4 << (pin * 4)); // Floating input
    } else {
        GPIOB->CRH &= ~(0xF << ((pin - 8) * 4));
        GPIOB->CRH |= (0x4 << ((pin - 8) * 4));
    }
}

void gpio_output_pb(uint8_t pin) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    if (pin < 8) {
        GPIOB->CRL &= ~(0xF << (pin * 4));
        GPIOB->CRL |= (0x3 << (pin * 4)); // Output push-pull 50MHz
    } else {
        GPIOB->CRH &= ~(0xF << ((pin - 8) * 4));
        GPIOB->CRH |= (0x3 << ((pin - 8) * 4)); // Output push-pull 50MHz
    }
}

void dht22_output_mode(void) {
    GPIOB->CRH &= ~(0xF << 4); // PB9
    GPIOB->CRH |= (0x3 << 4);  // Output 50MHz push-pull
}

// === DHT22 ===
uint8_t dht22_read(float *temperature) {
    uint8_t data[5] = {0};
    uint32_t timeout;

    dht22_output_mode();
    GPIOB->ODR &= ~(1 << DHT22_PIN);
    delay_ms(1);
    GPIOB->ODR |= (1 << DHT22_PIN);
    delay_us(30);
    gpio_input_pb(DHT22_PIN);

    timeout = 10000;
    while (!(GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
    if (!timeout) return 1;
    timeout = 10000;
    while ((GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
    if (!timeout) return 1;

    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            timeout = 10000;
            while (!(GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
            delay_us(40);
            data[i] <<= 1;
            if (GPIOB->IDR & (1 << DHT22_PIN)) data[i] |= 1;
            timeout = 10000;
            while ((GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
        }
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return 1;

    int16_t temp_raw = ((data[2] & 0x7F) << 8) | data[3];
    if (data[2] & 0x80) temp_raw = -temp_raw;
    *temperature = temp_raw * 0.1f;

    return 0;
}

// === Main ===
int main(void) {
    SystemInit72MHz();
    uart_init();
    timer_init();

    gpio_input_pb(DHT22_PIN);
    gpio_input_pa(MODE_BTN);
    gpio_input_pb(FAN_BTN);
    gpio_output_pb(RELAY_PIN);

    uart_send("Khoi dong OK (Fan Control)\r\n");

    float temp = 0;
    char buffer[64];
    uint8_t btn_last = 1;
    uint8_t fanbtn_last = 1;

    while (1) {
        // Nút chọn chế độ
        uint8_t btn = (GPIOA->IDR & (1 << MODE_BTN)) ? 1 : 0;
        if (btn_last == 1 && btn == 0) {
            mode ^= 1; // Toggle AUTO/MANUAL
            if (mode) {
                uart_send("Che do: MANUAL\r\n");
            } else {
                uart_send("Che do: AUTO\r\n");
                fan_manual = 0; // Tắt quạt khi về AUTO
            }
        }
        btn_last = btn;

        // Nút bật/tắt quạt khi MANUAL
        uint8_t fanbtn = (GPIOB->IDR & (1 << FAN_BTN)) ? 1 : 0;
        if (fanbtn_last == 1 && fanbtn == 0 && mode == 1) {
            fan_manual ^= 1; // Toggle fan
            if (fan_manual) uart_send("Quat: BAT (MANUAL)\r\n");
            else uart_send("Quat: TAT (MANUAL)\r\n");
        }
        fanbtn_last = fanbtn;

        // Đọc nhiệt độ
        if (dht22_read(&temp) == 0) {
            sprintf(buffer, "Nhiet do: %.1f C\r\n", temp);
            uart_send(buffer);
        } else {
            uart_send("Doc DHT22 that bai!\r\n");
        }

        // Điều khiển quạt
        if (mode == 0) { // AUTO
            if (temp > 34.0f) {
                GPIOB->ODR |= (1 << RELAY_PIN);
                uart_send("Quat: BAT (AUTO)\r\n");
            } else {
                GPIOB->ODR &= ~(1 << RELAY_PIN);
                uart_send("Quat: TAT (AUTO)\r\n");
            }
        } else { // MANUAL
            if (fan_manual) {
                GPIOB->ODR |= (1 << RELAY_PIN);
            } else {
                GPIOB->ODR &= ~(1 << RELAY_PIN);
            }
        }

        delay_ms(2000);
    }
}
