//#include "stm32f1xx.h"
//#include "uart_debug.h"
//#include <stdio.h>
//
//// ==== Định nghĩa chân ====
//#define DHT22_PIN     9     // PB9
//#define RELAY_PIN     8     // PB8 (điều khiển relay)
//#define UART_TX_PIN   9     // PA9
//
//// ==== Khai báo hàm ====
//void SystemInit72MHz(void);
//void timer_init(void);
//void delay_us(uint32_t us);
//void delay_ms(uint32_t ms);
//void dht22_output_mode(void);
//void dht22_input_mode(void);
//uint8_t dht22_read(float *temperature, float *humidity);
//
//// ==== System Clock 72MHz ====
//void SystemInit72MHz(void) {
//    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
//    RCC->CR |= RCC_CR_HSEON;
//    while (!(RCC->CR & RCC_CR_HSERDY));
//    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
//    RCC->CR |= RCC_CR_PLLON;
//    while (!(RCC->CR & RCC_CR_PLLRDY));
//    RCC->CFGR |= RCC_CFGR_SW_PLL;
//    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
//}
//
//// ==== Delay ====
//void timer_init(void) {
//    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//    TIM2->PSC = 72 - 1;
//    TIM2->ARR = 0xFFFF;
//    TIM2->CR1 |= TIM_CR1_CEN;
//}
//
//void delay_us(uint32_t us) {
//    TIM2->CNT = 0;
//    while (TIM2->CNT < us);
//}
//
//void delay_ms(uint32_t ms) {
//    while (ms--) delay_us(1000);
//}
//
//// ==== GPIO DHT22 ====
//void dht22_output_mode(void) {
//    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
//    GPIOB->CRH &= ~(0xF << 4);  // PB9
//    GPIOB->CRH |= (0x3 << 4);   // Output 50MHz push-pull
//}
//
//void dht22_input_mode(void) {
//    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
//    GPIOB->CRH &= ~(0xF << 4);  // PB9
//    GPIOB->CRH |= (0x4 << 4);   // Input floating
//}
//
//// ==== DHT22 ====
//uint8_t dht22_read(float *temperature, float *humidity) {
//    uint8_t data[5] = {0};
//    uint32_t timeout;
//
//    // Start signal ≥1.5ms
//    dht22_output_mode();
//    GPIOB->ODR &= ~(1 << DHT22_PIN);
//    delay_ms(2);
//    GPIOB->ODR |= (1 << DHT22_PIN);
//    delay_us(30);
//    dht22_input_mode();
//
//    // Wait response
//    timeout = 10000;
//    while (!(GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
//    if (!timeout) return 1;
//    timeout = 10000;
//    while ((GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
//    if (!timeout) return 1;
//
//    // Read 5 bytes
//    for (uint8_t i = 0; i < 5; i++) {
//        for (uint8_t j = 0; j < 8; j++) {
//            timeout = 10000;
//            while (!(GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
//            delay_us(40);
//            data[i] <<= 1;
//            if (GPIOB->IDR & (1 << DHT22_PIN)) data[i] |= 1;
//            timeout = 10000;
//            while ((GPIOB->IDR & (1 << DHT22_PIN)) && timeout--) delay_us(1);
//        }
//    }
//
//    // CRC
//    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return 1;
//
//    *humidity = ((data[0] << 8) | data[1]) * 0.1f;
//    int16_t temp_raw = ((data[2] & 0x7F) << 8) | data[3];
//    if (data[2] & 0x80) temp_raw = -temp_raw;
//    *temperature = temp_raw * 0.1f;
//
//    return 0;
//}
//
//// ==== MAIN ====
//int main(void) {
//    SystemInit72MHz();
//    uart_init();
//    timer_init();
//
//    // Khởi tạo chân Relay
//    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
//    GPIOB->CRH &= ~(0xF << 0);    // PB8
//    GPIOB->CRH |= (0x3 << 0);     // PB8 output push-pull
//    GPIOB->ODR &= ~(1 << RELAY_PIN); // Relay OFF ban đầu
//
//    uart_send("Khoi dong OK (DHT22 + Relay Fan)\r\n");
//
//    float temp = 0, humi = 0;
//    char buffer[64];
//
//    while (1) {
//        uint8_t dht_ok = dht22_read(&temp, &humi);
//
//        if (dht_ok == 0) {
//            sprintf(buffer, "Nhiet do: %.1f C, Do am: %.1f%%\r\n", temp, humi);
//            uart_send(buffer);
//
//            if (temp > 20.0f) {
//                uart_send("BAT QUAT (Relay ON)\r\n");
//                GPIOB->ODR |= (1 << RELAY_PIN);
//            } else if (temp < 17.0f) {
//                uart_send("TAT QUAT (Relay OFF)\r\n");
//                GPIOB->ODR &= ~(1 << RELAY_PIN);
//            }
//        } else {
//            uart_send("Doc DHT22 that bai!\r\n");
//        }
//
//        delay_ms(2000);
//    }
//}
