//#include "uart_debug.h"
//
//void uart_init(void) {
//    // APB2ENR : điều khiển các phần đc cấp xung clock
//    // bật các bit xung clock cho: uart, gpioa, afi(hỗ trợ chức năng ngoài cho các chân gpio)
//    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
//
//    // CRH = [PA15][PA14]...[PA9][PA8] (4 bit mỗi chân) -> 32 bit
//    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9); // phần PA9 trong CRH = 0000
//    GPIOA->CRH |= (0x0B << 4); // set mode PA9: TX = AF push-pull 50MHz
//
//    USART1->BRR = 72000000 / 9600; // tốc độ truyền data cho UART
//    //UE - USART Enable (cho phép USART hoạt động)
//    //TE – Transmitter Enable (cho phép truyền dữ liệu)
//    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
//}
//
//void uart_send(char *s) {
//    while (*s) {
//        // USART1->SR & USART_SR_TXE = 1 -> while(false) -> thoát khỏi while
//        // phần TXE của SR = 1 khi có thể ghi dữ liệu mới vào DR
//        // SR = 0000TXE000
//        // USART_SR_TXE = 00001000 (const) -> and với nhau lấy đc phần TXE của SR
//        while (!(USART1->SR & USART_SR_TXE));
//        USART1->DR = *s++;
//    }
//}
