//
// Created by ���ο� on 25-4-9.
//

#ifndef CTRBOARD_H7_ALL_USART_TASK_H
#define CTRBOARD_H7_ALL_USART_TASK_H

void usart_rx_semaphore_init(void);
void usart_tx_semaphore_init(void);
void USART1_DebugPrintf(const char *format, ...);

#endif //CTRBOARD_H7_ALL_USART_TASK_H
