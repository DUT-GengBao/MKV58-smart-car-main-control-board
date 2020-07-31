/*
 * tools.h
 *
 *  Created on: Feb 17, 2018
 *      Author: ZBT
 */

#ifndef CONTROL_TOOLS_H_
#define CONTROL_TOOLS_H_
void UART_eDMA_init(void);
void UART_eDMA_datasend(void);
void Buz_ring_ms(int ms);
void Wait_click(void);
void Buz_handler(void);
#endif /* CONTROL_TOOLS_H_ */
