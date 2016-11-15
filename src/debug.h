/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _DEBUG_H_    /* Guard against multiple inclusion */
#define _DEBUG_H_

#define DLOC_INITIALIZE                     0x01
#define DLOC_TIMER_ISR_ENTER                0x02
#define DLOC_TMR_ISR_LEAVE                  0x03
#define DLOC_ISR_BEFORE_MSG_SEND            0x04
#define DLOC_ISR_AFTER_MSG_SEND             0x05
#define DLOC_ISR_WAIT_MSG_RECEIVED          0x06
#define DLOC_ISR_MSG_RECEIVED               0x07
#define DLOC_QUEUE_ERROR                    0x08
#define DLOC_UART_ERROR                     0x09

#ifdef __cplusplus
extern "C" {
#endif

#include "system_definitions.h"
/**
 * Use this routine to output values (such as the characters of your team
 *  name) to 8 PIC i/o lines
 * @param output value
 */
void dbgOutputVal(unsigned char outVal);
void dbgOutputLoc(unsigned char outVal);
void errorHandling();

void turnLED_On();
void turnLED_Off();

void dbgLEDArray(unsigned char outVal);

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
