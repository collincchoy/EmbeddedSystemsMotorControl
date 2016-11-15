/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    overwatch_thread.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "overwatch_thread.h"
#include "motor_msq.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/
DRV_HANDLE uart_rx;
QueueHandle_t uart_rx_messageQueue;
OVERWATCH_THREAD_DATA overwatch_threadData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
/**
 * uart receive event handler
 * @param index
 *      sys module index
 */
void  APP_USARTReceiveEventHandler(const SYS_MODULE_INDEX index)
{
    // TODO: disabled event handler
  
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
        uint8_t rx_raw_message = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        //Location_message rx_loc_message = deserialize_LM(rx_raw_message)
        sendUartRxVal(rx_raw_message);       
        while (!uartRxSendToMsgQ(rx_raw_message));
    }
}

/*******************************************************************************
  Function:
    void OVERWATCH_THREAD_Initialize ( void )

  Remarks:
    See prototype in overwatch_thread.h.
 */

void OVERWATCH_THREAD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    overwatch_threadData.state = OVERWATCH_THREAD_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
        /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    uart_rx = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READ);
    if(uart_rx == DRV_HANDLE_INVALID) {
        dbgOutputVal(DLOC_UART_ERROR);
        errorHandling();
    }
    
    uart_rx_messageQueue = xQueueCreate(10, sizeof(char));
    DRV_USART_ByteReceiveCallbackSet(DRV_USART_INDEX_0, APP_USARTReceiveEventHandler);
    if (uart_rx_messageQueue == NULL) {
        dbgOutputVal(DLOC_QUEUE_ERROR);
        errorHandling();
    }
    
    if(motor_messageQueue == NULL) {
        initMotorMsgQ();
    }
    
    
    dbgOutputVal(0x0D);
}

/******************************************************************************
  Function:
    void OVERWATCH_THREAD_Tasks ( void )

  Remarks:
    See prototype in overwatch_thread.h.
 */
void OVERWATCH_THREAD_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( overwatch_threadData.state )
    {
        /* Application's initialstate. */
        case OVERWATCH_THREAD_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                overwatch_threadData.state = OVERWATCH_THREAD_STATE_SERVICE_TASKS;
            }
            break;
        }

        case OVERWATCH_THREAD_STATE_SERVICE_TASKS:
        {
          //  dbgOutputVal(0x01);
            // TODO: Disabled Overwatch Thread
            while(!uartRxReceiveFromMsgQ() 
                    || PLIB_USART_TransmitterBufferIsFull(USART_ID_1)){
                dbgOutputVal (0x0b);
            }
                uint8_t raw_message = receiveUartRxVal();
               // Location_message rx_loc_message;
               // Location_message rx_loc_message = (deserialize_LM(raw_message));
                //dbgOutputVal((uint8_t)rx_loc_message.token_num);
                dbgOutputVal(raw_message);
              
                
                sendMotorVal(raw_message);
                while(!sendToMotorMsgQ);
            
            break;
        }

        /* TODO: implement your application state machine.*/       

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

/* -------------- Overwatch_Thread Message queue Helpers -------------- */

/**
  * check if message received from queue
  * @return 
  *     1 is received, 0 not
  */
int uartRxReceiveFromMsgQ() { 
    unsigned char message;
//    dbgOutputVal(0x06);
    if (xQueueReceive(uart_rx_messageQueue, &message, portMAX_DELAY) == pdTRUE){
           return 1;
    }  
    return 0;
}

/**
 * check if message is successfully sent
 * @param millisecondsElapsed
 *      Elapsed time
 * @return  1 if is successfully sent, otherwise 0
 */
int uartRxSendToMsgQ(unsigned char message) {
    if (xQueueSend(uart_rx_messageQueue, &message, portMAX_DELAY) == pdTRUE)
           return 1;
    return 0;
}

/**
 * send message to message queue
 * @param message
 *      sent message
 */
void sendUartRxVal(unsigned char message) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(uart_rx_messageQueue, &message, &xHigherPriorityTaskWoken);
    /* signal end-of-irq and possible reschedule point */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/**
 * receive message from message queue
 * @return 
 *      received message
 */

unsigned char receiveUartRxVal() {
    unsigned char message;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueReceiveFromISR(uart_rx_messageQueue, (void *)&message, 0);
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    return message;
}
 

/*******************************************************************************
 End of File
 */
