/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    communication_thread.c

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

#include "communication_thread.h"

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

DRV_HANDLE    uart_tx;
QueueHandle_t uart_tx_messageQueue;
COMMUNICATION_THREAD_DATA communication_threadData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* Application's Timer Callback Function */
static int count = 0;
static int fakeRover_num = 1;
static int fakeToken_num = 0;
static uint8_t my_location_mess = 0;
Location_message location_data;
static void TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    // TODO: Disabled location timer interupt
    /*count++;
    while (count >10){
        if (fakeToken_num > 127)
            fakeToken_num = 0;
        fakeToken_num++;
        location_data.rover_num = fakeRover_num;
        location_data.token_num = fakeToken_num;
        my_location_mess = serialize_LM(location_data);

        sendUartTxVal(my_location_mess);
        while(! uartTxSendToMsgQ(my_location_mess)) {
            //dbgOutputLoc(0x07);
        }
        count = 0;    
    }*/
}

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* Application's Timer Setup Function */
static void TimerSetup( void )
{
    DRV_TMR_AlarmRegister(
        communication_threadData.handleTimer0, 
        COMMUNICATION_THREAD_TMR_DRV_PERIOD, 
        COMMUNICATION_THREAD_TMR_DRV_IS_PERIODIC,
        (uintptr_t)NULL, 
        TimerCallback);
    DRV_TMR_Start(communication_threadData.handleTimer0);
}

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void COMMUNICATION_THREAD_Initialize ( void )

  Remarks:
    See prototype in communication_thread.h.
 */

void COMMUNICATION_THREAD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    communication_threadData.state = COMMUNICATION_THREAD_STATE_INIT;

    communication_threadData.handleTimer0 = DRV_HANDLE_INVALID;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
  /*  uart_tx = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_WRITE);
    if(uart_tx == DRV_HANDLE_INVALID) {
        //dbgOutputLoc(DLOC_UART_ERROR);
        errorHandling();
    }
    //dbgOutputLoc(0x0A);
   
    uart_tx_messageQueue = xQueueCreate(10, sizeof(unsigned char));

    if (uart_tx_messageQueue == NULL) {
        //dbgOutputLoc(DLOC_QUEUE_ERROR);
        errorHandling();
    }*/
}


/******************************************************************************
  Function:
    void COMMUNICATION_THREAD_Tasks ( void )

  Remarks:
    See prototype in communication_thread.h.
 */

void COMMUNICATION_THREAD_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( communication_threadData.state )
    {
        /* Application's initial state. */
        case COMMUNICATION_THREAD_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (communication_threadData.handleTimer0 == DRV_HANDLE_INVALID)
            {
                communication_threadData.handleTimer0 = DRV_TMR_Open(COMMUNICATION_THREAD_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != communication_threadData.handleTimer0 );
            }
        
            if (appInitialized)
            {
                TimerSetup();
                PLIB_USART_TransmitterEnable (USART_ID_1);
            
                communication_threadData.state = COMMUNICATION_THREAD_STATE_SERVICE_TASKS;
            }
            break;
        }

        case COMMUNICATION_THREAD_STATE_SERVICE_TASKS:
        {
            // TODO: Disabled Communication Thread
            /*while(!uartTxReceiveFromMsgQ() 
                    || PLIB_USART_TransmitterBufferIsFull(USART_ID_1)){
            //dbgOutputLoc(0x04);
            }
            uint8_t rec = receiveUartTxVal();

            PLIB_USART_TransmitterByteSend (USART_ID_1, rec);*/
            break;
        }
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

/* -------------- Communication_Thread Message queue Helpers -------------- */

/**
  * check if message received from queue
  * @return 
  *     1 is received, 0 not
  */
int uartTxReceiveFromMsgQ() { 
    unsigned char message;
    if (xQueueReceive(uart_tx_messageQueue, &message, portMAX_DELAY) == pdTRUE)
           return 1;
    return 0;
}

/**
 * check if message is successfully sent
 * @param millisecondsElapsed
 *      Elapsed time
 * @return  1 if is successfully sent, otherwise 0
 */
int uartTxSendToMsgQ(unsigned char message) {
    if (xQueueSend(uart_tx_messageQueue, &message, portMAX_DELAY) == pdTRUE)
           return 1;
    return 0;
}

/**
 * send message to message queue
 * @param message
 *      sent message
 */
void sendUartTxVal(unsigned char message) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(uart_tx_messageQueue, &message, &xHigherPriorityTaskWoken);
    /* signal end-of-irq and possible reschedule point */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/**
 * receive message from message queue
 * @return 
 *      received message
 */

unsigned char receiveUartTxVal() {
    unsigned char message;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueReceiveFromISR(uart_tx_messageQueue, (void *)&message, &xHigherPriorityTaskWoken);
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    return message;
}

/*******************************************************************************
 End of File
 */
