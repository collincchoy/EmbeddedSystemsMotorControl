/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motor_control_thread.c

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

#include "motor_control_thread.h"
#include "motor_msq.h"
#include "debug.h"
#include "motor_msq.h"
#include "motor_message.h"

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

QueueHandle_t motor_messageQueue;
MOTOR_CONTROL_THREAD_DATA motor_control_threadData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* Application's Timer Callback Function */
// Timer Interrupt to read line sensor
static void TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    // TODO: Disabled Line Sensor Reading
    /*int covered = 0x33;
    int val = (uint16_t)(DRV_ADC_SamplesRead(0) - covered);
    //dbgOutputVal(val);
    DRV_ADC_Start();
    //Line_message new_mess;
    //new_mess.dark_level0 = val;
    //new_mess.dark_level1 = 0x1;
    //new_mess.dark_level2 = 0x2;
    //new_mess.dark_level3 = 0x3;
    //uint32_t ser_mess = serialize_MM(new_mess);
    sendMotorVal(val);
    while(! sendToMotorMsgQ(val)) {
        dbgOutputLoc(0x07);
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
        motor_control_threadData.handleTimer1, 
        MOTOR_CONTROL_THREAD_TMR_DRV_PERIOD, 
        MOTOR_CONTROL_THREAD_TMR_DRV_IS_PERIODIC,
        (uintptr_t)NULL, 
        TimerCallback);
    DRV_TMR_Start(motor_control_threadData.handleTimer1);
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
    void MOTOR_CONTROL_THREAD_Initialize ( void )

  Remarks:
    See prototype in motor_control_thread.h.
 */

void MOTOR_CONTROL_THREAD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_INIT;

    motor_control_threadData.handleTimer1 = DRV_HANDLE_INVALID;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    if(motor_messageQueue == NULL) {
        initMotorMsgQ();
    }
}


/******************************************************************************
  Function:
    void MOTOR_CONTROL_THREAD_Tasks ( void )

  Remarks:
    See prototype in motor_control_thread.h.
 */

void MOTOR_CONTROL_THREAD_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( motor_control_threadData.state )
    {
        /* Application's initial state. */
        case MOTOR_CONTROL_THREAD_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (motor_control_threadData.handleTimer1 == DRV_HANDLE_INVALID)
            {
                motor_control_threadData.handleTimer1 = DRV_TMR_Open(MOTOR_CONTROL_THREAD_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != motor_control_threadData.handleTimer1 );
            }
        
            if (appInitialized)
            {
                DRV_ADC_Open();
                DRV_ADC_Start(); 
                TimerSetup();
            
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_SERVICE_TASKS;
            }
            break;
        }

        case MOTOR_CONTROL_THREAD_STATE_SERVICE_TASKS:
        {
            // TODO: Disable reading from motor message ueue
            /*while(!receiveFromMotorMsgQ()){
            //dbgOutputLoc(0x04);
            }
            uint8_t rec = receiveMotorVal();
            dbgOutputVal(rec);*/
            dbgOutputVal(0xFF);
            dbgOutputLoc(0xFF);
            
            //PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_8, 0x1);
            //PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_5, 0x0); // Dir2
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0, 0x01); // OC1/En1
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_1, 0x01); // OC2/En2

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

/*******************************************************************************
 End of File
 */
