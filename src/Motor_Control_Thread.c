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
    if (motor_control_threadData.state == MOTOR_CONTROL_THREAD_STATE_DRIVE_SLOW)
    {
        dbgOutputVal(PWM1_cntr);
        PWM1_cntr++;
        drive_slow();
    }
    /*else if (motor_control_threadData.state == MOTOR_CONTROL_THREAD_STATE_HANG_RIGHT)
    {
        PWM1_cntr++;
        hangRight();
    }*/

}

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
    
    motor_control_threadData.isInManual = true;
    
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
    dbgOutputLoc(0x04);
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
                
                PWM1_cntr = 0;
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
            //dbgOutputVal(0xFF);

            stop();
            break;
        }
        
        case MOTOR_CONTROL_THREAD_STATE_DRIVE:
        {
            dbgOutputVal(0x04);
            drive();
            break;
        }
        
        case MOTOR_CONTROL_THREAD_STATE_HANG_LEFT:
        {
            dbgOutputVal(0x05);
            if (motor_control_threadData.isInManual)
                turnLeft();
            else
                hangLeft();
            break;
        }
        
        case MOTOR_CONTROL_THREAD_STATE_HANG_RIGHT:
        {
            dbgOutputVal(0x06);
            if (motor_control_threadData.isInManual)
                turnRight();
            else
                hangRight();
            break;
        }
        
        case MOTOR_CONTROL_THREAD_STATE_DRIVE_SLOW:
        {
            //dbgOutputVal(0x07);
            drive_slow();
            break;
        }
        
        case MOTOR_CONTROL_THREAD_STATE_DRIVE_REVERSE:
        {
            dbgOutputVal(0x08);
            reverse();
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
    
    /* Check Message Queue for Next-State Transitions */
    dbgOutputLoc(0x0E);
    if (receiveFromMotorMsgQ()) {
    //{}
        uint8_t rec = receiveMotorVal();

        //if (motor_control_threadData.state != MOTOR_CONTROL_THREAD_STATE_SERVICE_TASKS) {        
            if (rec == 0x77) { // w
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_DRIVE;
            }
            else if (rec == 0x73) { // s
                if (!motor_control_threadData.isInManual) {
                    motor_control_threadData.isInManual = true;
                }
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_SERVICE_TASKS;
            }
            else if (rec == 0x61) { // a
                PWM1_cntr = 0;
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_HANG_LEFT;
            }
            else if (rec == 0x64) { // d
                PWM1_cntr = 0;
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_HANG_RIGHT;
            }
            else if (rec == 0x65) { // e
                PWM1_cntr = 0;
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_DRIVE_SLOW;
            }
            else if (rec == 0x78) { // x
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_DRIVE_REVERSE;
            }
            else if (rec == 0x20) {
                motor_control_threadData.isInManual = false;
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_DRIVE;
            }
        /*}
        else {
            if (rec == 0x20) {
                motor_control_threadData.isInManual = false;
                motor_control_threadData.state = MOTOR_CONTROL_THREAD_STATE_DRIVE;
            }
        }*/
    }
    dbgOutputLoc(0xEE);
}

/* -------------- Overwatch_Thread Message queue Helpers -------------- */
void drive()
{
    MOTOR1_WRITE(0x1);
    MOTOR2_WRITE(0x1);
    
    MOTOR1_DIR(0x1);
    MOTOR2_DIR(0x1);
}

void stop()
{
    MOTOR1_WRITE(0x0);
    MOTOR2_WRITE(0x0);
}

void turnLeft()
{
    MOTOR1_WRITE(0x1);
    MOTOR2_WRITE(0x1);
    
    MOTOR1_DIR(0x0);
    MOTOR2_DIR(0x1);
}

void hangLeft()
{
    /*int PWM_PERIOD = 100;
    int PWM_DUTY_CYCLE = 25;
    int TRANSITION_TIME = PWM_PERIOD * (PWM_DUTY_CYCLE/100.0);
    
    if (PWM1_cntr < TRANSITION_TIME) {      
        // PWM signal to motor 1
        MOTOR1_WRITE(0x1);
        MOTOR1_DIR(0x1);
    }
    else if (PWM1_cntr < (PWM_PERIOD)) {
        MOTOR1_WRITE(0x0);
    }
    else {
        PWM1_cntr = 0;
    }*/
    
    MOTOR1_WRITE(0x0);
    
    MOTOR2_WRITE(0x1);
    MOTOR2_DIR(0x1);
}

void turnRight()
{ 
    MOTOR1_WRITE(0x1);
    MOTOR2_WRITE(0x1);
    
    MOTOR1_DIR(0x1);
    MOTOR2_DIR(0x0);
}

void hangRight()
{
    /*int PWM_PERIOD = 100;
    int PWM_DUTY_CYCLE = 25;
    int TRANSITION_TIME = PWM_PERIOD * (PWM_DUTY_CYCLE/100.0);
    
    if (PWM1_cntr < TRANSITION_TIME) {      
        // PWM signal to motor 1
        MOTOR2_WRITE(0x1);
        MOTOR2_DIR(0x1);
    }
    else if (PWM1_cntr < (PWM_PERIOD)) {
        MOTOR2_WRITE(0x0);
    }
    else {
        PWM1_cntr = 0;
    }*/
    
    MOTOR2_WRITE(0x0);
    
    MOTOR1_WRITE(0x1);
    MOTOR1_DIR(0x1);
}

void drive_slow()
{
    
    int PWM_PERIOD = 100;
    int PWM_DUTY_CYCLE = 25;
    int TRANSITION_TIME = PWM_PERIOD * (PWM_DUTY_CYCLE/100.0);
    
    if (PWM1_cntr < TRANSITION_TIME) {
        drive();
    }
    else if (PWM1_cntr < (PWM_PERIOD)) {
        stop();
    }
    else {
        PWM1_cntr = 0;
    }
}

void reverse()
{
    MOTOR1_WRITE(0x1);
    MOTOR2_WRITE(0x1);
    
    MOTOR1_DIR(0x0);
    MOTOR2_DIR(0x0);
}

void MOTOR1_WRITE(short rw)
{
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0, rw); // OC1/En1
}

void MOTOR1_DIR(short dir)
{
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, dir); // Dir1
}

void MOTOR2_WRITE(short rw)
{
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_1, rw); // OC2/En2
}

void MOTOR2_DIR(short dir)
{
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, dir); // Dir2
}

/*******************************************************************************
 End of File
 */
