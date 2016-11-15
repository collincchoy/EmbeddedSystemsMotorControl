/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "overwatch_thread.h"
#include "motor_control_thread.h"
#include "motor_msq.h"
#include "communication_thread.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void IntHandlerDrvAdc(void)
{
    uint16_t AdcVal1 = (uint16_t)DRV_ADC_SamplesRead(0);            
    uint16_t AdcVal2 = (uint16_t)DRV_ADC_SamplesRead(1);
    uint16_t AdcVal3 = (uint16_t)DRV_ADC_SamplesRead(2);            
    uint16_t AdcVal4 = (uint16_t)DRV_ADC_SamplesRead(3); 
    
    uint16_t THRESH = 150;
    /*if (AdcVal1 > THRESH || AdcVal2 > THRESH || AdcVal3 > THRESH || AdcVal4 > THRESH) {
        turnLED_On();
    }
    else {
        turnLED_Off();
    }*/
    dbgOutputVal(AdcVal2);
    dbgOutputLoc(AdcVal3);
    
    if ((AdcVal2 > THRESH && AdcVal3 > THRESH) && (AdcVal1 < THRESH && AdcVal4 < THRESH)) {
        turnLED_On();
        sendMotorVal('w'); // Go
    }
    else if ( AdcVal1 > THRESH && AdcVal4 < THRESH ) {
        sendMotorVal('d'); // Hang left to correct
    }
    else if ( AdcVal1 < THRESH && AdcVal4 > THRESH ) {
        sendMotorVal('a');
    }
    else {
        turnLED_Off();
        sendMotorVal('e');
        //sendMotorVal('s');
    }
    
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
}




    
void IntHandlerDrvTmrInstance0(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr0);
}
    
void IntHandlerDrvTmrInstance1(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr1);
}
 void IntHandlerDrvUsartInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
    DRV_USART_TasksReceive(sysObj.drvUsart0);
    DRV_USART_TasksError(sysObj.drvUsart0);
}
 
 
 

 

 

 

 

 
  
/*******************************************************************************
 End of File
*/

