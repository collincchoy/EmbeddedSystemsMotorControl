/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */
#include "motor_msq.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */
/** Descriptive Data Item Name

  @Summary
    Brief one-line summary of the data item.
    
  @Description
    Full description, explaining the purpose and usage of data item.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
  @Remarks
    Any additional remarks
 */
int global_data;


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */
     /**
 * check if message is successfully sent
 * @param millisecondsElapsed
 *      Elapsed time
 * @return  1 if is successfully sent, otherwise 0
 */
int sendToMotorMsgQ(unsigned char message) {
    if (xQueueSend(motor_messageQueue, &message, portMAX_DELAY) == pdTRUE)
           return 1;
    return 0;
}

/**
 * send message to message queue
 * @param message
 *      sent message
 */
void sendMotorVal(unsigned char message) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(motor_messageQueue, &message, &xHigherPriorityTaskWoken);
    /* signal end-of-irq and possible reschedule point */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/**
  * check if message received from queue
  * @return 
  *     1 is received, 0 not
  */
int receiveFromMotorMsgQ() { 
    unsigned char message;
    if (xQueuePeek(motor_messageQueue, &message, /*portMAX_DELAY*/( TickType_t ) 5) == pdTRUE)
           return 1;
    
    return 0;
}

unsigned char receiveMotorVal() {
    unsigned char message;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueReceiveFromISR(motor_messageQueue, (void *)&message, &xHigherPriorityTaskWoken);
    
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    return message;
}
    
    void initMotorMsgQ() {
        motor_messageQueue = xQueueCreate(10, sizeof(unsigned char));
    }
/* ************************************************************************** */

/* *****************************************************************************
 End of File
 */
