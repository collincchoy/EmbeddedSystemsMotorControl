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

#ifndef _MOTOR_MESSAGE_H    /* Guard against multiple inclusion */
#define _MOTOR_MESSAGE_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <stdint.h>
/*
* Four sensors
*
*
*/
typedef struct {
int dark_level0; 
int dark_level1; 
int dark_level2; 
int dark_level3; 
} Line_message;
/*
* 0-7 bits: level 0; 8-15 bits: level 1; 16-23 bits level2; 24-31 bits level3;
*/
Line_message deserialize_MM(uint32_t data);

uint32_t serialize_MM(Line_message line_data);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
