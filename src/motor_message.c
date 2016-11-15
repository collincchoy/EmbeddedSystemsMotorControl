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
#include "motor_message.h"

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

Line_message deserialize_MM(uint32_t data) {
    Line_message res;
    int dark_level0 = 0x000000FF & data;
    int dark_level1 = 0x000000FF & (data >> 8); 
    int dark_level2 = 0x000000FF & (data >> 16);
    int dark_level3 = 0x000000FF & (data >> 24); 
    res.dark_level0 = dark_level0;
    res.dark_level1 = dark_level1;
    res.dark_level2 = dark_level2;
    res.dark_level3 = dark_level3;
    return res;
}

uint32_t serialize_MM(Line_message line_data){
    uint32_t res;
    uint32_t res0 = (uint32_t)line_data.dark_level0;
    uint32_t res1 = (uint32_t)line_data.dark_level1;
    uint32_t res2 = (uint32_t)line_data.dark_level2;
    uint32_t res3 = (uint32_t)line_data.dark_level3;
    res = (res3 << 23)|(res2 << 15)|(res1 << 7)|(res0);
    return res;
}

/* ************************************************************************** */

/* *****************************************************************************
 End of File
 */
