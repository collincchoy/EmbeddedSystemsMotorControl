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
#include "location_message.h"

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


Location_message deserialize_LM(uint8_t data) {
    Location_message decoded;
    unsigned int rover_num = (unsigned int)(0xFFFF & (data >> 7));
    unsigned int token_num = (unsigned int)(0xFF7F & data); 
    decoded.rover_num = rover_num;
    decoded.token_num = token_num;
    return decoded;
}

uint8_t serialize_LM(Location_message location_data){
    uint8_t res;
    uint8_t first_bit = (uint8_t)(location_data.rover_num);
    uint8_t last_seven_bit = (uint8_t)(location_data.token_num);
    res = (first_bit << 7) | (last_seven_bit);
    return res;
}


/* *****************************************************************************
 End of File
 */
