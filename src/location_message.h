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

#ifndef _LOCATION_MESSAGE_H    /* Guard against multiple inclusion */
#define _LOCATION_MESSAGE_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif
    
typedef struct _Location_message{
        unsigned int rover_num;
        unsigned int token_num; 
} Location_message;



uint8_t serialize_LM (Location_message location_data);
Location_message deserialize_LM(uint8_t data);
/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
