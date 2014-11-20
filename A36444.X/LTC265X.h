#ifndef __LTC265X_H
#define __LTC265X_H

#include "ETM_SPI.h"
#include "ETM_IO_PORTS.h"


/*
  This module is used to interface with a LTC2654/LTC2656 over the SPI bus.
  Created from the LTC2656 Module (8 DAC device)

  Module Requirements
  ETM_SPI
  ETM_IO_PORTS

  Dan Parker
  2014_11_06

*/

/*
  DPARKER Validate
*/



typedef struct {
  unsigned char spi_port;
  unsigned long pin_chip_select_not;
  unsigned long pin_load_dac_not;

  unsigned long pin_dac_clear_not;
  unsigned long pin_por_select;
  unsigned char por_select_value;

} LTC265X;


#define LTC265X_SPI_CON_VALUE  0b0000010100111111 // This works for V1 and V2 because common bits are identical and there is no overlap on the non-identical bits
#define LTC265X_SPI_CON2_VALUE 0b0000000000000000 // This works for V1 and V2 because it is ignored in V1
#define LTC265X_SPI_STAT_VALUE 0b1010000000000000 // This works for V1 & V2 because they are identical

#define LTC265X_SPI_10_M_BIT    10000000
#define LTC265X_SPI_5_M_BIT      5000000
#define LTC265X_SPI_2_5_M_BIT    2500000
#define LTC265X_SPI_1_M_BIT      1000000
  
/* -------------- DAC Command Definitions ------------------- */
#define LTC265X_CMD_WRITE_TO_INPUT_REGISTER_N             0b00000000 // writes to input register N
#define LTC265X_CMD_UPDATE_DAC_REGISTER_N                 0b00010000 // updates output N
#define LTC265X_CMD_WRITE_TO_INPUT_REGISTER_N_UPDATE_ALL  0b00100000 // Writes to input register N and updates ALL outputs
#define LTC265X_CMD_WRITE_AND_UPDATE_N                    0b00110000 // Write to input register N and updates output N
#define LTC265X_CMD_POWER_DOWN_N                          0b01000000 // Powers down channel N
#define LTC265X_CMD_POWER_DOWN_CHIP                       0b01010000 // Powers down the chip and reference
#define LTC265X_CMD_SELECT_INTERNAL_REFERENCE             0b01100000 // Selects and Powers up the internal Reference
#define LTC265X_CMD_SELECT_EXTERNAL_REFERENCE             0b01110000 // Selects External Reference and powers down internal Reference
#define LTC265X_CMD_NO_OPERATION                          0b11110000 // No operation

#define LTC265X_ADR_A                                     0b00000000 // DAC A
#define LTC265X_ADR_B                                     0b00000001 // DAC B
#define LTC265X_ADR_C                                     0b00000010 // DAC C
#define LTC265X_ADR_D                                     0b00000011 // DAC D
#define LTC265X_ADR_E                                     0b00000100 // DAC E
#define LTC265X_ADR_F                                     0b00000101 // DAC F
#define LTC265X_ADR_G                                     0b00000110 // DAC G
#define LTC265X_ADR_H                                     0b00000111 // DAC H
#define LTC265X_ADR_ALL_DACS                              0b00001111 // ALL DACs

#define LTC265X_WRITE_AND_UPDATE_DAC_A                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_A)
#define LTC265X_WRITE_AND_UPDATE_DAC_B                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_B)
#define LTC265X_WRITE_AND_UPDATE_DAC_C                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_C)
#define LTC265X_WRITE_AND_UPDATE_DAC_D                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_D)
#define LTC265X_WRITE_AND_UPDATE_DAC_E                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_E)
#define LTC265X_WRITE_AND_UPDATE_DAC_F                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_F)
#define LTC265X_WRITE_AND_UPDATE_DAC_G                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_G)
#define LTC265X_WRITE_AND_UPDATE_DAC_H                    (LTC265X_CMD_WRITE_AND_UPDATE_N | LTC265X_ADR_H)

/* -------------- END DAC Command Definitions ------------------- */


void SetupLTC265X(LTC265X* ptr_LTC265X, unsigned char spi_port, unsigned long fcy_clk, unsigned long spi_bit_rate, unsigned long pin_chip_select_not, unsigned long pin_load_dac_not);
/*  
   Function Arguments:
   *ptr_LTC265X : This is pointer to the structure that defines a particular LTC265X chip

   This function should be called when somewhere during the startup of the processor.
   This will configure the SPI port (selected by spi_port).
*/

void ClearOutputsLTC265X(LTC265X* ptr_LTC265X);
/*  
   Function Arguments:
   *ptr_LTC265X : This is pointer to the structure that defines a particular LTC265X chip
              
   This function will set all the outputs to their Power On Reset value (either 0 or .5 based on the POR pin)
*/



unsigned char WriteLTC265X(LTC265X* ptr_LTC265X, unsigned int command_word, unsigned int data_word);

/*  
   Function Arguments:
   *ptr_LTC265X     : This is pointer to the structure that defines a particular LTC265X chip

   command_word     : This is the command word as defined above.  See the LTC265X spec sheet for more information
                      You may wish to #define more commands using the "N commands and a Channel"
		      For Example if you wanted a command that powered down only CHN B
		      #define LTC265X_CMD_POWER_DOWN_CHIP             (LTC265X_CMD_POWER_DOWN_N | LTC265X_ADR_B)
   data_word        : This is the data (Voltage) to be sent along with the command
                      Voltage is scaled to the chip reference - Internal Reference is selected in this SetupLTC265X()
		      If using 12 bit version of the chip, only the 12 MSB are used. The 4 LSB are don't care.

   This will send a 32 bit command to to the LTC265X and confirm the data sent to the LTC265
   If the data transmission is successful, the function will return 0
   If there is an error with the data transmission (error on SPI bus, or the data check failed), then the function will return the error code
   If there are any errors bad data may have been written to the DAC so the calling code should attempt to resend the data
*/

unsigned char WriteLTC265XTwoChannels(LTC265X* ptr_LTC265X, unsigned int command_word_one, unsigned int data_word_one, unsigned command_word_two, unsigned int data_word_two);


//unsigned char WriteLTC2654AllDacChannels(LTC265X* ptr_LTC265X, unsigned int *dac_array);
// DPARKER need to create this functions


unsigned char WriteLTC2656AllDacChannels(LTC265X* ptr_LTC265X, unsigned int *dac_array);
/*  
   Function Arguments:
   *ptr_LTC265X     : This is pointer to the structure that defines a particular LTC265X chip

   *ptr_data        : This is a pointer to the array (8 words) that contains the data for the 8 dac channels

   This will attempt to send out 8 commands (32 bits per command) and the SPI BUS
   The return data will be checked to ensure that the data was properly received
   If the entire data transfer is successful without errors, the function will return 0.
   If there are any errors on the spi bus or a data error is detected the function will exit immediately and return the error code
   If there are any errors bad data may have been written to the DAC so the calling code should attempt to resend the data
*/


extern unsigned int LTC265X_single_channel_error_count; // This global variable counts the number of times WriteLTC265X failed to properly write data.
extern unsigned int LTC265X_all_channel_error_count; // This global variable counts the number of times WriteLTC265XAllDacChannels failed to properly write data.

#endif
