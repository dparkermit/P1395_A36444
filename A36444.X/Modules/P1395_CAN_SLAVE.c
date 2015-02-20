#include <xc.h>
#include <timer.h>
#include "P1395_CAN_SLAVE.h"

// void ETMCanSlaveDoCan(void) - PUBLIC


void ETMCanSlaveProcessMessage(void);

void ETMCanSlaveExecuteCMD(ETMCanMessage* message_ptr);           
/* 
   This is the generic subroutine to handle commands.
   It will check that the message is valid and then call
   ETMCanSlaveExecuteCMDCommon (if it is a generic message that applies to all boards)
   ETMCanSlaveExecuteCMDBoardSpecific (if it is a command processed only by this board)  -- THIS IS LOCATED in file XXXXX

*/

void ETMCanSlaveExecuteCMDCommon(ETMCanMessage* message_ptr);
/*
  This handles commands that are common to all boards
*/

void ETMCanSlaveSetCalibrationPair(ETMCanMessage* message_ptr);

void ETMCanSlaveReturnCalibrationPair(ETMCanMessage* message_ptr);


void ETMCanSlaveTimedTransmit(void);
/*
  This uses TMR2 to schedule transmissions from the Slave to the Master
  TMR2 is set to expire at 100ms Interval
  Every 100ms a status message is set and 1 (of the 16) data log messages is sent
*/

void ETMCanSlaveSendStatus(void);
/*
  This function sends the status from the slave board to the master
*/


// void ETMCanSlaveLogData(void);  // this is a public function


void ETMCanSlaveCheckForTimeOut(void);
/*
  This looks to see if 
 */

void ETMCanSlaveSendUpdateIfNewNotReady(void);
/*
  This looks to see if there has been a status change
  If there is then a new status message is immediately sent to the master
*/

void ETMCanSlaveDoSync(ETMCanMessage* message_ptr);
/*
  This process the the sync message
*/



//void ETMCanReturnEEPromRegister(ETMCanMessage* message_ptr);






// void ETMCanInitialize(void) // Public Function


// local variables
unsigned int slave_data_log_index;
unsigned int previous_ready_status;  // DPARKER - Need better name

typedef struct {
  unsigned int reset_count;
  unsigned int can_timeout_count;
} PersistentData;
volatile PersistentData etm_can_persistent_data __attribute__ ((persistent));



// Global Variables
ETMCanMessageBuffer etm_can_rx_message_buffer;
ETMCanMessageBuffer etm_can_tx_message_buffer;

// Public Variables
unsigned int etm_can_next_pulse_level;
unsigned int etm_can_next_pulse_count;


// Public Debug and Status registers
ETMCanSystemDebugData local_debug_data;
ETMCanStatusRegister  etm_can_status_register;
ETMCanAgileConfig     etm_can_my_configuration;
ETMCanCanStatus       local_can_errors;
ETMCanSyncMessage     etm_can_sync_message;






void ETMCanSlaveDoCan(void) {
  ETMCanSlaveProcessMessage();
  ETMCanSlaveTimedTransmit();
  ETMCanSlaveCheckForTimeOut();
  ETMCanSlaveSendUpdateIfNewNotReady();


  local_debug_data.can_bus_error_count = local_can_errors.timeout;
  local_debug_data.can_bus_error_count += local_can_errors.message_tx_buffer_overflow;
  local_debug_data.can_bus_error_count += local_can_errors.message_rx_buffer_overflow;
  local_debug_data.can_bus_error_count += local_can_errors.data_log_rx_buffer_overflow;
  local_debug_data.can_bus_error_count += local_can_errors.address_error;
  local_debug_data.can_bus_error_count += local_can_errors.invalid_index;
  local_debug_data.can_bus_error_count += local_can_errors.unknown_message_identifier;
}




/*
  Slave to Slave  - LEVEL_BROADCAST  - Processed in Can interrupt
  Master to Slave - SYNC_BROADCAST   - Processed in Can interrupt
  Slave to Master - SLAVE_MASTER     - Not processed by Slave
  Master to Slave - MASTER_SLAVE     - Processed by ETMCanSlaveProcessMessage
  Slave to Master - SLAVE_STATUS     - Not processed by Slave
  Master to Slave - MASTER_REQUEST   - Processed by ETMCanSlaveProcessMessage (not at the moment this is only Calibration Request)
  Slave to Master - SLAVE_LOG        - Not processed by Slave

*/

void ETMCanSlaveProcessMessage(void) {
  ETMCanMessage next_message;
  while (ETMCanBufferNotEmpty(&etm_can_rx_message_buffer)) {
    ETMCanReadMessageFromBuffer(&etm_can_rx_message_buffer, &next_message);
    
    if ((next_message.identifier & 0b0000000001111000) != (ETM_CAN_MY_ADDRESS << 3)) {
      // It was not addressed to this board
      local_can_errors.address_error++;
    } else if (next_message.identifier == (ETM_CAN_MSG_CMD_RX | (ETM_CAN_MY_ADDRESS << 3))) {
      ETMCanSlaveExecuteCMD(&next_message);      
    } else if (next_message.identifier == (ETM_CAN_MSG_REQUEST_RX | (ETM_CAN_MY_ADDRESS << 3))) {
      ETMCanSlaveReturnCalibrationPair(&next_message);
    } else {
      local_can_errors.unknown_message_identifier++;
    } 
  }
  
  local_can_errors.message_tx_buffer_overflow = etm_can_tx_message_buffer.message_overwrite_count;
  local_can_errors.message_rx_buffer_overflow = etm_can_rx_message_buffer.message_overwrite_count;
}



void ETMCanSlaveExecuteCMD(ETMCanMessage* message_ptr) {
  /*
    Register allocations
    0xZ000 -> 0xZ0FF  -> Common Slave Commands and Set Values    - ETMCanSlaveExecuteCMDCommon()
    0xZ100 -> 0xZ1FF  -> Calibration Read/Write Registers        - ETMCanSlaveReturnCalibrationPair()
    0xZ200 -> 0xZ3FF  -> Slave Specific Commands and Set Values  - ETMCanSlaveExecuteCMD()
  */
  unsigned int index_word;
  index_word = message_ptr->word3;
  
  if ((index_word & 0xF000) != (ETM_CAN_MY_ADDRESS << 12)) {
    // The index is not addressed to this board
    local_can_errors.invalid_index++;
    return;
  }
  
  index_word &= 0x0FFF;

  if (index_word <= 0x00FF) {
    // It is a default command
    ETMCanSlaveExecuteCMDCommon(message_ptr);
  } else if (index_word <= 0x01FF) {
    // It is Calibration Pair Set Message
    ETMCanSlaveSetCalibrationPair(message_ptr);
  } else if (index_word <= 0x3FF) {
    // It is a board specific command
    ETMCanSlaveExecuteCMDBoardSpecific(message_ptr);
  } else {
    // It was not a command ID
    local_can_errors.invalid_index++;
  }
}



void ETMCanSlaveExecuteCMDCommon(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3;
  index_word &= 0x0FFF;
  
  switch (index_word) {
    
  case ETM_CAN_REGISTER_DEFAULT_CMD_RESET_MCU:
    __asm__ ("Reset");
    break;

  case ETM_CAN_REGISTER_DEFAULT_CMD_RESET_ANALOG_CALIBRATION:
    //ETMCanLoadDefaultAnalogCalibration();  // DPARKER Change to load default EEPROM
    break;
 
  default:
    // The default command was not recognized 
    local_can_errors.invalid_index++;
    break;
  }
}


void ETMCanSlaveSetCalibrationPair(ETMCanMessage* message_ptr) {
}

void ETMCanSlaveReturnCalibrationPair(ETMCanMessage* message_ptr) {
  ETMCanMessage return_msg;
  unsigned int eeprom_register;
  
  if ((message_ptr->word3 & 0xF000) != (ETM_CAN_MY_ADDRESS << 12)) {
    // The index is not addressed to this board
    local_can_errors.invalid_index++;
    return;
  }
  
  eeprom_register = (message_ptr->word3 & 0x0FFF);
  if ((eeprom_register < 0x0100) || (eeprom_register >= 0x200)) {
    // The eeprom_register is not a valid calibration point
    local_can_errors.invalid_index++;
    return;    
  }
  
  // The request is valid, return the data stored in eeprom
  return_msg.word3 = message_ptr->word3;
  return_msg.word2 = 0;
  //return_msg.word1 = ETMEEPromReadWord(eeprom_register + 1);  // DPARKER need to make eepromreadword generic for all processors/eeprom type
  //return_msg.word0 = ETMEEPromReadWord(eeprom_register);      // DPARKER need to make eepromreadword generic for all processors/eeprom type
  
  // Send Message Back to ECB with data
  ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &return_msg);
  MacroETMCanCheckTXBuffer();  // DPARKER - Figure out how to build this into ETMCanAddMessageToBuffer()
}


void ETMCanSlaveTimedTransmit(void) {
  // Sends the debug information up as log data  
  if (_T2IF) {
    // should be true once every 100mS
    _T2IF = 0;
    
    slave_data_log_index++;
    slave_data_log_index &= 0xF;
    
    ETMCanSlaveSendStatus(); // Send out the status every 100mS
    
    switch (slave_data_log_index) 
      {
      case 0x0:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_ERROR_0, 
			   local_debug_data.i2c_bus_error_count, 
			   local_debug_data.spi_bus_error_count,
			   local_debug_data.can_bus_error_count,
			   local_debug_data.scale_error_count);      
	break;
	
      case 0x1:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_ERROR_1, 
			   local_debug_data.reset_count, 
			   *(unsigned int*)&local_debug_data.self_test_results, 
			   0, 
			   0);
	break;
	
      case 0x2:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_0,
			   local_debug_data.debug_0, 
			   local_debug_data.debug_1,
			   local_debug_data.debug_2,
			   local_debug_data.debug_3);  
	break;
	
      case 0x3:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_1,
			   local_debug_data.debug_4,
			   local_debug_data.debug_5,
			   local_debug_data.debug_6,
			   local_debug_data.debug_7);
	break;
	
      case 0x4:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_2,
			   local_debug_data.debug_8,
			   local_debug_data.debug_9,
			   local_debug_data.debug_A,
			   local_debug_data.debug_B);
	break;
	
      case 0x5:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_DEBUG_3,
			   local_debug_data.debug_C,
			   local_debug_data.debug_D,
			   local_debug_data.debug_E,
			   local_debug_data.debug_F);
	break;
	
      case 0x6:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_0,
			   local_can_errors.CXEC_reg,
			   local_can_errors.error_flag,
			   local_can_errors.tx_1,
			   local_can_errors.tx_2);
	break;
	
      case 0x7:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_1,
			   local_can_errors.rx_0_filt_0,
			   local_can_errors.rx_0_filt_1,
			   local_can_errors.rx_1_filt_2,
			   local_can_errors.isr_entered);
	break;
	
      case 0x8:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_2,
			   local_can_errors.unknown_message_identifier,
			   local_can_errors.invalid_index,
			   local_can_errors.address_error,
			   local_can_errors.tx_0);
	break;
	
      case 0x9:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CAN_ERROR_3,
			   local_can_errors.message_tx_buffer_overflow,
			   local_can_errors.message_rx_buffer_overflow,
			   local_can_errors.data_log_rx_buffer_overflow,
			   local_can_errors.timeout);
	break;
	
      case 0xA:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_0,
			   etm_can_my_configuration.agile_number_high_word,
			   etm_can_my_configuration.agile_number_low_word,
			   etm_can_my_configuration.agile_dash,
			   etm_can_my_configuration.agile_rev_ascii);
	break;

      case 0xB:
	ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_DEFAULT_CONFIG_1,
			   etm_can_my_configuration.serial_number,
			   etm_can_my_configuration.firmware_branch,
			   etm_can_my_configuration.firmware_major_rev,
			   etm_can_my_configuration.firmware_minor_rev);
	break;
	
      case 0xC:
	ETMCanSlaveLogCustomPacketC();
	break;
	
      case 0xD:
	ETMCanSlaveLogCustomPacketD();
	break;

      case 0xE:
	ETMCanSlaveLogCustomPacketE();
	break;

      case 0xF:
	ETMCanSlaveLogCustomPacketF();
	break;
      }
  }
}


void ETMCanSlaveSendStatus(void) {
  ETMCanMessage message;
  message.identifier = ETM_CAN_MSG_STATUS_TX | (ETM_CAN_MY_ADDRESS << 3);

  message.word0 = _CONTROL_REGISTER;
  message.word1 = _FAULT_REGISTER;
  message.word2 = etm_can_status_register.data_word_A;
  message.word3 = etm_can_status_register.data_word_B;
  
  ETMCanTXMessage(&message, &CXTX1CON);
  local_can_errors.tx_1++;
}


void ETMCanSlaveLogData(unsigned int packet_id, unsigned int word3, unsigned int word2, unsigned int word1, unsigned int word0) {
  
  ETMCanMessage log_message;
  
  packet_id &= 0x000F;
  packet_id |= (ETM_CAN_MY_ADDRESS << 4);
  packet_id <<= 1;
  packet_id |= 0b0000011000000000;
  packet_id <<= 2;
  
  log_message.identifier = packet_id;
  log_message.identifier &= 0xFF00;
  log_message.identifier <<= 3;
  log_message.identifier |= (packet_id & 0x00FF);
  
  log_message.word0 = word0;
  log_message.word1 = word1;
  log_message.word2 = word2;
  log_message.word3 = word3;
  
  ETMCanAddMessageToBuffer(&etm_can_tx_message_buffer, &log_message);
  MacroETMCanCheckTXBuffer();
}

void ETMCanSlaveCheckForTimeOut(void) {
  if (_T3IF) {
    _T3IF = 0;
    local_can_errors.timeout++;
    etm_can_persistent_data.can_timeout_count = local_can_errors.timeout;
    _CONTROL_CAN_COM_LOSS = 1;
  }
}

void ETMCanSlaveSendUpdateIfNewNotReady(void) {
  if ((previous_ready_status == 0) && (_CONTROL_NOT_READY)) {
    // There is new condition that is causing this board to inhibit operation.
    // Send a status update upstream to Master
    ETMCanSlaveSendStatus();
  }
  previous_ready_status = _CONTROL_NOT_READY;
}



void ETMCanSlaveDoSync(ETMCanMessage* message_ptr) {
  // Sync data is available in CXRX0B1->CXRX0B4
  // At this time all that happens is that the chip watchdog is reset
  // DPARKER move to assembly and issure W0-W3, SR usage

  // It should be noted that if any of these registers are written ANYWHERE else, then they can be bashed
  // Therefore the Slave boards should NEVER WRITE ANYTHING in _SYNC_CONTROL_WORD

  _SYNC_CONTROL_WORD = message_ptr->word0;
  etm_can_sync_message.sync_1 = message_ptr->word1;
  etm_can_sync_message.sync_2 = message_ptr->word2;
  etm_can_sync_message.sync_3 = 0xFFFF;  // Use this to indicate that a sync message has been recieved
  
  ClrWdt();
  _CONTROL_CAN_COM_LOSS = 0;
  
  TMR3 = 0;

#ifdef __A36487
  // The Pulse Sync Board needs to see if it needs to inhibit X_RAYs
  // This can be based by an update to read/modify update to this PORT that is currently in process
  // It will get fixed the next time through the control loop, but there is nothing else we can do
  // Hopefully we are using good coding principles and not bashing a PORT register.
  if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV || _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY) {
    PIN_CPU_XRAY_ENABLE_OUT = !OLL_CPU_XRAY_ENABLE;
    // DPARKER how do we ensure that this is not bashed by a command in progress???
  }
#endif

}




void ETMCanSlaveInitialize(void) {
  if (_POR || _BOR) {
    // This was a power cycle;
    etm_can_persistent_data.reset_count = 0;
    etm_can_persistent_data.can_timeout_count = 0;
  } else {
    etm_can_persistent_data.reset_count++;
  }

  _SYNC_CONTROL_WORD = 0;
  etm_can_sync_message.sync_1 = 1;
  etm_can_sync_message.sync_2 = 2;
  etm_can_sync_message.sync_3 = 3;
  
  // Clear the processor reset flags
  _POR = 0;
  _BOR = 0;
  _SWR = 0;
  _EXTR = 0;
  _TRAPR = 0;
  _WDTO = 0;
  _IOPUWR = 0;

  local_debug_data.reset_count = etm_can_persistent_data.reset_count;
  local_can_errors.timeout = etm_can_persistent_data.can_timeout_count;

  _CXIE = 0;
  _CXIF = 0;
  _CXIP = ETM_CAN_INTERRUPT_PRIORITY;
  
  CXINTF = 0;
  
  CXINTEbits.RX0IE = 1; // Enable RXB0 interrupt
  CXINTEbits.RX1IE = 1; // Enable RXB1 interrupt
  CXINTEbits.TX0IE = 1; // Enable TXB0 interrupt
  CXINTEbits.ERRIE = 1; // Enable Error interrupt

  // DPARKER - Zero all the counters in the error structure.
  
  ETMCanBufferInitialize(&etm_can_rx_message_buffer);
  ETMCanBufferInitialize(&etm_can_tx_message_buffer);

  
  // ---------------- Set up CAN Control Registers ---------------- //
  
  // Set Baud Rate
  CXCTRL = CXCTRL_CONFIG_MODE_VALUE;
  while(CXCTRLbits.OPMODE != 4);
  
  CXCFG1 = ETM_CAN_CXCFG1_VALUE;
  CXCFG2 = CXCFG2_VALUE;
  
  
  // Load Mask registers for RX0 and RX1
  CXRXM0SID = ETM_CAN_SLAVE_RX0_MASK;
  CXRXM1SID = ETM_CAN_SLAVE_RX1_MASK;

  // Load Filter registers
  CXRXF0SID = ETM_CAN_MSG_LVL_FILTER;
  CXRXF1SID = ETM_CAN_MSG_SYNC_FILTER;
  CXRXF2SID = (ETM_CAN_MSG_SLAVE_FILTER | (ETM_CAN_MY_ADDRESS << 3));
  CXRXF3SID = ETM_CAN_MSG_FILTER_OFF;
  CXRXF4SID = ETM_CAN_MSG_FILTER_OFF;
  CXRXF5SID = ETM_CAN_MSG_FILTER_OFF;

  // Set Transmitter Mode
  CXTX0CON = CXTXXCON_VALUE_LOW_PRIORITY;
  CXTX1CON = CXTXXCON_VALUE_MEDIUM_PRIORITY;
  CXTX2CON = CXTXXCON_VALUE_HIGH_PRIORITY;

  CXTX0DLC = CXTXXDLC_VALUE;
  CXTX1DLC = CXTXXDLC_VALUE;
  CXTX2DLC = CXTXXDLC_VALUE;

  
  // Set Receiver Mode
  CXRX0CON = CXRXXCON_VALUE;
  CXRX1CON = CXRXXCON_VALUE;
  
  // Switch to normal operation
  CXCTRL = CXCTRL_OPERATE_MODE_VALUE;
  while(CXCTRLbits.OPMODE != 0);
  
  // Enable Can interrupt
  _CXIE = 1;

  // Configure T2
  T2CON = T2CON_VALUE;
  PR2 = PR2_VALUE;  
  TMR2 = 0;
  _T2IF = 0;
  _T2IE = 0;
  T2CONbits.TON = 1;

  // Configure T3
  T3CON = T3CON_VALUE;
  PR3 = PR3_VALUE;
  _T3IF = 0;
  _T3IE = 0;
  T3CONbits.TON = 1;

  etm_can_my_configuration.agile_number_high_word = ETM_CAN_AGILE_ID_HIGH;
  etm_can_my_configuration.agile_number_low_word  = ETM_CAN_AGILE_ID_LOW;
  etm_can_my_configuration.agile_dash             = ETM_CAN_AGILE_DASH;
  etm_can_my_configuration.agile_rev_ascii        = ETM_CAN_AGILE_REV;
  etm_can_my_configuration.serial_number          = ETM_CAN_SERIAL_NUMBER;
}




// Can interrupt ISR for slave modules

void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _CXInterrupt(void) {
  ETMCanMessage can_message;
    
  _CXIF = 0;
  local_can_errors.isr_entered++;
  
  if(CXRX0CONbits.RXFUL) {
    /*
      A message has been received in Buffer Zero
    */
    if (!CXRX0CONbits.FILHIT0) {
      // The command was received by Filter 0
      // It is a Next Pulse Level Command
      local_can_errors.rx_0_filt_0++;
      ETMCanRXMessage(&can_message, &CXRX0CON);
      etm_can_next_pulse_level = can_message.word2;
      etm_can_next_pulse_count = can_message.word3;
    } else {
      // The commmand was received by Filter 1
      // The command is a sync command.
      local_can_errors.rx_0_filt_1++;
      ETMCanRXMessage(&can_message, &CXRX0CON);
      ETMCanSlaveDoSync(&can_message);
    }
    CXINTFbits.RX0IF = 0; // Clear the Interuppt Status bit
  }
    
  if(CXRX1CONbits.RXFUL) {
    /* 
       A message has been recieved in Buffer 1
       This command gets pushed onto the command message buffer
    */
    local_can_errors.rx_1_filt_2++;
    ETMCanRXMessageBuffer(&etm_can_rx_message_buffer, &CXRX1CON);
    CXINTFbits.RX1IF = 0; // Clear the Interuppt Status bit
  }

  if ((!CXTX0CONbits.TXREQ) && (ETMCanBufferNotEmpty(&etm_can_tx_message_buffer))) {
    /*
      TX0 is empty and there is a message waiting in the transmit message buffer
      Load the next message into TX0
    */
    ETMCanTXMessageBuffer(&etm_can_tx_message_buffer, &CXTX0CON);
    CXINTFbits.TX0IF = 0;
    local_can_errors.tx_0++;
  }
  
  if (CXINTFbits.ERRIF) {
    // There was some sort of CAN Error
    // DPARKER - figure out which error and fix/reset
    local_can_errors.error_flag++;
    CXINTFbits.ERRIF = 0;
  } else {
    // FLASH THE CAN LED
    if (PIN_CAN_OPERATION_LED) {
      PIN_CAN_OPERATION_LED = 0;
    } else {
      PIN_CAN_OPERATION_LED = 1;
    }
  }
  local_can_errors.CXEC_reg = CXEC;
}

