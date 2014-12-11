#include "A36444.h"
#include "FIRMWARE_VERSION.h"
#include "ETM_EEPROM.h"
#include "LTC265X.h"

// This is firmware for the HV Lambda Board

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer 
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


LTC265X U14_LTC2654;
ETMEEProm U3_M24LC64F;
LambdaControlData global_data_A36444;


void InitializeA36444(void);
void DoStateMachine(void);
void EnableHVLambda(void);
void DisableHVLambda(void);
void DoA36444(void);


int main(void) {
  global_data_A36444.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}


void DoStateMachine(void) {
  switch (global_data_A36444.control_state) {
    
  case STATE_STARTUP:
    InitializeA36444();
    global_data_A36444.control_state = STATE_WAITING_FOR_CONFIG;
    break;
    
  case STATE_WAITING_FOR_CONFIG:
    ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_BOARD_WAITING_INITIAL_CONFIG);
    ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_SOFTWARE_DISABLE);
    DisableHVLambda();  
    while (global_data_A36444.control_state == STATE_WAITING_FOR_CONFIG) {
      DoA36444();
      ETMCanDoCan();
      
      if (!ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_BOARD_WAITING_INITIAL_CONFIG)) {
	global_data_A36444.control_state = STATE_STANDBY;
      }
      
      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	global_data_A36444.control_state = STATE_FAULT;
      }
    }
    break;

  case STATE_STANDBY:
    DisableHVLambda();  
    while (global_data_A36444.control_state == STATE_STANDBY) {
      DoA36444();
      ETMCanDoCan();
      
      if (!ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SOFTWARE_DISABLE)) {
	global_data_A36444.control_state = STATE_OPERATE;
      }
      
      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	global_data_A36444.control_state = STATE_FAULT;
      }
    }
    break;

  case STATE_OPERATE:
    EnableHVLambda(); 
    while (global_data_A36444.control_state == STATE_OPERATE) {
      DoA36444();
      ETMCanDoCan();
      
      if (global_data_A36444.run_post_pulse_process) {
	// Run this once after each pulse
	
	// Update the HV Lambda Program Values
	ETMAnalogScaleCalibrateDACSetting(&global_data_A36444.analog_output_high_energy_vprog);
	ETMAnalogScaleCalibrateDACSetting(&global_data_A36444.analog_output_low_energy_vprog);
	WriteLTC265XTwoChannels(&U14_LTC2654,
				LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36444.analog_output_high_energy_vprog.dac_setting_scaled_and_calibrated,
				LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36444.analog_output_low_energy_vprog.dac_setting_scaled_and_calibrated);
	
	
	global_data_A36444.no_pulse_counter = 0;
	global_data_A36444.run_post_pulse_process = 0;
      }
      
      
      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SOFTWARE_DISABLE)) {
	global_data_A36444.control_state = STATE_STANDBY;
      }
      
      if (ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	global_data_A36444.control_state = STATE_FAULT;
      }
    }
    break;


  case STATE_FAULT:
    DisableHVLambda();  
    while (global_data_A36444.control_state == STATE_FAULT) {
      DoA36444();
      ETMCanDoCan();
      if (!ETMCanCheckBit(etm_can_status_register.status_word_0, STATUS_BIT_SUM_FAULT)) {
	// The faults have been cleared
	global_data_A36444.control_state = STATE_WAITING_FOR_CONFIG;
      }
    }
    break;
        
  default:
    global_data_A36444.control_state = STATE_FAULT;
    break;
  }
  
}


void DoA36444(void) {
  
  if (_T5IF) {
    // Timer has expired so execute the scheduled code (should be once every 10ms unless the configuration file is changes
    _T5IF = 0;
    

    // If the system is faulted or inhibited set the red LED
    if (etm_can_status_register.status_word_0 & 0x0003) {
      // The board is faulted or inhibiting the system
      PIN_LED_A_RED = OLL_LED_ON;
    } else {
      PIN_LED_A_RED = !OLL_LED_ON;
    }

    // Update the digital input status pins
    if (PIN_LAMBDA_EOC == ILL_LAMBDA_AT_EOC) {
      ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_LAMBDA_AT_EOC);
    } else {
      ETMCanClearBit(&etm_can_status_register.status_word_0, STATUS_LAMBDA_AT_EOC);
    }
    
    if (PIN_LAMBDA_HV_ON_READBACK != ILL_LAMBDA_HV_ON) {
      ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_LAMBDA_READBACK_HV_OFF);
    } else {
      ETMCanClearBit(&etm_can_status_register.status_word_0, STATUS_LAMBDA_READBACK_HV_OFF);
    }

    if (PIN_LAMBDA_NOT_POWERED == ILL_LAMBDA_NOT_POWERED) {
      ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_LAMBDA_NOT_POWERED);
    } else {
      ETMCanClearBit(&etm_can_status_register.status_word_0, STATUS_LAMBDA_NOT_POWERED);
    }


    if (global_reset_faults) {
      etm_can_status_register.status_word_1 = 0x0000;
      global_reset_faults = 0;
    }

    // Update the digital input fault pins
    if (PIN_LAMBDA_SUM_FLT == ILL_LAMBDA_FAULT_ACTIVE) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_LAMBDA_SUM_FAULT);
    }

    if (PIN_LAMBDA_PHASE_LOSS_FLT == ILL_LAMBDA_FAULT_ACTIVE) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_LAMBDA_PHASE_LOSS_FAULT);
    }

    if (PIN_LAMBDA_OVER_TEMP_FLT == ILL_LAMBDA_FAULT_ACTIVE) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_LAMBDA_OVER_TEMP_FAULT);
    }

    if (PIN_LAMBDA_INTERLOCK_FLT == ILL_LAMBDA_FAULT_ACTIVE) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_LAMBDA_INTERLOCK_FAULT);
    }

    if (PIN_LAMBDA_LOAD_FLT == ILL_LAMBDA_FAULT_ACTIVE) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_LAMBDA_LOAD_FAULT);
    }

    // Do Math on the ADC inputs
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_lambda_vmon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_lambda_vpeak);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_lambda_imon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_lambda_heat_sink_temp);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_5v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_15v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_neg_15v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36444.analog_input_pic_adc_test_dac);

    etm_can_system_debug_data.debug_0 = global_data_A36444.analog_input_lambda_vmon.reading_scaled_and_calibrated;
    etm_can_system_debug_data.debug_1 = global_data_A36444.analog_input_lambda_vpeak.reading_scaled_and_calibrated;
    etm_can_system_debug_data.debug_2 = global_data_A36444.analog_input_lambda_imon.reading_scaled_and_calibrated;
    etm_can_system_debug_data.debug_3 = global_data_A36444.analog_input_lambda_heat_sink_temp.reading_scaled_and_calibrated;
    etm_can_system_debug_data.debug_4 = global_data_A36444.analog_input_5v_mon.reading_scaled_and_calibrated;
    etm_can_system_debug_data.debug_5 = global_data_A36444.analog_input_15v_mon.reading_scaled_and_calibrated;
    etm_can_system_debug_data.debug_6 = global_data_A36444.analog_input_neg_15v_mon.reading_scaled_and_calibrated;
    etm_can_system_debug_data.debug_7 = global_data_A36444.analog_input_pic_adc_test_dac.reading_scaled_and_calibrated;
  

    etm_can_system_debug_data.debug_8 = global_data_A36444.pulse_counter;

    // Look for faults on the Analog inputs
    if (ETMAnalogCheckOverAbsolute(&global_data_A36444.analog_input_lambda_heat_sink_temp)) {
      ETMCanSetBit(&etm_can_status_register.status_word_1, FAULT_LAMBDA_ANALOG_TEMP_OOR);
    }

    global_data_A36444.no_pulse_counter++;
    
    if (global_data_A36444.control_state != STATE_OPERATE) {
      // Update the HV Lambda Program Values
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36444.analog_output_high_energy_vprog);
      ETMAnalogScaleCalibrateDACSetting(&global_data_A36444.analog_output_low_energy_vprog);
      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36444.analog_output_high_energy_vprog.dac_setting_scaled_and_calibrated,
			      LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36444.analog_output_low_energy_vprog.dac_setting_scaled_and_calibrated);


      // Update the spare analog output and the DAC test output
      // DPARKER - may need to remove this
      WriteLTC265XTwoChannels(&U14_LTC2654,
			      LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36444.analog_output_spare.dac_setting_scaled_and_calibrated,
			      LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36444.analog_output_adc_test.dac_setting_scaled_and_calibrated);
      
    } else {
      if (global_data_A36444.no_pulse_counter >= HV_ON_LAMBDA_SET_POINT_REFRESH_RATE_WHEN_NOT_PULSING) {
	// A long time has passed without updating the Lambda Set points
	// Update the HV Lambda Program Values
	global_data_A36444.no_pulse_counter = 0;
	
	ETMAnalogScaleCalibrateDACSetting(&global_data_A36444.analog_output_high_energy_vprog);
	ETMAnalogScaleCalibrateDACSetting(&global_data_A36444.analog_output_low_energy_vprog);
	WriteLTC265XTwoChannels(&U14_LTC2654,
				LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36444.analog_output_high_energy_vprog.dac_setting_scaled_and_calibrated,
				LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36444.analog_output_low_energy_vprog.dac_setting_scaled_and_calibrated);
	
      } 
    }
    
  }
}




void InitializeA36444(void) {
  unsigned int n;


  // Initialize the status register and load the inhibit and fault masks
  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;
  etm_can_status_register.status_word_0_inhbit_mask = A36444_INHIBIT_MASK;
  etm_can_status_register.status_word_1_fault_mask  = A36444_FAULT_MASK;
  ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_BOARD_WAITING_INITIAL_CONFIG);
  ETMCanSetBit(&etm_can_status_register.status_word_0, STATUS_BIT_SOFTWARE_DISABLE);  

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;


  // Configure Inhibit Interrupt
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT1EP = 0; // Positive Transition

  
  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)


  // Configure T1 Inetrrupt
  _T1IP   = 5;
  
  // Initialize all I/O Registers
  TRISA = A36444_TRISA_VALUE;
  TRISB = A36444_TRISB_VALUE;
  TRISC = A36444_TRISC_VALUE;
  TRISD = A36444_TRISD_VALUE;
  TRISF = A36444_TRISF_VALUE;
  TRISG = A36444_TRISG_VALUE;

  // Flash LEDs at Startup
  PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
  PIN_LED_A_RED = !OLL_LED_ON;
  PIN_LED_B_GREEN = !OLL_LED_ON;
  

  for (n = 0; n < 5; n++) {
    PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_A_RED             = OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_B_GREEN           = OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_B_GREEN           = !OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_A_RED             = !OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

    PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    __delay32(600000);
    ClrWdt();

  }





  // Initialize TMR1
  TMR1  = 0;
  _T1IF = 0;
  T1CON = T1CON_VALUE;


  
  // Initialize TMR5
  PR5   = PR5_VALUE_10_MILLISECONDS;
  TMR5  = 0;
  _T5IF = 0;
  T5CON = T5CON_VALUE;


  
  
  // Initialize interal ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  // ADCSSL = ADCSSL_SETTING_SCAN_ALL;    // Set which analog pins are scanned
  ADCSSL = ADCSSL_SETTING_OPERATE;
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

  
  // Initialize LTC DAC
  SetupLTC265X(&U14_LTC2654, ETM_SPI_PORT_1, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);

  
  // Initialize the External EEprom
  ETMEEPromConfigureDevice(&U3_M24LC64F, EEPROM_I2C_ADDRESS_0, I2C_PORT, EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD);


  // Initialize the Can module
  ETMCanInitialize();


  // Initialize the Analog input data structures
  ETMAnalogInitializeInput(&global_data_A36444.analog_input_lambda_vmon, MACRO_DEC_TO_SCALE_FACTOR_16(.28125), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36444.analog_input_lambda_vpeak, MACRO_DEC_TO_SCALE_FACTOR_16(.28125), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION, 
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36444.analog_input_lambda_imon,  MACRO_DEC_TO_SCALE_FACTOR_16(.40179), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36444.analog_input_lambda_heat_sink_temp, MACRO_DEC_TO_SCALE_FACTOR_16(.78125), 10000, ANALOG_INPUT_NO_CALIBRATION,
			   LAMBDA_HEATSINK_OVER_TEMP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, TRIP_COUNTER_1Sec);

  ETMAnalogInitializeInput(&global_data_A36444.analog_input_5v_mon, MACRO_DEC_TO_SCALE_FACTOR_16(.10417), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36444.analog_input_15v_mon, MACRO_DEC_TO_SCALE_FACTOR_16(.26500), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36444.analog_input_neg_15v_mon, MACRO_DEC_TO_SCALE_FACTOR_16(.15625), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36444.analog_input_pic_adc_test_dac, MACRO_DEC_TO_SCALE_FACTOR_16(1), OFFSET_ZERO, ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP, NO_UNDER_TRIP, NO_TRIP_SCALE, NO_FLOOR, NO_COUNTER);



  // Initialize the Analog Output Data Structures
  ETMAnalogInitializeOutput(&global_data_A36444.analog_output_high_energy_vprog, MACRO_DEC_TO_SCALE_FACTOR_16(2.96296), OFFSET_ZERO, ANALOG_OUTPUT_NO_CALIBRATION,
			    HV_LAMBDA_MAX_VPROG, HV_LAMBDA_MIN_VPROG, HV_LAMBDA_DAC_ZERO_OUTPUT);

  ETMAnalogInitializeOutput(&global_data_A36444.analog_output_low_energy_vprog, MACRO_DEC_TO_SCALE_FACTOR_16(2.96296), OFFSET_ZERO, ANALOG_OUTPUT_NO_CALIBRATION,
			    HV_LAMBDA_MAX_VPROG, HV_LAMBDA_MIN_VPROG, HV_LAMBDA_DAC_ZERO_OUTPUT);

  ETMAnalogInitializeOutput(&global_data_A36444.analog_output_spare, MACRO_DEC_TO_SCALE_FACTOR_16(5.33333), OFFSET_ZERO, ANALOG_OUTPUT_NO_CALIBRATION,
			    10000, 0, 0);

  ETMAnalogInitializeOutput(&global_data_A36444.analog_output_adc_test, MACRO_DEC_TO_SCALE_FACTOR_16(1), OFFSET_ZERO, ANALOG_OUTPUT_NO_CALIBRATION,
			    0xFFFF, 0, 0);


  
}





void EnableHVLambda(void) {
  // Set the enable register in the DAC structure
  // The next time the DAC is updated it will be updated with the most recent high/low energy values
  global_data_A36444.analog_output_high_energy_vprog.enabled = 1;
  global_data_A36444.analog_output_low_energy_vprog.enabled = 1;
  
  // Set digital output to inhibit the lambda
  PIN_LAMBDA_INHIBIT = !OLL_INHIBIT_LAMBDA;
  
  // Set digital output to enable HV_ON of the lambda
  PIN_LAMBDA_ENABLE = OLL_ENABLE_LAMBDA;
}


void DisableHVLambda(void) {
  // Clear the enable register in the DAC structure
  // The next time the DAC is updated it will be updated with the "disabled" value
  global_data_A36444.analog_output_high_energy_vprog.enabled = 0;
  global_data_A36444.analog_output_low_energy_vprog.enabled = 0;
  
  // Set digital output to inhibit the lambda
  PIN_LAMBDA_INHIBIT = OLL_INHIBIT_LAMBDA;
  
  // Set digital output to disable HV_ON of the lambda
  PIN_LAMBDA_ENABLE = !OLL_ENABLE_LAMBDA;
}






void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  if (global_data_A36444.adc_ignore_current_sample) {
    // There was a pulse durring the sample sequence.  Throw the data away!!!
    global_data_A36444.adc_ignore_current_sample = 0;
  } else {
    // Copy Data From Buffer to RAM
    if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36444.analog_input_lambda_vmon.adc_accumulator           += ADCBUF0;
      global_data_A36444.analog_input_lambda_heat_sink_temp.adc_accumulator += ADCBUF1;
      global_data_A36444.analog_input_lambda_vpeak.adc_accumulator          += ADCBUF2;
      global_data_A36444.analog_input_lambda_imon.adc_accumulator           += ADCBUF3;

      global_data_A36444.analog_input_pic_adc_test_dac.adc_accumulator      += ADCBUF4;
      global_data_A36444.analog_input_5v_mon.adc_accumulator                += ADCBUF5;
      global_data_A36444.analog_input_15v_mon.adc_accumulator               += ADCBUF6;
      global_data_A36444.analog_input_neg_15v_mon.adc_accumulator           += ADCBUF7;

    } else {
      // read ADCBUF 8-15
      global_data_A36444.analog_input_lambda_vmon.adc_accumulator           += ADCBUF8;
      global_data_A36444.analog_input_lambda_heat_sink_temp.adc_accumulator += ADCBUF9;
      global_data_A36444.analog_input_lambda_vpeak.adc_accumulator          += ADCBUFA;
      global_data_A36444.analog_input_lambda_imon.adc_accumulator           += ADCBUFB;

      global_data_A36444.analog_input_pic_adc_test_dac.adc_accumulator      += ADCBUFC;
      global_data_A36444.analog_input_5v_mon.adc_accumulator                += ADCBUFD;
      global_data_A36444.analog_input_15v_mon.adc_accumulator               += ADCBUFE;
      global_data_A36444.analog_input_neg_15v_mon.adc_accumulator           += ADCBUFF;
      
    }
    
    global_data_A36444.accumulator_counter += 1;
    
    if (global_data_A36444.accumulator_counter >= 128) {

      global_data_A36444.analog_input_lambda_vmon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_lambda_vmon.filtered_adc_reading = global_data_A36444.analog_input_lambda_vmon.adc_accumulator;
      global_data_A36444.analog_input_lambda_vmon.adc_accumulator = 0;

      global_data_A36444.analog_input_lambda_heat_sink_temp.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_lambda_heat_sink_temp.filtered_adc_reading = global_data_A36444.analog_input_lambda_heat_sink_temp.adc_accumulator;
      global_data_A36444.analog_input_lambda_heat_sink_temp.adc_accumulator = 0;

      global_data_A36444.analog_input_lambda_vpeak.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_lambda_vpeak.filtered_adc_reading = global_data_A36444.analog_input_lambda_vpeak.adc_accumulator;
      global_data_A36444.analog_input_lambda_vpeak.adc_accumulator = 0;

      global_data_A36444.analog_input_lambda_imon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_lambda_imon.filtered_adc_reading = global_data_A36444.analog_input_lambda_imon.adc_accumulator;
      global_data_A36444.analog_input_lambda_imon.adc_accumulator = 0;

      global_data_A36444.analog_input_pic_adc_test_dac.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_pic_adc_test_dac.filtered_adc_reading = global_data_A36444.analog_input_pic_adc_test_dac.adc_accumulator;
      global_data_A36444.analog_input_pic_adc_test_dac.adc_accumulator = 0;

      global_data_A36444.analog_input_5v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_5v_mon.filtered_adc_reading = global_data_A36444.analog_input_5v_mon.adc_accumulator;
      global_data_A36444.analog_input_5v_mon.adc_accumulator = 0;

      global_data_A36444.analog_input_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_15v_mon.filtered_adc_reading = global_data_A36444.analog_input_15v_mon.adc_accumulator;
      global_data_A36444.analog_input_15v_mon.adc_accumulator = 0;

      global_data_A36444.analog_input_neg_15v_mon.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples 
      global_data_A36444.analog_input_neg_15v_mon.filtered_adc_reading = global_data_A36444.analog_input_neg_15v_mon.adc_accumulator;
      global_data_A36444.analog_input_neg_15v_mon.adc_accumulator = 0;


      global_data_A36444.accumulator_counter = 0;
    }
  }
}





void __attribute__((interrupt, shadow, no_auto_psv)) _INT3Interrupt(void) {
  /*    
    Assuming a sucessful trigger, this interupt will do the following conecptual steps
    (1) Inhibit the HV supply a set amount of time.
    (2) After the inhibt time has passed, !Inhibit the power supply, and start the EOC timer
    (3) Set the status bit that indicates a pulse occured
  */ 

  PIN_LAMBDA_INHIBIT = OLL_INHIBIT_LAMBDA;
  
  // Setup timer1 to time the inhibit period
  T1CONbits.TON = 0;
  TMR1 = 0;
  PR1 = TMR1_DELAY_HOLDOFF;
  T1CONbits.TON = 1;
  _T1IF = 0;

  if (global_data_A36444.run_post_pulse_process) {
    // We never completed the post pulse process.  
    // DPARKER, Increment some error
  }
  
  if (_T1IE) {
    // If timer one is enabled then we did not complete the Charge period before the next pulse
    // DPARKER, Increment some error
  }

  _T1IE = 0;

  while(!_T1IF);                                                   // what for the holdoff time to pass

  PIN_LAMBDA_INHIBIT = !OLL_INHIBIT_LAMBDA;

  // Set up Timer1 to produce interupt at end of charge period
  T1CONbits.TON = 0;
  TMR1 = 0;
  _T1IF = 0;
  _T1IE = 1;
  PR1 = (TMR1_LAMBDA_CHARGE_PERIOD - TMR1_DELAY_HOLDOFF);
  T1CONbits.TON = 1;

    
  
  
  global_data_A36444.run_post_pulse_process = 1;     // This tells the main control loop that a pulse has occured and that it should run the post pulse process once (and only once) 
  global_data_A36444.adc_ignore_current_sample = 1;  // This allows the internal ADC ISR to know that there was a pulse and to discard all the data from the sequence where the pulse occured

  global_data_A36444.pulse_counter++;
  _INT1IF = 0;
}


void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
  /*
    This interrupt indicates that the cap charger should have finished charging and it is time to enable the trigger pulse.

    This interrupt is called X us after charging starts
    If the lambda is not at EOC, it does not enable the trigger and sets the Lambda EOC Timeout Fault bit
    If the lambda is at EOC, It enables the trigger & sets status bits to show that the lambda is not charging and that the system is ready to fire.    
  */
  
  _T1IF = 0;         // Clear the interrupt flag
  _T1IE = 0;         // Disable the interrupt (This will be enabled the next time that a capacitor charging sequence starts)
  T1CONbits.TON = 0;   // Stop the timer from incrementing (Again this will be restarted with the next time the capacitor charge sequence starts)
  
  if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
    __delay32(DELAY_TCY_5US);
    if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
      __delay32(DELAY_TCY_5US);
      if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
	__delay32(DELAY_TCY_5US);
	if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
	  __delay32(DELAY_TCY_5US);
	  if (PIN_LAMBDA_EOC != ILL_LAMBDA_AT_EOC) {
	    global_data_A36444.eoc_not_reached_count++;
	  }
	} 
      }
    }
  }
}  



void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}



