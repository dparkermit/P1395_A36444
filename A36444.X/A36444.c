#include "A36444.h"
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

#define CALIBRATION_DATA_START_REGISTER             0x400

unsigned int ETMCanEEPromReadWord(unsigned int register_address);
void ETMCanReadCalibrationAnalogInput(AnalogInput* ptr_analog_input, unsigned char input_channel);
void ETMCanReadCalibrationAnalogOutput(AnalogOutput* ptr_analog_output, unsigned char output_channel);



void InitializeA36444(void);


LTC265X U14_LTC2654;
ETMEEProm U3_M24LC64F;
LambdaControlData global_data_A36444;


int main(void) {
  

  
  ETMCanInitialize();

  InitializeA36444();

  
  //#define ETM_CAN_REGISTER_CALIBRATION_TEST          0x0400
  //#define ETM_CAN_REGISTER_LOCAL_TEST                0x0100
  

  
  while (1) {
    ETMCanDoCan();
  }
}


#define A36444_INHIBIT_MASK        0b0001011000000100  
#define A36444_FAULT_MASK          0b0000000000000011  



void InitializeA36444(void) {
  
  etm_can_status_register.status_word_0 = 0x0000;
  etm_can_status_register.status_word_1 = 0x0000;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000; 
  etm_can_status_register.status_word_0_inhbit_mask = A36444_INHIBIT_MASK;
  etm_can_status_register.status_word_1_fault_mask  = A36444_FAULT_MASK;

  // Configure Inhibit Interrupt
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT1EP = 0; // Positive Transition
  
  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)
  
  
  // Initialize all I/O Registers
  TRISA = A36444_TRISA_VALUE;
  TRISB = A36444_TRISB_VALUE;
  TRISC = A36444_TRISC_VALUE;
  TRISD = A36444_TRISD_VALUE;
  TRISF = A36444_TRISF_VALUE;
  TRISG = A36444_TRISG_VALUE;

  // Initialize TMR1
  T1CON = T1CON_VALUE;
  TMR1  = 0;
  _T1IF = 0;
  
  // Initialize TMR5
  T5CON = T5CON_VALUE;
  TMR5  = 0;
  _T5IF = 0;
  PR5   = PR5_VALUE_10_MILLISECONDS;
  
  // Initialize interal ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING_SCAN_ALL;    // Set which analog pins are scanned
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

  
  // Initialize LTC DAC
  SetupLTC265X(&U14_LTC2654, ETM_SPI_PORT_1, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
  
  // Initialize the External EEprom
  ETMEEPromConfigureDevice(&U3_M24LC64F, EEPROM_I2C_ADDRESS_0, I2C_PORT, EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD);

  // LOAD Scaling and Calibration data for analog Inputs and outputs
  // Fixed values are generate from spreadsheet.
  // Calibration Data is read from EEProm
  global_data_A36444.analog_input_lambda_vmon.fixed_scale  = MACRO_DEC_TO_SCALE_FACTOR_16(.28125);
  global_data_A36444.analog_input_lambda_vmon.fixed_offset = 0;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_lambda_vmon, 0xFF);
  
  global_data_A36444.analog_input_lambda_vpeak.fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(.28125);
  global_data_A36444.analog_input_lambda_vpeak.fixed_offset = 0;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_lambda_vpeak, 0xFF);
  
  global_data_A36444.analog_input_lambda_imon.fixed_scale  = MACRO_DEC_TO_SCALE_FACTOR_16(.40179);
  global_data_A36444.analog_input_lambda_imon.fixed_offset = 0;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_lambda_imon, 0xFF);

  global_data_A36444.analog_input_lambda_heat_sink_temp.fixed_scale  = MACRO_DEC_TO_SCALE_FACTOR_16(.78125);
  global_data_A36444.analog_input_lambda_heat_sink_temp.fixed_offset = 10000;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_lambda_heat_sink_temp, 0xFF);

  global_data_A36444.analog_input_5v_mon.fixed_scale  = MACRO_DEC_TO_SCALE_FACTOR_16(.10417);
  global_data_A36444.analog_input_5v_mon.fixed_offset = 0;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_5v_mon, 0xFF);

  global_data_A36444.analog_input_15v_mon.fixed_scale  = MACRO_DEC_TO_SCALE_FACTOR_16(.26500);
  global_data_A36444.analog_input_15v_mon.fixed_offset = 0;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_15v_mon, 0xFF);

  global_data_A36444.analog_input_neg_15v_mon.fixed_scale  = MACRO_DEC_TO_SCALE_FACTOR_16(.15625);
  global_data_A36444.analog_input_neg_15v_mon.fixed_offset = 0;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_neg_15v_mon, 0xFF);

  global_data_A36444.analog_input_pic_adc_test_dac.fixed_scale  = MACRO_DEC_TO_SCALE_FACTOR_16(1);
  global_data_A36444.analog_input_pic_adc_test_dac.fixed_offset = 0;
  ETMCanReadCalibrationAnalogInput(&global_data_A36444.analog_input_pic_adc_test_dac, 0xFF);
  


  
  global_data_A36444.analog_output_high_energy_vprog.fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(2.96296);
  global_data_A36444.analog_output_high_energy_vprog.fixed_offset = 0;
  ETMCanReadCalibrationAnalogOutput(&global_data_A36444.analog_output_high_energy_vprog, 0xFF);

  global_data_A36444.analog_output_low_energy_vprog.fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(2.96296);
  global_data_A36444.analog_output_low_energy_vprog.fixed_offset = 0;
  ETMCanReadCalibrationAnalogOutput(&global_data_A36444.analog_output_low_energy_vprog, 0xFF);
  
  global_data_A36444.analog_output_spare.fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(5.33333);
  global_data_A36444.analog_output_spare.fixed_offset = 0;
  ETMCanReadCalibrationAnalogOutput(&global_data_A36444.analog_output_spare, 0xFF);

  global_data_A36444.analog_output_adc_test.fixed_scale = MACRO_DEC_TO_SCALE_FACTOR_16(1.00000);
  global_data_A36444.analog_output_adc_test.fixed_offset = 0;
  ETMCanReadCalibrationAnalogOutput(&global_data_A36444.analog_output_adc_test, 0xFF);
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


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}





void ETMCanReadCalibrationAnalogInput(AnalogInput* ptr_analog_input, unsigned char input_channel) {
  unsigned int cal_data_address;
  if (input_channel == 0xFF) {
    // this is a special case, set the calibration to gain of 1 and offset of zero
    ptr_analog_input->calibration_internal_scale = MACRO_DEC_TO_CAL_FACTOR_2(1);
    ptr_analog_input->calibration_internal_offset = 0;
    ptr_analog_input->calibration_external_scale = MACRO_DEC_TO_CAL_FACTOR_2(1);
    ptr_analog_input->calibration_external_offset = 0;
  } else {
    // load the adc internal calibration data 
    cal_data_address = CALIBRATION_DATA_START_REGISTER + input_channel*2;
    ptr_analog_input->calibration_internal_offset = ETMCanEEPromReadWord(cal_data_address);
    ptr_analog_input->calibration_internal_scale  = ETMCanEEPromReadWord(cal_data_address+1);
    // load_the adc external calibration data
    cal_data_address = CALIBRATION_DATA_START_REGISTER + 0x20 + input_channel*2;
    ptr_analog_input->calibration_external_offset = ETMCanEEPromReadWord(cal_data_address);
    ptr_analog_input->calibration_external_scale  = ETMCanEEPromReadWord(cal_data_address+1);    
  }
}

void ETMCanReadCalibrationAnalogOutput(AnalogOutput* ptr_analog_output, unsigned char output_channel) {
  unsigned int cal_data_address;
  if (output_channel == 0xFF) {
    // this is a special case, set the calibration to gain of 1 and offset of zero
    ptr_analog_output->calibration_internal_scale = MACRO_DEC_TO_CAL_FACTOR_2(1);
    ptr_analog_output->calibration_internal_offset = 0;
    ptr_analog_output->calibration_external_scale = MACRO_DEC_TO_CAL_FACTOR_2(1);
    ptr_analog_output->calibration_external_offset = 0;
  } else {
    // load the adc internal calibration data 
    cal_data_address = CALIBRATION_DATA_START_REGISTER + 0x40 + output_channel*2;
    ptr_analog_output->calibration_internal_offset = ETMCanEEPromReadWord(cal_data_address);
    ptr_analog_output->calibration_internal_scale  = ETMCanEEPromReadWord(cal_data_address+1);
    // load_the adc external calibration data
    cal_data_address = CALIBRATION_DATA_START_REGISTER + 0x60 + output_channel*2;
    ptr_analog_output->calibration_external_offset = ETMCanEEPromReadWord(cal_data_address);
    ptr_analog_output->calibration_external_scale  = ETMCanEEPromReadWord(cal_data_address+1);    
  }
}

unsigned int ETMCanEEPromReadWord(unsigned int register_address) {

#ifdef __EEPROM_INTERNAL
  //
#endif


#ifdef __EEPROM_EXTERNAL_FLASH
  ETMEEPromReadWord(&U3_M24LC64F, register_address);
#endif


#ifdef __EEPROM_EXTERNAL_FRAM
  //ETMEEPromReadWord(EXTERNAL_EEPROM_POINTER, register_address);
#endif

  return 0;
}
