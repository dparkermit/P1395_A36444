#ifndef __A36507_H
#define __A36507_H

#include <p30f6014a.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include "ETM_CAN_PUBLIC.h"
#include "ETM_ANALOG.h"



/*
  Hardware Module Resource Usage

  CAN1   - Used/Configured by ETM CAN 
  Timer2 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer3 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by LTC265X Module
  I2C    - Used/Configured by EEPROM Module


  Timer1 - Used to time to Lambda Charge Time and the Lambda Inhibit Time
  Timer5 - Used for 10msTicToc

  ADC Module - See Below For Specifics

*/


// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*

  RA12 - Fiber Trigger IN
  RA13 - Fiber Energy Select
  RA14 - Auto Inhibit Detect
  RA15 - Lambda EOC

  RD8  - Lambda Not Powered
  RD9  - Lambda Over Temp FLT
  RD10 - Lamdba Interlock FLT
  RD12 - Lambda Load FLT
  RD13 - Lambda Sum FLT
  RD14 - Lambda Phase Loss FLT
  RD15 - Lambda HV ON Readback
  
  RG14 - Reset Detect

  Analog Input Pins


  Pins that are overidden by a hardware module and should be left as inputs during port configuration
  RA9  ADC VREF-
  RA10 ADC VREF+

  RB0 PROGRAM
  RB1 PROGRAM
  RB3  - Analog Input
  RB4  - Analog Input
  RB5  - Analog Input
  RB6  - Analog Input
  RB12 - Analog Input
  RB13 - Analog Input
  RB14 - Analog Input
  RB15 - Analog Input

  RF0 CAN 1
  RF1 CAN 1
  RF6 SPI 1
  RF7 SPI 1
  RF8 SPI 1

  RG2 I2C
  RG3 I2C

  Pins that are configured by other software modules and should be left as inputs during port configuration
  RC1  (DAC LDAC)
  RG15 (DAC CS/LD)
  

*/

//   ------------------  Digital Output Pins ---------------
/*
  
  RD11 - Lamdba Voltage Select
  RD0 - Lambda Inhibit (This is also Output Compare 1 - If we want to use that module to generate Inhibit signal)
  RD1 - Lambda Enable


  RA7 - LED Operational
  RB8 - Test Point E
  RB9 - Test Point F
  RF4 - Test Point A
  RF5 - Test Point B
  RG0 - Test Point C
  RG1 - Test Point D
  RG12 - LED A RED
  RG13 - LED B GREEN
  
*/


#define A36444_TRISA_VALUE 0b1111011000000000 
#define A36444_TRISB_VALUE 0b1111000001111011 
#define A36444_TRISC_VALUE 0b0000000000000010 
#define A36444_TRISD_VALUE 0b1111011100000000 
#define A36444_TRISF_VALUE 0b0000000111000011 
#define A36444_TRISG_VALUE 0b1100000000001100



// -------- Digital Input Pins ----------//
#define PIN_FIBER_TRIGGER_IN                  _RA12
#define PIN_FIBER_ENERGY_SELECT               _RA13
#define PIN_AUTO_INHIBIT                      _RA14

#define PIN_LAMBDA_EOC                        _RA15
#define PIN_LAMBDA_HV_ON_READBACK             _RD15
#define PIN_LAMBDA_NOT_POWERED                _RD8

#define PIN_LAMBDA_SUM_FLT                    _RD13
#define PIN_LAMBDA_PHASE_LOSS_FLT             _RD14
#define PIN_LAMBDA_OVER_TEMP_FLT              _RD9
#define PIN_LAMBDA_INTERLOCK_FLT              _RD10
#define PIN_LAMBDA_LOAD_FLT                   _RD12



#define PIN_RESET_DETECT                      _RG14


#define ILL_HIGH_ENERGY_SELECTED              1
#define ILL_LAMBDA_AT_EOC                     1
#define ILL_LAMBDA_NOT_POWERED                1
#define ILL_LAMBDA_HV_ON                      1
#define ILL_LAMBDA_FAULT_ACTIVE               1




// ------- Digital Output Pins ---------//

#define PIN_LAMBDA_VOLTAGE_SELECT             _LATD11
#define PIN_LAMBDA_INHIBIT                    _LATD0
#define PIN_LAMBDA_ENABLE                     _LATD1

#define PIN_LED_OPERATIONAL_GREEN             _LATA7
#define PIN_LED_A_RED                         _LATG12
#define PIN_LED_B_GREEN                       _LATG13  // This is is configured by the CAN module to flash on CAN Bus activity

#define PIN_OUT_TP_A                          _LATF4
#define PIN_OUT_TP_B                          _LATF5
#define PIN_OUT_TP_C                          _LATG0
#define PIN_OUT_TP_D                          _LATG1
#define PIN_OUT_TP_E                          _LATB8
#define PIN_OUT_TP_F                          _LATB9

#define OLL_LED_ON                            0
#define OLL_SELECT_HIGH_ENERGY_MODE           1
#define OLL_INHIBIT_LAMBDA                    1
#define OLL_ENABLE_LAMBDA                     1


// ------------------------ CONFIGURE ADC MODULE ------------------- //

// ----------------- ANALOG INPUT PINS ---------------- //
/* 
   AN3 - Lambda Vmon
   AN4 - Lambda Heat Sink Temp
   AN5 - Lambda VPeak
   AN6 - Lambda Imon
   
   AN12 - ADC Test Input
   AN13 - 5V Mon
   AN14 - +15V Mon
   AN15 - -15V Mon
   
*/

/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 6 ADC Clock so total sample time is 9.0uS
  Conversion rate of 111KHz (13.888 Khz per Channel), 138 Samples per 10mS interrupt
  8 Samples per Interrupt, use alternating buffers
  Scan Through Selected Inputs

  // DPARKER it may be easier to read all the inputs all the time instead of reconfiguring the ADC, we don't need the high speed reading

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN3 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN3 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING          (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN12_ANA & ENABLE_AN13_ANA & ENABLE_AN14_ANA & ENABLE_AN15_ANA)
#define ADCSSL_SETTING_SCAN_ALL (SCAN_ALL)
#define ADCSSL_SETTING_STARTUP  (SKIP_SCAN_AN3 & SKIP_SCAN_AN5 & SKIP_SCAN_AN5 & SKIP_SCAN_AN6)
#define ADCSSL_SETTING_OPERATE  (SKIP_SCAN_AN12 & SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15)


/* 
   ---------- TMR1 Configuration -----------
   Timer1 - Used to time to Lambda Charge Time and the Lambda Inhibit Time
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/
    
#define T1CON_VALUE                    (T1_OFF & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_8 & T1_SOURCE_INT)
#define TMR1_DELAY_HOLDOFF_US          52        // 52uS
#define TMR1_LAMBDA_CHARGE_TIME_US     2400      // 2.400ms
#define TMR1_DELAY_HOLDOFF             (FCY_CLK_MHZ*TMR1_DELAY_HOLDOFF_US/8)    
#define TMR1_LAMBDA_CHARGE_PERIOD      (FCY_CLK_MHZ*TMR1_LAMBDA_CHARGE_TIME_US/8)




/* 
   TMR5 Configuration
   Timer5 - Used for 10msTicToc
   Period should be set to 10mS
   With 10Mhz Clock, x8 multiplier will yield max period of 17.7mS, 2.71uS per tick
*/

#define T5CON_VALUE                    (T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT)
#define PR5_PERIOD_US                  10000   // 10mS
#define PR5_VALUE_10_MILLISECONDS      (FCY_CLK_MHZ*PR5_PERIOD_US/8)



// -------------------- A36444 STATUS BIT CONFIGURATION ------------------------ //
#define STATUS_LAMBDA_AT_EOC                   STATUS_BIT_USER_DEFINED_8
#define STATUS_BIT_SOFTWARE_DISABLE            STATUS_BIT_USER_DEFINED_9
#define STATUS_LAMBDA_READBACK_HV_OFF          STATUS_BIT_USER_DEFINED_10
#define STATUS_LAMBDA_HIGH_ENERGY              STATUS_BIT_USER_DEFINED_11
#define STATUS_LAMBDA_NOT_POWERED              STATUS_BIT_USER_DEFINED_12



// -------------------- A36444 FAULTS/WARNINGS CONFIGURATION-------------------- //
#define FAULT_LAMBDA_SUM_FAULT                 FAULT_BIT_USER_DEFINED_1
#define FAULT_LAMBDA_PHASE_LOSS_FAULT          FAULT_BIT_USER_DEFINED_2
#define FAULT_LAMBDA_OVER_TEMP_FAULT           FAULT_BIT_USER_DEFINED_3
#define FAULT_LAMBDA_INTERLOCK_FAULT           FAULT_BIT_USER_DEFINED_4
#define FAULT_LAMBDA_LOAD_FAULT                FAULT_BIT_USER_DEFINED_5
#define FAULT_LAMBDA_EOC_WARNING               FAULT_BIT_USER_DEFINED_6
#define FAULT_LAMBDA_ANALOG_TEMP_OOR           FAULT_BIT_USER_DEFINED_7


// -------------------- FAULT CONFIGURATION -------------------------------------- //
#define A36444_INHIBIT_MASK        0b0000000000000000  
#define A36444_FAULT_MASK          0b0000000000000000  

//#define A36444_INHIBIT_MASK        0b0001011000000100  
//#define A36444_FAULT_MASK          0b0000000000000011  







typedef struct {
  AnalogInput analog_input_lambda_vmon;               // 1V per LSB
  AnalogInput analog_input_lambda_vpeak;              // 1V per LSB
  AnalogInput analog_input_lambda_imon;               // 100uA per LSB
  AnalogInput analog_input_lambda_heat_sink_temp;     // 1 mili Deg C per LSB

  AnalogInput analog_input_5v_mon;                    // 1mV per LSB
  AnalogInput analog_input_15v_mon;                   // 1mV per LSB
  AnalogInput analog_input_neg_15v_mon;               // 1mV per LSB
  AnalogInput analog_input_pic_adc_test_dac;          // 62.5uV per LSB


  AnalogOutput analog_output_high_energy_vprog;       // 1V per LSB
  AnalogOutput analog_output_low_energy_vprog;        // 1V per LSB

  AnalogOutput analog_output_spare;                   // 1mV per LSB
  AnalogOutput analog_output_adc_test;                // 62.5uV per LSB

  unsigned int accumulator_counter;
  unsigned int adc_ignore_current_sample;
  unsigned int eoc_not_reached_count;
  unsigned int control_state;
  unsigned int led_divider;

} LambdaControlData;

extern LambdaControlData global_data_A36444;



#define STATE_STARTUP                0x10
#define STATE_WAITING_FOR_CONFIG     0x20
#define STATE_STANDBY                0x30
#define STATE_OPERATE                0x40
#define STATE_FAULT                  0x50



#endif
