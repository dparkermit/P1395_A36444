#ifndef __A36444_SETTINGS_H
#define __A36444_SETTINGS_H


// Settings for the HV Lambda 
#define HV_LAMBDA_MAX_VPROG            19000
#define HV_LAMBDA_MIN_VPROG            3000
#define HV_LAMBDA_DAC_ZERO_OUTPUT      0x0000    // This will set the DAC output to Zero (program of Zero as well)



#define LAMBDA_HEATSINK_OVER_TEMP      57000     // 57 Deg C
#define TRIP_COUNTER_100mS             10
#define TRIP_COUNTER_1Sec              100




#define LAMBDA_HOLDOFF_TIME_US         1000        // 400 uS
#define LAMBDA_MAX_CHARGE_TIME_US      2400      // 2.4mS
#define RETRIGGER_BLANKING_US          1100       // 500us


#define HV_ON_LAMBDA_SET_POINT_REFRESH_RATE_WHEN_NOT_PULSING            200             // 2 seconds


#define POWER_UP_DELAY   500                            // 5 Seconds
#define TIME_WAIT_FOR_LAMBDA_TO_SET_FAULT_OUTPUTS   10  // 100mS



#define PWR_5V_OVER_FLT        5200                   // 5.2 V
#define PWR_5V_UNDER_FLT       4800                   // 4.8 V

#define PWR_15V_OVER_FLT       15500                  // 15.5 V
#define PWR_15V_UNDER_FLT      14500                  // 14.5 V

#define PWR_NEG_15V_OVER_FLT   15500                  // -15.5 V
#define PWR_NEG_15V_UNDER_FLT  14500                  // -14.5 V

#define ADC_DAC_TEST_VALUE     0x8000                 // Default Test Value
#define ADC_DAC_TEST_OVER_FLT  0x8200                 // 1.01562 of test value 
#define ADC_DAC_TEST_UNDER_FLT 0x7E00                 // .984375 of test value

#endif
