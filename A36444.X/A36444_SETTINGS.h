#ifndef __A36444_SETTINGS_H
#define __A36444_SETTINGS_H


// Settings for the HV Lambda 
#define HV_LAMBDA_MAX_VPROG            18000
#define HV_LAMBDA_MIN_VPROG            3000
#define HV_LAMBDA_DAC_ZERO_OUTPUT      0x0000    // This will set the DAC output to Zero (program of Zero as well)



#define LAMBDA_HEATSINK_OVER_TEMP      57000     // 57 Deg C
#define TRIP_COUNTER_100mS             10
#define TRIP_COUNTER_1Sec              100




#define LAMBDA_HOLDOFF_TIME_US         52        // 52 uS
#define LAMBDA_MAX_CHARGE_TIME_US      2400      // 2.4mS


#endif
