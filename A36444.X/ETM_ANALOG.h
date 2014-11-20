#ifndef __ETM_ANALOG
#define __ETM_ANALOG
#include "ETM_SCALE.h"


/*
  Possible faults on analog input
  
  Over condition - Absolute (Single Sample) - Does this need some sort of delay
  Under condition - Absolute (Single Sample) - Does this need some sort of delay
  
  Over condition - Relative (N samples out of range)
  Under condition - Relative (N samples out of range)

  Do we want to build the fault masking into the analog input module

  DPARKER - I think it makes sense to have a configuration integer that says to ignore a particular fault
  
*/


typedef struct {
  unsigned long adc_accumulator;
  unsigned int filtered_adc_reading;
  unsigned int reading_scaled_and_calibrated;

  // -------- These are used to calibrate and scale the ADC Reading to Engineering Units ---------
  unsigned int fixed_scale;
  signed int   fixed_offset;
  unsigned int calibration_internal_scale;
  signed int   calibration_internal_offset;
  unsigned int calibration_external_scale;
  signed int   calibration_external_offset;


  // --------  These are used for fault detection ------------------ 
  unsigned int over_trip_point_absolute;          // If the value exceeds this it will trip immediatly
  unsigned int under_trip_point_absolute;         // If the value is less than this it will trip immediatly
  unsigned int target_value;                      // This is the target value (probably set point) in engineering units
  unsigned int relative_trip_point_scale;         // This will be something like 5%, 10%, ect
  unsigned int relative_trip_point_floor;         // If target_value * relative_trip_point_floor is less than the floor value, the floor value will be used instead
                                                  // Trip Points = target_value +/- GreaterOf [(target_value*relative_trip_point_scale) OR (relative_trip_point_floor)] 
  unsigned int over_trip_counter;                 // This counts the number of samples over the relative_over_trip_point (will decrement each sample over test is false)
  unsigned int under_trip_counter;                // This counts the number of samples under the relative_under_trip_point (will decrement each sample under test is false)
  unsigned int relative_counter_fault_limit;      // The over / under trip counter must reach this value to generate a fault

} AnalogInput;



typedef struct {
  unsigned int set_point;
  unsigned int dac_setting_scaled_and_calibrated;
  unsigned int enabled;

  unsigned int max_set_point;
  unsigned int min_set_point;
  unsigned int disabled_dac_set_point;

  // -------- These are used to calibrate and scale the ADC Reading to Engineering Units ---------
  unsigned int fixed_scale;
  signed int   fixed_offset;
  unsigned int calibration_internal_scale;
  signed int   calibration_internal_offset;
  unsigned int calibration_external_scale;
  signed int   calibration_external_offset;

  // DPARKER add minimum and maximum set points
  // DPARKER figure out when to check (probably in EMScaleCalibrateDACSetting

} AnalogOutput;


void ETMAnalogScaleCalibrateDACSetting(AnalogOutput* ptr_analog_output);
void ETMAnalogScaleCalibrateADCReading(AnalogInput* ptr_analog_input);

void ETMAnalogSetOutput(AnalogOutput* ptr_analog_output, unsigned int new_set_point);


unsigned int ETMAnalogCheckOverAbsolute(AnalogInput* ptr_analog_input);
unsigned int ETMAnalogCheckUnderAbsolute(AnalogInput* ptr_analog_input);
unsigned int ETMAnalogCheckOverRelative(AnalogInput* ptr_analog_input);
unsigned int ETMAnalogCheckUnderRelative(AnalogInput* ptr_analog_input);



#endif
