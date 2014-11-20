#include "ETM_ANALOG.h"




void ETMAnalogScaleCalibrateDACSetting(AnalogOutput* ptr_analog_output) {
  unsigned int temp;
  // Convert from engineering units to the DAC scale
  temp = ETMScaleFactor16(ptr_analog_output->set_point, ptr_analog_output->fixed_scale, ptr_analog_output->fixed_offset);
  
  // Calibrate for known gain/offset errors of this board
  temp = ETMScaleFactor2(temp, ptr_analog_output->calibration_internal_scale, ptr_analog_output->calibration_internal_offset);

  // Calibrate the DAC output for known gain/offset errors of the external circuitry
  temp = ETMScaleFactor2(temp, ptr_analog_output->calibration_external_scale, ptr_analog_output->calibration_external_offset);

  if (!ptr_analog_output->enabled) {
    temp = ptr_analog_output->disabled_dac_set_point;
  }
  
  ptr_analog_output->dac_setting_scaled_and_calibrated = temp;
}

void ETMAnalogScaleCalibrateADCReading(AnalogInput* ptr_analog_input) {
  unsigned int temp;
  // Calibrate the adc reading based on the known gain/offset errors of the external circuitry
  temp = ETMScaleFactor16(ptr_analog_input->filtered_adc_reading, ptr_analog_input->calibration_external_scale, ptr_analog_input->calibration_external_offset);

  // Calibrate the adc reading based on the known gain/offset errors of this board
  temp = ETMScaleFactor2(temp, ptr_analog_input->calibration_internal_scale, ptr_analog_input->calibration_internal_offset);
  
  // Scale the analog input to engineering units based on the fixed scale (and offset but normally not required) for this application
  temp = ETMScaleFactor16(temp, ptr_analog_input->fixed_scale, ptr_analog_input->fixed_offset);

  ptr_analog_input->reading_scaled_and_calibrated = temp;
}

void ETMAnalogSetOutput(AnalogOutput* ptr_analog_output, unsigned int new_set_point) {
  if (new_set_point > ptr_analog_output->max_set_point) {
    new_set_point = ptr_analog_output->max_set_point;
  }

  if (new_set_point < ptr_analog_output->min_set_point) {
    new_set_point = ptr_analog_output->min_set_point;
  }
  ptr_analog_output->set_point = new_set_point;
}

unsigned int ETMAnalogCheckOverAbsolute(AnalogInput* ptr_analog_input) {
  if (ptr_analog_input->reading_scaled_and_calibrated > ptr_analog_input->over_trip_point_absolute) {
    return 1;
  } else {
    return 0;
  }
}

unsigned int ETMAnalogCheckUnderAbsolute(AnalogInput* ptr_analog_input) {
  if (ptr_analog_input->reading_scaled_and_calibrated < ptr_analog_input->under_trip_point_absolute) {
    return 1;
  } else {
    return 0;
  }
}


unsigned int ETMAnalogCheckOverRelative(AnalogInput* ptr_analog_input) {
  unsigned int compare_point;
  compare_point = ETMScaleFactor2(ptr_analog_input->target_value, ptr_analog_input->relative_trip_point_scale, 0);
  if (compare_point < ptr_analog_input->relative_trip_point_floor) {
    compare_point = ptr_analog_input->relative_trip_point_floor;
  }
  
  if ((0xFF00 - compare_point) > ptr_analog_input->target_value) {
    compare_point += ptr_analog_input->target_value;
  } else {
    compare_point = 0xFF00;// We can't set this to 0xFFFF otherwise we would never get an over trip relative
  }
  
  if (ptr_analog_input->reading_scaled_and_calibrated > compare_point) {
    // We are out of range
    ptr_analog_input->over_trip_counter++;
  } else {
    if (ptr_analog_input->over_trip_counter > 0) {
      ptr_analog_input->over_trip_counter--;
    }
  }
  
  if (ptr_analog_input->over_trip_counter >= ptr_analog_input->relative_counter_fault_limit) {
    if (ptr_analog_input->over_trip_counter >= (ptr_analog_input->relative_counter_fault_limit * 2)) {
      ptr_analog_input->over_trip_counter = ptr_analog_input->relative_counter_fault_limit * 2;
    }
    return 1;
  } else {
    return 0;
  }
}


unsigned int ETMAnalogCheckUnderRelative(AnalogInput* ptr_analog_input) {
  unsigned int compare_point;
  compare_point = ETMScaleFactor2(ptr_analog_input->target_value, ptr_analog_input->relative_trip_point_scale, 0);
  if (compare_point < ptr_analog_input->relative_trip_point_floor) {
    compare_point = ptr_analog_input->relative_trip_point_floor;
  }
  
  if (compare_point > ptr_analog_input->target_value) {
    compare_point = ptr_analog_input->target_value - compare_point;
  } else {
    // In this case we will never get an under relative fault
    compare_point = 0x0000; 
  }
  
  if (ptr_analog_input->reading_scaled_and_calibrated < compare_point) {
    // We are out of range
    ptr_analog_input->under_trip_counter++;
  } else {
    if (ptr_analog_input->under_trip_counter > 0) {
      ptr_analog_input->under_trip_counter--;
    }
  }
  
  if (ptr_analog_input->under_trip_counter >= ptr_analog_input->relative_counter_fault_limit) {
    if (ptr_analog_input->under_trip_counter >= (ptr_analog_input->relative_counter_fault_limit * 2)) {
      ptr_analog_input->under_trip_counter = ptr_analog_input->relative_counter_fault_limit * 2;
    }
    return 1;
  } else {
    return 0;
  }
}
