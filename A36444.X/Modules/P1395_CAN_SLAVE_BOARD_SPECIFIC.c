#include "P1395_CAN_SLAVE.h"

#ifdef __A36224_500
#include "A36224_500.h"
#endif


#ifdef __A36444
#include "A36444.h"
#endif

#ifdef __A36224
#include "A36224_000.h"
#endif

#ifdef __A36487
#include "A36487.h"
#endif

#ifdef __A36582
#include "A36582.h"
#endif


void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
      
#ifdef __A36224
    case ETM_CAN_REGISTER_COOLING_CMD_SF6_PULSE_LIMIT_OVERRIDE:
      OverrideSF6PulseLimit();
      break;
      
    case ETM_CAN_REGISTER_COOLING_CMD_RESET_BOTTLE_COUNT:
      global_data_A36224_000.SF6_bottle_counter=700;
      break;
      
    case ETM_CAN_REGISTER_COOLING_CMD_SF6_LEAK_LIMIT_OVERRIDE:
      OverrideSF6LowPressure();
      break;
#endif
      

      
#ifdef __A36224_500
    case ETM_CAN_REGISTER_HEATER_MAGNET_SET_1_CURRENT_SET_POINT:
      ETMAnalogSetOutput(&global_data_A36224_500.analog_output_heater_current, message_ptr->word1);
      ETMAnalogSetOutput(&global_data_A36224_500.analog_output_electromagnet_current, message_ptr->word0);
      _CONTROL_NOT_CONFIGURED = 0;
      break;
      
#endif
      
#ifdef __A36444
    case ETM_CAN_REGISTER_HV_LAMBDA_SET_1_LAMBDA_SET_POINT:
      ETMAnalogSetOutput(&global_data_A36444.analog_output_high_energy_vprog, message_ptr->word1); 
      ETMAnalogSetOutput(&global_data_A36444.analog_output_low_energy_vprog,message_ptr->word2);
      _CONTROL_NOT_CONFIGURED = 0;
      break;
#endif
      
      
#ifdef __A36487     //A36487 PULSE SYNC BOARD
      unsigned int temp;
    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_0:
      temp = (message_ptr->word3 & 0xFF00) >> 8;
      psb_params.grid_delay_high3 = temp;
      temp = message_ptr->word3 & 0x00FF;
      psb_params.grid_delay_high2 = temp;
      temp = (message_ptr->word2 & 0xFF00) >> 8;
      psb_params.grid_delay_high1 = temp;
      temp = message_ptr->word2 & 0x00FF;
      psb_params.grid_delay_high0 = temp;
      temp = (message_ptr->word1 & 0xFF00) >> 8;
      psb_params.pfn_delay_high = temp;
      temp = message_ptr->word1 & 0x00FF;
      psb_params.rf_delay_high = temp;
      psb_data.counter_config_received |= 0b0001;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_HIGH_ENERGY_TIMING_REG_1:
      temp = (message_ptr->word3 & 0xFF00) >> 8;
      psb_params.grid_width_high3 = temp;
      temp = message_ptr->word3 & 0x00FF;
      psb_params.grid_width_high2 = temp;
      temp = (message_ptr->word2 & 0xFF00) >> 8;
      psb_params.grid_width_high1 = temp;
      temp = message_ptr->word2 & 0x00FF;
      psb_params.grid_width_high0 = temp;
      temp = (message_ptr->word1 & 0xFF00) >> 8;
      psb_params.afc_delay_high = temp;
      temp = message_ptr->word1 & 0x00FF;
      psb_params.spare_delay_high = temp;
      psb_data.counter_config_received |=0b0010;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_0:
      temp = (message_ptr->word3 & 0xFF00) >> 8;
      psb_params.grid_delay_low3 = temp;
      temp = message_ptr->word3 & 0x00FF;
      psb_params.grid_delay_low2 = temp;
      temp = (message_ptr->word2 & 0xFF00) >> 8;
      psb_params.grid_delay_low1 = temp;
      temp = message_ptr->word2 & 0x00FF;
      psb_params.grid_delay_low0 = temp;
      temp = (message_ptr->word1 & 0xFF00) >> 8;
      psb_params.pfn_delay_low = temp;
      temp = message_ptr->word1 & 0x00FF;
      psb_params.rf_delay_low = temp;
      psb_data.counter_config_received |= 0b0100;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_LOW_ENERGY_TIMING_REG_1:
      temp = (message_ptr->word3 & 0xFF00) >> 8;
      psb_params.grid_width_low3 = temp;
      temp = message_ptr->word3 & 0x00FF;
      psb_params.grid_width_low2 = temp;
      temp = (message_ptr->word2 & 0xFF00) >> 8;
      psb_params.grid_width_low1 = temp;
      temp = message_ptr->word2 & 0x00FF;
      psb_params.grid_width_low0 = temp;
      temp = (message_ptr->word1 & 0xFF00) >> 8;
      psb_params.afc_delay_low = temp;
      temp = message_ptr->word1 & 0x00FF;
      psb_params.spare_delay_low = temp;
      psb_data.counter_config_received |= 0b1000;
      break;

    case ETM_CAN_REGISTER_PULSE_SYNC_SET_1_CUSTOMER_LED_OUTPUT:
      psb_data.led_state = message_ptr->word0;
      break;
#endif
      
      
    default:
      local_can_errors.invalid_index++;
      break;
    }
}


void ETMCanSlaveLogCustomPacketC(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */

#ifdef __A36224
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_FLOW_0,
		     global_data_A36224_000.analog_input_flow_1.reading_scaled_and_calibrated,     //HVPS flow
		     global_data_A36224_000.analog_input_flow_0.reading_scaled_and_calibrated,     //Magnetron flow
		     0,                                              //Linac flow
		     global_data_A36224_000.analog_input_flow_2.reading_scaled_and_calibrated     //Circulator
		     );
#endif
#ifdef __A36224_500
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_HEATER_MAGNET_SLOW_READINGS, 
		     global_data_A36224_500.analog_input_heater_current.reading_scaled_and_calibrated,
		     global_data_A36224_500.analog_input_heater_voltage.reading_scaled_and_calibrated,
		     global_data_A36224_500.analog_input_electromagnet_current.reading_scaled_and_calibrated,
		     global_data_A36224_500.analog_input_electromagnet_voltage.reading_scaled_and_calibrated
		     );
#endif


#ifdef __A36444
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_FAST_PROGRAM_VOLTAGE,
		     etm_can_next_pulse_count,
		     global_data_A36444.analog_output_high_energy_vprog.set_point,
		     global_data_A36444.analog_output_low_energy_vprog.set_point,
		     global_data_A36444.analog_input_lambda_vpeak.reading_scaled_and_calibrated
		     );
#endif

#ifdef __A36487
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_FAST_TRIGGER_DATA,
		     psb_data.pulses_on,
		     (psb_data.trigger_input << 8) & psb_data.trigger_filtered,
		     (psb_data.grid_width << 8) & psb_data.grid_delay,
		     0);
#endif


#ifdef __A36582
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_FAST_PREVIOUS_PULSE,
		     global_data_A36582.sample_index,
		     global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated,
		     global_data_A36582.imag_external_adc.reading_scaled_and_calibrated,
		     _STATUS_ARC_DETECTED
		     );
#endif

}

void ETMCanSlaveLogCustomPacketD(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */

#ifdef __A36224
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_FLOW_1,
		     0,   //Coolant spare word 1
		     0,      //Coolant spare word 0
		     0,  //HX coolant flow
		     0  //coolant Flow (unused)
		     );
#endif

#ifdef __A36224_500
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_HEATER_MAGNET_SLOW_SET_POINTS, 
		     global_data_A36224_500.analog_output_heater_current.set_point,
		     0,
		     global_data_A36224_500.analog_output_electromagnet_current.set_point,
		     0
		     );
#endif


#ifdef __A36444
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_HV_LAMBDA_SLOW_SET_POINT,
		     global_data_A36444.eoc_not_reached_count,
		     global_data_A36444.analog_input_lambda_vmon.reading_scaled_and_calibrated,
		     global_data_A36444.analog_input_lambda_imon.reading_scaled_and_calibrated,
		     global_data_A36444.analog_input_lambda_heat_sink_temp.reading_scaled_and_calibrated
		     );
#endif


#ifdef __A36487
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_0,
		     (psb_params.grid_delay_high3 << 8) & psb_params.grid_delay_high2,
		     (psb_params.grid_delay_high1 << 8) &  psb_params.grid_delay_high0,
		     (psb_params.pfn_delay_high << 8) & psb_params.rf_delay_high,
		     (psb_params.grid_width_high3 << 8) & psb_params.grid_width_high2);
#endif


#ifdef __A36582
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_FILTERED_PULSE,
		     0,
		     global_data_A36582.arc_this_hv_on,
		     global_data_A36582.filt_int_adc_low,
		     global_data_A36582.filt_int_adc_high
		     );
#endif


}

void ETMCanSlaveLogCustomPacketE(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
#ifdef __A36224
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_COOLING_SLOW_ANALOG_READINGS,
		     global_data_A36224_000.analog_input_coolant_temp.reading_scaled_and_calibrated,
		     global_data_A36224_000.analog_input_SF6_pressure.reading_scaled_and_calibrated,
		     global_data_A36224_000.analog_input_cabinet_temp.reading_scaled_and_calibrated,
		     0       //linac temp
		     );
#endif
  // There is no E packet for HV Lamdba, leave blank

#ifdef __A36487
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_1,
		     (psb_params.grid_width_high1 << 8) & psb_params.grid_width_high0,
		     (psb_params.afc_delay_high << 8) & psb_params.spare_delay_high,
		     (psb_params.grid_delay_low3 << 8) & psb_params.grid_delay_low2,
		     (psb_params.grid_delay_low1 << 8) & psb_params.grid_delay_low0);
#endif

#ifdef __A36582
  ETMCanSlaveLogData(
		     // DPARKER need to confirm this pointer math
		     ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_ARCS,
		     *((unsigned int*)&global_data_A36582.arc_total + 1),          // This is the high word
		     *((unsigned int*)&global_data_A36582.arc_total),              // This is the low word
		     *((unsigned int*)&global_data_A36582.pulse_this_hv_on + 1),   // This is the high word
		     *((unsigned int*)&global_data_A36582.pulse_this_hv_on)        // This is the high word
		     );
#endif
}

void ETMCanSlaveLogCustomPacketF(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  
  // There is no F packet for HV Lamdba, leave blank
  
#ifdef __A36487
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_PULSE_SYNC_SLOW_TIMING_DATA_1,
		     (psb_params.pfn_delay_low << 8) & psb_params.rf_delay_low,
		     (psb_params.grid_width_low3 << 8) & psb_params.grid_width_low2,
		     (psb_params.grid_width_low1 << 8) & psb_params.grid_width_low0,
		     (psb_params.afc_delay_low << 8) & psb_params.spare_delay_low);
#endif
  
  
  
#ifdef __A36582
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_PULSE_COUNT,
		     *((unsigned int*)&global_data_A36582.pulse_total + 3),        // This is the most significant word
		     *((unsigned int*)&global_data_A36582.pulse_total + 2),        
		     *((unsigned int*)&global_data_A36582.pulse_total + 1),        
		     *((unsigned int*)&global_data_A36582.pulse_total)             // This is the least significant word
		     );
#endif

}


  



