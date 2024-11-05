/*
 * light_sensor.c
 *
 *  Created on: Aug 21, 2024
 *      Author: philbush
 */

#include "light_sensor.h"
#include "gpio.h"
#include "tx_api.h"
#include "i2c.h"

// @formatter:off
static Light_Sensor *light_self;
static as7341_gpio_int_struct gpio_struct;

// Struct functions
static light_return_code_t  _light_sensor_self_test (void);
static light_return_code_t  _light_sensor_setup_sensor (void);
static light_return_code_t  _light_sensor_read_all_channels (void);
static light_return_code_t  _light_sensor_start_timer ( uint16_t timeout_in_minutes );
static light_return_code_t  _light_sensor_stop_timer ( void );
static light_return_code_t  _light_sensor_process_measurements (void);
static light_return_code_t  _light_sensor_get_samples_averages (void);
static void                 _light_sensor_get_raw_measurements (light_raw_counts *buffer);
static void                 _light_sensor_get_basic_counts (light_basic_counts *buffer);
static void                 _light_sensor_get_single_measurement (uint16_t *raw_measurement, uint32_t *basic_count, light_channel_index_t which_channel);
static void                 _light_sensor_on (void);
static void                 _light_sensor_off (void);
// Functions neccessary for the AS7341 pins
static bool                 __as7341_wait_on_int (uint32_t timeout_ms);
static GPIO_PinState        __get_as7341_int_pin_state ( void );
static GPIO_PinState        __get_as7341_gpio_pin_state ( void );
static void                 __set_as7341_int_pin_state ( GPIO_PinState state );
static void                 __set_as7341_gpio_pin_state ( GPIO_PinState state );
// I/O functions for the sensor
static int32_t              _light_sensor_i2c_init ( void );
static int32_t              _light_sensor_i2c_deinit ( void );
static int32_t              _light_sensor_i2c_read_blocking ( void *unused_handle, uint16_t bus_address,
                                                          uint16_t reg_address, uint8_t *read_data,
                                                          uint16_t data_length );
static int32_t              _light_sensor_i2c_write_blocking ( void *unused_handle, uint16_t bus_address,
                                                           uint16_t reg_address, uint8_t *write_data,
                                                           uint16_t data_length );
static void                 _light_sensor_ms_delay ( uint32_t delay );
// Helper functions
static void                 __raw_to_basic_counts (void);
static void                 __get_mins_maxes (void);

// @formatter:on
/**
 * @brief  Initialize the Light_Sensor struct.
 * @param  struct_ptr:= pointer to Light_Sensor struct
 * @param  i2c_handle:= handle for I2C onboard peripheral
 * @param  timer:= pointer to TX_TIMER instance
 * @param  int_pin_sema:= pointer to semaphore used to evaluate falling edges on Int pin
 * @param  light_Sensor_i2c_sema:= semaphore used to determine TxRx completion status from I2C interrupt transactions
 * @retval Void
 */
void light_sensor_init ( Light_Sensor *struct_ptr, microSWIFT_configuration *global_config,
                         light_basic_counts *samples_series_buffer, I2C_HandleTypeDef *i2c_handle,
                         TX_TIMER *timer, TX_SEMAPHORE *int_pin_sema,
                         TX_SEMAPHORE *light_sensor_i2c_sema )
{
  light_self = struct_ptr;
  light_self->gpio_handle = &gpio_struct;

  light_self->global_config = global_config;
  light_self->i2c_handle = i2c_handle;
  light_self->int_pin_sema = int_pin_sema;
  light_self->i2c_sema = light_sensor_i2c_sema;
  light_self->timer = timer;

  light_self->smux_assignment_low_channels.adc_assignments[0] = F1;
  light_self->smux_assignment_low_channels.adc_assignments[1] = F2;
  light_self->smux_assignment_low_channels.adc_assignments[2] = F3;
  light_self->smux_assignment_low_channels.adc_assignments[3] = F4;
  light_self->smux_assignment_low_channels.adc_assignments[4] = F5;
  light_self->smux_assignment_low_channels.adc_assignments[5] = F6;

  light_self->smux_assignment_high_channels.adc_assignments[0] = F7;
  light_self->smux_assignment_high_channels.adc_assignments[1] = F8;
  light_self->smux_assignment_high_channels.adc_assignments[2] = NIR;
  light_self->smux_assignment_high_channels.adc_assignments[3] = CLEAR;
  light_self->smux_assignment_high_channels.adc_assignments[4] = DARK;
  light_self->smux_assignment_high_channels.adc_assignments[5] = ADC_DISABLE;

  light_self->gpio_handle->int_pin.port = AS7341_INT_GPIO_Port;
  light_self->gpio_handle->int_pin.pin = AS7341_INT_Pin;
  light_self->gpio_handle->gpio_pin.port = AS7341_GPIO_GPIO_Port;
  light_self->gpio_handle->gpio_pin.pin = AS7341_GPIO_Pin;
  light_self->gpio_handle->wait_on_int = __as7341_wait_on_int;
  light_self->gpio_handle->get_int_pin_state = __get_as7341_int_pin_state;
  light_self->gpio_handle->get_gpio_pin_state = __get_as7341_gpio_pin_state;
  light_self->gpio_handle->set_int_pin_state = __set_as7341_int_pin_state;
  light_self->gpio_handle->set_gpio_pin_state = __set_as7341_gpio_pin_state;

  light_self->fet.port = LIGHT_FET_GPIO_Port;
  light_self->fet.pin = LIGHT_FET_Pin;

  memset (&(light_self->raw_counts), 0, sizeof(light_self->raw_counts));
  memset (&(light_self->basic_counts), 0, sizeof(light_self->basic_counts));

  light_self->as7341_current_reg_bank = REG_BANK_UNKNOWN;

  light_self->sensor_gain = GAIN_64X;

  light_self->timer_timeout = false;

  light_self->total_samples = 0;
  light_self->samples_series = samples_series_buffer;
  memset (&(light_self->samples_min), 0xFFFFFFFF, sizeof(light_basic_counts));
  memset (&(light_self->samples_max), 0, sizeof(light_basic_counts));
  memset (&(light_self->samples_averages_accumulator), 0, sizeof(light_basic_counts));

  light_self->self_test = _light_sensor_self_test;
  light_self->setup_sensor = _light_sensor_setup_sensor;
  light_self->read_all_channels = _light_sensor_read_all_channels;
  light_self->start_timer = _light_sensor_start_timer;
  light_self->stop_timer = _light_sensor_stop_timer;
  light_self->process_measurements = _light_sensor_process_measurements;
  light_self->get_samples_averages = _light_sensor_get_samples_averages;
  light_self->get_raw_measurements = _light_sensor_get_raw_measurements;
  light_self->get_basic_counts = _light_sensor_get_basic_counts;
  light_self->get_single_measurement = _light_sensor_get_single_measurement;
  light_self->on = _light_sensor_on;
  light_self->off = _light_sensor_off;
}

/**
 * @brief  Deinitialize the I2C interface.
 * @param  Void
 * @retval Void
 */
void light_deinit ( void )
{
  if ( light_self->dev_ctx.deinit != NULL )
  {
    light_self->dev_ctx.deinit ();
  }
}

/**
 * @brief  Timer timeout callback
 * @param  expiration_input:= Unused input
 * @retval Void
 */
void light_timer_expired ( ULONG expiration_input )
{
  (void) expiration_input;
  light_self->timer_timeout = true;
}

/**
 * @brief  Get the timout status.
 * @param  Void
 * @retval Bool representing timeout status (true = timeout occurred)
 */
bool light_get_timeout_status ( void )
{
  return light_self->timer_timeout;
}

/**
 * @brief  Test the light sensor. This involves checking the ID, setting up the sensor, and taking
 *         initial measurements.
 * @param  Void
 * @retval light_return_code_t indicating success or specific failure type
 */
static light_return_code_t _light_sensor_self_test ( void )
{
  light_return_code_t ret = LIGHT_SUCCESS;
  as7341_auxid_reg_t aux_id;
  as7341_revid_reg_t rev_id;
  uint8_t id;
  bool asic_initialized = false;

  // Initialize the I/O interface
  if ( as7341_register_io_functions (&light_self->dev_ctx, _light_sensor_i2c_init,
                                     _light_sensor_i2c_deinit, _light_sensor_i2c_write_blocking,
                                     _light_sensor_i2c_read_blocking, _light_sensor_ms_delay,
                                     light_self->gpio_handle)
       != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Wait for the asic to boot
  while ( !asic_initialized )
  {
    as7341_get_initialization_status (&light_self->dev_ctx, &asic_initialized);
  }

  if ( as7341_spectral_meas_config (&light_self->dev_ctx, false) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  tx_thread_sleep (2);

  if ( as7341_power (&light_self->dev_ctx, true) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  light_self->dev_ctx.bus_read (NULL, AS7341_I2C_ADDR, AUXID_REG_ADDR, (uint8_t*) &aux_id, 1);
  light_self->dev_ctx.bus_read (NULL, AS7341_I2C_ADDR, REVID_REG_ADDR, (uint8_t*) &rev_id, 1);

  // Check the chip ID
  if ( as7341_get_id (&light_self->dev_ctx, &id) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  if ( (id != AS7341_ID) || (aux_id.auxid != AS7341_AUXID) || (rev_id.rev_id != AS7341_REVID) )
  {
    return LIGHT_I2C_ERROR;
  }

  ret = light_self->setup_sensor ();
  if ( ret != LIGHT_SUCCESS )
  {
    return ret;
  }

  // Read all the channels
  ret = light_self->read_all_channels ();

  return ret;
}

/**
 * @brief  Apply the specific configuration to the sensor
 * @param  Void
 * @retval light_return_code_t indicating success or specific failure type
 */
static light_return_code_t _light_sensor_setup_sensor ( void )
{
  light_return_code_t ret = LIGHT_SUCCESS;
  as7341_az_config_reg_t az_config;

  // Make sure we're starting with SP_EN bit cleared
  if ( as7341_spectral_meas_config (&light_self->dev_ctx, false) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Set ASTEP and ATIME to obtain the integration time from the following formula : t_int = (ATMIE + 1) X (ASTEP + 1) * 2.78us
  // We want an integration time of 182ms, so we'll set ATIME = 0, ASTEP = 65534
  ret = as7341_set_astep (&light_self->dev_ctx, 65534);
  ret |= as7341_set_atime (&light_self->dev_ctx, 0);
  if ( ret != LIGHT_SUCCESS )
  {
    return ret;
  }

  // Disable wait time, though we still need WTIME to be longer than integration time
  ret = as7341_set_wait_time (&light_self->dev_ctx, 250);
  ret |= as7341_wait_config (&light_self->dev_ctx, false);
  if ( ret != LIGHT_SUCCESS )
  {
    return ret;
  }

  // Set the gain
  ret = as7341_set_again (&light_self->dev_ctx, light_self->sensor_gain);
  if ( ret != LIGHT_SUCCESS )
  {
    return ret;
  }

  // Set the GPIO pin to input and output?
  uint8_t gpio_2 = 0x06, edge_reg = 0;
  ret = light_self->dev_ctx.bus_write (NULL, AS7341_I2C_ADDR, GPIO_2_REG_ADDR, (uint8_t*) &gpio_2,
                                       1);

  ret = light_self->dev_ctx.bus_read (NULL, AS7341_I2C_ADDR, EDGE_REG_ADDR, (uint8_t*) &edge_reg,
                                      1);

  if ( ret != LIGHT_SUCCESS )
  {
    return ret;
  }

  // Setup auto-zero to occur every cycle
  az_config.az_nth_iteration.nth_iteration = 1;
  ret = as7341_auto_zero_config (&light_self->dev_ctx, az_config.az_nth_iteration);

  return ret;
}

/**
 * @brief  Read all channels, data will be stored in light_self->raw_counts and light_self->basic_counts.
 * @param  Void
 * @retval light_return_code_t indicating success or specific failure type
 */
static light_return_code_t _light_sensor_read_all_channels ( void )
{
  bool data_ready = false;
  uint32_t loop_counter = 0;

  // Integration mode SYNS --> GPIO pin triggers samples to start
  if ( as7341_set_integration_mode (&light_self->dev_ctx, SYNS_MODE) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Make sure we're starting with SP_EN bit cleared
  if ( as7341_spectral_meas_config (&light_self->dev_ctx, false) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Set the SMUX to the lower channels
  if ( as7341_config_smux (&light_self->dev_ctx,
                           &(light_self->smux_assignment_low_channels)) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Enable spectral measurements
  if ( as7341_spectral_meas_config (&light_self->dev_ctx, true) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  while ( !data_ready )
  {
    // We're triggering sampling via GPIO pin, so we'll do that here
    light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_SET);
    tx_thread_relinquish ();
    light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_RESET);

    tx_thread_sleep (1);

    if ( as7341_get_data_ready (&light_self->dev_ctx, &data_ready) != AS7341_OK )
    {
      return LIGHT_I2C_ERROR;
    }

    loop_counter++;
  }

  if ( as7341_get_all_channel_data (
      &light_self->dev_ctx, ((as7341_all_channel_data_struct*) &light_self->raw_counts.f1_chan))
       != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  if ( as7341_spectral_meas_config (&light_self->dev_ctx, false) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Set the SMUX to the upper channels and repeat process
  if ( as7341_config_smux (&light_self->dev_ctx,
                           &(light_self->smux_assignment_high_channels)) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  if ( as7341_spectral_meas_config (&light_self->dev_ctx, true) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  data_ready = false;
  loop_counter = 0;

  while ( !data_ready )
  {
    // We're triggering sampling via GPIO pin, so we'll do that here
    light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_SET);
    tx_thread_relinquish ();
    light_self->gpio_handle->set_gpio_pin_state (GPIO_PIN_RESET);

    tx_thread_sleep (1);

    if ( as7341_get_data_ready (&light_self->dev_ctx, &data_ready) != AS7341_OK )
    {
      return LIGHT_I2C_ERROR;
    }

    loop_counter++;
  }

  if ( as7341_get_all_channel_data (
      &light_self->dev_ctx, ((as7341_all_channel_data_struct*) &light_self->raw_counts.f7_chan))
       != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  if ( as7341_spectral_meas_config (&light_self->dev_ctx, false) != AS7341_OK )
  {
    return LIGHT_I2C_ERROR;
  }

  // Convert everything to basic counts
  __raw_to_basic_counts ();

  return LIGHT_SUCCESS;
}

/**
 * @brief  Start the thread timer.
 * @param  timeout_in_minutes:= timeout, in minutes
 * @retval light_return_code_t indicating success or specific failure type
 */
static light_return_code_t _light_sensor_start_timer ( uint16_t timeout_in_minutes )
{
  uint16_t timeout = TX_TIMER_TICKS_PER_SECOND * 60 * timeout_in_minutes;
  light_return_code_t ret = LIGHT_SUCCESS;

  if ( tx_timer_change (light_self->timer, timeout, 0) != TX_SUCCESS )
  {
    ret = LIGHT_TIMER_ERROR;
    return ret;
  }

  if ( tx_timer_activate (light_self->timer) != TX_SUCCESS )
  {
    ret = LIGHT_TIMER_ERROR;
  }

  return ret;
}

/**
 * @brief  Stop the thread timer.
 * @param  void
 * @retval light_return_code_t indicating success or specific failure type
 */
static light_return_code_t _light_sensor_stop_timer ( void )
{
  return (tx_timer_deactivate (light_self->timer) == TX_SUCCESS) ?
      LIGHT_SUCCESS : LIGHT_TIMER_ERROR;
}

/**
 * @brief  Process the last round of measurements
 * @param  void
 * @retval light_return_code_t indicating success or specific failure type
 */
static light_return_code_t _light_sensor_process_measurements ( void )
{
  uint32_t *basic_count_ptr = &light_self->basic_counts.f1_chan;
  uint32_t *averages_ptr = &light_self->samples_averages_accumulator.f1_chan;
  // Update min/ max
  __get_mins_maxes ();

  if ( light_self->total_samples == light_self->global_config->total_light_samples )
  {
    return LIGHT_DONE_SAMPLING;
  }

  // Store the most recent samples in the time series
  memcpy (&(light_self->samples_series[light_self->total_samples]), &light_self->basic_counts,
          sizeof(light_basic_counts));

  for ( int i = 0; i < 12; i++, basic_count_ptr++, averages_ptr++ )
  {
    *averages_ptr += *basic_count_ptr;
  }

  light_self->total_samples++;

  return LIGHT_SUCCESS;
}

/**
 * @brief  Average the samples.
 * @param  void
 * @retval light_return_code_t indicating success or specific failure type
 */
static light_return_code_t _light_sensor_get_samples_averages ( void )
{
  uint32_t *averages_ptr = &light_self->samples_averages_accumulator.f1_chan;

  if ( light_self->total_samples == 0 )
  {
    return LIGHT_PARAMETERS_INVALID;
  }

  for ( int i = 0; i < 12; i++, averages_ptr++ )
  {
    *averages_ptr /= light_self->total_samples;
  }

  return LIGHT_SUCCESS;
}

static void _light_sensor_get_raw_measurements ( light_raw_counts *buffer )
{
  memcpy (buffer, &(light_self->raw_counts), sizeof(light_raw_counts));
}

static void _light_sensor_get_basic_counts ( light_basic_counts *buffer )
{
  memcpy (buffer, &(light_self->basic_counts), sizeof(light_basic_counts));
}

static void _light_sensor_get_single_measurement ( uint16_t *raw_measurement, uint32_t *basic_count,
                                                   light_channel_index_t which_channel )
{
  uint16_t *raw = &light_self->raw_counts.f1_chan;
  uint32_t *basic = &light_self->basic_counts.f1_chan;

  if ( (which_channel < 0) || (which_channel > DARK_CHANNEL) )
  {
    *raw_measurement = 0;
    *basic_count = 0;
    return;
  }

  *raw_measurement = *(raw + which_channel);
  *basic_count = *(basic + which_channel);
}

static void _light_sensor_on ( void )
{
  HAL_GPIO_WritePin (light_self->fet.port, light_self->fet.pin, GPIO_PIN_SET);
}

static void _light_sensor_off ( void )
{
  HAL_GPIO_WritePin (light_self->fet.port, light_self->fet.pin, GPIO_PIN_RESET);
}

static bool __as7341_wait_on_int ( uint32_t timeout_ms )
{
  ULONG timeout = (timeout_ms / (1000 / TX_TIMER_TICKS_PER_SECOND)) + 1;

  if ( tx_semaphore_get (light_self->int_pin_sema, timeout) != TX_SUCCESS )
  {
    return false;
  }

  return true;
}

static GPIO_PinState __get_as7341_int_pin_state ( void )
{
  return HAL_GPIO_ReadPin (light_self->gpio_handle->int_pin.port,
                           light_self->gpio_handle->int_pin.pin);
}

static GPIO_PinState __get_as7341_gpio_pin_state ( void )
{
  return HAL_GPIO_ReadPin (light_self->gpio_handle->gpio_pin.port,
                           light_self->gpio_handle->gpio_pin.pin);
}

void __set_as7341_int_pin_state ( GPIO_PinState state )
{
  HAL_GPIO_WritePin (light_self->gpio_handle->int_pin.port, light_self->gpio_handle->int_pin.pin,
                     state);
}

void __set_as7341_gpio_pin_state ( GPIO_PinState state )
{
  HAL_GPIO_WritePin (light_self->gpio_handle->gpio_pin.port, light_self->gpio_handle->gpio_pin.pin,
                     state);
}

static int32_t _light_sensor_i2c_init ( void )
{
  return i2c1_init ();
}

static int32_t _light_sensor_i2c_deinit ( void )
{
  return i2c1_deinit ();
}

static int32_t _light_sensor_i2c_read_blocking ( void *unused_handle, uint16_t bus_address,
                                                 uint16_t reg_address, uint8_t *read_data,
                                                 uint16_t data_length )
{
  (void) unused_handle;
  int32_t ret = AS7341_OK;

// Why could they possibly need to put a bit in there for different register banks? This
// could so easily be handled in the asic
//  if ( (reg_address < 0x80)
  if ( ((reg_address >= 0x64) && (reg_address < 0x80))
       && (reg_address != CFG0_REG_ADDR)
       && ((light_self->as7341_current_reg_bank == REG_BANK_80_PLUS)
           || (light_self->as7341_current_reg_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_60_74) != AS7341_OK )
    {
      light_self->as7341_current_reg_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }

    light_self->as7341_current_reg_bank = REG_BANK_60_74;
  }
  else if ( ((reg_address >= 0x80) || (reg_address < 0x64))
            && (reg_address != CFG0_REG_ADDR)
            && ((light_self->as7341_current_reg_bank == REG_BANK_60_74)
                || (light_self->as7341_current_reg_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_80_PLUS) != AS7341_OK )
    {
      light_self->as7341_current_reg_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }

    light_self->as7341_current_reg_bank = REG_BANK_80_PLUS;
  }

  if ( HAL_I2C_Mem_Read_IT (light_self->i2c_handle, bus_address, reg_address, 1, read_data,
                            data_length)
       != HAL_OK )
  {
    ret = AS7341_ERROR;
  }

  if ( tx_semaphore_get (light_self->i2c_sema, LIGHT_I2C_TIMEOUT) != TX_SUCCESS )
  {
    ret = LIGHT_TIMEOUT;
  }

  return ret;
}

static int32_t _light_sensor_i2c_write_blocking ( void *unused_handle, uint16_t bus_address,
                                                  uint16_t reg_address, uint8_t *write_data,
                                                  uint16_t data_length )
{
  (void) unused_handle;
  int32_t ret = AS7341_OK;

// Why could they possibly need to put a bit in there for different register banks? This
// could so easily be handled in the asic
//  if ( (reg_address < 0x80)
  if ( ((reg_address >= 0x64) && (reg_address < 0x80))
       && (reg_address != CFG0_REG_ADDR)
       && ((light_self->as7341_current_reg_bank == REG_BANK_80_PLUS)
           || (light_self->as7341_current_reg_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_60_74) != AS7341_OK )
    {
      light_self->as7341_current_reg_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }

    light_self->as7341_current_reg_bank = REG_BANK_60_74;
  }
  else if ( ((reg_address >= 0x80) || (reg_address < 0x64))
            && (reg_address != CFG0_REG_ADDR)
            && ((light_self->as7341_current_reg_bank == REG_BANK_60_74)
                || (light_self->as7341_current_reg_bank == REG_BANK_UNKNOWN)) )
  {
    if ( as7341_set_register_bank (&light_self->dev_ctx, REG_BANK_80_PLUS) != AS7341_OK )
    {
      light_self->as7341_current_reg_bank = REG_BANK_UNKNOWN;
      return AS7341_ERROR;
    }

    light_self->as7341_current_reg_bank = REG_BANK_80_PLUS;
  }

  if ( HAL_I2C_Mem_Write_IT (light_self->i2c_handle, bus_address, reg_address, 1, write_data,
                             data_length)
       != HAL_OK )
  {
    ret = AS7341_ERROR;
  }

  if ( tx_semaphore_get (light_self->i2c_sema, LIGHT_I2C_TIMEOUT) != TX_SUCCESS )
  {
    ret = LIGHT_TIMEOUT;
  }

  return ret;
}

static void _light_sensor_ms_delay ( uint32_t delay )

{
  UINT delay_ticks = (delay == 0) ?
      0 : delay / TX_TIMER_TICKS_PER_SECOND;

  if ( delay == 0 )
  {
    tx_thread_relinquish ();
  }
  else
  {
    tx_thread_sleep (delay_ticks);
  }
}

static void __raw_to_basic_counts ( void )
{
  uint16_t *raw_cnt_ptr = &(light_self->raw_counts.f1_chan);
  uint32_t *basic_cnt_ptr = &(light_self->basic_counts.f1_chan);

  if ( light_self->sensor_gain == GAIN_0_5X )
  {
    for ( int i = 0; i < 12; i++, raw_cnt_ptr++, basic_cnt_ptr++ )
    {
      *basic_cnt_ptr = *raw_cnt_ptr >> 1;
    }
  }
  else
  {
    for ( int i = 0; i < 12; i++, raw_cnt_ptr++, basic_cnt_ptr++ )
    {
      *basic_cnt_ptr = *raw_cnt_ptr << (light_self->sensor_gain - 1);
    }
  }
}

static void __get_mins_maxes ( void )
{
  uint32_t *basic_count, *min, *max;
  basic_count = &light_self->basic_counts.f1_chan;
  min = &light_self->samples_min.f1_chan;
  max = &light_self->samples_max.f1_chan;

  for ( int i = 0; i < 12; i++, basic_count++, min++, max++ )
  {
    if ( *basic_count < *min )
    {
      *min = *basic_count;
    }

    if ( *basic_count > *max )
    {
      *max = *basic_count;
    }
  }
}
