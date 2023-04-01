#include <18F46K22.h>
#device ADC = 8
#FUSES NOWDT //! No Watch Dog Timer

#use delay(internal = 64mhz)

#define b_high() output_high(PIN_c1)
#define b_low() output_low(PIN_c1)
#define l_high() output_high(PIN_a4)
#define l_low() output_low(PIN_a4)
#define r_high() output_high(PIN_c0)
#define r_low() output_low(PIN_c0)
#define f_high() output_high(PIN_e1)
#define f_low() output_low(PIN_e1)
#define i2c_busy() output_low(pin_C2)   // D2
#define i2c_ready() output_high(pin_C2)

#define b_toggle() output_toggle(PIN_c1) // 35
#define l_toggle() output_toggle(PIN_a4) // 23
#define r_toggle() output_toggle(PIN_c0) // 32
#define f_toggle() output_toggle(PIN_e1) // 26

//!#define I2C_INT pin_C2 // D2

#define callibrate_key() !input(pin_e2)

#use i2c(i2c1, slave, address = 0xA0, FORCE_HW, SLOW)
#use rs232(baud = 115200, parity = N, xmit = PIN_c6, rcv = pin_D7, bits = 8, stream = blt) //? bluetooth terminal//password:123

unsigned int32 digital_value = 0;
unsigned int8 data[3] = {0};            //? Buffer to send digital data to master
unsigned int8 njl_values[20] = {0};     //? ADC value of each sensor
unsigned int8 threshold_values[20];     //? threshhold value of each njl sensor
unsigned int8 min_njl[20], max_njl[20]; //? Maximum & Minimum value of sensor uses to measure threshold
unsigned int8 njl_value = 0;
unsigned int8 ledc = 0;
const unsigned int8 njl_channels[20] = {13, 11, 9, 8, 10, 12, 26, 25, 24, 19, 17, 23, 22, 21, 20, 4, 3, 2, 1, 0};
int1 callibration_done = 0;
int counter, counter_key = 0;


void check_sensors();
void convert32to24(unsigned int32 value, unsigned int8 *msb, unsigned int8 *mid, unsigned int8 *lsb);

#INT_SSP
void ssp_interrupt()
{
   int state = i2c_isr_state();

   if (state < 0x80) // Master is sending data
   {
      i2c_read();
   }

   if (state >= 0x80) // Master is requesting data from slave
   {
      i2c_write(data[state - 0x80]);
      
//!      if (state == 81)
//!      {
//!         data[0] = 0;
//!         data[1] = 0;
//!         data[0] = 0;
//!         
//!         i2c_busy();
//!      }
   }
}

void main()
{
   i2c_busy();
   set_analog_pins(PIN_A0, PIN_A1, PIN_A2, PIN_A3, PIN_A5, PIN_B0, PIN_B1, PIN_B2, PIN_B3, PIN_B4, PIN_B5, PIN_C5, PIN_C7, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6);
   setup_adc(ADC_CLOCK_DIV_64 | ADC_TAD_MUL_0);
   enable_interrupts(global);
   enable_interrupts(INT_SSP);
   delay_ms(500);

   for (int j = 0; j < 3; j++)
   {
      f_high();
      delay_ms(60);
      l_high();
      delay_ms(60);
      b_high();
      delay_ms(60);
      r_high();
      delay_ms(60);
      f_low();
      delay_ms(60);
      l_low();
      delay_ms(60);
      b_low();
      delay_ms(60);
      r_low();
      delay_ms(60);
   }

   for (int i = 0; i < 20; i++)
   {
      threshold_values[i] = read_eeprom(i);
      min_njl[i] = 255;
      max_njl[i] = 0;
   }

   b_low();
   f_low();
   l_low();
   r_low();

   while (True)
   {
//!      if (callibrate_key()) // callibration
//!      {
//!         counter_key++;
//!         counter_key %= 20;
//!         while (callibrate_key())
//!         {
//!         }
//!      }
//!      check_sensors();
//!      printf("Sensor %u: %u\r\n", counter_key, njl_values[counter_key]);
//!      printf("MIN %u: %u    MAX %u: %u\r\n", counter_key, min_njl[counter_key], counter_key, max_njl[counter_key]);
      
//!      r_toggle();
//!      delay_ms(5);
      
      
      if (callibrate_key() && !callibration_done) // callibration
      {
         while (callibrate_key())
         {
            for (int i = 0; i < 20; i++)
            {
               
               min_njl[i] = 255;
               max_njl[i] = 0;
            }
         }
         while (!callibrate_key())
         {
            for (int i = 0; i < 20; i++)
            {
               set_adc_channel(njl_channels[i]);
               delay_us(10);
               njl_value = read_adc();
               if (njl_value > max_njl[i])
                  max_njl[i] = njl_value;
               if (njl_value < min_njl[i])
                  min_njl[i] = njl_value;
            }
            if ((ledc++) > 250)
            {
               ledc = 0;
               b_toggle();
               f_toggle();
               r_toggle();
               l_toggle();
            }
         }
         for (int i = 0; i < 20; i++)
         {
            threshold_values[i] = max_njl[i] - 40;
//!            threshold_values[i] = (float)max_njl[i] * 0.65 + (float)min_njl[i] * 0.35;
//!            threshold_values[i] = (float)max_njl[i] * 0.5 + (float)min_njl[i] * 0.5;
            write_eeprom(i, threshold_values[i]);
            delay_us(10);
         }
         b_low();
         f_low();
         l_low();
         r_low();

         callibration_done = 1;
      }
      

      check_sensors();
      
      if (counter > 1)
      {
         b_high();
      }
      else
      {
         b_low();
      }
//!      uint32_to_uint8(digital_value, data);
      convert32to24(digital_value, &data[0], &data[1], &data[2]);
//!      printf("%d\r\n", njl_values[0]);
//!      i2c_ready();
   }
}

void check_sensors()
{
   counter = 0;
   digital_value = 0;
   for (unsigned int8 i = 0; i < 20; i++)
   {
      set_adc_channel(njl_channels[i]);
//!      delay_us(10);
      delay_us(15);
      njl_values[i] = read_adc();

      if (njl_values[i] > threshold_values[i])
      {
         bit_set(digital_value, i);
//!         printf("%u\r\n", i);
         counter++;
      }
      else
      {
         bit_clear(digital_value, i);
      }
   }
}

void convert32to24(unsigned int32 value, unsigned int8 *msb, unsigned int8 *mid, unsigned int8 *lsb)
{
    *lsb = (unsigned int8)value;
    value >>= 8;
    *mid = (unsigned int8)value;
    value >>= 8;
    *msb = (unsigned int8)value;
}

