#include <18F46K22.h>
#device ADC = 8
//!#FUSES NOMCLR,NOPUT,NOCPD,NOWRTD,NOEBTR,NOWRTB,NOWRT,NOWDT
#FUSES NOWDT,NOMCLR //! No Watch Dog Timer
//!#use delay(clock=64000000, internal=16000000)
#use delay(internal=16mhz)
//!#use delay(internal=64mhz)

//!#use delay(internal = 64mhz)

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
//!unsigned int8 data[3] = {0};            //? Buffer to send digital data to master
unsigned int8 data[4] = {10, 20, 30, 40};            //? Buffer to send digital data to master
unsigned int8 njl_values[20] = {0};     //? ADC value of each sensor
unsigned int8 threshold_values[20];     //? threshhold value of each njl sensor
unsigned int8 min_njl[20], max_njl[20]; //? Maximum & Minimum value of sensor uses to measure threshold
unsigned int8 njl_value = 0;
unsigned int8 ledc = 0;
const unsigned int8 njl_channels[20] = {13, 11, 9, 8, 10, 12, 26, 25, 24, 19, 17, 23, 22, 21, 20, 4, 3, 2, 1, 0};
int1 callibration_done = 0;
int counter, counter_key = 0;
unsigned int16 counter_4ms = 0;
const unsigned int32 bit_values[20] = {0x00000001, 0x00000002, 0x00000004, 0x00000008, 0x00000010, 0x00000020, 0x00000040, 0x00000080, 0x00000100, 0x00000200, 0x00000400, 0x00000800, 0x00001000, 0x00002000, 0x00004000, 0x00008000, 0x00010000, 0x00020000, 0x00040000, 0x00080000};


void check_sensors();
void convert32to24(unsigned int32 value, unsigned int8 *msb, unsigned int8 *mid, unsigned int8 *lsb);

#INT_TIMER1
void timer1_interrupt()
{
   counter_4ms++;
}

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
//!   setup_wdt(WDT_32MS);
   
   i2c_busy();
   set_analog_pins(PIN_A0, PIN_A1, PIN_A2, PIN_A3, PIN_A5, PIN_B0, PIN_B1, PIN_B2, PIN_B3, PIN_B4, PIN_B5, PIN_C5, PIN_C7, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6);
   setup_adc(ADC_CLOCK_DIV_64 | ADC_TAD_MUL_0);
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_1); //? 4ms
   enable_interrupts(global);
   enable_interrupts(INT_SSP);
   enable_interrupts(INT_TIMER1);
   
//!   delay_ms(500);
//!
//!   for (int j = 0; j < 3; j++)
//!   {
      f_high();
      delay_ms(15);
//      l_high();

//!      b_high();
//!      delay_ms(60);
//!      r_high();
//!      delay_ms(60);
      f_low();
//!      delay_ms(60);
//!      l_low();
//!      delay_ms(60);
//!      b_low();
//!      delay_ms(60);
//!      r_low();
//!      delay_ms(60);
//!   }

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
   
   //setup_wdt(WDT_4MS);
   
   while (True)
   {
//!      restart_wdt();
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

//!      printf("%d\r\n", njl_values[6]);
      
      
      if (callibrate_key() && !callibration_done) // callibration
      {
         while (callibrate_key())
         {
//!            restart_wdt();
            for (int i = 0; i < 20; i++)
            {
               min_njl[i] = 255;
               max_njl[i] = 0;
            }
         }
         while (!callibrate_key())
         {
//!            restart_wdt();
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
//!            restart_wdt();

//!            threshold_values[i] = max_njl[i] - 40;
//!            threshold_values[i] = ((max_njl[i] - min_njl[i]) / 4) + min_njl[i];
//!            threshold_values[i] = ((max_njl[i] - min_njl[i]) / 8) + min_njl[i];
            threshold_values[i] = ((max_njl[i] - min_njl[i]) / 5) + min_njl[i];
//!            threshold_values[i] = ((max_njl[i] - min_njl[i]) / 3) + min_njl[i];
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
      
      
//!      if (digital_value & 0b00001111100000000000)
//!      if (digital_value & 0b00000000000000000111)
//!      {
//!         b_high();
//!      }
//!      else
//!      {
//!         b_low();
//!      }
//!      
//!      
//!      if (njl_values[13] > threshold_values[13])
//!      if (digital_value & 0b00001111100000000000)
//!      {
//!         l_high();
//!      }
//!      else
//!      {
//!         l_low();
//!      } 
//!      
     if (counter > 1)
      {
         b_high();
      }
      else
      {
         b_low();
      }
      if (counter > 2)
      {
         l_high();
         
      }
      else
      {
         l_low();
      }
      if (counter > 3)
      {
         f_high();
      }
      else
      {
         f_low();
      }


//!      for (unsigned int8 i = 0; i < 20; i++)
//!      {
//!         printf("%u", digital_value&bit_values[i]);
//!         printf("%ld", (digital_value&bit_values[i]));
//!      }
//!      printf("\r\n");

//!     printf("%lu\r\n", digital_value);
     
      if (counter_4ms > 20)
      {
         r_toggle();
         counter_4ms = 0;
      }
//!      uint32_to_uint8(digital_value, data);
//!      convert32to24(digital_value, &data[0], &data[1], &data[2]);
      data[2] = digital_value & 0x000000FF;
      digital_value >>= 8;
      data[1] = digital_value & 0x000000FF;
      digital_value >>= 8;
      data[0] = digital_value & 0x000000FF;
      
      
      data[3] = 80;
      
      
//!      printf("%lu   H: %d   M: %d   L: %d\r\n", digital_value, data[0], data[1], data[2]);
      
//!      printf("%d\r\n", data[0]);
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
//!         bit_set(digital_value, i);
         digital_value |= bit_values[i];
//!         printf("%u\r\n", i);
         counter++;
      }
//!      else
//!      {
//!         bit_clear(digital_value, i);
//!         digital_value &= ~bit_values[i];
//!      }
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

