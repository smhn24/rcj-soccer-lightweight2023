#include <Wire.h>
#include <SPI.h>

#define GY_955_ADDRESS 0x29
#define SS_PIN 10

int angle = 0;
volatile byte received;
volatile bool process_it = false;
uint8_t counter = 123;

void bno055_config() {
  Wire.begin();
  Wire.setClock(400000);  // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
  delay(100);
  Wire.beginTransmission(GY_955_ADDRESS);
  Wire.write(0x3F);  // Power Mode
  Wire.write(0x20);  // Normal:0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)
  Wire.endTransmission();
  delay(700);
  Wire.beginTransmission(GY_955_ADDRESS);
  Wire.write(0x3E);  // Power Mode
  Wire.write(0x00);  // Normal:0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(GY_955_ADDRESS);
  Wire.write(0x3D);  // Operation Mode
  Wire.write(0x0C);  //NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011)
  Wire.endTransmission();
  delay(100);
}

int bno055_read() {
  int angle;
  Wire.beginTransmission(GY_955_ADDRESS);
  Wire.write(0x1A);
  Wire.endTransmission(false);
  Wire.requestFrom(GY_955_ADDRESS, 2, true);
  angle = (int)((int16_t)(Wire.read() | Wire.read() << 8) / 16.00);  //in Degrees unit
  return angle;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MISO, OUTPUT);
  SPCR = (1 << SPE) | (1 << SPIE);
  sei();
  process_it = false;
  delay(1000);
  bno055_config();
  Serial.begin(115200);
}

// SPI interrupt routine
ISR(SPI_STC_vect) {
  process_it = true;
  received = SPDR;
}

void loop() {
  angle = bno055_read();
  Serial.println(angle);
  if (process_it) {
    if (received == 'L') {
      SPDR = lowByte(angle);
    } else if (received == 'H') {
      SPDR = highByte(angle);
    }
    process_it = false;
  }
}
