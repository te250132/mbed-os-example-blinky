/**********************************************
MLX90363 Example rotary program for mbed board
Purpose: Demonstrate getting angular data from
MLX90363 via GET1a message.
Revision: 001
Date: 1-Nov-13
Changes: Initial Revision
**********************************************/
#include "mbed.h"
SPI spi(p5, p6, p7); //MOSI, MISO, CLK
DigitalOut ss1(p8); //SS Pin for Die 1
Serial pc(USBTX, USBRX); //Create a virtual serial port
int main(void) {
char u8_spi_mode = 1; //SPI mode
char u8_spi_bits = 8; //number of bits per mbed write command
char u8_spi_read_buffer[8];
char u8_spi_write_buffer[8];
float f32_angle_degrees;
unsigned int u16_angle_lsb;
char u8_error_lsb;
char u8_rollcnt_dec;
char u8_virtualgain_dec;
char u8_crc_dec;
//360 degrees is mapped to 14 bits = 360/2^14 = 0.02197
const float f32_lsb_to_dec_degrees = 0.02197;
//Serial port setup
pc.baud(115200);
//SPI Bus setup
spi.format(u8_spi_bits, u8_spi_mode);
spi.frequency(500000);
ss1 = 1; //SS to deselected state
while (1){
//issue GET1 message
u8_spi_write_buffer[0] = 0x00;
u8_spi_write_buffer[1] = 0x00;
u8_spi_write_buffer[2] = 0xFF;
u8_spi_write_buffer[3] = 0xFF;
u8_spi_write_buffer[4] = 0x00;
u8_spi_write_buffer[5] = 0x00;
u8_spi_write_buffer[6] = 0x13;
u8_spi_write_buffer[7] = 0xEA;
ss1 = 0;
for (int i = 0; i < 8; i++){
u8_spi_read_buffer[i] = spi.write(u8_spi_write_buffer[i]);
}
ss1 = 1;
wait_ms(1);
//issue NOP message
u8_spi_write_buffer[0] = 0x00;
u8_spi_write_buffer[1] = 0x00;
u8_spi_write_buffer[2] = 0xAA;
u8_spi_write_buffer[3] = 0xAA;
u8_spi_write_buffer[4] = 0x00;
u8_spi_write_buffer[5] = 0x00;
u8_spi_write_buffer[6] = 0xD0;
u8_spi_write_buffer[7] = 0xAB;
ss1 = 0;
for (int i = 0; i < 8; i++){
u8_spi_read_buffer[i] = spi.write(u8_spi_write_buffer[i]);
}
ss1 = 1;
//Extract and convert the angle to degrees
//remove error bits and shift to high byte
u16_angle_lsb = (u8_spi_read_buffer[1] & 0x3F) << 8;
//add LSB of angle
u16_angle_lsb = u16_angle_lsb + u8_spi_read_buffer[0];
//convert to decimal degrees
f32_angle_degrees = u16_angle_lsb * f32_lsb_to_dec_degrees;
//Extract the error bits
u8_error_lsb = u8_spi_read_buffer[1] >> 6;
//Extract the CRC
u8_crc_dec = u8_spi_read_buffer[7];
//Extract the virtual gain byte
u8_virtualgain_dec = u8_spi_read_buffer[4];
//Extract the rolling counter
u8_rollcnt_dec = u8_spi_read_buffer[6] & 0x3F;
//Send results to serial port
pc.printf("Angle (LSb): %d\r\n", u16_angle_lsb);
pc.printf("Angle (Dec): %2.2f\r\n", f32_angle_degrees);
pc.printf("Error Bits (Dec): %d\r\n", u8_error_lsb);
pc.printf("CRC (Dec): %d\r\n", u8_crc_dec);
pc.printf("Virtual Gain (Dec): %d\r\n", u8_virtualgain_dec);
pc.printf("Rolling Counter (Dec): %d\r\n", u8_rollcnt_dec);
wait(0.5);
}
}
