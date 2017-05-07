#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k
//#define SL_WR 0b01000000
//#define SL_RD 0b01000001
#include "i2c_master_noint.h"


void i2c_master_setup(void) {
  ANSELBbits.ANSB2=0; // Turning off default ANSEL function for B2 pin
  ANSELBbits.ANSB3=0; // Turning off default ANSEL function for B3 pin
  I2C2BRG = 53;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2  Baudrate to 400kHz
                                    // look up PGD for your PIC32
                                    //  PGD Typically value is 104 ns for PIC32MX250F128B
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
  
  I2C_write(0x10,0b10000010); // accelerometer ON with 1.66 kHz, +-2g resol, 100 Hz antialiasing
  I2C_write(0x11,0b10001000); // gyroscope ON with 1.66 kHz, 1000 dps sens
  I2C_write(0x12,0b00000100); // setting CTRL3,auto address increment during multi read set to ON 
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent,
    //hardware turns SEN to 0 automatically after start bit is sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

void I2C_write(char addr, char data){
    i2c_master_start(); //sending start bit
    i2c_master_send(SL_WR); //sending Write Control bit, last bit 0 as this is a write
    i2c_master_send(addr); //sending address of register
    i2c_master_send(data); //sending data to write to register
    i2c_master_stop(); // sending stop bit, releases control of bus
}

char I2C_read(char addr){
    char val;
    i2c_master_start(); //sending start bit, this time to write register to read 
    i2c_master_send(SL_WR); //sending Write Control bit, last bit 0 as this is a write
    i2c_master_send(addr); //sending address of PORT register
    i2c_master_restart(); //restarting transfer this time to read
    i2c_master_send(SL_RD); //sending Read Control bit, last bit 1 as this is a read
    val=i2c_master_recv(); //receiving data from register
    i2c_master_ack(1); // sending NACK to signal end of requests
    i2c_master_stop(); // sending stop bit, releases control of bus
    return val;
}

void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length){
    unsigned short i;
    
    i2c_master_start(); //sending start bit, this time to write register to read 
    i2c_master_send(address); //sending Write Control bit, last bit 0 as this is a write
    i2c_master_send(reg); //sending address of PORT register
    i2c_master_restart(); //restarting transfer this time to read
    i2c_master_send(address+1); //sending Read Control bit, last bit 1 as this is a read
    for(i=0;i<length-1;i++){
    data[i]=i2c_master_recv(); //receiving data from register
    i2c_master_ack(0);
    }
    data[length-1]=i2c_master_recv();
    i2c_master_ack(1); // sending NACK to signal end of requests
    i2c_master_stop(); // sending stop bit, releases control of bus;
}

short combine(unsigned char l, unsigned char h){ //combines IMU 16bits ints which are split to 2 8byte vals
    short c;
    c=(h<<8)|l; // combining msbs with lsbs
    return c;
}

void IMU_multiRead(unsigned char address, unsigned char reg, short * compData, int length){
    int d;
    char datap[length*2];
    I2C_read_multiple(address, reg , datap, length*2);
    for(d=0;d<length;d++){
        compData[d]=combine(datap[2*d],datap[2*d+1]);
    }
}


  