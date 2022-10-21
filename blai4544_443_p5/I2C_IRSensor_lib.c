#include <plib.h>
#include "LCDlib.h"
#include "CerebotMX7cK.h"
#include "I2C_IRSensor_lib.h"

void init_I2C1_IRSensor(){
    OpenI2C1(I2C_EN, BRG_VAL);
}


/*
 Function polls EEPROM for page write completion
 * INPUT:
 *      slaveAddr, the address of the slave device to poll
 * OUTPUT:
 *      NA
 */
void wait_I2C_Xfer(int slaveAddr){
    StartI2C1();
    IdleI2C1();
    
    // Continue polling
    while (MasterWriteI2C1((slaveAddr << 1) | 0)){
        RestartI2C1();
        IdleI2C1();
    }
    
    //Final restart
    StopI2C1();
    IdleI2C1();
}


/*
 * INPUT:
 *      slaveAddr, addres of the device to read from
 *      mem_addr, address to start reading
 *      i2cData, data buffer to store read values
 *      length, length of data to read
 * OUTPUT:
 *      int, exit status of function
 *      0   :   All good, no errors
 *      1   :   Address overflow
 *      2   :   Address underflow
 *      3   :   Invalid negative length
 *      4   :   Invalid length, length won't fit in EEPROM
 *      5   :   Error while writing
 */
int I2C1_IR_Read(unsigned short int* sensorTemp, unsigned short int length){
    
    // Temporary char for message
    unsigned short int sensorReading = 0;
    
    // Make control, high address, and low address
    unsigned char write_err = 0;
    
    // Array to store data in 
    unsigned char i2cData[3];
    StartI2C1();
    IdleI2C1();
    
    // write to IR sensor
    write_err |= MasterWriteI2C1(0xB4); // IR sensor address
    write_err |= MasterWriteI2C1(0x07); // RAM address
    
    if (write_err){
        LCD_puts("Write Error in\nRead EEPROM");
        return 5;
    }
    
    // Restart I2C and write read control byte
    RestartI2C1();
    IdleI2C1();
    
    // Tell IR sensor to read
    write_err |= MasterWriteI2C1(0xB5);// Read command!
    
    // Iterable variable made from length input
    int i = 0;
    
    // Read each byte in i2cData
    while (i < length){
        
        // Set each byte of data to a position in array
        i2cData[i] = MasterReadI2C1();// Read!
                
        // Increment the byte counter
        i++;
        
        AckI2C1();
        IdleI2C1();
    }
        
    // Stop reading
    StopI2C1();
    IdleI2C1();
    
    // Re-arrange LSB to MSB spot and vice versa
                    // Cast MSB to int & shift, cast LSB to int & or both
    sensorReading = (((short int) i2cData[1]) << 8) | ((short int) i2cData[0]);
    
    // Put the data where it needs to be!
    // Send only the raw data so that the processing can happen elsewhere
    *sensorTemp = sensorReading;
    
    //return no error
    return 0;
}
