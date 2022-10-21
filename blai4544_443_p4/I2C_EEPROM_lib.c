#include <plib.h>
#include "LCDlib.h"
#include "CerebotMX7cK.h"
#include "I2C_EEPROM_lib.h"

void init_EEPROM(){
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
 Input: mem_addr, the memory addr to check
 *      len, the length of data to write
 * 
 * OUTPUT:
 *      0, all good! No problem
 *      1, overflow
 */
int memCheck( int mem_addr, int length){
    if (mem_addr >= 0x800){ // addr larger than memory
        LCD_puts("Addr not valid\nOverflow");
        return 1;
    }
    else if(mem_addr < 0x000){// addr is negative
        LCD_puts("Addr not valid\nUnderflow");
        return 2;
    }
    else if (length < 0){// len is negative
        LCD_puts("Len is negative\n");
        return 3;
    }
    else if(length > 32768){// len won't fit in EEPROM
        LCD_puts("Len > EEPROM\n");
        return 4;
    }
    return 0;
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
int I2C_IR_Read(int slaveAddr, int mem_addr, char* i2cData, int length){
    
    // Temporary char for message
    char temp[10];
    
    // Make control, high address, and low address
    unsigned char write_err = 0;
    unsigned int highAddr = ((mem_addr & 0xFF00)>>8);
    unsigned int lowAddr = mem_addr & 0x00FF;
    int i = 0;
    
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
    int len = length;
    
    // Read each byte in i2cData
    //while (len--){
    while (len > 0){
        
        *i2cData = MasterReadI2C1();// Read!
        *i2cData++;
        
        AckI2C1();
        IdleI2C1();
    }
    
    // Stop reading with a NAK
    NotAckI2C1();
    IdleI2C1();
    StopI2C1();
    IdleI2C1();
    
    //return no error
    return 0;
}

/*
 * INPUT:
 *      slaveAddr, addres of the device to write to
 *      mem_addr, address to start writing
 *      i2cData, data buffer to store values to write
 *      length, length of data to write
 * 
 * OUTPUT:
 *      int, exit status of function
 *      0   :   All good, no errors
 *      1   :   Address overflow
 *      2   :   Address underflow
 *      3   :   Invalid negative length
 *      4   :   Invalid length, length won't fit in EEPROM
 *      5   :   Error while writing
 */
int writeEEPROM(int slaveAddr, int mem_addr, char* i2cData, int length){
    
    // Is the memory address valid?
    int mem_error = memCheck(mem_addr, length);
    
    // Check for return error
    if(mem_error != 0){
        return mem_error;
    }
    
    unsigned int write_err = 0;
    int i = 0;
    int highAddr = (mem_addr & 0xFF00)>>8;// set highAddr
    int lowAddr = mem_addr & 0x00FF;// set lowAddr
    int pagePos = mem_addr;
    
    // Start I2C, write control & addr bytes
    StartI2C1();
    IdleI2C1();
    write_err |= MasterWriteI2C1(((slaveAddr << 1) | 0));//Write control byte
    write_err |= MasterWriteI2C1(highAddr);// Write highAddr
    write_err |= MasterWriteI2C1(lowAddr);// Write lowAddr
    
    // Iterable variable made from length input
    int len = length;
    
    while(len--){
        write_err |= MasterWriteI2C1(i2cData[i++]);
        pagePos++;
        
        if((pagePos % 64)== 0){// check if page boundary
            
            // Stop so EEPROM can do internal write
            StopI2C1();
            IdleI2C1();
            
            // Poll EEPROM for completion
            wait_I2C_Xfer(slaveAddr);
            
            // send ctrl byte as well as the next mem addr
            StartI2C1();
            IdleI2C1();
            write_err |= MasterWriteI2C1(((slaveAddr << 1)|0));//control byte
            write_err |= MasterWriteI2C1((pagePos & 0xFF00)>>8);// high mem_addr
            write_err |= MasterWriteI2C1(pagePos & 0x00FF);// low mem addr
            
            if (write_err){// Something went wrong while writing
                LCD_puts("Write Error in\nWrite EEPROM");
                return 5;
            }
        }
    }
    // Stop I2C activities and idle
    StopI2C1();
    IdleI2C1();
    
    //Wait for EEPROM to do internal page write
    wait_I2C_Xfer(slaveAddr);
    
    // Stop I2C activities and idle
    StopI2C1();
    IdleI2C1();
    
    //Return no error
    return 0;
}

