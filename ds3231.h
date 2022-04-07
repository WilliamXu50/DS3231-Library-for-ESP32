#ifndef _DS3231_H_
#define _DS3231_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driver/i2c.h"

//DS3231 slave address for I2C communication
#define DS3231_SLAVE_ADDR           0x68

// I2C pin configurations
#define I2C_MASTER_SCK      19
#define I2C_MASTER_SDA      18
#define DS3231_INTR_PIN     21

/*  Register addresses for DS3231 time-keeping registers, 
 *  alarm registers and control & status registers
 */
#define DS3231_SECONDS_REG_ADDR     0x00
#define DS3231_MINUTES_REG_ADDR     0x01
#define DS3231_HOURS_REG_ADDR       0x02
#define DS3231_DAY_REG_ADDR         0x03
#define DS3231_DATE_REG_ADDR        0x04
#define DS3231_MONTH_REG_ADDR       0x05
#define DS3231_YEAR_REG_ADDR        0x06

#define DS3231_ALARM_1_SECONDS_REG_ADDR     0x07
#define DS3231_ALARM_1_MINUTES_REG_ADDR     0x08
#define DS3231_ALARM_1_HOURS_REG_ADDR       0x09
#define DS3231_ALARM_1_DAY_DATE_REG_ADDR    0x0A

#define DS3231_ALARM_2_MINUTES_REG_ADDR     0x0B
#define DS3231_ALARM_2_HOURS_REG_ADDR       0x0C
#define DS3231_ALARM_2_DAY_DATE_REG_ADDR    0x0D

#define DS3231_CONTROL_REG_ADDR     0x0E
#define DS3231_STATUS_REG_ADDR      0x0F

#define RECEIVED_ACK        0x00
#define RECEIVED_NOT_ACK    0x01

//Control register bit masks
#define EN_OSC_BITMASK      (1<<7)
#define BATTERY_SQ_WAVE_EN_BITMASK      (1<<6)
#define CONV_TEMP_BITMASK       (1<<5)
#define RATE_SELECT_2_BITMASK       (1<<4)
#define RATE_SELECT_1_BITMASK       (1<<3)
#define INTR_CTRL_BITMASK           (1<<2)
#define ALARM2_INTR_EN_BITMASK      (1<<1)
#define ALARM1_INTR_EN_BITMASK      (1<<0)

//Status register bit masks
#define STATUS_OSF_BITMASK      (1<<7)
#define STATUS_EN_32KHZ_BITMASK     (1<<3)
#define STATUS_BSY_BITMASK      (1<<2)
#define STATUS_AF1_BITMASK      (1<<1)
#define STATUS_AF2_BITMASK      (1<<0)

//Square wave frequencies bitmasks
#define DS3231_CONTROL_RATE_1HZ         0b00000000
#define DS3231_CONTROL_RATE_1024HZ      0b00001000
#define DS3231_CONTROL_RATE_4096HZ      0b00010000
#define DS3231_CONTROL_RATE_8192HZ      0b00011000

//Struct for holding the I2C port number & DS3231 slave address
typedef struct{
    i2c_port_t i2c_port_num;
    uint8_t i2c_dev_addr;
}ds3231_t;

//Time registers read from DS3231 are stored here
typedef struct{
    uint8_t secs;
    uint8_t mins;
    uint8_t hours;
    
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
}ds3231_timestamp_t;

//Status register flags are stored in here
typedef struct{
    bool osf;
    bool en_32khz;
    bool bsy;
    bool alarm1;
    bool alarm2;
}ds3231_status_reg_t;

/*  Enum values for selecting a square wave output frequency
*   These enum values will be used as indexes to access the different frequency
*   bitmasks in sq_wave_rate_sel[4] array;
*/
typedef enum{
    DS3231_SQ_WAVE_1HZ=0,
    DS3231_SQ_WAVE_1024HZ=1,
    DS3231_SQ_WAVE_4096HZ=2,
    DS3231_SQ_WAVE_8192HZ=3
}ds3231_sq_wave_rate;

static uint8_t sq_wave_rate_sel[4]={
    DS3231_CONTROL_RATE_1HZ,
    DS3231_CONTROL_RATE_1024HZ,
    DS3231_CONTROL_RATE_4096HZ,
    DS3231_CONTROL_RATE_8192HZ
};

/* Helper functions to convert from decimal to BCD and vice versa*/
uint8_t bcd_to_dec(uint8_t bcd);

uint8_t dec_to_bcd(uint8_t dec);

ds3231_t ds3231_init(i2c_port_t i2c_port_num, uint8_t i2c_dev_addr);


/* Writes a data buffer to DS3231, starting at base address 'reg_base_addr'
*
*  To send a byte to DS3231 via I2C, the master needs to send a chain commands to DS3231 as follows:
*
*  | START CMD | -> | 7-bit slave address + R/W bit + ACK bit| -> 
*  | Register address + ACK bit | -> | Data byte 1 + ACK bit | -> | Data byte 2 + ACK bit | -> .... -> 
*  | Data byte n + ACK bit | ->| STOP CMD |
*
*  These commands are to be queued into the command handle (i2c_cmd_handle_t), which is really just a void ptr
*
*  START CMD: This command is queued using i2c_master_start()
*             This command signals the DS3231 that an I2C data transfer is to be commenced
*
*  7-Bit Slave Addr, R/W Bit & ACK Bit: The 7-bit slave address and the read/write bit is 
                                        queued together as 1-byte using i2c_master_write_byte()
*                                       and by enabling 'ack_en' arg to check for acknowledge bit
*
*  Register Address: The specific register address which we wish to write data to.
*                    Queued using i2c_master_write_byte() and by enabling 'ack_en' arg 
*                    to check for acknowledge bit
*
*  Data buffer: Actual data which we wish to write to a DS3231 register. 
*               Queued using i2c_master_write() to transfer an entire buffer,
*               and by enable checking the acknowledge bit after each byte sent
*
*  STOP CMD: This command signals DS3231 that the I2C data transfer is finished
*            This command is queued using i2c_master_stop
*
*  After queuing the above chain of data & signals to the command list (i2c_cmd_handle_t),
*  i2c_master_cmd_begin() will beginning sending the command list to the I2C bus -> DS3231
*

*   Parameters:
*   - i2c_port_num: The I2C port number that is configured for DS3231
*   - slave_addr: The DS3231 slave address
*   - reg_base_addr: The starting register address of DS3231 that data is written to 
      (after each byte written, the DS3231 will automatically increment the register pointer)
*   - data_buf: Pointer to a data buffer in which the data is to be written to DS3231
*   - size: The number of bytes to be written
*/
esp_err_t ds3231_i2c_write(i2c_port_t i2c_port_num, uint8_t slave_addr, uint8_t reg_base_addr, uint8_t* data_buff, size_t num_bytes);

/*  Read a stream of bytes from DS3231 starting at the base addr 'reg_base_addr'
 *  To initiate I2C master read, the master has to do the following:
    - Send START command
    - Send DS3231 slave address + WR bit + ACK
    - Send starting register address + ACK:

    - Send a repeated START command
    - Send the DS3231 slave address again + RD bit +ACK
    - Start reading data from DS3231 byte by byte (each byte, except the last) will be 
      acknowledged by the master
      (last byte doesn't need to be acknowledged)
    - Send the STOP command (end of communication)

*   Data Reception Diagram:

    |START| -> |DS3231 SLAVE ADDR + WRITE + ACK| -> |REGISTER ADDR + ACK| ->
    |REPEATED START| -> |DS3231 SLAVE ADDR + READ + ACK| -> 
    |DATA1 + ACK| -> |DATA2 + ACK| -> .... -> |LAST DATA + NACK| -> |STOP|

*   Parameters:
*   - i2c_port_num: The I2C port number that is configured for DS3231
*   - slave_addr: The DS3231 slave address
*   - reg_base_addr: The starting register address of DS3231 that is to be read 
      (after each byte read, the DS3231 will automatically increment the register pointer)
*   - read_buf: Pointer to a data buffer in which the data read from I2C will be stored
*   - size: The number of bytes to be read
*/
esp_err_t ds3231_i2c_read(i2c_port_t i2c_port_num, uint8_t slave_addr, uint8_t reg_base_addr, uint8_t* read_buf, size_t size);

/*  Set the DS3231 time registers (seconds, minutes & hours)
*   Seconds: 0 -> 59
*   Minutes: 0 -> 59
*   Hours: 0 -> 23
*
*   NOTE: The seconds, minutes and hours values will wrap around if exceeded their boundaries
*/
esp_err_t ds3231_set_time(ds3231_t* dev, uint8_t secs, uint8_t mins, uint8_t hrs);

/*  Read the DS3231 time registers (seconds, minutes & hours)  */
esp_err_t ds3231_get_time(ds3231_t* dev, ds3231_timestamp_t* time);

/*  Read the DS3231 status register */
esp_err_t ds3231_get_status(ds3231_t* dev, ds3231_status_reg_t* status_reg);

/*  Set the DS3231 calender registers (day of the week, day of the month, month & year)
*   Day of the week: 1 -> 7 or 0 -> 6 (user preference)
*   Day of the month: 1 -> 31
*   Month: 1 -> 12
*   Year: 0 -> 99
*
*   NOTE: This function will also check for the validity of the date entered (the day of the month and the month)
*/
esp_err_t ds3231_set_calendar(ds3231_t* dev, uint8_t day, uint8_t date, uint8_t month, uint8_t year);

/*  Read the DS3231 calender registers (day of the week, day of the month, month & year) */
esp_err_t ds3231_get_calendar(ds3231_t* dev, ds3231_timestamp_t* time);

/*  Set the square-wave output frequency */
esp_err_t ds3231_set_square_wave(ds3231_t* dev, ds3231_sq_wave_rate rate);
#endif