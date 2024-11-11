/**************************************************************************************************
 * File: i2c.h
 * Author: Jacob Clarey
 * Date: 10/29/2024
 * Description: This header is for the I2C bare metal drivers.
 *************************************************************************************************/

#ifndef __I2C_H__
#define __I2C_H__

#include <stm32f301x8.h>

#ifndef STM32_SADDR
#define STM32_SADDR (0x32U)
#endif

#ifndef I2C_BUS
#define I2C_BUS I2C2
#endif

/// @brief Example usage: I2C_Master_Init(I2C1, GPIOC, 6, 7);
/// @param i2c is the i2c bus to enable.
/// @param port is the port the SCL and SDA pins are on.
/// @param sclPin is the SCL pin number.
/// @param sdaPin is the SDA pin number.
void I2C_Master_Init(I2C_TypeDef *i2c, GPIO_TypeDef *port, uint8_t sclPin,
                     uint8_t sdaPin);

/// @brief Example usage: I2C_Master_Transmit(I2C1, saddr, array_of_bytes,
/// number_of_bytes);
/// @param i2c is the i2c bus to enable.
/// @param saddr is the slave address to transmit to.
/// @param data is the data to transmit.
/// @param size is the number of bytes to transmit.
/// @return Returns 1 when the transmit is complete.
uint8_t I2C_Master_Transmit(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data,
                            uint16_t size);

/// @brief Example usage: I2C_Master_Transmit_Maddr(I2C1, saddr, maddr_bytes,
/// number_of_bytes_in_maddr, array_of_bytes, number_of_bytes);
/// @param i2c is the i2c bus to enable.
/// @param saddr is the slave address to transmit to.
/// @param maddr is an array of bytes where [0] is the uppermost 8 bits and [n]
/// is the lowermost 8 bits of the memory address.
/// @param maddrSize is the number of bytes in the memory address.
/// @param data is the data to transmit.
/// @param size is the the number of bytes to transmit.
/// @return Returns 1 when the transmit is complete.
uint8_t I2C_Master_Transmit_Maddr(I2C_TypeDef *i2c, uint8_t saddr,
                                  uint8_t *maddr, uint8_t maddrSize,
                                  uint8_t *data, uint16_t size);

/// @brief Example usage: I2C_Master_SuperTransmit_Maddr(I2C1, saddr,
/// maddr_bytes, number_of_bytes_in_maddr, array_of_bytes, number_of_bytes);
/// @param i2c is the i2c bus to enable.
/// @param saddr is the slave address to transmit to.
/// @param maddr is an array of bytes where [0] is the uppermost 8 bits and [n]
/// is the lowermost 8 bits of the memory address.
/// @param maddrSize is the number of bytes in the memory address.
/// @param data is the data to transmit.
/// @param size is the the number of bytes to transmit.
/// @return Returns 1 when the transmit is complete.
uint8_t I2C_Master_SuperTransmit_Maddr(I2C_TypeDef *i2c, uint8_t saddr,
                                       uint8_t *maddr, uint8_t maddrSize,
                                       uint8_t *data, uint16_t size);

/// @brief Example usage: I2C_Master_Receive(I2C1, saddr, maddr_bytes,
/// number_of_bytes_in_maddr, number_of_bytes_to_receive);
/// @param i2c is the i2c bus to enable.
/// @param saddr is the slave address to transmit to.
/// @param maddr is an array of bytes where [0] is the uppermost 8 bits and [n]
/// is the lowermost 8 bits of the memory address.
/// @param maddrSize is the number of bytes in the memory address.
/// @param rxSize is the number of bytes to receive.
/// @return Returns received data as array.
uint8_t *I2C_Master_Receive(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *maddr,
                            uint8_t maddrSize, uint8_t rxSize);

/// @brief Example usage: I2C_Slave_Init(I2C1, GPIOC, 6, 7);
/// @param i2c is the i2c bus to enable.
/// @param port is the port the SCL and SDA pins are on.
/// @param sclPin is the SCL pin number.
/// @param sdaPin is the SDA pin number.
void I2C_Slave_Init(I2C_TypeDef *i2c, GPIO_TypeDef *port, uint8_t sclPin,
                    uint8_t sdaPin);

/// @brief Example usage: I2C_Slave_Transmit(I2C1, data_to_send);
/// @param i2c is the i2c bus to listen on.
/// @param data is the array of data to transmit on request.
/// @return Returns 1 when the transmit is complete.
uint8_t I2C_Slave_Transmit(I2C_TypeDef *i2c, uint8_t *data);

/// @brief Example usage: I2C_Slave_Receive(I2C1);
/// @param i2c is the i2c bus to listen on.
/// @return Returns received data as array.
uint8_t *I2C_Slave_Receive(I2C_TypeDef *i2c);

#endif  // __I2C_H__

/* EOF */