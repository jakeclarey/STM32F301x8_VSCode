/**************************************************************************************************
 * File: i2c.c
 * Author: Jacob Clarey
 * Date: 10/29/2024
 * Description: This source is for the I2C bare metal drivers.
 *************************************************************************************************/

#include "i2c.h"

/// This function initializes the given i2c peripheral at 100kHz as a master
/// device.
void I2C_Master_Init(I2C_TypeDef *i2c, GPIO_TypeDef *port, uint8_t sclPin, uint8_t sdaPin)
{
    switch ((uint32_t)port)
    {
    case (uint32_t)GPIOA:
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
        break;

    case (uint32_t)GPIOB:
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
        break;

    case (uint32_t)GPIOC:
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
        break;

    default:
        return;
    }

    port->MODER &= ~((3U << (2 * sclPin)) | (3U << (2 * sdaPin)));
    port->MODER |= (2U << (2 * sclPin)) | (2U << (2 * sdaPin));

    port->OTYPER |= (1U << sclPin) | (1U << sdaPin);

    // port->PUPDR &= ~((3U << (2 * sclPin)) | (3U << (2 * sdaPin)));
    // port->PUPDR |= (1U << (2 * sclPin)) | (1U << (2 * sdaPin));

    if (sclPin < 8U)
    {
        port->AFR[0] &= ~(0xFU << (4 * sclPin));
        port->AFR[0] |= (4U << (4 * sclPin));
    }
    else
    {
        port->AFR[1] &= ~(0xFU << (4 * (sclPin - 8)));
        port->AFR[1] |= (4U << (4 * (sclPin - 8)));
    }

    if (sdaPin < 8U)
    {
        port->AFR[0] &= ~(0xFU << (4 * sdaPin));
        port->AFR[0] |= (4U << (4 * sdaPin));
    }
    else
    {
        port->AFR[1] &= ~(0xFU << (4 * (sdaPin - 8)));
        port->AFR[1] |= (4U << (4 * (sdaPin - 8)));
    }

    /* Allow time for pins to be set before enabling the  I2C RCC clock (avoid
     * accidental busy state) */
    for (uint32_t i = 0; i < 100000; i++)
    {
        asm("NOP");
    }

    switch ((uint32_t)i2c)
    {
    case (uint32_t)I2C1:
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        break;

    default:
        return;
    }

    /* Following steps are from page 701 of the STM32F446RE reference manual */
    i2c->CR1 &= ~(I2C_CR1_PE | I2C_CR1_ANFOFF | I2C_CR1_DNF | I2C_CR1_NOSTRETCH);
    i2c->TIMINGR = 0x00901D23U; // set frequency to 100kHz assuming 8MHz i2c clock

    i2c->CR1 |= I2C_CR1_PE;
}

/// This functions operates by first doing the setup steps for an i2c master
/// transmission. Then, the process continues with a loop to ensure all data
/// bytes are sent. The function returns 0 when the transfer is completed.
uint8_t I2C_Master_Transmit(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *data, uint16_t size)
{
    while (i2c->ISR & I2C_ISR_BUSY)
    {
    }

    /* Following steps are from page 718 of the STM32F446RE reference manual */
    i2c->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD);
    i2c->CR2 |= (size << I2C_CR2_NBYTES_Pos);

    /* Allow AUTOEND (I modified the provided step, opposite of manual)*/
    i2c->CR2 |= I2C_CR2_AUTOEND;

    i2c->CR2 |= saddr << (I2C_CR2_SADD_Pos + 1);

    /* Flush TXDR (I added this line in, wasn't in manual) */
    i2c->ISR |= I2C_ISR_TXE;

    /* Set to write operation (I added this line in, wasn't in manual) */
    i2c->CR2 &= ~I2C_CR2_RD_WRN;

    /* Continuation of steps from page 718 */
    i2c->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < size; i++)
    {
        while (!(i2c->ISR & I2C_ISR_TXE))
        {
        }

        i2c->TXDR = data[i];
    }

    return 0;
}

uint8_t I2C_Master_Transmit_Maddr(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *maddr, uint8_t maddrSize, uint8_t *data,
                                  uint16_t size)
{
    while (i2c->ISR & I2C_ISR_BUSY)
    {
    }

    /* Following steps are from page 718 of the STM32F446RE reference manual */
    i2c->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD);
    i2c->CR2 |= (maddrSize << I2C_CR2_NBYTES_Pos);

    /* Disable AUTOEND */
    i2c->CR2 &= ~I2C_CR2_AUTOEND;

    i2c->CR2 |= saddr << (I2C_CR2_SADD_Pos + 1);

    /* Flush TXDR (I added this line in, wasn't in manual) */
    i2c->ISR |= I2C_ISR_TXE;

    /* Set to write operation (I added this line in, wasn't in manual) */
    i2c->CR2 &= ~I2C_CR2_RD_WRN;

    /* Continuation of steps from page 718 */
    i2c->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < maddrSize; i++)
    {
        while (!(i2c->ISR & I2C_ISR_TXE))
        {
        }

        i2c->TXDR = maddr[i];
    }

    /* Following steps are from page 724 of the STM32F446RE reference manual */
    while (!(i2c->ISR & I2C_ISR_TC))
    {
    }

    /* Following steps are from page 718 of the STM32F446RE reference manual */
    i2c->CR2 &= ~I2C_CR2_NBYTES;
    i2c->CR2 |= (size << I2C_CR2_NBYTES_Pos);

    /* Flush TXDR (I added this line in, wasn't in manual) */
    i2c->ISR |= I2C_ISR_TXE;

    /* Set to write operation (I added this line in, wasn't in manual) */
    i2c->CR2 &= ~I2C_CR2_RD_WRN;

    /* Continuation of steps from page 718 */
    i2c->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < size; i++)
    {
        while (!(i2c->ISR & I2C_ISR_TXE))
        {
        }

        i2c->TXDR = data[i];
    }

    while (!(i2c->ISR & I2C_ISR_TC))
    {
    }

    i2c->CR2 |= I2C_CR2_STOP;

    return 0;
}

/// This function transmits more than 255 bytes in a "single" transmission. One
/// function call for a large array of data to send using the RELOAD
/// functionality of  I2C.
uint8_t I2C_Master_SuperTransmit_Maddr(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *maddr, uint8_t maddrSize,
                                       uint8_t *data, uint16_t size)
{
    uint8_t blocks = size / 256;
    while (i2c->ISR & I2C_ISR_BUSY)
    {
    }

    /* Following steps are from page 718 of the STM32F446RE reference manual */
    i2c->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD);
    i2c->CR2 |= (maddrSize << I2C_CR2_NBYTES_Pos);

    /* Disable AUTOEND */
    i2c->CR2 &= ~I2C_CR2_AUTOEND;

    i2c->CR2 |= saddr << (I2C_CR2_SADD_Pos + 1);

    /* Flush TXDR (I added this line in, wasn't in manual) */
    i2c->ISR |= I2C_ISR_TXE;

    /* Set to write operation (I added this line in, wasn't in manual) */
    i2c->CR2 &= ~I2C_CR2_RD_WRN;

    /* Continuation of steps from page 718 */
    i2c->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < maddrSize; i++)
    {
        while (!(i2c->ISR & I2C_ISR_TXE))
        {
        }

        i2c->TXDR = maddr[i];
    }

    /* Following steps are from page 724 of the STM32F446RE reference manual */
    while (!(i2c->ISR & I2C_ISR_TC))
    {
    }

    /*************************************************************************/
    /* Completed maddr transmit */
    /*************************************************************************/

    /* Following steps are from page 719 of the STM32F446RE reference manual */
    i2c->CR2 &= ~I2C_CR2_NBYTES;

    if (size < 256)
    {
        i2c->CR2 |= (size << I2C_CR2_NBYTES_Pos);
        i2c->CR2 &= ~I2C_CR2_RELOAD;
    }
    else
    {
        i2c->CR2 |= (255U << I2C_CR2_NBYTES_Pos);
        i2c->CR2 |= I2C_CR2_RELOAD;
    }

    i2c->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < blocks; i++)
    {
        for (uint8_t j = 0; j < 255; j++)
        {
            while (!(i2c->ISR & I2C_ISR_TXE))
            {
            }

            i2c->TXDR = data[j + (255 * i)];
        }

        while (!(i2c->ISR & I2C_ISR_TCR))
        {
        }

        size -= 255;

        if (size < 255)
        {
            i2c->CR2 |= (size << I2C_CR2_NBYTES_Pos);
        }
        else
        {
            i2c->CR2 |= (255U << I2C_CR2_NBYTES_Pos);
        }
    }

    for (uint8_t i = 0; i < size; i++)
    {
        while (!(i2c->ISR & I2C_ISR_TXE))
        {
        }

        i2c->TXDR = data[i + (255 * blocks)];
    }

    while (!(i2c->ISR & I2C_ISR_TCR))
    {
    }

    i2c->CR2 |= I2C_CR2_STOP;
    i2c->CR2 &= ~I2C_CR2_RELOAD;

    return 0;
}

/// This function operates by first sending a memory address prior to a restart
/// condition. This memory address is formatted as an array in order to be
/// compataible with 16-bit and 32-bit I2C EEPROM chips. After the memory
/// address is sent, a number of bytes is expected to be read out at the end of
/// which the Master will stop the reception.
uint8_t *I2C_Master_Receive(I2C_TypeDef *i2c, uint8_t saddr, uint8_t *maddr, uint8_t maddrSize, uint8_t rxSize)
{
    static uint8_t rcvBuffer[255];

    while (i2c->ISR & I2C_ISR_BUSY)
    {
    }

    /* Following steps are from page 718 of the STM32F446RE reference manual */
    i2c->CR2 &= ~(I2C_CR2_NBYTES | I2C_CR2_SADD);
    i2c->CR2 |= (maddrSize << I2C_CR2_NBYTES_Pos);

    /* Disable AUTOEND */
    i2c->CR2 &= ~I2C_CR2_AUTOEND;

    i2c->CR2 |= saddr << (I2C_CR2_SADD_Pos + 1);

    /* Flush TXDR (I added this line in, wasn't in manual) */
    i2c->ISR |= I2C_ISR_TXE;

    /* Set to write operation (I added this line in, wasn't in manual) */
    i2c->CR2 &= ~I2C_CR2_RD_WRN;

    /* Continuation of steps from page 718 */
    i2c->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < maddrSize; i++)
    {
        while (!(i2c->ISR & I2C_ISR_TXE))
        {
        }

        i2c->TXDR = maddr[i];
    }

    /* Following steps are from page 724 of the STM32F446RE reference manual */
    while (!(i2c->ISR & I2C_ISR_TC))
    {
    }

    i2c->CR2 &= ~I2C_CR2_NBYTES;
    i2c->CR2 |= (rxSize << I2C_CR2_NBYTES_Pos);

    /* Set to read operation (I added this line in, wasn't in manual) */
    i2c->CR2 |= I2C_CR2_RD_WRN;

    i2c->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0; i < rxSize; i++)
    {
        while (!(i2c->ISR & I2C_ISR_RXNE))
        {
        }

        rcvBuffer[i] = i2c->RXDR;
    }

    return rcvBuffer;
}

/// This function first uses to switches to initialize the clock for the chosen
/// gpio port and the chosen i2c peripheral. After that, it will initialize the
/// i2c peripheral into a slave mode device with the slave address set in the
/// header.
void I2C_Slave_Init(I2C_TypeDef *i2c, GPIO_TypeDef *port, uint8_t sclPin, uint8_t sdaPin)
{
    switch ((uint32_t)port)
    {
    case (uint32_t)GPIOA:
        RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
        break;

    case (uint32_t)GPIOB:
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
        break;

    case (uint32_t)GPIOC:
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
        break;

    default:
        return;
    }

    port->MODER &= ~((3U << (2 * sclPin)) | (3U << (2 * sdaPin)));
    port->MODER |= (2U << (2 * sclPin)) | (2U << (2 * sdaPin));

    port->OTYPER |= (1U << sclPin) | (1U << sdaPin);

    // port->PUPDR &= ~((3U << (2 * sclPin)) | (3U << (2 * sdaPin)));
    // port->PUPDR |= (1U << (2 * sclPin)) | (1U << (2 * sdaPin));

    if (sclPin < 8U)
    {
        port->AFR[0] &= ~(0xFU << (4 * sclPin));
        port->AFR[0] |= (4U << (4 * sclPin));
    }
    else
    {
        port->AFR[1] &= ~(0xFU << (4 * (sclPin - 8)));
        port->AFR[1] |= (4U << (4 * (sclPin - 8)));
    }

    if (sdaPin < 8U)
    {
        port->AFR[0] &= ~(0xFU << (4 * sdaPin));
        port->AFR[0] |= (4U << (4 * sdaPin));
    }
    else
    {
        port->AFR[1] &= ~(0xFU << (4 * (sdaPin - 8)));
        port->AFR[1] |= (4U << (4 * (sdaPin - 8)));
    }

    /* Allow time for pins to be set before enabling the  I2C RCC clock (avoid
     * accidental busy state) */
    for (uint32_t i = 0; i < 100000; i++)
    {
        asm("NOP");
    }

    switch ((uint32_t)i2c)
    {
    case (uint32_t)I2C1:
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        break;

    case (uint32_t)I2C2:
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        break;

    default:
        return;
    }

    /* Following steps are from page 701 of the STM32F446RE reference manual */
    i2c->CR1 &= ~(I2C_CR1_PE | I2C_CR1_ANFOFF | I2C_CR1_DFN | I2C_CR1_NOSTRETCH);
    i2c->TIMINGR = 0x00901D23U; // set frequency to 100kHz assuming 16MHz i2c clock

    /* Following steps are from page 706 of the STM32F446RE reference manual */
    i2c->OAR1 &= ~(I2C_OAR1_OA1EN | I2C_OAR1_OA1 | I2C_OAR1_OA1MODE);
    i2c->OAR2 &= ~I2C_OAR2_OA2EN;
    i2c->OAR1 |= STM32_SADDR << (I2C_OAR1_OA1_Pos + 1);
    i2c->OAR1 |= I2C_OAR1_OA1EN;

    i2c->CR1 |= I2C_CR1_SBC; // required to send multiple bytes from slave back

    i2c->CR1 |= I2C_CR1_PE; // enable i2c peripheral
}

/// This function operates first by reading a byte of how many bytes to transmit
/// back. After this, it waits for the restart condition and begins the
/// tranmission back to the master.
uint8_t I2C_Slave_Transmit(I2C_TypeDef *i2c, uint8_t *data)
{
    uint32_t temp, txSize = 0;
    while (!(i2c->ISR & I2C_ISR_ADDR))
    {
    }
    temp = i2c->CR1 & (I2C_ISR_ADDCODE | I2C_ISR_DIR);
    temp = temp;

    i2c->ICR |= I2C_ICR_ADDRCF;

    while (!(i2c->ISR & (I2C_ISR_RXNE)))
    {
    }

    /* First received byte is the number of bytes that will be transmitted back */
    txSize = i2c->RXDR;

    /* Wait for restart with saddr reception */
    while (!(i2c->ISR & I2C_ISR_ADDR))
    {
    }
    temp = i2c->CR1 & (I2C_ISR_ADDCODE | I2C_ISR_DIR);
    temp = temp;

    i2c->CR2 &= ~I2C_CR2_NBYTES;
    i2c->CR2 |= (txSize << I2C_CR2_NBYTES_Pos);

    i2c->ICR |= I2C_ICR_ADDRCF;

    for (uint8_t i = 0; i < txSize; i++)
    {
        while (!(i2c->ISR & (I2C_ISR_TXE)))
        {
        }

        i2c->TXDR = data[i];
    }

    return 0;
}

/// This functions operates by setting the device in a while loop waiting to
/// receive its slave address. After which, it will receive first a number of
/// bytes to expect to receive and the store the bytes into a receive buffer.
/// This receive buffer is returned.
uint8_t *I2C_Slave_Receive(I2C_TypeDef *i2c)
{
    static uint8_t rcvBuffer[255];
    uint32_t temp, rxSize = 0;
    while (!(i2c->ISR & I2C_ISR_ADDR))
    {
    }
    temp = i2c->CR1 & (I2C_ISR_ADDCODE | I2C_ISR_DIR);
    temp = temp;

    i2c->ICR |= I2C_ICR_ADDRCF;

    while (!(i2c->ISR & (I2C_ISR_RXNE)))
    {
    }

    /* First received byte is the number of bytes that will follow */
    rxSize = i2c->RXDR;

    for (uint8_t i = 0; i < rxSize; i++)
    {
        while (!(i2c->ISR & (I2C_ISR_RXNE)))
        {
        }

        rcvBuffer[i] = i2c->RXDR;
    }

    return rcvBuffer;
}

/* EOF */