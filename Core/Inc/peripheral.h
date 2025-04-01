/*
 * sensor.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Vatsal
 */

#ifndef INC_PERIPHERAL_H_
#define INC_PERIPHERAL_H_

#include "main.h"

#define MAX_HANDLES 4

/*
 * Index through global transaction manager
 */
typedef enum
{
	I2C_1,
	I2C_2,
	SPI_1,
	SPI_2,
} handle_e;

/*
 * Common peripheral functionality
 */
typedef struct
{
	handle_e handle;

	uint32_t tx_buff_length;
	uint32_t rx_buff_length;
	uint8_t *tx_buff;
	uint8_t *rx_buff;

	uint8_t read;
	uint8_t ready;
} transaction_s;

/*
 * I2C peripheral
 */
typedef struct
{
	uint8_t address;
	transaction_s data;
	void (*i2c_cb)();

} i2c_per_s;

/*
 * SPI peripheral
 */
typedef struct
{
	uint16_t cs;
	GPIO_TypeDef *port;
	transaction_s data;
	void (*spi_cb)();

} spi_per_s;

/*
 * Peripheral Union
 */
typedef union
{
	i2c_per_s *i2c;
	spi_per_s *spi;
} peripheral_u;

/*
 * Return safety for handle pointer
 */
typedef union
{
	I2C_HandleTypeDef *i2c_handle;
	SPI_HandleTypeDef *spi_handle;
} handle_ptr_u;

extern peripheral_u transactions[MAX_HANDLES];

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;


/*
 * User must populate the SPI tx_buff, and rx_buff will be populated with the result
 */
HAL_StatusTypeDef rtx_spi(spi_per_s *spi_peripheral, uint8_t blocking);

/*
 * Callback called after interrupt transaction is complete
 */
HAL_StatusTypeDef rtx_spi_cb(handle_e spi_handle);

/*
 * User must populate the I2C tx_buff, and rx_buff will be populated with the result
 */
HAL_StatusTypeDef rtx_i2c(i2c_per_s *i2c_peripheral);

/*
 * Callback called after interrupt transaction is complete
 */
HAL_StatusTypeDef rtx_i2c_cb(handle_e i2c_handle);

#endif /* INC_PERIPHERAL_H_ */
