/*
 * peripheral.c
 *
 *  Created on: Mar 20, 2025
 *      Author: Vatsal
 */
#include "peripheral.h"

peripheral_u transactions[MAX_HANDLES];


/*
 * Get handle
 */

handle_ptr_u get_transaction_handle(handle_e handle)
{
	handle_ptr_u ret = {0};

	switch (handle){
		case I2C_1:
			ret.i2c_handle = &hi2c1;
			break;
		case I2C_2:
			ret.i2c_handle = &hi2c2;
			break;
		case SPI_1:
			ret.spi_handle = &hspi1;
			break;
		case SPI_2:
			ret.spi_handle = &hspi2;
			break;
	}

	return ret;
}

/*
 * User must populate the SPI tx_buff, and rx_buff will be populated with the result
 */
HAL_StatusTypeDef rtx_spi(spi_per_s *spi_peripheral, uint8_t blocking)
{
	if(spi_peripheral->data.rx_buff == NULL
			&& spi_peripheral->data.tx_buff == NULL)
		return HAL_ERROR;

	if(spi_peripheral->data.tx_buff_length != spi_peripheral->data.rx_buff_length)
		return HAL_ERROR;

	if(transactions[spi_peripheral->data.handle].spi != NULL)
		return HAL_BUSY;

	SPI_HandleTypeDef *handle = get_transaction_handle(spi_peripheral->data.handle).spi_handle;

	if(handle == NULL)
		return HAL_ERROR;

	HAL_StatusTypeDef status;
	
	/*
	 * Enqueue ongoing transaction
	 */
	spi_peripheral->data.ready = FALSE;
	transactions[spi_peripheral->data.handle].spi = spi_peripheral;

	if(spi_peripheral->cs_ext_handle == FALSE)
		HAL_GPIO_WritePin(spi_peripheral->port, spi_peripheral->cs, GPIO_PIN_RESET);

	if(blocking == FALSE)
	{
		if(spi_peripheral->data.read == TRUE)
			status = HAL_SPI_TransmitReceive_DMA(handle,
					spi_peripheral->data.tx_buff,
					spi_peripheral->data.rx_buff,
					spi_peripheral->data.tx_buff_length);
		else
			status = HAL_SPI_Transmit_DMA(handle,
					spi_peripheral->data.tx_buff,
					spi_peripheral->data.tx_buff_length);
	}
	else
	{
		if(spi_peripheral->data.read == TRUE)
			status = HAL_SPI_TransmitReceive(handle,
					spi_peripheral->data.tx_buff,
					spi_peripheral->data.rx_buff,
					spi_peripheral->data.tx_buff_length,
					HAL_MAX_DELAY);
		else
			status = HAL_SPI_Transmit(handle,
					spi_peripheral->data.tx_buff,
					spi_peripheral->data.tx_buff_length,
					HAL_MAX_DELAY);
	}


	if(blocking == TRUE)
		rtx_spi_cb(spi_peripheral->data.handle);

	return status;
}

/*
 * Callback called after interrupt transaction is complete
 */
HAL_StatusTypeDef rtx_spi_cb(handle_e spi_handle)
{
	if(spi_handle != SPI_1 && spi_handle != SPI_2)
		return HAL_ERROR;

	if(transactions[spi_handle].spi->cs_ext_handle == FALSE)
		HAL_GPIO_WritePin(transactions[spi_handle].spi->port, transactions[spi_handle].spi->cs, GPIO_PIN_SET);

	// Assume transaction completed successfully and dequeue
	if(transactions[spi_handle].spi->spi_cb != NULL)
		transactions[spi_handle].spi->spi_cb();


	transactions[spi_handle].spi->data.ready = TRUE;
	transactions[spi_handle].spi = NULL;

	return HAL_OK;
}


/*
 * User must populate the I2C tx_buff, and rx_buff will be populated with the result
 */
HAL_StatusTypeDef rtx_i2c(i2c_per_s *i2c_peripheral)
{
	if(i2c_peripheral->data.rx_buff == NULL
			&& i2c_peripheral->data.tx_buff == NULL)
		return HAL_ERROR;


	if(transactions[i2c_peripheral->data.handle].i2c != NULL)
		return HAL_BUSY;

	I2C_HandleTypeDef *handle = get_transaction_handle(i2c_peripheral->data.handle).i2c_handle;

	if(handle == NULL)
		return HAL_ERROR;

	/*
	 * Enqueue ongoing transaction
	 */
	i2c_peripheral->data.ready = FALSE;
	transactions[i2c_peripheral->data.handle].i2c = i2c_peripheral;

	HAL_StatusTypeDef status;
 	status = HAL_I2C_Master_Transmit_IT(handle,
			i2c_peripheral->address << 1,
			i2c_peripheral->data.tx_buff,
			i2c_peripheral->data.tx_buff_length);

	return status;
}

/*
 * Callback called after interrupt transaction is complete
 */
HAL_StatusTypeDef rtx_i2c_cb(handle_e i2c_handle)
{
	if(i2c_handle != I2C_1 && i2c_handle != I2C_2)
		return HAL_ERROR;

	// Check if a read operation needs to be triggered
	if(transactions[i2c_handle].i2c->data.read == TRUE)
	{
		I2C_HandleTypeDef *handle = get_transaction_handle(i2c_handle).i2c_handle;

		if(handle == NULL)
			return HAL_ERROR;

		HAL_StatusTypeDef status;
		status = HAL_I2C_Master_Receive_IT(handle,
				transactions[i2c_handle].i2c->address << 1,
				transactions[i2c_handle].i2c->data.rx_buff,
				transactions[i2c_handle].i2c->data.rx_buff_length);

		// No need to read on second callback
		transactions[i2c_handle].i2c->data.read = FALSE;

		return status;
	}

	if(transactions[i2c_handle].i2c->i2c_cb != NULL)
		transactions[i2c_handle].i2c->i2c_cb();

	transactions[i2c_handle].i2c->data.ready = TRUE;
	transactions[i2c_handle].i2c = NULL;

	return HAL_OK;
}
