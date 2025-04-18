/*
 * camera.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Vatsal
 */

#include "main.h"
#include "camera.h"
#include "display.h"

#include "ov2640_init_regs.h"

camera_s camera;

uint8_t camera_rx_buff[CAMERA_BUFFER_LENGTH];
uint8_t camera_tx_buff[CAMERA_BUFFER_LENGTH];

uint8_t populate_camera_tx_buff(uint8_t reset, uint8_t byte)
{
	static int buff_pos = 0;

	if(reset == TRUE)
		buff_pos = 0;

	if(buff_pos >= CAMERA_BUFFER_LENGTH)
	{
		buff_pos = 0;
		return buff_pos;
	}

	camera_tx_buff[buff_pos++] = byte;
	return buff_pos;
}

void set_camera_spi_read(uint8_t read, uint16_t buffer_length)
{
	camera.cam_spi.data.read = read;
	camera.cam_spi.data.tx_buff_length = buffer_length;
	camera.cam_spi.data.rx_buff_length = buffer_length;
}


void set_camera_i2c_read(uint8_t read, uint16_t rx_buffer_length, uint16_t tx_buffer_length)
{
	camera.cam_i2c.data.read = read;
	camera.cam_i2c.data.tx_buff_length = tx_buffer_length;
	camera.cam_i2c.data.rx_buff_length = rx_buffer_length;
}

void write_spi(uint8_t byte)
{
    uint16_t length = populate_camera_tx_buff(TRUE, byte);
    set_camera_spi_read(FALSE, length);
    rtx_spi(&(camera.cam_spi), TRUE);
}

void discard_spi(uint16_t length)
{
    set_camera_spi_read(FALSE, length);
    rtx_spi(&(camera.cam_spi), TRUE);
}

void write_spi_reg(uint8_t reg_addr, uint8_t reg_val)
{
    populate_camera_tx_buff(TRUE, ARDUCAM_SPI_WRITE_REG(reg_addr));
    uint16_t length = populate_camera_tx_buff(FALSE, reg_val);
    set_camera_spi_read(FALSE, length);
    rtx_spi(&(camera.cam_spi), TRUE);
}

uint8_t read_spi_reg(uint8_t reg_addr)
{
    populate_camera_tx_buff(TRUE, ARDUCAM_SPI_READ_REG(reg_addr));
    uint16_t length = populate_camera_tx_buff(FALSE, 0x00);
    set_camera_spi_read(TRUE, length);
    rtx_spi(&(camera.cam_spi), TRUE);
    return camera_rx_buff[1];
}

void write_i2c_reg(uint8_t reg_addr, uint8_t reg_val)
{
    populate_camera_tx_buff(TRUE, reg_addr);
    uint16_t length = populate_camera_tx_buff(FALSE, reg_val);
    set_camera_i2c_read(FALSE, 0, length);
    while(rtx_i2c(&(camera.cam_i2c)) == HAL_BUSY);
}

void write_i2c_regs(const sensor_reg_s *next)
{
    uint16_t reg_addr = 0;
    uint16_t reg_val = 0;
    while ((reg_addr != 0xff) | (reg_val != 0xff))
    {
      reg_addr = next->reg;
      reg_val = next->value;
      write_i2c_reg(reg_addr, reg_val);
      next++;
    }
}

uint8_t read_i2c_reg(uint8_t reg_addr)
{
    populate_camera_tx_buff(TRUE, reg_addr);
    set_camera_i2c_read(TRUE, 1, 1);
    while(rtx_i2c(&(camera.cam_i2c)) == HAL_BUSY);
    HAL_Delay(100);
    return camera_rx_buff[0];
}

uint8_t cam_cs_low()
{
    if(camera.cam_spi.cs_ext_handle == TRUE)
        return FALSE;

    HAL_GPIO_WritePin(camera.cam_spi.port, camera.cam_spi.cs, GPIO_PIN_RESET);
    camera.cam_spi.cs_ext_handle = TRUE;

    return TRUE;
}

uint8_t cam_cs_high()
{
    if(camera.cam_spi.cs_ext_handle == FALSE)
        return FALSE;

    HAL_GPIO_WritePin(camera.cam_spi.port, camera.cam_spi.cs, GPIO_PIN_SET);
    camera.cam_spi.cs_ext_handle = FALSE;

    return TRUE;
}

void clear_fifo()
{
    write_spi_reg(ARDUCAM_REG_FIFO_CTRL, ARDUCAM_SPI_FIFO_CLEAR);
}

void start_capture()
{
    write_spi_reg(ARDUCAM_REG_FIFO_CTRL, ARDUCAM_SPI_FIFO_START);
}


uint8_t is_capture_done()
{
    uint8_t trig_reg = read_spi_reg(ARDUCHIP_TRIG);

    if((trig_reg & CAP_DONE_MASK) == 0x00)
        return FALSE;

    return TRUE;
}

uint32_t read_fifo_length()
{
    uint8_t length1 = read_spi_reg(ARDUCAM_REG_FIFO_SIZE1);
    uint8_t length2 = read_spi_reg(ARDUCAM_REG_FIFO_SIZE2);
    uint8_t length3 = read_spi_reg(ARDUCAM_REG_FIFO_SIZE3);
    
    return ((uint32_t)(length3 & 0x7F) << 16) | ((uint32_t)(length2 & 0xFF) << 8) | (uint32_t)(length1 & 0xFF);
}

void reset_camera_tx_buffer()
{
    for(int i = 0; i < CAMERA_BUFFER_LENGTH; i++)
        camera_tx_buff[i] = 0x00;
}

void set_fifo_burst_read()
{
    write_spi(ARDUCAM_SPI_BURST_READ);
    write_spi(0xFF);
    reset_camera_tx_buffer();
}

uint8_t is_camera_spi_data_ready()
{
    return camera.cam_spi.data.ready;
}

uint8_t *get_camera_rx_buffer()
{
    return camera.cam_spi.data.rx_buff;
}

uint16_t get_camera_read_fifo_length()
{
    return camera.last_fifo_read_length;
}

uint8_t pipe_fifo(uint32_t buff_len)
{
    static uint16_t packet = 0;

    if(camera.fifo_length < buff_len)
    {
        set_camera_spi_read(TRUE, camera.fifo_length);
        rtx_spi(&(camera.cam_spi), TRUE);
        
        camera.last_fifo_read_length = camera.fifo_length;
        camera.fifo_length = 0;

        reset_camera_tx_buffer();   
        clear_fifo();

        packet = 0;
        return FALSE;
    }
    
    // Discard dummy byte
    if(packet == 0)
        discard_spi(1);

    set_camera_spi_read(TRUE, buff_len);
    rtx_spi(&(camera.cam_spi), TRUE);

    camera.fifo_length -= (buff_len);
    camera.last_fifo_read_length = buff_len;

    packet++;

    return TRUE;
}


uint8_t init_camera()
{
    /*
     * Initialize SPI
     */
    camera.cam_spi.cs = CAM_CS_Pin;
	camera.cam_spi.port = CAM_CS_GPIO_Port;
    camera.cam_spi.spi_cb = NULL;

	camera.cam_spi.data.handle = SPI_1;
	camera.cam_spi.data.tx_buff = camera_tx_buff;
	camera.cam_spi.data.rx_buff = camera_rx_buff;
    
    /*
     * Initialize I2C
     */
    camera.cam_i2c.address = OV2640_I2C_ADDR;
    camera.cam_i2c.i2c_cb = NULL;

    camera.cam_i2c.data.handle = I2C_2;
    camera.cam_i2c.data.tx_buff = camera_tx_buff;
    camera.cam_i2c.data.rx_buff = camera_rx_buff;

    HAL_GPIO_WritePin(camera.cam_spi.port, camera.cam_spi.cs, GPIO_PIN_SET);
    
    write_spi_reg(0x07, 0x80);
    HAL_Delay(100);

    read_spi_reg(0x07);

    write_spi_reg(ARDUCHIP_TEST1, 0x55);
    HAL_Delay(100);

    uint8_t val = read_spi_reg(ARDUCHIP_TEST1);

    /*
     * Expected response
     */
    if(val != 0x55)
        return FALSE;

    /*
     * Validate that I2C communication is established
     */
    write_i2c_reg(OV2640_REG_SENSOR_RESET, 0x01);

    uint8_t pidh = read_i2c_reg(OV2640_REG_PIDH), pidl = read_i2c_reg(OV2640_REG_PIDL);
    /*
     * Expected response
     */
    if(pidh != 0x26 || pidl != 0x42)
        return FALSE;

    /*
     * Start camera initialization for QVGA mode
     */
    write_i2c_reg(OV2640_REG_COM7, OV2640_RESET_ALL);


    /* Initialize QVGA */
    //write_i2c_regs(OV2640_QVGA);

    /* Initialize JPEG*/
    write_i2c_regs(OV2640_JPEG_INIT);
    write_i2c_regs(OV2640_YUV422);
    write_i2c_regs(OV2640_JPEG);
    write_i2c_reg(OV2640_REG_SENSOR_RESET, 0x01);
    write_i2c_reg(OV2640_REG_COM10, 0x00);
    write_i2c_regs(OV2640_1024x768_JPEG);

    return TRUE;
}

uint8_t capture_and_pipe(uint8_t new_capture, uint8_t *buff, uint32_t buff_len)
{
    static capture_step_e step_id = 0;
    
    if(new_capture == TRUE)
        step_id = 0;

    switch (step_id)
    {
        case CAPTURE_PREP:
        {            
            clear_fifo();
            start_capture();
            step_id = CAPTURE_IN_PROGRESS;
            break;
        }
        case CAPTURE_IN_PROGRESS:
        {
            if(is_capture_done() == TRUE)
                step_id = CAPTURE_METADATA;
            break;
        }
        case CAPTURE_METADATA:
        {
            camera.fifo_length = read_fifo_length();
            if(cam_cs_low() == FALSE)
                Error_Handler();
            set_fifo_burst_read();
            step_id = CAPTURE_DATA;
            
            if(buff != NULL)
                camera.cam_spi.data.rx_buff = buff;
            break;
        }
        case CAPTURE_DATA:
        {
            if(pipe_fifo(buff_len) == TRUE)
                break;
            step_id = CAPTURE_END;
            break;
        }
        case CAPTURE_END:
        {
            if(cam_cs_high() == FALSE)
                Error_Handler();
            step_id = CAPTURE_INVALID;
            camera.cam_spi.data.rx_buff = camera_rx_buff;
            break;
        }
        default:
            break;
    }

    return step_id;
}

#if 0
uint8_t buffer_fifo()
{
    static uint16_t row = 0;

    if(camera.fifo_length <= CAMERA_BUFFER_LENGTH || row >= 240)
    {
        /* Discard last few bytes of FIFO */
        reset_camera_tx_buffer();   
        clear_fifo();
        row = 0;
        return FALSE;
    }

    /*static uint8_t display_buff[CAMERA_BUFFER_LENGTH];

    uint16_t k = 0;

    for(uint16_t j = 0; j < 240; j++)
    {
        uint8_t vl = read_spi_reg(ARDUCAM_SPI_SINGLE_READ);
        uint8_t vh = read_spi_reg(ARDUCAM_SPI_SINGLE_READ);

        display_buff[2 * (j % 80 + k * 80)] = vl;
        display_buff[2 * (j % 80 + k * 80) + 1] = vh;
        k++;
        k = k % 3;

        if(k >= CAMERA_BUFFER_LENGTH)
            break;

    }

    write_display_row(row, display_buff, CAMERA_BUFFER_LENGTH);*/
    
    // Discard dummy byte
    if(row == 0)
    {
        discard_spi(1);
        prep_display();
    }

    uint8_t *curr_buff = camera.cam_spi.data.rx_buff;

    for(uint16_t i = 0; i < CAM_ROWS_CAPTURED; i++)
    {        
        discard_spi(CAMERA_DISCARD_BUFFER_LENGTH);

        camera.cam_spi.data.rx_buff = curr_buff + i * CAMERA_ROW_LENGTH;
        set_camera_spi_read(TRUE, CAMERA_ROW_LENGTH);
        rtx_spi(&(camera.cam_spi), TRUE);

        discard_spi(CAMERA_DISCARD_BUFFER_LENGTH);
    }

    camera.cam_spi.data.rx_buff = curr_buff;
    write_display_row(curr_buff, CAMERA_BUFFER_LENGTH);

    row += CAM_ROWS_CAPTURED;

    camera.fifo_length -= (CAMERA_BUFFER_LENGTH);

    return TRUE;
}

uint8_t capture_and_display(uint8_t new_capture)
{
    static capture_step_e step_id = 0;
    
    if(new_capture == TRUE)
        step_id = 0;

    switch (step_id)
    {
        case CAPTURE_PREP:
        {
            clear_fifo();
            start_capture();
            step_id = CAPTURE_IN_PROGRESS;
            break;
        }
        case CAPTURE_IN_PROGRESS:
        {
            if(is_capture_done() == TRUE)
                step_id = CAPTURE_METADATA;
            break;
        }
        case CAPTURE_METADATA:
        {
            camera.fifo_length = read_fifo_length();
            if(cam_cs_low() == FALSE)
                Error_Handler();
            set_fifo_burst_read();
            step_id = CAPTURE_DATA;
            break;
        }
        case CAPTURE_DATA:
        {
            if(buffer_fifo() == TRUE)
                break;
            step_id = CAPTURE_END;
            break;
        }
        case CAPTURE_END:
        {
            if(cam_cs_high() == FALSE)
                Error_Handler();
            step_id = CAPTURE_INVALID;
            break;
        }
        default:
            break;
    }

    return step_id;
}

#endif
