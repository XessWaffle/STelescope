/*
 * camera.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Vatsal
 */

#include "main.h"
#include "camera.h"
#include "display.h"

const sensor_reg_s qvga_init[CAM_QVGA_INIT_LENGTH] = 
{
	{0xff, 0x0}, 
	{0x2c, 0xff}, 
	{0x2e, 0xdf}, 
	{0xff, 0x1}, 
	{0x3c, 0x32}, 
	{0x11, 0x0}, 
	{0x9, 0x2}, 
	{0x4, 0xa8}, 
	{0x13, 0xe5}, 
	{0x14, 0x48}, 
	{0x2c, 0xc}, 
	{0x33, 0x78}, 
	{0x3a, 0x33}, 
	{0x3b, 0xfb}, 
	{0x3e, 0x0}, 
	{0x43, 0x11}, 
	{0x16, 0x10}, 
	{0x39, 0x2}, 
	{0x35, 0x88}, 

	{0x22, 0xa}, 
	{0x37, 0x40}, 
	{0x23, 0x0}, 
	{0x34, 0xa0}, 
	{0x6, 0x2}, 
	{0x6, 0x88}, 
	{0x7, 0xc0}, 
	{0xd, 0xb7}, 
	{0xe, 0x1}, 
	{0x4c, 0x0}, 
	{0x4a, 0x81}, 
	{0x21, 0x99}, 
	{0x24, 0x40}, 
	{0x25, 0x38}, 
	{0x26, 0x82}, 
	{0x5c, 0x0}, 
	{0x63, 0x0}, 
	{0x46, 0x22}, 
	{0xc, 0x3a}, 
	{0x5d, 0x55}, 
	{0x5e, 0x7d}, 
	{0x5f, 0x7d}, 
	{0x60, 0x55}, 
	{0x61, 0x70}, 
	{0x62, 0x80}, 
	{0x7c, 0x5}, 
	{0x20, 0x80}, 
	{0x28, 0x30}, 
	{0x6c, 0x0}, 
	{0x6d, 0x80}, 
	{0x6e, 0x0}, 
	{0x70, 0x2}, 
	{0x71, 0x94}, 
	{0x73, 0xc1}, 
	{0x3d, 0x34}, 
	{0x12, 0x4}, 
	{0x5a, 0x57}, 
	{0x4f, 0xbb}, 
	{0x50, 0x9c}, 
	{0xff, 0x0}, 
	{0xe5, 0x7f}, 
	{0xf9, 0xc0}, 
	{0x41, 0x24}, 
	{0xe0, 0x14}, 
	{0x76, 0xff}, 
	{0x33, 0xa0}, 
	{0x42, 0x20}, 
	{0x43, 0x18}, 
	{0x4c, 0x0}, 
	{0x87, 0xd0}, 
	{0x88, 0x3f}, 
	{0xd7, 0x3}, 
	{0xd9, 0x10}, 
	{0xd3, 0x82}, 
	{0xc8, 0x8}, 
	{0xc9, 0x80}, 
	{0x7c, 0x0}, 
	{0x7d, 0x0}, 
	{0x7c, 0x3}, 
	{0x7d, 0x48}, 
	{0x7d, 0x48}, 
	{0x7c, 0x8}, 
	{0x7d, 0x20}, 
	{0x7d, 0x10}, 
	{0x7d, 0xe}, 
	{0x90, 0x0}, 
	{0x91, 0xe}, 
	{0x91, 0x1a}, 
	{0x91, 0x31}, 
	{0x91, 0x5a}, 
	{0x91, 0x69}, 
	{0x91, 0x75}, 
	{0x91, 0x7e}, 
	{0x91, 0x88}, 
	{0x91, 0x8f}, 
	{0x91, 0x96}, 
	{0x91, 0xa3}, 
	{0x91, 0xaf}, 
	{0x91, 0xc4}, 
	{0x91, 0xd7}, 
	{0x91, 0xe8}, 
	{0x91, 0x20}, 
	{0x92, 0x0}, 

	{0x93, 0x6}, 
	{0x93, 0xe3}, 
	{0x93, 0x3}, 
	{0x93, 0x3}, 
	{0x93, 0x0}, 
	{0x93, 0x2}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x93, 0x0}, 
	{0x96, 0x0}, 
	{0x97, 0x8}, 
	{0x97, 0x19}, 
	{0x97, 0x2}, 
	{0x97, 0xc}, 
	{0x97, 0x24}, 
	{0x97, 0x30}, 
	{0x97, 0x28}, 
	{0x97, 0x26}, 
	{0x97, 0x2}, 
	{0x97, 0x98}, 
	{0x97, 0x80}, 
	{0x97, 0x0}, 
	{0x97, 0x0}, 
	{0xa4, 0x0}, 
	{0xa8, 0x0}, 
	{0xc5, 0x11}, 
	{0xc6, 0x51}, 
	{0xbf, 0x80}, 
	{0xc7, 0x10}, 
	{0xb6, 0x66}, 
	{0xb8, 0xa5}, 
	{0xb7, 0x64}, 
	{0xb9, 0x7c}, 
	{0xb3, 0xaf}, 
	{0xb4, 0x97}, 
	{0xb5, 0xff}, 
	{0xb0, 0xc5}, 
	{0xb1, 0x94}, 
	{0xb2, 0xf}, 
	{0xc4, 0x5c}, 
	{0xa6, 0x0}, 
	{0xa7, 0x20}, 
	{0xa7, 0xd8}, 
	{0xa7, 0x1b}, 
	{0xa7, 0x31}, 
	{0xa7, 0x0}, 
	{0xa7, 0x18}, 
	{0xa7, 0x20}, 
	{0xa7, 0xd8}, 
	{0xa7, 0x19}, 
	{0xa7, 0x31}, 
	{0xa7, 0x0}, 
	{0xa7, 0x18}, 
	{0xa7, 0x20}, 
	{0xa7, 0xd8}, 
	{0xa7, 0x19}, 
	{0xa7, 0x31}, 
	{0xa7, 0x0}, 
	{0xa7, 0x18}, 
	{0x7f, 0x0}, 
	{0xe5, 0x1f}, 
	{0xe1, 0x77}, 
	{0xdd, 0x7f}, 
	{0xc2, 0xe}, 
	
	{0xff, 0x0}, 
	{0xe0, 0x4}, 
	{0xc0, 0xc8}, 
	{0xc1, 0x96}, 
	{0x86, 0x3d}, 
	{0x51, 0x90}, 
	{0x52, 0x2c}, 
	{0x53, 0x0}, 
	{0x54, 0x0}, 
	{0x55, 0x88}, 
	{0x57, 0x0}, 
	
	{0x50, 0x92}, 
	{0x5a, 0x50}, 
	{0x5b, 0x3c}, 
	{0x5c, 0x0}, 
	{0xd3, 0x4}, 
	{0xe0, 0x0}, 
	
	{0xff, 0x0}, 
	{0x5, 0x0}, 
	
	{0xda, 0x8}, 
	{0xd7, 0x3}, 
	{0xe0, 0x0}, 
	
	{0x5, 0x0}, 

	
	{0xff,0xff}
};

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

uint8_t is_camera_spi_data_ready()
{
    return camera.cam_spi.data.ready;
}

uint8_t *get_camera_rx_buffer()
{
    return camera.cam_spi.data.rx_buff;
}

uint8_t pipe_fifo(uint32_t buff_len)
{
    static uint16_t packet = 0;

    if(camera.fifo_length < buff_len)
    {
        /* Discard last few bytes of FIFO */
        reset_camera_tx_buffer();   
        clear_fifo();
        packet = 0;
        return FALSE;
    }
    
    // Discard dummy byte
    if(packet == 0)
    {
        discard_spi(1);
        prep_display();
    }

    set_camera_spi_read(TRUE, buff_len);
    rtx_spi(&(camera.cam_spi), TRUE);

    camera.fifo_length -= (buff_len);

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

    uint16_t reg_addr = 0;
    uint16_t reg_val = 0;
    const sensor_reg_s *next = qvga_init;
    while ((reg_addr != 0xff) | (reg_val != 0xff))
    {
      reg_addr = next->reg;
      reg_val = next->value;
      write_i2c_reg(reg_addr, reg_val);
      next++;
    }

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
