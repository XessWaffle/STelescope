/*
 * gc9a01a.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Vatsal
 *
 *  Macros generated via AI
 */

#ifndef INC_GC9A01A_H_
#define INC_GC9A01A_H_


// GC9A01A Command Definitions
#define GC9A01A_SWRESET 0x01   ///< Software Reset (maybe, not documented)
#define GC9A01A_RDDID 0x04     ///< Read display identification information
#define GC9A01A_RDDST 0x09     ///< Read Display Status
#define GC9A01A_SLPIN 0x10     ///< Enter Sleep Mode
#define GC9A01A_SLPOUT 0x11    ///< Sleep Out
#define GC9A01A_PTLON 0x12     ///< Partial Mode ON
#define GC9A01A_NORON 0x13     ///< Normal Display Mode ON
#define GC9A01A_INVOFF 0x20    ///< Display Inversion OFF
#define GC9A01A_INVON 0x21     ///< Display Inversion ON
#define GC9A01A_DISPOFF 0x28   ///< Display OFF
#define GC9A01A_DISPON 0x29    ///< Display ON
#define GC9A01A_CASET 0x2A     ///< Column Address Set
#define GC9A01A_RASET 0x2B     ///< Row Address Set
#define GC9A01A_RAMWR 0x2C     ///< Memory Write
#define GC9A01A_PTLAR 0x30     ///< Partial Area
#define GC9A01A_VSCRDEF 0x33   ///< Vertical Scrolling Definition
#define GC9A01A_TEOFF 0x34     ///< Tearing Effect Line OFF
#define GC9A01A_TEON 0x35      ///< Tearing Effect Line ON
#define GC9A01A_MADCTL 0x36    ///< Memory Access Control
#define GC9A01A_VSCRSADD 0x37  ///< Vertical Scrolling Start Address
#define GC9A01A_IDLEOFF 0x38   ///< Idle mode OFF
#define GC9A01A_IDLEON 0x39    ///< Idle mode ON
#define GC9A01A_COLMOD 0x3A    ///< Pixel Format Set
#define GC9A01A_CONTINUE 0x3C  ///< Write Memory Continue
#define GC9A01A_TEARSET 0x44   ///< Set Tear Scanline
#define GC9A01A_GETLINE 0x45   ///< Get Scanline
#define GC9A01A_SETBRIGHT 0x51 ///< Write Display Brightness
#define GC9A01A_SETCTRL 0x53   ///< Write CTRL Display
#define GC9A01A1_POWER7 0xA7   ///< Power Control 7
#define GC9A01A_TEWC 0xBA      ///< Tearing effect width control
#define GC9A01A1_POWER1 0xC1   ///< Power Control 1
#define GC9A01A1_POWER2 0xC3   ///< Power Control 2
#define GC9A01A1_POWER3 0xC4   ///< Power Control 3
#define GC9A01A1_POWER4 0xC9   ///< Power Control 4
#define GC9A01A_RDID1 0xDA     ///< Read ID 1
#define GC9A01A_RDID2 0xDB     ///< Read ID 2
#define GC9A01A_RDID3 0xDC     ///< Read ID 3
#define GC9A01A_FRAMERATE 0xE8 ///< Frame rate control
#define GC9A01A_SPI2DATA 0xE9  ///< SPI 2DATA control
#define GC9A01A_INREGEN2 0xEF  ///< Inter register enable 2
#define GC9A01A_GAMMA1 0xF0    ///< Set gamma 1
#define GC9A01A_GAMMA2 0xF1    ///< Set gamma 2
#define GC9A01A_GAMMA3 0xF2    ///< Set gamma 3
#define GC9A01A_GAMMA4 0xF3    ///< Set gamma 4
#define GC9A01A_IFACE 0xF6     ///< Interface control
#define GC9A01A_INREGEN1 0xFE  ///< Inter register enable 1

// MADCTL Bit Definitions (Memory Access Control)
#define GC9A01A_MADCTL_MY   0x80  // Row address order (1 = bottom to top)
#define GC9A01A_MADCTL_MX   0x40  // Column address order (1 = right to left)
#define GC9A01A_MADCTL_MV   0x20  // Row/Column exchange (1 = swapped)
#define GC9A01A_MADCTL_ML   0x10  // Vertical refresh order (1 = bottom to top)
#define GC9A01A_MADCTL_RGB  0x00  // RGB color order
#define GC9A01A_MADCTL_BGR  0x08  // BGR color order
#define GC9A01A_MADCTL_MH   0x04  // Horizontal refresh order (1 = right to left)

// Display Dimensions
#define GC9A01A_WIDTH       240
#define GC9A01A_HEIGHT      240

// Color Definitions (RGB565)
#define GC9A01A_COLOR_BLACK   0x0000
#define GC9A01A_COLOR_WHITE   0xFFFF
#define GC9A01A_COLOR_RED     0xF800
#define GC9A01A_COLOR_GREEN   0x07E0
#define GC9A01A_COLOR_BLUE    0x001F


#endif /* INC_GC9A01A_H_ */
