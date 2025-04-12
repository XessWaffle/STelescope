#ifndef DS3231_REGISTERS_H
#define DS3231_REGISTERS_H

// DS3231 I2C Slave Address (7-bit)
#define DS3231_I2C_ADDRESS 0x68

// Register Addresses
#define DS3231_REG_SECONDS      0x00
#define DS3231_REG_MINUTES      0x01
#define DS3231_REG_HOURS        0x02
#define DS3231_REG_DAY          0x03
#define DS3231_REG_DATE         0x04
#define DS3231_REG_MONTH        0x05
#define DS3231_REG_YEAR         0x06
#define DS3231_REG_ALARM1_SEC   0x07
#define DS3231_REG_ALARM1_MIN   0x08
#define DS3231_REG_ALARM1_HR    0x09
#define DS3231_REG_ALARM1_DAY   0x0A
#define DS3231_REG_ALARM2_MIN   0x0B
#define DS3231_REG_ALARM2_HR    0x0C
#define DS3231_REG_ALARM2_DAY   0x0D
#define DS3231_REG_CONTROL      0x0E
#define DS3231_REG_STATUS       0x0F
#define DS3231_REG_AGING        0x10
#define DS3231_REG_TEMP_MSB     0x11
#define DS3231_REG_TEMP_LSB     0x12

// Bit Definitions for Each Register

// Seconds Register (0x00)
// Bits 0-6: Seconds (0-59, BCD), Bit 7: Reserved
#define DS3231_SECONDS_MASK     0x7F

// Minutes Register (0x01)
// Bits 0-6: Minutes (0-59, BCD), Bit 7: Reserved
#define DS3231_MINUTES_MASK     0x7F

// Hours Register (0x02)
// In 24-hour mode:
//   Bits 0-5: Hours (0-23, BCD), Bits 6-7: Reserved
// In 12-hour mode:
//   Bits 0-4: Hours (1-12, BCD), Bit 5: AM/PM (0=AM, 1=PM), Bit 6: 12/24 mode
#define DS3231_HOURS_12HR_MODE  (1 << 6) // 1 = 12-hour, 0 = 24-hour
#define DS3231_HOURS_AMPM       (1 << 5) // 1 = PM, 0 = AM (12-hour mode)
#define DS3231_HOURS_24HR_MASK  0x3F     // Hours mask for 24-hour mode
#define DS3231_HOURS_12HR_MASK  0x1F     // Hours mask for 12-hour mode

// Day Register (0x03)
// Bits 0-2: Day of week (1-7), Bits 3-7: Reserved
#define DS3231_DAY_MASK         0x07

// Date Register (0x04)
// Bits 0-5: Date (1-31, BCD), Bits 6-7: Reserved
#define DS3231_DATE_MASK        0x3F

// Month Register (0x05)
// Bits 0-4: Month (1-12, BCD), Bit 7: Century, Bits 5-6: Reserved
#define DS3231_MONTH_CENTURY    (1 << 7)
#define DS3231_MONTH_MASK       0x1F

// Year Register (0x06)
// Bits 0-7: Year (0-99, BCD)
#define DS3231_YEAR_MASK        0xFF

// Alarm 1 Seconds Register (0x07)
// Bits 0-6: Alarm 1 Seconds (0-59, BCD), Bit 7: A1M1 (Alarm 1 Mask Bit 1)
#define DS3231_ALARM1_SEC_A1M1  (1 << 7)
#define DS3231_ALARM1_SEC_MASK  0x7F

// Alarm 1 Minutes Register (0x08)
// Bits 0-6: Alarm 1 Minutes (0-59, BCD), Bit 7: A1M2 (Alarm 1 Mask Bit 2)
#define DS3231_ALARM1_MIN_A1M2  (1 << 7)
#define DS3231_ALARM1_MIN_MASK  0x7F

// Alarm 1 Hours Register (0x09)
// Bit 7: A1M3 (Alarm 1 Mask Bit 3)
// In 24-hour mode:
//   Bits 0-5: Hours (0-23, BCD), Bit 6: Reserved
// In 12-hour mode:
//   Bits 0-4: Hours (1-12, BCD), Bit 5: AM/PM, Bit 6: 12/24 mode
#define DS3231_ALARM1_HR_A1M3   (1 << 7)
#define DS3231_ALARM1_HR_12HR   (1 << 6)
#define DS3231_ALARM1_HR_AMPM   (1 << 5)
#define DS3231_ALARM1_HR_24HR_MASK 0x3F
#define DS3231_ALARM1_HR_12HR_MASK 0x1F

// Alarm 1 Day/Date Register (0x0A)
// Bit 7: A1M4 (Alarm 1 Mask Bit 4), Bit 6: DY/DT (0=Day, 1=Date)
// When DY/DT = 0: Bits 0-2: Day (1-7)
// When DY/DT = 1: Bits 0-5: Date (1-31, BCD)
#define DS3231_ALARM1_DAY_A1M4  (1 << 7)
#define DS3231_ALARM1_DAY_DYDT  (1 << 6) // 1 = Date, 0 = Day
#define DS3231_ALARM1_DAY_DAY_MASK 0x07
#define DS3231_ALARM1_DAY_DATE_MASK 0x3F

// Alarm 2 Minutes Register (0x0B)
// Bits 0-6: Alarm 2 Minutes (0-59, BCD), Bit 7: A2M2 (Alarm 2 Mask Bit 2)
#define DS3231_ALARM2_MIN_A2M2  (1 << 7)
#define DS3231_ALARM2_MIN_MASK  0x7F

// Alarm 2 Hours Register (0x0C)
// Bit 7: A2M3 (Alarm 2 Mask Bit 3)
// In 24-hour mode:
//   Bits 0-5: Hours (0-23, BCD), Bit 6: Reserved
// In 12-hour mode:
//   Bits 0-4: Hours (1-12, BCD), Bit 5: AM/PM, Bit 6: 12/24 mode
#define DS3231_ALARM2_HR_A2M3   (1 << 7)
#define DS3231_ALARM2_HR_12HR   (1 << 6)
#define DS3231_ALARM2_HR_AMPM   (1 << 5)
#define DS3231_ALARM2_HR_24HR_MASK 0x3F
#define DS3231_ALARM2_HR_12HR_MASK 0x1F

// Alarm 2 Day/Date Register (0x0D)
// Bit 7: A2M4 (Alarm 2 Mask Bit 4), Bit 6: DY/DT (0=Day, 1=Date)
// When DY/DT = 0: Bits 0-2: Day (1-7)
// When DY/DT = 1: Bits 0-5: Date (1-31, BCD)
#define DS3231_ALARM2_DAY_A2M4  (1 << 7)
#define DS3231_ALARM2_DAY_DYDT  (1 << 6) // 1 = Date, 0 = Day
#define DS3231_ALARM2_DAY_DAY_MASK 0x07
#define DS3231_ALARM2_DAY_DATE_MASK 0x3F

// Control Register (0x0E)
#define DS3231_CONTROL_EOSC     (1 << 7) // Enable Oscillator (0 = enabled, 1 = disabled)
#define DS3231_CONTROL_BBSQW    (1 << 6) // Battery-Backed Square-Wave Enable
#define DS3231_CONTROL_CONV     (1 << 5) // Convert Temperature
#define DS3231_CONTROL_RS2      (1 << 4) // Rate Select 2
#define DS3231_CONTROL_RS1      (1 << 3) // Rate Select 1
#define DS3231_CONTROL_INTCN    (1 << 2) // Interrupt Control (0 = SQW, 1 = Alarm)
#define DS3231_CONTROL_A2IE     (1 << 1) // Alarm 2 Interrupt Enable
#define DS3231_CONTROL_A1IE     (1 << 0) // Alarm 1 Interrupt Enable
// Square-wave frequency (RS2, RS1):
#define DS3231_SQW_1HZ          (0 << 4 | 0 << 3)
#define DS3231_SQW_1024HZ       (0 << 4 | 1 << 3)
#define DS3231_SQW_4096HZ       (1 << 4 | 0 << 3)
#define DS3231_SQW_8192HZ       (1 << 4 | 1 << 3)

// Status Register (0x0F)
#define DS3231_STATUS_OSF       (1 << 7) // Oscillator Stop Flag
#define DS3231_STATUS_EN32KHZ   (1 << 3) // Enable 32kHz Output
#define DS3231_STATUS_BSY       (1 << 2) // Busy (temperature conversion)
#define DS3231_STATUS_A2F       (1 << 1) // Alarm 2 Flag
#define DS3231_STATUS_A1F       (1 << 0) // Alarm 1 Flag

// Aging Offset Register (0x10)
// Bits 0-7: Aging Offset (signed 8-bit value)
#define DS3231_AGING_MASK       0xFF

// Temperature Registers (0x11, 0x12)
// Temp MSB (0x11): Bits 0-7: Integer part (signed)
// Temp LSB (0x12): Bits 6-7: Fractional part (0.25°C steps), Bits 0-5: Reserved
#define DS3231_TEMP_MSB_MASK    0xFF
#define DS3231_TEMP_LSB_MASK    0xC0
#define DS3231_TEMP_LSB_SHIFT   6

#endif // DS3231_REGISTERS_H