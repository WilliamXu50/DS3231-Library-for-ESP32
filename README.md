# DS3231-Library-for-ESP32
Simple library for interfacing DS3231 RTC to ESP32

# Description
This is a simple library containing functions to interface a DS3231 real-time clock module to ESP32. Right now, it can perform reading to and writing from DS3231 registers using I2C protocol. The DS3231 also contains a temperature register, but I probably won't touch it much since I'm only focused on the time and alarms at the moment.

# Functionality So Far:
  - Read/write to DS3231 registers (individual register or sequential read/write)
  - Set time (secs, mins, hrs) and calendar (day of the week, day of the month, month, year)
  - Read time and calendar

# Todos:
  - Write functions to set alarm
  - Write function to output square waves
