# STM32CubeIDE Source File
> This is the API of sensors, sdcards, activators and other objects.


## Temperature Sensor
* auther: Yueh Feng Tsai
* email: frank232026407@gmail.com
* [Datasheet](./datasheet/Temperature.pdf)
* Manufacture number: 2521020222501

---
## Pressure Sensor
* Auther: Zhi-Kai Xu
* Email: zkxu.ii12@nycu.edu.tw
* Sensor name: WSEN-PADS Absolute Pressure Sensor
* Order code: 2511020213301
* Datasheet: https://www.we-online.com/components/products/datasheet/2511020213301.pdf
* User Manual: https://www.we-online.com/components/products/manual/2511020213301_WSEN-PADS%202511020213301%20Manual_rev2.2.pdf
* Available interface: *I2C*
* TODO:
    - Single conversion mode
    - Enable low-noise configuration
    - Enable low pass filter
    - FIFO
    - Interrupt

---
## Accelerator
* Auther: Zhi-Kai Xu
* Email: zkxu.ii12@nycu.edu.tw
* Sensor name: WSEN-ITDS 3 Axis Acceleration Sensor
* Order code: 2533020201601
* Datasheet: https://www.we-online.com/components/products/datasheet/2533020201601.pdf
* User Manual: https://www.we-online.com/components/products/manual/2533020201601_WSEN-ITDS%202533020201601%20Manual_rev2.3.pdf
* Available interface: *I2C*
* TODO:
    - Single data conversion mode
    - Config filter path
    - Config filter BW
    - FIFO
    - Interrupt


---
## 6-axis IMU
* Auther: Zhi-Kai Xu
* Email: zkxu.ii12@nycu.edu.tw
* Sensor name: Absolute Pressure Sensor WSEN-PADS
* Order code: 2511020213301
* Datasheet: https://www.we-online.com/components/products/datasheet/2511020213301.pdf
* User Manual: https://www.we-online.com/components/products/manual/2511020213301_WSEN-PADS%202511020213301%20Manual_rev2.2.pdf
* Available interface: *I2C* and *SPI*
* TODO:
    - Config filters of accelerator and gyroscope
    - FIFO
    - Interrupt