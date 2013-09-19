raspi_interface
===============

Hardware interface for use of Raspberry Pi with ROS using bosch_drivers and wiringPi

This package is an additional hardware interface to work with the bosch_drivers architecture. It utilizes the awesome wiringPi library to access GPIOs, SPI, I2C and RS232. This package is still under heavy development.

#### current status:
* I2C - implemented and tested on BMC050
* GPIO - roughly tested
* SPI - very roughly tested
* RS232 - implemented but not working


Installation and Test
-----------

    git clone git://git.drogon.net/wiringPi
    cd wiringPi; ./build; cd ..
    svn co http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/bosch_drivers
    git clone https://github.com/kalectro/raspi_interface
    rosmake raspi_interface
    rosrun raspi_interface gpio_test
