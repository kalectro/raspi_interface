/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

//\Author Kai Franke

#ifndef RASPI_INTERFACE_H_
#define RASPI_INTERFACE_H_

#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <bosch_drivers_hardware_interface.hpp>
#include <bosch_drivers_parameters.hpp>

#define MAX_SPI_CHANNELS 2
#define MIN_SPI_FREQUENCY 5e5
#define MAX_SPI_FREQUENCY 32e6

using namespace bosch_drivers_common;

class RaspiInterface: public bosch_hardware_interface
{
  /**
   * \brief The Raspberry Pi hardware interface.
   * This code is meant to run on a raspberry pi with the wiringPi package installed
   * Get the wiringPi source code from http://wiringpi.com/
   */

public:
 
  RaspiInterface();

  ~RaspiInterface();

 
  bool initialize();

  /**
   * \brief Reads \a num_bytes from the requested device on the specified \a protocol at the specified protocol \a frequency
   * \var   int device_address the way that the sensor identifies itself
   * \var   interface_protocol protocol the defined protocol
   * \var   int frequency the frequency of the interface protocol
   * \var   int* flags additional information necessary to read from that particular interface protocol.
   * \var   uint8_t reg_address the starting address of the data in the sensor.
   * \var   uint8_t* data the name of the array where the data will be stored.
   * \var   uint8_t num_bytes the number of bytes to be read from the sensor.
   * \return \a num_bytes or a value less than zero, if the read failed.
   */
  ssize_t read( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes );
  
  /**
   * \brief Writes \a num_bytes from the requested device on the specified \a protocol at the specified protocol \a frequency
   * \var   int device_address the way that the sensor itentifies itself
   * \var   interface_protocol protocol the defined protocol
   * \var   int frequency the frequency of the interface protocol
   * \var   int* flags additional information necessary to write to that particular interface protocol.
   * \var   uint8_t reg_address the starting address in the sensor's registers where the data will be written to.
   * \var   uint8_t* data the name of the array where the data will be output from.
   * \var   uint8_t num_bytes the number of bytes to be written to the sensor.
   * \return \a num_bytes or a value less than zero, if the write failed.
   */
  ssize_t write( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes );

  /**
   * \brief  Returns true if the input protocol is supported by the hardware interface.
   * \param  interface_protocol protocol the input protocol.
   * \return true, if the hardware interface supports reading and writing on that particular protocol.
   */
  bool supportedProtocol( interface_protocol protocol );
  
  /**
   * \return  the way the sensor identifies itself
   */
  std::string getID();

private:

  ssize_t raspiGpioWrite( uint8_t pin, bool value );
  ssize_t raspiGpioRead( uint8_t flags, uint8_t pin, uint8_t* value );
  ssize_t raspiSpiRead( int frequency, uint8_t flags, uint8_t* data, size_t num_bytes );
  
  // is set to false when object is created and set to true after initialize() was successful
  bool is_initialized_;
  
  // save which SPI channels (CS lines) are currently in used
  bool use_spi[MAX_SPI_CHANNELS];
  
};
#endif //RASPI_INTERFACE_H_

