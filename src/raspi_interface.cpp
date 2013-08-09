/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Robert Bosch LLC.
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

#include "raspi_interface/raspi_interface.hpp"

/**********************************************************************/
// Constructor
/**********************************************************************/
RaspiInterface::RaspiInterface():
  is_initialized_( false )
{
}


/**********************************************************************/
// Destructor
/**********************************************************************/
RaspiInterface::~RaspiInterface()
{
  // close all serial connections
  for (std::map<std::string,int>::iterator it=serial_devices_.begin(); it!=serial_devices_.end(); ++it)
  {
    serialClose(it->second);
  }
}


/**********************************************************************/
// Initialize:   
/**********************************************************************/
bool RaspiInterface::initialize()
{
  // Prevent Double-initialization!
  if( is_initialized_ == true )
  {
    ROS_INFO( "RaspiInterface::initialize(): WiringPi already initialized." );
    return true;
  }

  // Setup WiringPi
  int init_reply = wiringPiSetup ();
  
  // Set all SPI channels not to use by default
  for( ssize_t i = 0; i < MAX_SPI_CHANNELS; ++i )
  {
    use_spi[i] = false;
  }
  
  if( init_reply != 0 )
  {
    ROS_ERROR( "WiringPi not initialized properly. Returned %i", init_reply);
    return false;
  }

  is_initialized_ = true;
  ROS_INFO( "RaspiInterface::initialize(): WiringPi initialized." );
  return true;
}

/**********************************************************************/
// Read
/**********************************************************************/
ssize_t RaspiInterface::read( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{
  int error_code = 0;
  
  // Check initialization:
  if( is_initialized_ == false )
  {
    return -1;
  }
 
  switch( protocol )
  {
    case GPIO:
    {
      error_code = raspiGpioRead( (uint8_t)flags[0], reg_address, data );
      break;
    }
    case SPI:
    {
      error_code = raspiSpi( frequency, (uint8_t)flags[0], reg_address, data, num_bytes );
      break;
    }
    case RS232:
    {
      error_code = raspiRs232Read( frequency, flags[0], data, num_bytes );
    } break;
    default:
    {
      ROS_ERROR("Raspberry Pi does not support reading through this protocol.");
      return -1;
    }
  }
  return error_code;
}


/**********************************************************************/
// Write
/**********************************************************************/
ssize_t RaspiInterface::write( int device_address, interface_protocol protocol, int frequency, int* flags, uint8_t reg_address, uint8_t* data, size_t num_bytes )
{ 
  int error_code = 0;

  // Check initialization:
  if( is_initialized_ == false )
  {
    return -1;
  }
 
  switch( protocol )
  {
    case GPIO:
    {
      error_code = raspiGpioWrite( reg_address, (bool)data[0] );
      break;
    }
    case SPI:
    {
      error_code = raspiSpi( frequency, (uint8_t)flags[0], reg_address, data, num_bytes );
      break;
    }
    case RS232:
    {
      error_code = raspiRs232Write( frequency, data, num_bytes );
    } break;
    default:
    {
      ROS_ERROR( "Raspberry Pi does not support writing through this protocol." );
      error_code = -1;
    }
  }
 
  return error_code; // bytes written, or error. 
}


/**********************************************************************/
// supportedProtocol
/**********************************************************************/
bool RaspiInterface::supportedProtocol( interface_protocol protocol )
{
  switch( protocol )
  {
  case GPIO:
  case SPI:
  case RS232:
    return true;
  default:
    return false;
  }
}


/**********************************************************************/
std::string RaspiInterface::getID()
/**********************************************************************/
{
  return "Raspberry Pi";
}


/**********************************************************************/
/**********************************************************************/
ssize_t RaspiInterface::raspiGpioWrite( uint8_t pin, bool value )
{

  // check if selected pin is valid for Raspberry Pi
  if( pin > 16 )
  {
    ROS_ERROR("The selected Pin number is not available for GPIO");
    ROS_ERROR("Select Pins 0 through 16 instead");
    return -1;
  }
  
  pinMode (pin, OUTPUT);
  digitalWrite (pin, value);
  
  return 1;
}

/**********************************************************************/
/**********************************************************************/
ssize_t RaspiInterface::raspiGpioRead( uint8_t flags, uint8_t pin, uint8_t* value )
{

  // check if selected pin is valid for Raspberry Pi
  if( pin > 16 )
  {
    ROS_ERROR("The selected Pin number is not available for GPIO");
    ROS_ERROR("Select Pins 0 through 16 instead");
    return -1;
  }
  
  pinMode( pin, INPUT );
  
  switch ((gpio_input_mode) flags )
  {
    case FLOATING:
    {
      pullUpDnControl( pin, PUD_OFF );
    } break;
    case PULLUP:
    {
      pullUpDnControl( pin, PUD_UP );
    } break;
    case PULLDOWN:
    {
      pullUpDnControl( pin, PUD_DOWN );
    } break;
    default:
    {
      ROS_ERROR("ArduinoInterface::arduinoGpioRead The selected input mode is not known");
      return -1;
    }
  }
  
  value[0] = digitalRead( pin );
  
  return 1;
}

/**********************************************************************/
/**********************************************************************/
ssize_t RaspiInterface::raspiSpi( int frequency, uint8_t flags, uint8_t reg_address, uint8_t* data, size_t num_bytes ) 
{
  //  Decrypt flags:
  uint8_t spi_slave_select = (flags >> 4);
  uint8_t bit_order = (flags>>2) & 0x01;
  uint8_t mode = flags & 0x03;
  
  // Sanity check for decrypted flags
  if( mode != bosch_drivers_common::SPI_MODE_0 )
  {
    ROS_ERROR( "Only mode 0 is implemented in wiringPi at this point" );
    return -1;
  }
  if( bit_order != 0 )
  {
    ROS_ERROR( "Only MSB first is implemented in wiringPi at this point" );
    return -1;
  }
  if( spi_slave_select == bosch_drivers_common::NULL_DEVICE )
  {
    ROS_ERROR( "NULL_DEVICE functionality not implemented yet" );
    return -1;
  }
  if( spi_slave_select >= MAX_SPI_CHANNELS )
  {
    ROS_ERROR( "Maximum SPI channel is %i, you asked for channel %i", MAX_SPI_CHANNELS-1, spi_slave_select );
    return -1;
  }
  if( frequency < MIN_SPI_FREQUENCY || frequency > MAX_SPI_FREQUENCY )
  {
    ROS_WARN( "The requested frequency of %i is out of bounds. Setting frequency to 1MHz", frequency );
    frequency = 1e6;
  }
  
  
  // setup SPI channel if it has not been setup yet
  if( use_spi[spi_slave_select] == false )
  {
    if( wiringPiSPISetup (spi_slave_select, frequency) == -1 )
    {
      ROS_ERROR( "RaspiInterface::initializeSPI(): SPI channel 0 not initialized properly.");
      return false;
    }
    use_spi[spi_slave_select] = true;
    ROS_INFO( "SPI channel %u initialized.", spi_slave_select );
  }
  
  // transfer the address register:
  wiringPiSPIDataRW( spi_slave_select, &reg_address, 1 );
  
  // read/write from/to SPI bus
  wiringPiSPIDataRW( spi_slave_select, data, num_bytes );
  
  return num_bytes;
}

ssize_t RaspiInterface::raspiRs232Write( int frequency, uint8_t* data, size_t num_bytes )
{
  // convert uint8_t* to string
  std::string complete( data, data + num_bytes );
  ROS_INFO("complete string: %s", reinterpret_cast<const char*>(&complete[0]) ); 
  // split string at first colon
  size_t delimiter = complete.find_first_of( ':' );
  if( delimiter == std::string::npos )
  {
    ROS_ERROR( "No colon found in data string! Example: /dev/ttyUSB0:helloWorld" );
    return -1;
  }  
  std::string device  = complete.substr( 0, delimiter );
  std::string command = complete.substr( delimiter + 1 );
  const char* cdevice = reinterpret_cast<const char*>(&device[0]);
  
  // open new serial device if not opened yet
  if( serial_devices_.count(device) != 1 )
  {
    // open serial interface using wiringPi
    ROS_INFO("Opening serial interface %s.", cdevice );
    int file_descriptor = serialOpen( cdevice, frequency );
    // check if serial device was opened successfully
    if( file_descriptor == -1 )
    {
      ROS_ERROR("Opening serial device %s failed :(", cdevice );
      return -1;
    }
    // create new hash entry
    serial_devices_[device] = file_descriptor;    
    ROS_INFO( "Successfully opened serial port %s", cdevice );
  }
  // write command to RS232 connection
  serialPuts( serial_devices_[device], reinterpret_cast<const char*>(&command[0]) );
  
  return command.size();
}

ssize_t RaspiInterface::raspiRs232Read( int frequency, int device_name_length, uint8_t* data, size_t num_bytes )
{
  std::string device( data, data + device_name_length );
  const char* cdevice = reinterpret_cast<const char*>(&device[0]);
  
  // open new serial device if not opened yet
  if( serial_devices_.find(cdevice) == serial_devices_.end() )
  {
    // open serial interface using wiringPi
    int file_descriptor = serialOpen( cdevice, frequency );
    // check if serial device was opened successfully
    if( file_descriptor == -1 )
    {
      ROS_ERROR("Opening serial device %s failed :(", cdevice );
      return -1;
    }
    // create new hash entry
    serial_devices_[device] = file_descriptor;  
    ROS_INFO( "Successfully opened serial port %s", cdevice );
  }
  // read from RS232
  unsigned int index = 0;
  int temp = 0;
  while( temp != -1 && num_bytes-- > 0 )
  {
    temp = serialGetchar( serial_devices_[device] );
    data[index++] = static_cast<uint8_t>(temp);
  }
  
  if( index != num_bytes )
  {
    ROS_WARN( "You asked for %zd bytes but I only read %i due to error or timeout", num_bytes, index );
  }
  
  return index;
}
