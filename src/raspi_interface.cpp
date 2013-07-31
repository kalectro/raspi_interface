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
  
  switch ((gpio_input_mode) flags )
  {
    case FLOATING:
    case PULLUP: break;
    case PULLDOWN:
    {
      ROS_ERROR("The selected input mode is not available for Raspberry Pi");
      ROS_ERROR("Select FLOATING instead");
      return -1;
    }break;
    default:
    {
      ROS_ERROR("ArduinoInterface::arduinoGpioRead The selected input mode is not known");
      return -1;
    }
  }
  pinMode (pin, OUTPUT);
  digitalWrite (pin, value);
  
  return 1;
}