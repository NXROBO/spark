/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
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
*   * Neither the name of the ISR University of Coimbra nor the names of its
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
* Author: Gonçalo Cabrita on 01/10/2010
*********************************************************************/
#include <stdexcept>
#include <termios.h>
#include <string>
#include <vector>
#include <stdint.h>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#define MAX_LENGTH 128

namespace cereal
{
	//! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
	#define DEF_EXCEPTION(name, parent) \
	class name : public parent { \
  		public: \
    	name(const char* msg) : parent(msg) {} \
  	}
  
  	//! A standard exception
  	DEF_EXCEPTION(Exception, std::runtime_error);

	//! An exception for use when a timeout is exceeded
  	DEF_EXCEPTION(TimeoutException, Exception);

	#undef DEF_EXCEPTION

	/*! \class CerealPort CerealPort.h "inc/CerealPort.h"
	 *  \brief C++ serial port class for ROS.
	 *
	 * This class was based on the serial port code found on the hokuyo_node as suggested by Blaise Gassend on the ros-users mailling list.
	 */
	class CerealPort
	{
		public:
		//! Constructor
		CerealPort();
		//! Destructor
		~CerealPort();

		//! Open the serial port
		/*! 
		* This opens the serial port for communication. Wrapper for open.
		* 
		* \param port_name   A null terminated character array containing the name of the port.
		* \param baud_rate   Baud rate of the serial port. Defaults to 115200.
		*
		*/
		void open(const char * port_name, int baud_rate = 115200);

		//! Close the serial port
		/*!
		* This call essentially wraps close.
		*/
		void close();

		//! Check whether the port is open or not
		bool portOpen() { return fd_ != -1; }

		//! Get the current baud rate
		int baudRate() { return baud_; }

		//! Write to the port
		/*!
		*  This function allows to send data through the serial port. Wraper for write.
		*
		*  \param data    Data to send in a character array or null terminated c string.
		*  \param length  Number of bytes being sent. Defaults to -1 if sending a c string.
		* 
		*  \return Number of bytes writen.	
		*/
		int write(const char * data, int length = -1);
		
		//! Read from the port
		/*!
		*  This function allows to read data from the serial port. Simple wrapper for read.
		*
		*  \param data    		Data coming from the serial port.
		*  \param max_length  	Maximum length of the incoming data.
		*  \param timeout 		Timeout in milliseconds.
		*
		*  \return Number of bytes read.
		*/
	    int read(char * data, int max_length, int timeout = -1);

		//! Read a fixed number of bytes from the serial port
		/*!
		*  This function allows to read a fixed number of data bytes from the serial port, no more, no less.
		*
		*  \param data    Data coming from the serial port.
		*  \param length  Fixed length of the incoming data.
		*  \param timeout Timeout in milliseconds.
		*
		*  \sa read()
		* 
		*  \return Number of bytes read.
		*/
	    int readBytes(char * data, int length, int timeout = -1);

	   	//! Read a line from the serial port
		/*!
		*  This function allows to read a line from the serial port. Data is return as char*
		*
		*  \param data    	Data coming from the serial port.
		*  \param length  	Length of the incoming data.
		*  \param timeout	Timeout in milliseconds.
		* 
		*  \sa readLine(std::string*, int)
		*
		*  \return Number of bytes read.
		*/
	    int readLine(char * data, int length, int timeout = -1);
	    
	    //! Read a line from the serial port
		/*!
		*  This function allows to read a line from the serial port. Data is return as std::string
		*
		*  \param data    	Data coming from the serial port.
		*  \param timeout	Timeout in milliseconds.
		* 
		*  \sa readLine(char*, int, int)
		*
		*  \return Whether the read was successful or not.
		*/
	    bool readLine(std::string * data, int timeout = -1);
	    
	    //! Read from the serial port between a start char and an end char
		/*!
		*  This function allows to read data from the serial port between a start and an end char.
		*
		*  \param data    	Data coming from the serial port.
		*  \param start		Start character of the incoming data stream.
		*  \param end		End character of the incoming data stream.
		*  \param timeout	Timeout in milliseconds.
		* 
		*  \return Whether the read was successful or not.
		*/
		//TODO: int readBetween(char * data, int length, char start, char end, int timeout = -1);
	    bool readBetween(std::string * data, char start, char end, int timeout = -1);

	    //! Wrapper around tcflush
	    int flush();
	    
	    //*** Stream functions ***
	    
	    //! Start a stream of read()
		/*!
		*  Stream version of the read function.
		*
		*  \param f    		Callback boost function to receive the data.
		*
		*  \sa read()
		* 
		*  \return True if successful false if a stream is already running.
		*/
	    bool startReadStream(boost::function<void(char*, int)> f);
	    
	    //! Start a stream of readLine(std::string*, int)
		/*!
		*  Stream version of the readLine(std::string*, int) function.
		*
		*  \param f    		Callback boost function to receive the data.
		*
		*  \sa readLine(std::string*, int)
		* 
		*  \return True if successful false if a stream is already running.
		*/
	    bool startReadLineStream(boost::function<void(std::string*)> f);
	    
	    //! Start a stream of readBetween()
		/*!
		*  Stream version of the readBetween() function.
		*
		*  \param f    		Callback boost function to receive the data.
		*  \param start		Start character of the incoming data stream.
		*  \param end		End character of the incoming data stream.
		*
		*  \sa readBetween()
		* 
		*  \return True if successful false if a stream is already running.
		*/
	    bool startReadBetweenStream(boost::function<void(std::string*)> f, char start, char end);
	    
	    //! Stop streaming
		void stopStream();
		//! Pause streaming
		void pauseStream();
		//! Resume streaming
		void resumeStream();
		
		private:
		//! File descriptor
	   	int fd_;
	   	//! Baud rate
		int baud_;
		
		//std::vector<char> leftovers;
		
		//! Thread for a stream of read()
		/*!
		*  Stream version of the read function. 
		*/
		void readThread();
		
		//! Thread for a stream of readLine(std::string*, int)
		/*!
		*  Stream version of the readLine function.
		*/
	    void readLineThread();
	    
	    //! Thread for a stream of readBetween()
		/*!
		*  Stream version of the readBetween function.
		*
		*  \param start		Start character of the incoming data stream.
		*  \param end		End character of the incoming data stream.	
		* 
		*/
	    void readBetweenThread(char start, char end);
		
		//! Stream thread
		boost::thread * stream_thread_;
		
		//! Stream read callback boost function
		boost::function<void(char*, int)> readCallback;
		//! Stream readLine callback boost function
		boost::function<void(std::string*)> readLineCallback;
		//! Stream readBetween callback boost function
		boost::function<void(std::string*)> readBetweenCallback;
		
		//! Whether streaming is paused or not
		bool stream_paused_;
		//! Whether streaming is stopped or not
		bool stream_stopped_;
	};

}

// EOF
