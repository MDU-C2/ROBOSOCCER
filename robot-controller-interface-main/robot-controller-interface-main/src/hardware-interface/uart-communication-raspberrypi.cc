 
 /* main.cc
 *==============================================================================
 * Author: Aaiza A. Khan, Shruthi P. Kunnon
 * Creation date: 2024-11-14
 * Last modified: 2024-12-12 by Aaiza A. Khan
 * Description: Used for communication between raspberry pi and STM 32
 * via UART.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */

/* C++ standard library headers */
#include <iostream>    
#include <thread>      
#include <cstring>     

/* Other .h files */
#include <fcntl.h>     
#include <unistd.h>    
#include <termios.h>   
#include <sys/select.h> 

/* Project .h files */
/* It does not have any accompanying .h file */

namespace robot_controller_interface
{
namespace hardware_interface
{

int setup_uart(const char* device) {
  /* Open the UART device file with read/write permissions */
  int file_descriptor = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (file_descriptor == -1) {
      perror("Unable to open UART");
      return -1;/* Return -1 if unable to open the UART */
  }

  struct termios options;
  tcgetattr(file_descriptor, &options);/* Get current UART settings */

  /* Set baud rate to 115200 for both input and output communication */
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  /* Set 8N1 (8 data bits, no parity, 1 stop bit) */
  options.c_cflag &= ~PARENB; /* Clear parity bit */
  options.c_cflag &= ~CSTOPB; /* Clear stop bit */
  options.c_cflag &= ~CSIZE;  /* Clear data size bits */
  options.c_cflag |= CS8;  /* Set data size to 8 bits */
  options.c_cflag &= ~CRTSCTS;   /* Disable hardware flow control */

  /* Enable receiver, disable modem control lines */
  options.c_cflag |= (CLOCAL | CREAD);

  /* Set raw input mode (no echo, no processing) */
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);

  /* Set raw output mode */
  options.c_oflag &= ~OPOST;

  /* Apply the settings */
  tcsetattr(file_descriptor, TCSANOW, &options);

  /* Flush the UART input and output buffers to ensure clean communication */
  tcflush(file_descriptor, TCIOFLUSH);

  return file_descriptor;
}

/* Function to send data through UART */
void transmit_data(int file_descriptor, const char* message) {
  write(file_descriptor, message, std::strlen(message));
}

/*  Function to receive data from UART */
void receive_data(int file_descriptor) {
  fd_set read_file_descriptors;
  struct timeval timeout;
  char buffer[256];
  int ret;
  int bytes_read;

  while (true) {
    /* Clear and set the file descriptor */
    FD_ZERO(&read_file_descriptors);
    FD_SET(file_descriptor, &read_file_descriptors);

    /* setting timeout */
    timeout.tv_sec =5;
    timeout.tv_usec = 0;

    /* wait for data on the UART file descriptor */
    ret = select(file_descriptor + 1, &read_file_descriptors, nullptr, nullptr,
        &timeout);
    if (ret == -1) {
      /* Handle error and exit the loop. */
      perror("select() failed");
      return;
    }
    else if (ret > 0) {
      if (FD_ISSET(file_descriptor, &read_file_descriptors)) {
        /* Data is available to read it into the buffer */
        bytes_read = read(file_descriptor, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
          /* Null-terminate the received data and print it */
          buffer[bytes_read] = '\0';  // Null-terminate the received string
          std::cout << "Received: " << buffer << std::endl;
        }
      }
    }
    else {
      /* No data received within the timeout period */
      std::cout << "Timeout while waiting for data" << std::endl;
    }
  }
}

} /* namespace hardware_interface */
} /* namespace robot_controller_interface */

int main() {
  /* Default path to UART device on Raspberry Pi */
  const char* uart_device = "/dev/serial0";
  int uart_file_descriptor =robot_controller_interface::hardware_interface
      ::setup_uart(uart_device);
  if (uart_file_descriptor == -1) {
    return 1; /* Error initializing UART */
  }

  /* Launch separate thread/process for receiving data */
  std::thread receiver_thread
      (robot_controller_interface::hardware_interface::receive_data,
          uart_file_descriptor);

  /* Main loop to periodically transmit data through UART */
  const char* message = "Hello STM32!\n";
  while (true) {
    robot_controller_interface::hardware_interface::
        transmit_data(uart_file_descriptor, message);
    /* Log the transmitted message to the console */
    std::cout << "Sent: " << message << std::endl;
    sleep(1);  /* Wait for 1 second before sending the next message */
  }

    receiver_thread.join();  /* Wait for receiver thread to finish */
    close(uart_file_descriptor);  /* Close the UART file descriptor */
    return 0;
}


