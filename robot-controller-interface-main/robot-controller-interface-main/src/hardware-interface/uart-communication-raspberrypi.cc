#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/select.h>
#include <thread>

using namespace std;

int setup_uart(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open UART");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // Set baud rate to 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // Set 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;   // Disable hardware flow control

    // Enable receiver, disable modem control lines
    options.c_cflag |= (CLOCAL | CREAD);

    // Set raw input mode (no echo, no processing)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Set raw output mode
    options.c_oflag &= ~OPOST;

    // Apply the settings
    tcsetattr(fd, TCSANOW, &options);

    // Flush the UART buffer
    tcflush(fd, TCIOFLUSH); // Clear both input and output buffers

    return fd;
}

void transmit_data(int fd, const char* message) {
    write(fd, message, strlen(message));
}

void receive_data(int fd) {
    fd_set readfds;
    struct timeval timeout;
    char buffer[256];

    while (true) {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);

        timeout.tv_sec =5;  // 2-second timeout
        timeout.tv_usec = 0;

        int ret = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
        if (ret == -1) {
            perror("select() failed");
            return;
        } else if (ret > 0) {
            if (FD_ISSET(fd, &readfds)) {
                int bytes_read = read(fd, buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';  // Null-terminate the received string
                    cout << "Received: " << buffer << endl;
                }
            }
        } else {
            cout << "Timeout while waiting for data" << endl;
        }
    }
}

int main() {
    const char* uart_device = "/dev/serial0";  // Default UART device on Raspberry Pi
    int uart_fd = setup_uart(uart_device);
    if (uart_fd == -1) {
        return 1;  // Error initializing UART
    }

    // Launch separate thread/process for receiving data
    std::thread receiver_thread(receive_data, uart_fd);

    // Main loop: Transmitting data
    const char* message = "Hello STM32!\n";
    while (true) {
        transmit_data(uart_fd, message);
        cout << "Sent: " << message << endl;
        sleep(1);  // Transmit data every 1 second
    }

    receiver_thread.join();  // Wait for receiver thread to finish (optional in this case)
    close(uart_fd);  // Close UART file descriptor when done

    return 0;
}
