#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

int main() {
    int uart = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
    if (uart < 0) {
        perror("Unable to open UART");
        return 1;
    }

    // Configure UART settings
    struct termios options;
    tcgetattr(uart, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB;     // No parity
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8 data bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // No flow control
    options.c_oflag &= ~OPOST;      // Raw output
    tcsetattr(uart, TCSANOW, &options);

    const char *msg = "GIDON\n";
    write(uart, msg, strlen(msg));
    printf("Sent: %s", msg);

    // Read response
    char buf[100];
    int n = read(uart, buf, sizeof(buf) - 1);
    if (n > 0) {
        buf[n] = '\0';
        printf("Received: %s\n", buf);
    } else {
        printf("No response received\n");
    }

    close(uart);
    return 0;
}
