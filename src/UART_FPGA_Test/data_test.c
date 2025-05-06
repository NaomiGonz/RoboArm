#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

// Helper to convert float to 4 bytes
void float_to_bytes(float val, uint8_t *bytes) {
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = val;
    for (int i = 0; i < 4; ++i) bytes[i] = u.b[i];
}

// Helper to convert 4 bytes to float
float bytes_to_float(uint8_t *bytes) {
    union {
        float f;
        uint8_t b[4];
    } u;
    for (int i = 0; i < 4; ++i) u.b[i] = bytes[i];
    return u.f;
}

int main() {
    int uart = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_SYNC);
    if (uart < 0) {
        perror("open");
        return 1;
    }

    // Setup UART
    struct termios options;
    tcgetattr(uart, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB; options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;  options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(uart, TCSANOW, &options);

    // 5 hardcoded joint angles
    float joint_angles[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    uint8_t tx_buf[20];
    for (int i = 0; i < 5; ++i) {
        float_to_bytes(joint_angles[i], &tx_buf[i * 4]);
    }

    // Send joint angles
    write(uart, tx_buf, 20);
    tcdrain(uart);

    // Read cartesian points back
    uint8_t rx_buf[60];
    int total_read = 0;
    while (total_read < 60) {
        int n = read(uart, &rx_buf[total_read], 60 - total_read);
        if (n > 0) total_read += n;
        else { perror("read"); break; }
    }

    // Parse received data
    printf("Received cartesian points:\n");
    for (int i = 0; i < 5; ++i) {
        float x = bytes_to_float(&rx_buf[i * 12 + 0]);
        float y = bytes_to_float(&rx_buf[i * 12 + 4]);
        float z = bytes_to_float(&rx_buf[i * 12 + 8]);
        printf("Point %d: (%.4f, %.4f, %.4f)\n", i+1, x, y, z);
    }

    close(uart);
    return 0;
}
