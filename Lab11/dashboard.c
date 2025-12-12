#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#define DEVICE_PATH "/dev/iot_sensor"
#define LOG_PATH    "/dev/iot_log"

#define BUFFER_SIZE 128
#define LOG_SIZE    2048

#define IOC_MAGIC        'k'
#define IOC_SET_SENSOR   _IOW(IOC_MAGIC, 1, int)
#define IOC_SET_UNITS    _IOW(IOC_MAGIC, 2, int)
#define IOC_SET_RANGE    _IOW(IOC_MAGIC, 3, struct range)

struct range {
    int min;
    int max;
};

const char *sensor_name(int t) {
    if (t == 0) return "Temperature";
    if (t == 1) return "Humidity";
    return "Pressure";
}

const char *unit_name(int u) {
    return (u == 0) ? "Celsius" : "Fahrenheit";
}

double current_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

int main(void) {
    int fd = open(DEVICE_PATH, O_RDWR);
    int log_fd = open(LOG_PATH, O_RDONLY);

    if (fd < 0 || log_fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    int sensor = 0;   // temperature
    int units  = 0;   // celsius
    struct range r = {0, 50};

    ioctl(fd, IOC_SET_SENSOR, &sensor);
    ioctl(fd, IOC_SET_UNITS, &units);
    ioctl(fd, IOC_SET_RANGE, &r);

    while (1) {
        char buffer[BUFFER_SIZE];
        char log_buffer[LOG_SIZE];
        ssize_t n;
        double start, end;

        printf("\033[H\033[J");
        printf("=== IoT Sensor Dashboard ===\n");
        printf("Sensor Type: %s\n", sensor_name(sensor));
        if (sensor == 0)
            printf("Units: %s\n", unit_name(units));
        printf("Range: %d to %d\n\n", r.min, r.max);

        lseek(fd, 0, SEEK_SET);
        start = current_time_ms();
        n = read(fd, buffer, BUFFER_SIZE - 1);
        end = current_time_ms();

        if (n < 0) {
            printf("Sensor Reading: ERROR (%s)\n", strerror(errno));
        } else {
            buffer[n] = '\0';
            printf("Sensor Reading: %s", buffer);
        }

        printf("Read Latency: %.3f ms\n\n", end - start);

        lseek(log_fd, 0, SEEK_SET);
        n = read(log_fd, log_buffer, LOG_SIZE - 1);
        if (n > 0) {
            log_buffer[n] = '\0';
            printf("Recent Logs:\n%s\n", log_buffer);
        }

        printf("\nCommands:\n");
        printf("  t = Temperature | h = Humidity | p = Pressure\n");
        printf("  c = Celsius     | f = Fahrenheit\n");
        printf("  r min max       | q = Quit\n");
        printf("> ");
        fflush(stdout);

        char cmd[64];
        if (!fgets(cmd, sizeof(cmd), stdin))
            continue;

        if (cmd[0] == 'q')
            break;
        if (cmd[0] == 't') { sensor = 0; ioctl(fd, IOC_SET_SENSOR, &sensor); }
        if (cmd[0] == 'h') { sensor = 1; ioctl(fd, IOC_SET_SENSOR, &sensor); }
        if (cmd[0] == 'p') { sensor = 2; ioctl(fd, IOC_SET_SENSOR, &sensor); }
        if (cmd[0] == 'c') { units = 0; ioctl(fd, IOC_SET_UNITS, &units); }
        if (cmd[0] == 'f') { units = 1; ioctl(fd, IOC_SET_UNITS, &units); }

        if (cmd[0] == 'r') {
            int a, b;
            if (sscanf(cmd + 1, "%d %d", &a, &b) == 2) {
                r.min = a;
                r.max = b;
                ioctl(fd, IOC_SET_RANGE, &r);
            }
        }
    }

    close(fd);
    close(log_fd);
    return 0;
}
