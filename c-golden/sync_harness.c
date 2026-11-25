// Runs gpsctl's REAL autoBaudRate() with its real synchronizers (ascii and ubx from the libraries, nmea from gpsctl.c's
// static) on a pty.   usage: sync_harness <pty> <ascii|nmea|ubx> <minBaud>      prints "BAUD n" or "FAIL"
#define main gpsctl_main_unused
#include "gpsctl.c"
#undef main
#include <fcntl.h>

int main(int argc, char **argv) {
    if (argc != 4) return 2;
    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 2; }
    baudRateSynchronizer *sy = !strcmp(argv[2], "ascii") ? asciiBaudRateSynchronizer
                             : !strcmp(argv[2], "nmea")  ? nmeaBaudRateSynchronizer : ubxSynchronizer;
    slReturn r = setTermOptions(fd, 921600, 8, 1, false, false);
    if (isErrorReturn(r)) { printf("FAIL setopts\n"); return 1; }
    r = autoBaudRate(fd, atoi(argv[3]), sy, 0);
    if (isErrorReturn(r)) printf("FAIL\n"); else printf("BAUD %d\n", getReturnInfoInt32(r));
    return 0;
}
