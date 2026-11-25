// Runs one of gpsctl's REAL config functions against a (fake) receiver on a pty.
// usage: cfg_harness <pty slave path> <ini file> nav5|tp5
#include "ublox.c"
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>

dictionary *gpsctlConf = NULL;

int main(int argc, char **argv) {
    if (argc != 4) { fprintf(stderr, "usage: %s <port> <ini> nav5|tp5\n", argv[0]); return 2; }
    int fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return 2; }
    struct termios t;
    tcgetattr(fd, &t); cfmakeraw(&t); tcsetattr(fd, TCSANOW, &t);
    gpsctlConf = iniparser_load(argv[2]);
    if (!gpsctlConf) { fprintf(stderr, "can't load ini\n"); return 2; }
    slReturn r = (strcmp(argv[3], "nav5") == 0) ? ubxConfigNavEngine(fd, 0) : ubxConfigTimePulse(fd, 0);
    if (isErrorReturn(r)) { printf("C-RESULT error: %s\n", getReturnMsg(r)); freeReturn(r); }
    else { printf("C-RESULT ok\n"); freeReturn(r); }
    iniparser_freedict(gpsctlConf);
    close(fd);
    return 0;
}
