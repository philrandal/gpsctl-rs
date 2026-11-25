// Runs gpsctl's REAL configuration commands against a (fake) receiver on a pty, with a gpsctl.conf read from a file.
// usage: cmd_harness <pty> <ini file or -> save | reset | nmeadata <0|1> | msg <0-8> <0|1> | nmeaver <hex> | sat | tp | nav | timing
// Prints RESULT ok|error, then "ERR: <message>", and one "WARN" line per Warning: gpsctl prints (stdout or stderr).
#include "sl_serial.h"
#include <stdio.h>
#include "ublox.c"
#include <fcntl.h>
#include <termios.h>

dictionary *gpsctlConf = NULL;

int main(int argc, char **argv) {
    if (argc < 4) return 2;
    int fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return 2; }
    struct termios t;
    tcgetattr(fd, &t); cfmakeraw(&t); tcsetattr(fd, TCSANOW, &t);
    gpsctlConf = strcmp(argv[2], "-") ? iniparser_load(argv[2]) : dictionary_new(0);
    if (gpsctlConf == NULL) gpsctlConf = dictionary_new(0);
    const char *cmd = argv[3];
    int v = 1;  // verbosity 1: the non-fatal problems are printed as "Warning:"
    slReturn r;
    if (!strcmp(cmd, "save")) r = ubxSaveConfig(fd, v);
    else if (!strcmp(cmd, "reset")) r = ubxReset(fd, v);
    else if (!strcmp(cmd, "nmeadata")) r = ubxSetNMEAData(fd, v, atoi(argv[4]) != 0);
    else if (!strcmp(cmd, "msg")) r = ubxEnableNMEAMsg(fd, v, (nmeaMSG) atoi(argv[4]), atoi(argv[5]) != 0);
    else if (!strcmp(cmd, "nmeaver")) r = ubxConfigNMEAVersion(fd, v, (uint8_t) strtol(argv[4], NULL, 16));
    else if (!strcmp(cmd, "sat")) r = ubxConfigSatellites(fd, v);
    else if (!strcmp(cmd, "tp")) r = ubxConfigTimePulse(fd, v);
    else if (!strcmp(cmd, "nav")) r = ubxConfigNavEngine(fd, v);
    else if (!strcmp(cmd, "timing")) r = ubxConfigForTiming(fd, v);
    else return 2;
    fflush(stdout);
    if (isErrorReturn(r)) { printf("RESULT error\n"); fprintf(stderr, "ERR: %s\n", getReturnMsg(r)); } else printf("RESULT ok\n");
    freeReturn(r);
    return 0;
}
