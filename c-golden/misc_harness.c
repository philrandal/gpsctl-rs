// Runs gpsctl's REAL ubxGetFix / ubxGetSatellites / isNMEAmsgEnabled / ubxEnableNMEAMsg / ubxChangeBaudRate against a
// (fake) receiver on a pty and prints what they decoded, or the error.  Host baud changes are logged as HOSTBAUD n.
// usage: misc_harness <pty> fix | sats | nmeaon | rate | msgq <0-8> | msgset <0-8> <0|1> | baud <rate>
#include "sl_serial.h"
#include <stdio.h>
#define setTermOptionsBaud(fd, b) (printf("HOSTBAUD %d\n", (b)), setTermOptionsBaud((fd), (b)))
#include "ublox.c"
#undef setTermOptionsBaud
#include <fcntl.h>
#include <termios.h>

dictionary *gpsctlConf = NULL;

static void done(slReturn r) {
    if (isErrorReturn(r)) { printf("RESULT error\n"); fprintf(stderr, "ERR: %s\n", getReturnMsg(r)); } else printf("RESULT ok\n");
    freeReturn(r);
}

int main(int argc, char **argv) {
    if (argc < 3) return 2;
    int fd = open(argv[1], O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return 2; }
    struct termios t;
    tcgetattr(fd, &t); cfmakeraw(&t); tcsetattr(fd, TCSANOW, &t);
    const char *cmd = argv[2];
    if (!strcmp(cmd, "fix")) {
        ubxFix f; memset(&f, 0, sizeof f);
        slReturn r = ubxGetFix(fd, 0, &f);
        if (isOkReturn(r)) {
            printf("FIX %d %d %d %d %d %.9f %d %u %d %d %d %d %d %u %.9f %.9f %d %d %u %u %d %u %.9f %.9f\n",
                f.year, f.month, f.day, f.hour, f.minute, f.second, f.nanoseconds_correction, f.time_accuracy_ns,
                f.time_resolved, f.date_valid, f.time_valid, f.fix_is_3d, f.fix_valid, f.number_of_satellites_used,
                f.longitude_deg, f.latitude_deg, f.height_above_ellipsoid_mm, f.height_above_sea_level_mm,
                f.height_accuracy_mm, f.horizontal_accuracy_mm, f.ground_speed_mm_s, f.ground_speed_accuracy_mm_s,
                f.heading_deg, f.heading_accuracy_deg);
        }
        done(r);
    } else if (!strcmp(cmd, "sats")) {
        ubxSatellites s; memset(&s, 0, sizeof s);
        slReturn r = ubxGetSatellites(fd, 0, &s);
        if (isOkReturn(r)) {
            printf("SATS %d\n", s.numberOfSatellites);
            for (int i = 0; i < s.numberOfSatellites; i++) {
                ubxSatellite *a = s.satellites + i;
                printf("SAT %d %u %u %d %d %.1f %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", a->gnssID, a->satelliteID,
                    a->cno, a->elevation, a->azimuth, a->pseudoRangeResidualM, a->signalQuality, a->used, a->health,
                    a->diffCorr, a->smoothed, a->orbitSource, a->haveEphemeris, a->haveAlmanac, a->haveAssistNowOff,
                    a->haveAssistNowAuto, a->sbasCorrUsed, a->rtcmCorrUsed, a->prCorrUsed, a->crCorrUsed);
                // doCorrUsed on its own line item to keep the format string readable
                printf("SATDO %d\n", a->doCorrUsed);
            }
            free(s.satellites);
        }
        done(r);
    } else if (!strcmp(cmd, "msgq") && argc == 4) {
        slReturn r = isNMEAmsgEnabled(fd, 0, (nmeaMSG) atoi(argv[3]));
        if (isOkReturn(r)) printf("ENABLED %d\n", getReturnInfoBool(r));
        done(r);
    } else if (!strcmp(cmd, "msgset") && argc == 5) {
        done(ubxEnableNMEAMsg(fd, 0, (nmeaMSG) atoi(argv[3]), atoi(argv[4]) != 0));
    } else if (!strcmp(cmd, "nmeaon")) {
        slReturn r = isNmeaOn(fd);
        if (isOkReturn(r)) printf("NMEAON %d\n", getReturnInfoBool(r));
        done(r);
    } else if (!strcmp(cmd, "rate")) {
        ubxConfig c; memset(&c, 0, sizeof c);
        slReturn r = ubxGetConfig(fd, 0, &c);
        if (isOkReturn(r)) printf("RATE %u %u %d\n", c.measRateMs, c.navRate, (int) c.timeRef);
        done(r);
    } else if (!strcmp(cmd, "rx") && argc == 6) {
        // pollUbx(class id) with an empty body; prints the reply body or the error
        ubxType type = { (uint8_t) strtol(argv[3], NULL, 16), (uint8_t) strtol(argv[4], NULL, 16) };
        slBuffer *pb = create_slBuffer(0, LittleEndian);
        ubxMsg pm = createUbxMsg(type, pb), ans;
        slReturn r = pollUbx(fd, pm, atoi(argv[5]), &ans);
        if (isOkReturn(r)) {
            printf("POLLBODY");
            for (size_t i = 0; i < size_slBuffer(ans.body); i++) printf(" %02X", buffer_slBuffer(ans.body)[i]);
            printf("\n");
            free(ans.body);
        }
        done(r);
    } else if (!strcmp(cmd, "baud") && argc == 4) {
        done(ubxChangeBaudRate(fd, (unsigned) atoi(argv[3]), 0));
    } else return 2;
    close(fd);
    return 0;
}
