// Golden data for MON-VER decoding, from the REAL copyBodyString() in ublox.c (included so the static is reachable).
// NOTE: the software/hardware/extension slicing and the extension-count formula are a COPY of ubxGetVersion()
// (ublox.c ~lines 1040-1062) because that function polls a port; keep in sync if it changes.
// Sample bodies are made-up test inputs, NOT captured from a receiver (except where noted).
#include "ublox.c"
#include <stdio.h>

dictionary *gpsctlConf = NULL;

static void put(byte *body, size_t off, size_t width, const char *s) {   // NUL-padded field
    memset(body + off, 0, width);
    memcpy(body + off, s, strlen(s) < width ? strlen(s) : width);
}

static void emit(const char *name, const byte *body, size_t bodyLen) {
    printf("ver %s : ", name);
    for (size_t i = 0; i < bodyLen; i++) printf("%s%02X", i ? " " : "", body[i]);
    printf(" : ");
    const char *buf = (const char *) body;
    char *sw = copyBodyString(buf, bodyLen, 0, 30);
    char *hw = copyBodyString(buf, bodyLen, 30, 10);
    int bodySize = (int) bodyLen;
    int ne = (bodySize > 40) ? ((bodySize - 40) + 29) / 30 : 0;
    printf("%d|%s|%s", ne, sw, hw);
    free(sw); free(hw);
    for (int i = 0; i < ne; i++) {
        char *e = copyBodyString(buf, bodyLen, 40 + 30 * (size_t) i, 30);
        printf("|%s", e);
        free(e);
    }
    printf("\n");
}

int main(void) {
    byte b[400];

    memset(b, 0, sizeof b);                                   // typical M8-style layout, 4 extensions
    put(b, 0, 30, "ROM CORE 3.01 (107888)"); put(b, 30, 10, "00080000");
    put(b, 40, 30, "FWVER=SPG 3.01"); put(b, 70, 30, "PROTVER=18.00");
    put(b, 100, 30, "GPS;GLO;GAL;BDS"); put(b, 130, 30, "SBAS;IMES;QZSS");
    emit("typical", b, 160);

    memset(b, 0, sizeof b); put(b, 0, 30, "EXT CORE 1.00"); put(b, 30, 10, "00040007");
    emit("noext", b, 40);

    memset(b, 0, sizeof b); put(b, 0, 30, "A"); put(b, 30, 10, "B"); b[40] = 'X';
    emit("oneByteExt", b, 41);

    memset(b, 'S', 30); memset(b + 30, 'H', 10); memset(b + 40, 'A', 30);     // no NUL anywhere: fields must be cut
    emit("nonTerminated", b, 70);

    memset(b, 0, sizeof b); put(b, 0, 30, "SW"); put(b, 30, 10, "HW"); memcpy(b + 40, "FIRST", 5); memcpy(b + 70, "SECOND", 6);
    emit("partialSecond", b, 76);

    memset(b, 0, sizeof b); memcpy(b, "SHORTSW", 7);
    emit("short10", b, 10);
    emit("short35", b, 35);
    emit("empty", b, 0);
    return 0;
}
