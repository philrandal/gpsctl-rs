// Golden data for CFG-NMEA, from the REAL ublox.c (included so its statics are reachable):
//   poll   : the empty-body poll frame
//   set    : read-modify-write of byte 1 on a receiver-supplied body (ubxConfigNMEAVersion logic), or "unchanged"/"error"
//   conv   : the config text -> version byte, via the real getConfigNMEAVersion() plus the force-to-4.1 rule (a "-"
//            stands for the empty text).
#include "ublox.c"
#include <stdio.h>

dictionary *gpsctlConf = NULL;

static void hex(const byte *p, size_t n) { for (size_t i = 0; i < n; i++) printf("%s%02X", i ? " " : "", p[i]); }

static void emit_frame(ubxMsg m) {
    printf("B5 62 %02X %02X %02X %02X", m.type.class, m.type.id,
           (unsigned) (size_slBuffer(m.body) & 0xFF), (unsigned) (size_slBuffer(m.body) >> 8));
    for (size_t i = 0; i < size_slBuffer(m.body); i++) printf(" %02X", buffer_slBuffer(m.body)[i]);
    printf(" %02X %02X", m.checksum.ck_a, m.checksum.ck_b);
}

static void set_case(const char *name, const byte *body, size_t len, uint8_t newVer) {
    ubxType t = { UBX_CFG, UBX_CFG_NMEA };
    slBuffer *b = copy_slBuffer(len, LittleEndian, (byte *) body);
    ubxMsg dev = createUbxMsg(t, b);                      // what the receiver would have sent
    printf("set %s %02X : ", name, newVer); hex(body, len); printf(" : ");
    slReturn sz = validateBodySize(&dev, 4, "CFG-NMEA");  // exactly the check ubxConfigNMEAVersion() makes
    if (isErrorReturn(sz)) { printf("error\n"); freeReturn(sz); return; }  /* validateBodySize() already freed the body */
    freeReturn(sz);
    if (get_uint8_slBuffer(b, 1) != newVer) {
        put_uint8_slBuffer(b, 1, newVer);
        emit_frame(createUbxMsg(t, b));
        printf("\n");
    } else printf("unchanged\n");
    free(b);
}

// The config text -> version byte through the REAL getConfigNMEAVersion() (default 0x41 for an invalid setting), then
// the same force-to-4.1-or-higher rule ubxConfigSatellites() applies when BeiDou or Galileo is on.
static int conv(const char *text, int bdsOrGal) {
    gpsctlConf = dictionary_new(0);
    iniparser_set(gpsctlConf, "nmea", NULL);
    iniparser_set(gpsctlConf, "nmea:version", text);
    int nmeaver = getConfigNMEAVersion("nmea:version", 0x41);
    dictionary_del(gpsctlConf);
    gpsctlConf = NULL;
    if (bdsOrGal && nmeaver < 0x41) nmeaver = 0x41;
    return (uint8_t) nmeaver;
}

int main(void) {
    ubxType t = { UBX_CFG, UBX_CFG_NMEA };
    slBuffer *empty = create_slBuffer(0, LittleEndian);
    printf("poll : "); emit_frame(createUbxMsg(t, empty)); printf("\n"); free(empty);

    // Sample receiver bodies (test inputs only, not claims about any particular firmware): 20, 12 and 4 bytes.
    const byte b20[] = {0x00,0x23,0x08,0x00, 0x00,0x00,0x00,0x00, 0x01,0x00,0x00,0x00, 0x01,0x01,0x00,0x00, 0x00,0x00,0x00,0x00};
    const byte b20b[] = {0x07,0x40,0x14,0x1C, 0x03,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x01,0x00,0x00,0x00, 0x00,0x00,0x00,0x00};
    const byte b12[] = {0x00,0x21,0x08,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00};
    const byte b4[]  = {0x00,0x23,0x08,0x00};
    const byte b3[]  = {0x00,0x23,0x08};
    set_case("20to41", b20, sizeof b20, 0x41);
    set_case("20same", b20b, sizeof b20b, 0x40);
    set_case("20to23", b20b, sizeof b20b, 0x23);
    set_case("12to41", b12, sizeof b12, 0x41);
    set_case("4to40",  b4,  sizeof b4,  0x40);
    set_case("3short", b3,  sizeof b3,  0x41);

    const char *vals[] = {"2.1", "21", "2.3", "23", "4.0", "40", "4.1", "41", "4.10", "410", "4.11", "411", "3.0",
                          "2.0", "4.15", "41.5", "2.5", "abc", "", "0", "-4.1", "1e3", "99999", "4.0m", "4.2", "0x41"};
    for (size_t i = 0; i < sizeof vals / sizeof vals[0]; i++)
        for (int f = 0; f <= 1; f++)
            printf("conv %s %d : %02X\n", vals[i][0] ? vals[i] : "-", f, conv(vals[i], f));
    return 0;
}
