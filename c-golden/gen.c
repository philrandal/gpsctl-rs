// Generates golden CFG-NAV5 frames using the REAL scaleRound() and Fletcher checksum from gpsctl's ublox.c
// (included directly so its static functions are reachable).  The body offsets below mirror ubxConfigNavEngine().
#include "ublox.c"
#include <stdio.h>

dictionary *gpsctlConf = NULL;   // only referenced by functions this harness never calls

typedef struct { const char *name; int dyn, fix; double alt, altVar; int minElev; double pdop, tdop; int pacc, tacc,
                 hold, dgnss, nsv, cno, holdDist, utc; } params;

static void emit(const params *p) {
    slBuffer *b = create_slBuffer(36, LittleEndian);
    put_uint16_slBuffer(b,  0, 0x05F7);
    put_uint8_slBuffer(b,  2, p->dyn);
    put_uint8_slBuffer(b,  3, p->fix);
    put_uint32_slBuffer(b,  4, (uint32_t) scaleRound(p->alt, 100.0));
    put_uint32_slBuffer(b,  8, (uint32_t) scaleRound(p->altVar, 10000.0));
    put_uint8_slBuffer(b, 12, p->minElev);
    put_uint16_slBuffer(b, 14, (uint16_t) scaleRound(p->pdop, 10.0));
    put_uint16_slBuffer(b, 16, (uint16_t) scaleRound(p->tdop, 10.0));
    put_uint16_slBuffer(b, 18, p->pacc);
    put_uint16_slBuffer(b, 20, p->tacc);
    put_uint8_slBuffer(b, 22, p->hold);
    put_uint8_slBuffer(b, 23, p->dgnss);
    put_uint8_slBuffer(b, 24, p->nsv);
    put_uint8_slBuffer(b, 25, p->cno);
    put_uint16_slBuffer(b, 28, p->holdDist);
    put_uint8_slBuffer(b, 30, p->utc);
    ubxType t = { UBX_CFG, UBX_CFG_NAV5 };
    ubxMsg m = createUbxMsg(t, b);
    // name dyn fix alt altVar minElev pdop tdop pacc tacc hold dgnss nsv cno holdDist utc : frame-hex
    printf("%s %d %d %.6f %.6f %d %.6f %.6f %d %d %d %d %d %d %d %d : B5 62 %02X %02X 24 00", p->name, p->dyn, p->fix, p->alt,
           p->altVar, p->minElev, p->pdop, p->tdop, p->pacc, p->tacc, p->hold, p->dgnss, p->nsv, p->cno, p->holdDist, p->utc,
           t.class, t.id);
    for (size_t i = 0; i < 36; i++) printf(" %02X", buffer_slBuffer(b)[i]);
    printf(" %02X %02X\n", m.checksum.ck_a, m.checksum.ck_b);
    free(b);
}

int main(void) {
    params v[] = {
        {"defaults",   2, 2,  0.0,   0.0,    20, 10.0, 10.0, 40, 40, 0, 60, 8, 20, 0, 3},
        {"alt1.15",    2, 2,  1.15,  0.0,    20, 10.0, 10.0, 40, 40, 0, 60, 8, 20, 0, 3},
        {"altneg1.15", 2, 2, -1.15,  0.0,    20, 10.0, 10.0, 40, 40, 0, 60, 8, 20, 0, 3},
        {"dop2.3",     3, 3,  0.0,   0.0001, 15,  2.3,  2.3, 100, 300, 5, 30, 6, 25, 10, 3},
        {"dop0.05",    4, 1,  1234.56, 0.0005, 5, 0.05, 99.95, 65535, 1, 255, 255, 12, 45, 65535, 6},
        {"altneg0.005",2, 2, -0.005, 1.15,   20, 10.0, 10.0, 40, 40, 0, 60, 8, 20, 0, 3},
        {"hold5",      2, 2,  0.0,   0.0,    20, 10.0, 10.0, 40, 40, 5, 60, 8, 20, 0, 3},
        {"hold7",      2, 2,  0.0,   0.0,    20, 10.0, 10.0, 40, 40, 7, 60, 8, 20, 0, 3},
        {"hold255",    2, 2,  0.0,   0.0,    20, 10.0, 10.0, 40, 40, 255, 60, 8, 20, 0, 3},
        {"utcchina",   2, 2,  0.0,   0.0,    20, 10.0, 10.0, 40, 40, 0, 60, 8, 20, 0, 7},
    };
    for (size_t i = 0; i < sizeof v / sizeof v[0]; i++) emit(&v[i]);
    return 0;
}
