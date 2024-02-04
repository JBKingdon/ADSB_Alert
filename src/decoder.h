/**
 * Functions for decoding ADSB messages
*/

#ifndef ADSB_DECODE_H
#define ADSB_DECODE_H

#include <stdint.h>
#include <stdbool.h>

// Enable binary data output for readsb
// #define BEAST_OUTPUT

#define MAX_CONTACTS 10

extern uint32_t totalMessages;
extern uint32_t totalCorrected;
extern float maxRange;

// Struct for holding distance & bearing values
typedef struct distance_bearing {
    float distance;
    float bearing;
} distance_bearing_t;

// Struct from dump1090 for holding aircraft info
// TODO review the fields to see if any are unnecessary, and check required sizes
struct aircraft {
    uint32_t      addr;           // ICAO address
    char          callsign[10];   // Flight number
    uint16_t      wake_class;
    // unsigned char signalLevel[8]; // Last 8 Signal Amplitudes    // JBK how to measure signal strength?
    int           altitude;       // Altitude
    int           speed;          // Velocity
    int           track;          // Angle of flight
    int           vert_rate;      // Vertical rate.

    // JBK Need to figure out a way to get abs time - from the GPS?
    // time_t        seen;           // Time at which the last packet was received
    // time_t        seenLatLon;     // Time at which the last lat long was calculated
    // uint64_t      timestamp;      // Timestamp at which the last packet was received
    // uint64_t      timestampLatLon;// Timestamp at which the last lat long was calculated
    uint32_t      timestamp;      // Timestamp at which the last packet was received
    uint32_t      timestampLatLon;// Timestamp at which the last lat long was calculated

    uint16_t      messages;       // Number of Mode S messages received
    int           modeA;          // Squawk
    int           modeC;          // Altitude
    // long          modeAcount;     // Mode A Squawk hit Count
    // long          modeCcount;     // Mode C Altitude hit Count
    int           modeACflags;    // Flags for mode A/C recognition

    // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
    int           odd_cprlat;
    int           odd_cprlon;
    int           even_cprlat;
    int           even_cprlon;
    uint32_t      odd_cprtime;
    uint32_t      even_cprtime;
    double        lat, lon;       // Coordinates obtained from CPR encoded data
    int           bFlags;         // Flags related to valid fields in this structure
    // struct aircraft *next;        // Next aircraft in our linked list // JBK use a static array for reliability on embedded system
    float         range;        // Should this use a distance_bearing_t?
    float         bearing;
};

typedef struct aircraft aircraft_t;

typedef struct distance_bearing distance_bearing_t;

// Public functions:

bool decodeModeS(uint8_t *bitstream, int bits);


#endif // ADSB_DECODE_H
