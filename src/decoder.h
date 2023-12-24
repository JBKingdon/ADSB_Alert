/**
 * Functions for decoding ADSB messages
*/

#ifndef ADSB_DECODE_H
#define ADSB_DECODE_H

#include <stdint.h>

// Enable binary data output for readsb
// #define BEAST_OUTPUT

bool decodeModeS(uint8_t *bitstream, int bits);

#endif // ADSB_DECODE_H
