/**
 * Functions for decoding ADSB messages
*/

#ifndef ADSB_DECODE_H
#define ADSB_DECODE_H

bool decodeModeS(const uint8_t *bitstream, int bits);

#endif // ADSB_DECODE_H
