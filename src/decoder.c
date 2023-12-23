/**
 * Functions for decoding ADSB messages
 * 
 * Much of this was either derived or copied directly from dump1090 by Salvatore Sanfilippo (and others)
 *   https://github.com/antirez/dump1090
 * 
 * Technical info was also gleaned from the excellent "The 1090 Megahertz Riddle" by Junzi Sun
 *   https://mode-s.org/decode/index.html
 * 
 * Huge thanks and respect to both.
*/

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "decoder.h"

/**
 * Read N bits from the bitstream starting at the specified index
 * 
 * @param input bitstream to read from as an array of uint8_t, values must be 0 or 1
 * @param start index in the bitstream to start reading from
 * @param nBits number of bits to read
 * @return the value of the bits read
*/
uint32_t readNBits(const uint8_t *input, const int start, const int nBits)
{
  uint32_t res = 0;

  for(int i=0; i<nBits; i++) {
    res = res << 1;
    uint8_t t = input[start+i];
    if (t > 1) {
      printf("ERROR bad bitsream value %d at index %d\n", t, start+i);
      return 0;
    }
    res |= t;
  }

  return res;
}


/**
 * read 8 bits from the bitstream starting at the specified index
 * 
 * NB The caller is responsible for making sure there are enough values in the stream
 * 
 * @param input bitstream to read from as an array of uint8_t, values must be 0 or 1
 * @param start index in the bitstream to start reading from
*/
uint8_t read8Bits(const uint8_t *input, const int start)
{
  const u_int8_t res = readNBits(input, start, 8);

  return res;
}

/**
 * read 24 bits from the bitstream starting at the specified index
 * 
 * NB The caller is responsible for making sure there are enough values in the stream
 * 
 * @param input bitstream to read from as an array of uint8_t, values must be 0 or 1
 * @param start index in the bitstream to start reading from
 * 
*/
uint32_t read24Bits(const uint8_t *input, const int start)
{
  uint32_t res = readNBits(input, start, 24);

  return res;
}


// Parity table for MODE S Messages.
// The table contains 112 elements, every element corresponds to a bit set
// in the message, starting from the first bit of actual data after the
// preamble.
//
// For messages of 112 bit, the whole table is used.
// For messages of 56 bits only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table
// for which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum at the
// end of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes have
// the CRC xored with the sender address as they are reply to interrogations,
// but a casual listener can't split the address from the checksum.
//
const uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

/**
 * modified to operate on the bitsream array which is one byte per bit
*/
uint32_t modeSChecksum(const uint8_t *bitstream, int bits) 
{
  uint32_t   crc = 0;

  int offset = (bits == 112) ? 0 : (112-56);

  const uint32_t * pCRCTable = &modes_checksum_table[offset];
  int j;

  // We don't really need to include the checksum itself
  bits -= 24;
  for(j = 0; j < bits; j++) {
    // If bit is set, xor with corresponding table entry.
    if (bitstream[j]) {crc ^= *pCRCTable;} 
    pCRCTable++;
  }

  if ((crc & 0xFF000000) != 0) printf("ERROR crc & 0xFF000000 != 0\n"); // if this doesn't show up we can get rid of the mask below

  return (crc & 0x00FFFFFF); // JBK - it shouldn't be possible for any of the high bits to be set, so this is unnecessary?
}

bool decodeDF17DF18(uint8_t *bitstream, int bits)
{
  // printf("ADS-B squitter message\n");

  const uint32_t expectedCrc = modeSChecksum(bitstream, 112);
  const uint32_t actualCrc = read24Bits(bitstream, 88);
  bool crcOk = (actualCrc == expectedCrc);
  // printf("Expected CRC: %08lx, actual CRC: %08lx, %s\n", expectedCrc, actualCrc, crcOk ? "MATCHED" : "failed");

  if (!crcOk) {
      // It's relatively cheap to check for single bit errors
      uint32_t error = actualCrc ^ expectedCrc;
      // printf("checking for 1 bit error...\n");
      const int BITS_TO_TEST = 112-24;  // Don't need to check the last 24 bits as they will never match
      for (int i=0; i < BITS_TO_TEST; i++) {
          if (error == modes_checksum_table[i]) {
              // printf("1 bit error at index %d\n", i);
              bitstream[i] = 1 - bitstream[i];
              crcOk = true; // Assumes that if we found a value matching the error there was only the single error. Mostly true
              break;  // we found our 1 bit, so break the loop
          }
      }
  }

  if (crcOk) {

    // TODO eventually we will need to be updating the contact database instead of printing debug
    #ifndef BEAST_OUTPUT
    uint8_t b1 = read8Bits(bitstream, 8);
    uint8_t b2 = read8Bits(bitstream, 16);
    uint8_t b3 = read8Bits(bitstream, 24);
    uint32_t address = (b1 << 16) | (b2 << 8) | b3;

    // The 'type' of the message part is in 5 bits starting at offset 32
    const uint8_t type = readNBits(bitstream, 32, 5);
    printf("Address: %06lx Type: %d\n", address, type);
    #else
    // Output in beast binary format for decoding by readsb
    // https://wiki.jetvision.de/wiki/Mode-S_Beast:Contents
    // <esc> "3" : 6 byte MLAT timestamp, 1 byte signal level, 14 byte Mode-S long frame, where <esc> is 0x1a
    // Any 0x1a values in the body have to be doubled up (esc-esc), so the actual message length is unknown at first
    uint8_t beastMsg[41]; // 2 + 6 + 1 + (14 * 2) + 4
    memset(beastMsg, 0, 12); // zero out the first part of the message (timestamp and signal, rounded up so that memset can write words)
    beastMsg[0] = 0x1a;
    beastMsg[1] = 0x33;
    // leave mlat and signal level 0?
    int outptr = 9;
    for(int i=0; i<14; i++) {
      uint8_t x = read8Bits(bitstream, i*8);
      beastMsg[outptr++] = x;
      if (x == 0x1a) {
        // double up esc chars
        beastMsg[outptr++] = 0x1a;
      }
    }
    beastMsg[outptr++] = 's';
    beastMsg[outptr++] = 'e';
    beastMsg[outptr++] = 'p';
    beastMsg[outptr++] = 0;
    fwrite(beastMsg, 1, outptr, stdout);
    #endif
  }

  return crcOk;
}

/**
 * decodeModeS from bitstream
 * @param bitstream bitstream input data not including the preamble, values must be 0 or 1
 * @param bits number of bits in the bitstream
*/
bool decodeModeS(uint8_t *bitstream, int bits)
{
  bool crcOk = false;
  // printf(".");
  // if (1) return false;

  // printf("decodeModeS_NEW\n");

  // Read 5 bits for the DF
  const int dfStart = 0;
  uint8_t df = 0;

  for(int i=0; i<5; i++) {
    df = df << 1;
    if (bitstream[dfStart+i] > 1) {
      printf("ERROR bad bitsream value %d at index %d\n", bitstream[dfStart+i], dfStart+i);
      return false;
    }
    df |= bitstream[dfStart+i];
  }

  // printf("DF: %02x (%d)\n", df, df);

  switch (df) {
    case 0:
    case 4:
    case 5:
    case 11:
      // printf("df0,4,5,11\n");
      // if (preambleStart + 56 < (ADC_CONVERTED_DATA_BUFFER_SIZE/16)) {
      //   uint8_t b1 = read8Bits(bitstream, preambleStart + 32);
      //   uint8_t b2 = read8Bits(bitstream, preambleStart + 40);
      //   uint8_t b3 = read8Bits(bitstream, preambleStart + 48);
      //   uint32_t address = (b1 << 16) | (b2 << 8) | b3;
      //   printf("Address: %08lx\n", address);
      // } else {
      //   printf("packet too close to the end to read address\n");
      // }
      break;
    case 17:  // These are what we are expecting for ADS-B squitter messages
        // DF starts at +8
        // CA at +13
        // Address at +16
        // ME (payload) at +40
        // Parity at +96
      
        // FALLTHROUGH to DF18

    case 18:
      crcOk = decodeDF17DF18(bitstream, bits);
      break;
      
    default:
      // if ((df & 0b11000) == 0b11000) {
      //   printf("extended length message\n");
      // }
      break;
  }

  // printf("decodeModeS_NEW done\n");

  return crcOk;
}

