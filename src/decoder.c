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
#include <math.h>

#include "stm32h7xx_hal.h"

#include "decoder.h"
#include "contactManager.h"

#include "localConfig.h"

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

  return crc;
}

/**
 * Calculate the distance between two sets of coordinates in km
 * See https://www.movable-type.co.uk/scripts/latlong.html
 * 
 * 
*/
void calculateDistance(double lat1, double lon1, double lat2, double lon2, distance_bearing_t *result)
{
  // Radius of earth in km
  const double R = 6371;
  const double DegreesToRadians = M_PI / 180.0;

  // Convert to radians
  const double lat1R = lat1 * DegreesToRadians;
  const double lat2R = lat2 * DegreesToRadians;
  const double dLatR = (lat2 - lat1) * DegreesToRadians;
  const double dLonR = (lon2 - lon1) * DegreesToRadians;

  // Haversine formula

  const double x = sin(dLatR/2);
  const double y = sin(dLonR/2);

  const double a = (x * x) + cos(lat1R) * cos(lat2R) * (y * y);

  const double c = 2 * atan2(sqrt(a), sqrt(1-a));

  result->distance = R * c;

  // Can we merge in bearing calculation? λ longitude in radians, φ latitude in radians
  
  // const y = Math.sin(λ2-λ1) * Math.cos(φ2);  // cos(lat2) is used for both
  // const x = Math.cos(φ1)*Math.sin(φ2) -      // cos(lat1) is used for both
  //           Math.sin(φ1)*Math.cos(φ2)*Math.cos(λ2-λ1);
  // const θ = Math.atan2(y, x);
  // const brng = (θ*180/Math.PI + 360) % 360; // in degrees

  const double yb = sin(dLonR) * cos(lat2R);
  const double xb = cos(lat1R) * sin(lat2R) - sin(lat1R) * cos(lat2R) * cos(dLonR);

  const double brng = atan2(yb, xb) * 180 / M_PI;

  // TODO may need to remap the output range

  result->bearing = brng;
}


//
//=========================================================================
//
// Always positive MOD operation, used for CPR decoding.
//
int cprModFunction(int a, int b) {
    int res = a % b;
    if (res < 0) res += b;
    return res;
}
//
//=========================================================================
//
// The NL function uses the precomputed table from 1090-WP-9-14
//
int cprNLFunction(double lat) {
    if (lat < 0) lat = -lat; // Table is simmetric about the equator
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}
//
//=========================================================================
//
int cprNFunction(double lat, int fflag) {
    int nl = cprNLFunction(lat) - (fflag ? 1 : 0);
    if (nl < 1) nl = 1;
    return nl;
}
//
//=========================================================================
//
double cprDlonFunction(double lat, int fflag, int surface) {
    return (surface ? 90.0 : 360.0) / cprNFunction(lat, fflag);
}

// Based on code in dump1090

//=========================================================================
//
// This algorithm comes from:
// 1090-WP29-07-Draft_CPR101 (which also defines decodeCPR() )
//
// There is an error in this document related to CPR relative decode.
// Should use trunc() rather than the floor() function in Eq 38 and related for deltaZI.
// floor() returns integer less than argument
// trunc() returns integer closer to zero than argument.
// Note:   text of document describes trunc() functionality for deltaZI calculation
//         but the formulae use floor().
//

// XXX This has been hacked around for testing. Need to restore self-relative calc and reasonableness tests.

// int decodeCPRrelative(struct aircraft *a, int fflag, int surface, double lonRel, double latRel) {
/**
 * latCPR, lonCPR need to be doubles for the expressions in this function to operate correctly, but are integer outside of here
 *
*/
int decodeCPRrelative(aircraft_t *a, uint32_t address, int fflag, int surface, double latRel, double lonRel, double latCPR, double lonCPR) 
{
  double AirDlat;
  double AirDlon;
  // double latCPR;
  // double lonCPR;
  double lonr, latr;  // coords that we are calculating relative to (inputs)
  double rlon, rlat;  // result values (outputs)
  int j, m;
  aircraft_t * contact = a;

  // if (a->bFlags & MODES_ACFLAGS_LATLON_REL_OK) { // Ok to try aircraft relative first
  //     latr = a->lat;
  //     lonr = a->lon;
  // } else if (Modes.bUserFlags & MODES_USER_LATLON_VALID) { // Try ground station relative next
  //     latr = Modes.fUserLat;
  //     lonr = Modes.fUserLon;
  // } else {
  //     return (-1); // Exit with error - can't do relative if we don't have ref.
  // }

  lonr = lonRel;
  latr = latRel;

  if (fflag)
  { // odd
    AirDlat = (surface ? 90.0 : 360.0) / 59.0;
    // latCPR = a->odd_cprlat;
    // lonCPR = a->odd_cprlon;
  }
  else
  { // even
    AirDlat = (surface ? 90.0 : 360.0) / 60.0;
    // latCPR = a->even_cprlat;
    // lonCPR = a->even_cprlon;
  }

  // Compute the Latitude Index "j"
  j = (int)(floor(latr / AirDlat) +
            trunc(0.5 + cprModFunction((int)latr, (int)AirDlat) / AirDlat - latCPR / 131072));
  rlat = AirDlat * (j + latCPR / 131072);
  if (rlat >= 270)
    rlat -= 360;

  // Check to see that the latitude is in range: -90 .. +90
  if (rlat < -90 || rlat > 90)
  {
    // a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
    printf("rlat out of range\n");
    return (-1); // Time to give up - Latitude error
  }

  // Check to see that answer is reasonable - ie no more than 1/2 cell away
  // if (fabs(rlat - a->lat) > (AirDlat/2)) {
  //     // a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
  //     printf("rlat unreasonable\n");
  //     return (-1);                               // Time to give up - Latitude error
  // }

  // Compute the Longitude Index "m"
  AirDlon = cprDlonFunction(rlat, fflag, surface);
  m = (int)(floor(lonr / AirDlon) +
            trunc(0.5 + cprModFunction((int)lonr, (int)AirDlon) / AirDlon - lonCPR / 131072));
  rlon = AirDlon * (m + lonCPR / 131072);
  if (rlon > 180)
    rlon -= 360;

  // Check to see that answer is reasonable - ie no more than 1/2 cell away
  // if (fabs(rlon - a->lon) > (AirDlon/2)) {
  //     // a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
  //     printf("rlon unreasonable\n");
  //     return (-1);                               // Time to give up - Longitude error
  // }

  // a->lat = rlat;
  // a->lon = rlon;

  // a->seenLatLon      = a->seen;
  // a->timestampLatLon = a->timestamp;
  // a->bFlags         |= (MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK);

  distance_bearing_t db;

  // the order of coords doesn't affect the distance, but does affect the bearing.
  // We want the bearing from the receiver to the aircraft, not the aircraft to the receiver.
  calculateDistance(latRel, lonRel, rlat, rlon, &db);

  // float d = db.distance;
  // printf("lat %f lon %f, range %.2fkm (%.2fnm), bearing %.1f degrees\n", rlat, rlon, d, d / 1.852, db.bearing);

  // Do we need to create a new contact?
  if (contact == NULL)
  {
    // First check if we need to make space in the list
    if (contactListIsFull())
    {
      printf("Contact list is full\n");
      // See if the new contact is closer than the most distant existing entry and swap it out if so
      int mostDistantIndex = findMostDistantContact();
      aircraft_t *mostDistantContact =  getContact(mostDistantIndex);
      if (mostDistantContact) {
        if (db.distance < mostDistantContact->range) {
          removeAircraft(mostDistantIndex);
        }
      }
    }
    contact = addAircraft(address);
  }


  if (contact) {
    uint32_t tNow = HAL_GetTick();
    contact->timestamp = tNow;
    contact->timestampLatLon = tNow;

    contact->bearing = db.bearing;
    contact->range = db.distance;
    contact->lat = rlat;
    contact->lon = rlon;
    if (fflag)
    { // odd
      contact->odd_cprlat = latCPR;
      contact->odd_cprlon = lonCPR;
      contact->odd_cprtime = tNow;
    } else { // even
      contact->even_cprlat = latCPR;
      contact->even_cprlon = lonCPR;
      contact->even_cprtime = tNow;
    }
    contact->messages++;
  } // if (we have a contact entry to update)

  return (0);
}

/**
 * initial experiment at decoding gps position messages
 * 
 * @param msgBitstream the bitstream corresponding to the 'message' part of the ADSB data (56 bits long)
 * 
 * +-------+-------+--------+---------+------+------+-------------+-------------+
 * | TC, 5 | SS, 2 | SAF, 1 | ALT, 12 | T, 1 | F, 1 | LAT-CPR, 17 | LON-CPR, 17 |
 * +-------+-------+--------+---------+------+------+-------------+-------------+
*/
void decodePositionMessage(uint32_t address, uint8_t *msgBitstream, uint8_t type)
{
  // uint16_t alt = readNBits(msgBitstream, 8, 12);
  // TODO decode alt depending on 'type' for baro or gps altitude

  uint8_t frameFlag = readNBits(msgBitstream, 21, 1);
  uint32_t latCpr = readNBits(msgBitstream, 22, 17);
  uint32_t lonCpr = readNBits(msgBitstream, 39, 17);

  // printf("alt %u, ff %u latCpr %lx lonCpr %lx\n", alt, frameFlag, latCpr, lonCpr);

  // do we have an existing contact record for this address?
  aircraft_t *aircraft = findAircraft(address);

  // TODO set surface flag based on 'type' (surface are types 5 through 8)
  decodeCPRrelative(aircraft, address, frameFlag, false, LAT, LON, latCpr, lonCpr);
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
    // printf("Address: %06lx Type: %d\n", address, type);

    // 9 to 18 and 20 to 22 are airborne position reports
    if (((type >= 9) && (type <= 18)) || ((type >= 20) && (type <= 22))) {
      decodePositionMessage(address, &(bitstream[32]), type);
    }

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

