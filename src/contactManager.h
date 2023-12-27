#ifndef _CONTACT_MGR

#define _CONTACT_MGR

#define CONTACT_MAX_AGE 60000

#include <stdint.h>
#include "decoder.h"

aircraft_t * findAircraft(uint32_t address);
int findOldestContact();
int findMostDistantContact();
bool contactListIsFull();
aircraft_t * addAircraft(uint32_t address);
void removeAircraft(uint32_t address);
int32_t getNumContacts();
aircraft_t * getContact(uint32_t index);

#endif