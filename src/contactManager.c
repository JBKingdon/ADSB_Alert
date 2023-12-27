// Maintain the list of contacts

#include <stddef.h>
#include <string.h>
#include <stm32h7xx_hal.h>
#include <stdio.h>
#include "decoder.h"
#include "contactManager.h"

aircraft_t contacts[MAX_CONTACTS];
int num_contacts = 0;

int32_t getNumContacts()
{
    return num_contacts;
}

/**
 * return the aircraft_t struct for the aircraft with the given index
 * 
 * @return the aircraft_t struct for the aircraft with the given index or NULL if the index is invalid
*/
aircraft_t * getContact(uint32_t index)
{
    if (index < num_contacts) {
        return &contacts[index];
    }
    return NULL;
}

/**
 * Return the index of the oldest contact in the list
 * 
 * TODO This will need more work if we need to cope with wrapping of the timestamps
*/
int findOldestContact()
{
    int32_t oldest = -1;
    uint32_t oldestTime = 0xFFFFFFFF;

    for(int i = 0; i < num_contacts; i++)
    {
        if (contacts[i].timestamp < oldestTime) {
            oldestTime = contacts[i].timestamp;
            oldest = i;
        }
    }
    return oldest;
}

/**
 * return the index of the most distant aircraft in the list
 * 
 * @return the index of the most distant aircraft in the list or -1 if the list is empty
*/
int findMostDistantContact()
{
    int32_t mostDistant = -1;
    float mostDistantDistance = 0;
    for(int i = 0; i < num_contacts; i++)
    {
        if (contacts[i].range > mostDistantDistance)
        {
            mostDistantDistance = contacts[i].range;
            mostDistant = i;
        }
    }
    return mostDistant;
}

/**
 * Find an aircraft in the list of contacts
 * 
 * Also removes at most one aged contact from the list.
 * 
 * @param address The address of the aircraft to find
 * @return A pointer to the aircraft_t struct for the aircraft, or NULL if not found.
*/
aircraft_t * findAircraft(uint32_t address)
{
    aircraft_t *res = NULL;
    int32_t oldContactIndex = -1;
    const uint32_t tNow = HAL_GetTick();

    for(int i = 0; i < num_contacts; i++)
    {
        if(contacts[i].addr == address) {
            res = &contacts[i];
            break;
        } else if (contacts[i].timestamp + CONTACT_MAX_AGE < tNow) {
            oldContactIndex = i;
        }
    }

    if (oldContactIndex != -1) {
        // We found an old contact, so we remove it
        // printf("removing contact %lu of %d\n", oldContactIndex, num_contacts);
        for(int i = oldContactIndex; i < num_contacts-1; i++)
        {
            contacts[i] = contacts[i+1];
        }
        num_contacts--;
    }

    return res;
}

/**
 * return true if the list is full
*/
bool contactListIsFull()
{
    bool result = (num_contacts >= MAX_CONTACTS);
    return result;
}

/**
 * Add a new aircraft to the list of contacts
 * 
 * The returned aircraft_t is initialised with the given address
 * 
 * Doesn't check for duplicates, so the caller is responsible
 * 
 * 
 * @param address The address of the aircraft to be added
 * @return The aircraft_t struct for the new aircraft, or NULL if the list is full.
*/
aircraft_t * addAircraft(uint32_t address)
{
    if (num_contacts < MAX_CONTACTS) {
        uint32_t newAircraft = num_contacts++;

        // clear all the fields of the struct before re-using it
        memset(&contacts[newAircraft], 0, sizeof(aircraft_t));

        contacts[newAircraft].addr = address;
        return &contacts[newAircraft];
    }

    // return null to indicate failure
    return NULL;
}

/**
 * Remove a given aircraft from the list of contacts
 * 
 * TODO this could be optimised with a single memcpy
*/
void removeAircraft(uint32_t address)
{
    bool found = false;
    for(int i = 0; i < num_contacts; i++)
    {
        if (found) {
            // We already found the match, so now we shuffle all the entries down by one
            contacts[i-1] = contacts[i];
        } else if (contacts[i].addr == address) {
            // We found the match, so now we set the flag to start shuffling down the entries
            found = true;
        }
    }

    // Decrement the number of contacts
    if (found) {
        num_contacts--;
    }
}