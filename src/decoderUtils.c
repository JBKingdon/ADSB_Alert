/**
 * A collection of helper functions from dump1090 by Salvatore Sanfilippo
 * See https://github.com/antirez/dump1090
 * 
 * License info from dump1090:
        // Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
        //
        // All rights reserved.
        //
        // Redistribution and use in source and binary forms, with or without
        // modification, are permitted provided that the following conditions are
        // met:
        //
        //  *  Redistributions of source code must retain the above copyright
        //     notice, this list of conditions and the following disclaimer.
        //
        //  *  Redistributions in binary form must reproduce the above copyright
        //     notice, this list of conditions and the following disclaimer in the
        //     documentation and/or other materials provided with the distribution.
        //
        // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        // "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        // LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
        // A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
        // HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
        // SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
        // LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
        // DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
        // THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
        // (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
        // OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



//=========================================================================
//
// Input format is : 00:A4:A2:A1:00:B4:B2:B1:00:C4:C2:C1:00:D4:D2:D1
//
int ModeAToModeC(unsigned int ModeA)
{
    unsigned int FiveHundreds = 0;
    unsigned int OneHundreds = 0;

    if ((ModeA & 0xFFFF888B)            // D1 set is illegal. D2 set is > 62700ft which is unlikely
        || ((ModeA & 0x000000F0) == 0)) // C1,,C4 cannot be Zero
    {
        return -9999;
    }

    if (ModeA & 0x0010)
    {
        OneHundreds ^= 0x007;
    } // C1
    if (ModeA & 0x0020)
    {
        OneHundreds ^= 0x003;
    } // C2
    if (ModeA & 0x0040)
    {
        OneHundreds ^= 0x001;
    } // C4

    // Remove 7s from OneHundreds (Make 7->5, snd 5->7).
    if ((OneHundreds & 5) == 5)
    {
        OneHundreds ^= 2;
    }

    // Check for invalid codes, only 1 to 5 are valid
    if (OneHundreds > 5)
    {
        return -9999;
    }

    // if (ModeA & 0x0001) {FiveHundreds ^= 0x1FF;} // D1 never used for altitude
    if (ModeA & 0x0002)
    {
        FiveHundreds ^= 0x0FF;
    } // D2
    if (ModeA & 0x0004)
    {
        FiveHundreds ^= 0x07F;
    } // D4

    if (ModeA & 0x1000)
    {
        FiveHundreds ^= 0x03F;
    } // A1
    if (ModeA & 0x2000)
    {
        FiveHundreds ^= 0x01F;
    } // A2
    if (ModeA & 0x4000)
    {
        FiveHundreds ^= 0x00F;
    } // A4

    if (ModeA & 0x0100)
    {
        FiveHundreds ^= 0x007;
    } // B1
    if (ModeA & 0x0200)
    {
        FiveHundreds ^= 0x003;
    } // B2
    if (ModeA & 0x0400)
    {
        FiveHundreds ^= 0x001;
    } // B4

    // Correct order of OneHundreds.
    if (FiveHundreds & 1)
    {
        OneHundreds = 6 - OneHundreds;
    }

    return ((FiveHundreds * 5) + OneHundreds - 13);
}
