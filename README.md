# ADSB_Alert
Experiments with a ADSB warning device

Inspired by MustardTiger on the ExpressLRS discord (see https://github.com/PicoADSB), I'm trying out a simple ADSB receiver using R820T for the tuner and STM32H7 for the processing. The aim is to create a simple portable ADSB monitor that can alert a pilot to close-by traffic.

Initial prototyping and testing is being done with the R820 beakout board by Eric Brombaugh https://github.com/emeb/r820t2, and a WeAct studio H723 development board https://github.com/WeActStudio/WeActStudio.MiniSTM32H723

Current status: Added message output for consumption by readsb. Contacts confirmed out to 50NM

Preamble and first 5 bits of a DF17 packet. Sampled at 8 msps, dc offset removed, rectified and filtered
![image](https://github.com/JBKingdon/ADSB_Alert/assets/12351913/9e1567e0-9f94-45af-9570-c25a7aae8cf3)

Acknowledgements and Licenses

This project builds on the work of several open source projects whose work is gratefully acknowledged.
I need to gather together the complete list and the Licenses of each. Obviously at the moment this project
is at a very early stage and things are considerably in flux, but I'd like to mention the following 
while I'm getting my act together:

- Code for initialising the R820T from https://github.com/emeb/r820t2
- Code for decoding ADSB message from https://github.com/antirez/dump1090
- The LCD driver and graphics library from https://github.com/deividAlfa/ST7789-STM32-uGUI
- Algorithms for handling coordinates from https://www.movable-type.co.uk/scripts/latlong.html
- Excellent ADSB documentation from https://mode-s.org/decode/index.html
