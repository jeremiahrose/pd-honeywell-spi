pd-honeywell-spi
================

Pure Data external for Honeywell TruStability SPI pressure sensors.

Defines a Pure Data object called 'honeywell_spi' that retrieves 
pressure, temperature and status readings from a Honeywell HSC pressure 
sensor with an SPI interface. 

Tested on the 1 Psi HSCDANT001PGSA3 but should work with other Honeywell SPI sensors. 
No calculations or adjustments are done and the output provided is the pure digital 
data coming the sensor's onboard analog to digital converter.

To build run run 'make' from inside the pd-honeywell-spi directory, then copy the
resulting pd object to your Pure Data path.

License: GPLv3.0
