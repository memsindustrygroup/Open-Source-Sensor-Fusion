The initial open source sensor fusion release is based upon Freescale's commercial product at http://www.freescale.com/sensorfusion.
The Sources directory is a proper subset of that product, minus the I2C, UART & GPIO drivers, as well as the MQX Lite RTOS.  All of that 
code is generated via the Freescale Processor Expert Tool.  The Freescale kit contains multiple templates corresponding to different 
hardware sets available at the URL above.   To keep things simple, we did not duplicate all of that material here.  You can download
Freescale templates and Kinetis Design Studio IDE (both of which are also free) and generate the missing files.

Alternately, we will be providing guidance with regard to creating ports to alternate RTOSs.
