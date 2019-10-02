# madivaru-lib
General purpose C libraries especially for bare metal embedded systems.
NOTE! The C++ part is heavily under construction.

## CLI API

* mdv_cli_api.h
* mdv_cli_api.c

**A generic application programming interface for creating command line 
interfaces (CLI).**

This library can be used to create and control an input parser for user defined 
CLI commands and their parameters. It interprets an input stream and acts on the 
commands by invoking their associated callback functions. The host application 
handles the callback functions and performs the actions.

## Sensor Framework

* mdv_sensor_api.h
* mdv_sensor_api.c

**A generic application programming interface and sensor driver definition for 
applications using different types sensors.**

This library can be used to write unified interface drivers for different types 
of sensors and to manage them by using a simple API. The drivers, the API, and a 
sensor I/O and management service this library forms a framework. The framework 
is flexible and can be used to manage sensors from simple ones with single input 
and output to complex entities with multiple inputs and outputs, calibration, 
configuration, and separate processes.

## Serial port API

* mdv_serialport.h
* mdv_serialport.c

**A simple generic driver definition and API for serial port communication.**

This library can be used to write unified interface drivers for universal 
asynchronous receiver transmitters (UART) and to use them via a simple API. 
Using the serial port library helps implementation of host applications that 
are not aware of the detailed implementation of the UART in embedded systems or 
the serial port implementation in personal computers.

## SPI API

* mdv_spi_api.h
* mdv_spi_api.c

**A simple generic driver definition and API for synchronous peripheral 
interface (SPI) communication.**

This library can be used to write unified interface drivers for SPI devices and 
to use them via a simple API. Using the SPI library helps implementation of host 
applications that are not aware of the defailted implementation of the SPI port 
peripheral.

## Utilities

### Average16

* mdv_average16.h
* mdv_average16.c

**A library for 16-bit average value calculations.**

This library supports two modes of average value calculations, a *contiguous* 
mode and a *floating* mode. 

The **contiguous mode** uses a buffer of *n* samples to calculate average from a 
contiguous signal. This can be used also a simple low-pass filter.

The **floating mode** calculates an average value for all input samples since 
the reset of the calculator.

### CRC-16

* mdv_crc16.h
* mdv_crc16.c

**A library for 16-bit cyclic redundancy check (CRC) value calculations.**

This library supports calculation of 16-bit CRC values in both single and 
continued modes and for both single byte and buffered calculations. 

The CRC calculation is performed by using the **CRC-16-IBM** polynomial 
(0xA001).

### CRC-32

* mdv_crc32.h
* mdv_crc32.c

**A library for 32-bit cyclic redundancy check (CRC) value calculations.**

This library supports calculation of 32-bit CRC values in both single and 
continued modes. The calculation is performed by using a look-up table.

### FIFO

* mdv_fifo.h
* mdv_fifo.c

**A library for creating and managing custom data first-in, first-out (FIFO) 
buffers.**

This library supports FIFOs of any type and size of data up to 65,535 bytes of 
size per data item. The FIFO can contain up to 4,294,967,295 data items 
(32-bit).

### Software timer

* mdv_timer.h
* mdv_timer.c
* mdv_timer_system.h
* mdv_timer_system.c

**A library for creating and managing software timers with different time 
bases.**

This library can be used to create unlimited amount of software timers with 
different time bases (timer systems). 

A timer system is ticked by a system timer, for example a timer overflow 
interrupt. The frequency of the software timer (the time base) depends on the 
timer tick frequency. The timer is a simple structure that gets a time stamp 
from the current timer counter when "started". The counter is advanced (ticked) 
continuously on the background by the system timer. When compared with the 
current timer counter value, the timer results into the time lapse between the 
initial state of time and the current state of time. This result can be used to 
create delays without stopping the whole application. The timer system can
detect timer stuck errors (e.g. timer interrupt not running properly).
