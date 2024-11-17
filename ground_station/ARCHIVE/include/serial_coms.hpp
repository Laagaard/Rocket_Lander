/*
File: serial_coms.h
Project: DART
Description: Header file for serial communcation functionality
*/

#include <windows.h> // serial communcation
#include <iostream>

class serial_port
{
public:
    /// @brief serial_port constructor
    serial_port();

    /// @brief Method to close the the serial port handle
    /// @param hSerial The serial port handle to close
    void close(HANDLE hSerial);

    /// @brief Method to read bytes from serial port
    /// @param hSerial Windows serial port HANDLE object
    /// @param n The number of bytes to read
    void read_bytes(HANDLE hSerial, int n);

    /// @brief Method to write bytes to serial port
    /// @param hSerial Windows serial port HANDLE object
    /// @param n The number of bytes to write
    void write_bytes(HANDLE hSerial, int n);
};