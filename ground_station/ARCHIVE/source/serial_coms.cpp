/*
File: serial_coms.cpp
Project: DART
Description: Source file for serial communcation functionality
*/

#include "serial_coms.hpp"

serial_port::serial_port()
{
    HANDLE hSerial; // serial port object

    /*
    1. Serial port to open
    2. Whether to read to write to the port
    3. Will pretty much always be 0
    4. Will pretty much always be 0
    5. Windows should only open an existing file (serial ports already exist)
    6. Tells Windows we don't want to do anything fancy here
    7. Will pretty much always be 0
    */
    hSerial = CreateFile(L"COM9", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

    DCB dcbSerialParams = {0}; // serial port parameters object - clear all fields
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams); // set size parameter

    // Set parameters in use by the serial port
    if (!GetCommState(hSerial, &dcbSerialParams)){
        std::cerr << "Error getting state" << std::endl;
    }

    dcbSerialParams.BaudRate = CBR_19200; // set baud rate
    dcbSerialParams.ByteSize = 8; // set byte size (bits?)
    // Set stop bits?
    // Set parity?

    /*
    If no data is coming into the serial port, attempting to read from the port can cause the application to hang
    while waiting for data to show up. Ways to fix this are to use multithreading (messy) or just tell Windows
    not to wait for data to show up (easier, implemented below).
    */

    COMMTIMEOUTS timeouts = {0}; // clear all fields
    timeouts.ReadIntervalTimeout = 50; // duration (milliseconds) to wait between receiving characters before timeout
    timeouts.ReadTotalTimeoutConstant = 50; // duration (milliseconds) to wait before returning
    timeouts.ReadTotalTimeoutMultiplier = 10; // additional duration (milliseconds) to wait before returning for each byte that was requested in the read operation
    timeouts.WriteTotalTimeoutConstant = 50; // duration (milliseconds) to wait before returning
    timeouts.WriteTotalTimeoutMultiplier = 10; // additional duration (milliseconds) to wait before returning for each byte that was written in the write operation

    // Apply the settings to the serial port
    if (!SetCommTimeouts(hSerial, &timeouts)){
        std::cerr << "Error applying settings to serial port" << std::endl;
    }
}

void serial_port::read_bytes(HANDLE hSerial, int n)
{
    char szBuff[] = {0}; // buffer to store the data in
    DWORD dwBytesRead = 0; // the number of bytes read during the operation

    // Read n bytes from the serial port; show error if unsuccessful
    if (!ReadFile(hSerial, szBuff, n, &dwBytesRead, NULL)){
        std::cerr << "Error reading bytes" << std::endl;
    }
}

void serial_port::write_bytes(HANDLE hSerial, int n)
{
    char szBuff[] = {0}; // buffer to store the data in
    DWORD dwBytesWritten = 0; // the number of bytes read during the operation

    // Write n bytes to the serial port; show error if unsuccessful
    if (!ReadFile(hSerial, szBuff, n, &dwBytesWritten, NULL)){
        std::cerr << "Error reading bytes" << std::endl;
    }
}

void serial_port::close(HANDLE hSerial)
{
    // Failing to close the serial port handle is BAD (like, the port being unavailble until reboot)
    CloseHandle(hSerial);
}