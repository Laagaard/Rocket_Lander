/*
File: spp.hpp
Project: DART
Description: Header for Space Packet Protocol struct & helper functions
*/

#pragma once

#include <bitset>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <vector>

#define BIT_PERS_BYTE                   8

#define PACKET_VERSION_NUMBER_BITS      3
#define PACKET_TYPE_BITS                1
#define SECONDARY_HEADER_FLAG_BITS      1
#define APID_BITS                       11
#define SEQUENCE_FLAG_BITS              2
#define PACKET_SEQUENCE_COUNT_BITS      14
#define PACKET_DATA_LENGTH_BITS         16

#define PRIMARY_HEADER_BITS PACKET_VERSION_NUMBER_BITS + PACKET_TYPE_BITS + SECONDARY_HEADER_FLAG_BITS + APID_BITS + SEQUENCE_FLAG_BITS + PACKET_SEQUENCE_COUNT_BITS + PACKET_DATA_LENGTH_BITS

#define TIME_CODE_BITS                  64

#define CMD_USER_DATA_FIELD_BITS        1