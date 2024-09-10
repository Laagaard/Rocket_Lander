/*
File: spp.cpp
Project: DART
Description: Source for Space Packet Protocol struct & helper functions
Source: https://public.ccsds.org/Pubs/133x0b2e1.pdf
*/

#include "spp.hpp"

struct primary_header_t
{
    std::bitset<PACKET_VERSION_NUMBER_BITS> packet_version_number;
    std::bitset<PACKET_TYPE_BITS> packet_type;
    std::bitset<SECONDARY_HEADER_FLAG_BITS> secondary_header_flag;
    std::bitset<APID_BITS> apid;
    std::bitset<SEQUENCE_FLAG_BITS> sequence_flags;
    std::bitset<PACKET_SEQUENCE_COUNT_BITS> packet_sequence_count;
    std::bitset<PACKET_DATA_LENGTH_BITS> packet_data_length;
};

struct user_data_field_cmd_t
{
    std::bitset<1> abort_bit;
};

struct data_field_cmd_t
{
    std::bitset<TIME_CODE_BITS> time_code;
    user_data_field_cmd_t user_data;
};

struct cmd_packet_t
{
    primary_header_t primary_header;
    data_field_cmd_t data;
};

std::string assemble_cmd_packet(data_field_cmd_t data, int sequence_count, std::bitset<1> abort_bit)
{
    primary_header_t primary_header;
    primary_header.packet_version_number = {000}; // required value by SPP standard
    primary_header.packet_type = {1}; // required value for CMD packets as defined by SPP standard
    primary_header.secondary_header_flag = {1}; // 1 - secondary header present
    primary_header.apid = {1}; // 1 - indicates packet recipient is the DART vehicle
    primary_header.sequence_flags = {11}; // 11 - indicates packet contains unsegmented user data
    primary_header.packet_sequence_count = sequence_count; // indicates packet order
    primary_header.packet_data_length = sizeof(data) - 1; // definition per SPP standard

    std::chrono::time_point time_now = std::chrono::system_clock::now(); // current system time
    auto time_now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(time_now); // convert to `std::chrono::milliseconds` object
    long long ms = time_now_ms.time_since_epoch().count() % 1000; // obtain milliseconds from end of timestamp
    std::time_t today_time = std::chrono::system_clock::to_time_t(time_now); // convert to `std::time_t` object
    // std::cout << std::put_time(std::localtime(&today_time), "%m/%d/%y %H:%M:%S.") << ms << std::endl;

    data.time_code = std::bitset<TIME_CODE_BITS> {time_now_ms.time_since_epoch().count()};
    data.user_data.abort_bit = abort_bit;

    std::bitset<PRIMARY_HEADER_BITS + TIME_CODE_BITS + CMD_USER_DATA_FIELD_BITS> cmd_packet;

    std::string primary_header_string = primary_header.packet_version_number.to_string() + primary_header.packet_type.to_string() + primary_header.secondary_header_flag.to_string() + primary_header.apid.to_string() + primary_header.sequence_flags.to_string() + primary_header.packet_sequence_count.to_string() + primary_header.packet_data_length.to_string();
    std::string data_field_string = data.time_code.to_string() + data.user_data.abort_bit.to_string();

    std::string cmd_packet_string = primary_header_string + data_field_string;

    return cmd_packet_string;
}