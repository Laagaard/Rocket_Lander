# `static_fire_test`

## Dependencies
- Download [PuTTY](https://www.putty.org/) (SSH and telnet client - used to write live load cell data to PC text file)

## Setup
- PuTTY Configuration:

    - Session Settings:

        1. Connection Type: **Serial**
        2. Serial Line: **COM port connected to Arduino** (e.g., COM9)

    - Logging Settings:

        1. Session Logging: **Printable output** 
        2. Log file name: `{PATH_TO_DESIRED_DIRECTORY}\&Y-&M-&D-&T_static_fire_test_data.txt`
        2. Deselect "Include header"
