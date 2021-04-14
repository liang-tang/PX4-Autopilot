/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

class AP_WindVane_NMEA
{
public:
    // constructor
    AP_WindVane_NMEA();
    virtual ~AP_WindVane_NMEA();

    // try and decode NMEA message
    bool decode(char c);
    float get_wind_speed() { return _wind_speed_ms; }
    float get_wind_dir() { return _wind_dir_deg; }

private:
    // decode each term
    bool decode_latest_term();

    // convert from char to hex value for checksum
    int16_t char_to_hex(char a);

    // latest values read in
    float _wind_speed_ms;
    float _wind_dir_deg;

    char _term[15];            // buffer for the current term within the current sentence
    uint8_t _term_offset;      // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;      // term index within the current sentence
    uint8_t _checksum;         // checksum accumulator
    bool _term_is_checksum;    // current term is the checksum
    bool _sentence_valid;      // is current sentence valid so far
    bool _sentence_done;       // true if this sentence has already been decoded
};

