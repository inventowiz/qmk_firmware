/*
  Copyright 2017 Gabriel Young <gabeplaysdrums@live.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <util/delay.h>
#include "matrix.h"
//#include <print.h>

static matrix_row_t scan_col(void) {
    // Each of the 8 columns is read off pins as below
    //   7  6  5  4  3  2  1  0
    // ,--,--,--,--,--,--,--,--,
    // |D1|C2|D4|D0|B6|B4|C4|C7|
    // `--`--`--`--`--`--`--`--`
    matrix_row_t tmp = (
        (PINC&(1<<7) ? 0 : ((matrix_row_t)1<<0)) |
        (PINC&(1<<4) ? 0 : ((matrix_row_t)1<<1)) |
        (PINB&(1<<4) ? 0 : ((matrix_row_t)1<<2)) |
        (PINB&(1<<6) ? 0 : ((matrix_row_t)1<<3)) |
        (PIND&(1<<0) ? 0 : ((matrix_row_t)1<<4)) |
        (PIND&(1<<4) ? 0 : ((matrix_row_t)1<<5)) |
        (PINC&(1<<2) ? 0 : ((matrix_row_t)1<<6)) |
        (PIND&(1<<1) ? 0 : ((matrix_row_t)1<<7))
    );
    //uprintf("col_val=%d, pinB=%d, pinC=%d, pinD=%d\n",(uint32_t)tmp,(uint8_t)PINB,(uint8_t)PINC,(uint8_t)PIND);
    return tmp;
}                                                //    MM
                                                 //    IOC
static void select_row(uint8_t row) {            //    SSL
    switch (row) {                               //  B OIKA          col mux     mux_bug  extras
        case  0: PORTB = (PORTB & ~0b00101111) | 0b00000111; break; // A A011** A110
        case  1: PORTB = (PORTB & ~0b00101111) | 0b00000000; break; // B N000
        case  2: PORTB = (PORTB & ~0b00101111) | 0b00100000; break; // C B000
        case  3: PORTB = (PORTB & ~0b00101111) | 0b00001000; break; // D N100
        case  4: PORTB = (PORTB & ~0b00101111) | 0b00001100; break; // E N110
        case  5: PORTB = (PORTB & ~0b00101111) | 0b00100010; break; // F B001
        case  6: PORTB = (PORTB & ~0b00101111) | 0b00100110; break; // G B011
        case  7: PORTB = (PORTB & ~0b00101111) | 0b00101010; break; // H B101
        case  8: PORTB = (PORTB & ~0b00101111) | 0b00101110; break; // I B111
        case  9: PORTB = (PORTB & ~0b00101111) | 0b00101100; break; // J B110
        case 10: PORTB = (PORTB & ~0b00101111) | 0b00000001; break; // K A000
        case 11: PORTB = (PORTB & ~0b00101111) | 0b00001110; break; // L N111
        case 12: PORTB = (PORTB & ~0b00101111) | 0b00000011; break; // M A001** A100
        case 13: PORTB = (PORTB & ~0b00101111) | 0b00101000; break; // N B100
        case 14: PORTB = (PORTB & ~0b00101111) | 0b00001001; break; // O A100** A001
        case 15: PORTB = (PORTB & ~0b00101111) | 0b00100100; break; // P B010
        case 16: PORTB = (PORTB & ~0b00101111) | 0b00001111; break; // Q A111
        case 17: PORTB = (PORTB & ~0b00101111) | 0b00000100; break; // R N010
        case 18: PORTB = (PORTB & ~0b00101111) | 0b00001011; break; // S A101         CONN_29
        case 19: PORTB = (PORTB & ~0b00101111) | 0b00001010; break; // T N101         CONN_11
        case 20: PORTB = (PORTB & ~0b00101111) | 0b00000110; break; // U N011         CONN_9
        case 21: PORTB = (PORTB & ~0b00101111) | 0b00000010; break; // V N001         CONN_7
        case 22: PORTB = (PORTB & ~0b00101111) | 0b00000101; break; // W A010         CONN_33
        case 23: PORTB = (PORTB & ~0b00101111) | 0b00000111; break; // X A011** A110  CONN_27
    }
    //uprintf("row %d selected: ",row);
}

void matrix_init_custom(void) {
    /* Row output pins & Column input pins */ 
    // DDR, low for input, high output direction
    // PORT high/low write default state
    
    // outputs
    DDRB  |= 0b10101111;
    DDRC  |= 0b01100000;
    
    // inputs
    DDRB  &=  ~0b01010000;
    DDRC  &=  ~0b10010100;
    DDRD  &=  ~0b00010011;

    // Set pull ups on all inputs
    PORTB |=  0b01010000;
    PORTC |=  0b10010100;
    PORTD |=  0b00010011;

    //print("matrix init complete\n");
}

// matrix is 24 uint8_t.
// we select the row (one of 24), then read the column
bool matrix_scan_custom(matrix_row_t current_matrix[]) {

    bool has_changed = false;
    
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        matrix_row_t orig = current_matrix[row];
        select_row(row);
        _delay_us(3);
        current_matrix[row] = scan_col();
        has_changed |= (orig != current_matrix[row]);
    }
    return has_changed;
}
