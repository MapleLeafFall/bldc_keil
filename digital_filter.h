/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

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

/*
 * digital_filter.h
 *
 *  Created on: 24 nov 2012
 *      Author: benjamin
 */

#ifndef DIGITAL_FILTER_H_
#define DIGITAL_FILTER_H_

#include <stdint.h>

// Functions
void filter_fft(int dir, int m, float *real, float *imag);
void filter_dft(int dir, int len, float *real, float *imag);
void filter_fftshift(float *data, int len);
void filter_hamming(float *data, int len);
void filter_zeroPad(float *data, float *result, int dataLen, int resultLen);
void filter_create_fir_lowpass(float *filter_vector, float f_break, int bits, int use_hamming);
float filter_run_fir_iteration(float *vector, float *filter, int bits, uint32_t offset);
void filter_add_sample(float *buffer, float sample, int bits, uint32_t *offset);

#ifndef M_PI 
#define M_PI       3.14159265358979323846f 
#endif  

#endif /* DIGITAL_FILTER_H_ */
