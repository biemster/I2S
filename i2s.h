/*
    I2SIn and I2SOut for Raspberry Pi Pico
    Implements one or more I2S interfaces using DMA

    Copyright (c) 2022 Earle F. Philhower, III <earlephilhower@yahoo.com>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef __I2S_H__
#define __I2S_H__

#include "audioringbuffer.h"

void I2S_init(PinMode direction);

bool I2S_setBCLK(pin_size_t pin);
bool I2S_setDATA(pin_size_t pin);
bool I2S_setBitsPerSample(int bps);
bool I2S_setBuffers(size_t bufferWords, int32_t silenceSample = 0);
bool I2S_setFrequency(int newFreq);

bool I2S_begin(long sampleRate) {
    I2S_setFrequency(sampleRate);
    return I2S_begin();
}

bool I2S_begin();
void I2S_end();

// from Stream
int I2S_available();
int I2S_read();
int I2S_peek();
void I2S_flush();

size_t I2S_write(const uint8_t *buffer, size_t size);
int I2S_availableForWrite();

// Try and make I2S_write() do what makes sense, namely write
// one sample (L or R) at the I2S configured bit width
size_t I2S_write(uint8_t s) {
    return _writeNatural(s & 0xff);
}
size_t I2S_write(int8_t s) {
    return write((uint8_t)s);
}
size_t I2S_write(uint16_t s) {
    return _writeNatural(s & 0xffff);
}
size_t I2S_write(int16_t s) {
    return write((uint16_t)s);
}
size_t I2S_write(uint32_t s) {
    return _writeNatural(s);
}
size_t I2S_write(int32_t s) {
    return write((uint32_t)s);
}

// Write 32 bit value to port, user responsible for packing/alignment, etc.
size_t I2S_write(int32_t val, bool sync);

// Write sample to I2S port, will block until completed
size_t I2S_write8(int8_t l, int8_t r);
size_t I2S_write16(int16_t l, int16_t r);
size_t I2S_write24(int32_t l, int32_t r); // Note that 24b must have values left-aligned (i.e. 0xABCDEF00)
size_t I2S_write32(int32_t l, int32_t r);

// Read 32 bit value to port, user responsible for packing/alignment, etc.
size_t I2S_read(int32_t *val, bool sync);

// Read samples from I2S port, will block until data available
bool I2S_read8(int8_t *l, int8_t *r);
bool I2S_read16(int16_t *l, int16_t *r);
bool I2S_read24(int32_t *l, int32_t *r); // Note that 24b reads will be left-aligned (see above)
bool I2S_read32(int32_t *l, int32_t *r);

// Note that these callback are called from **INTERRUPT CONTEXT** and hence
// should be in RAM, not FLASH, and should be quick to execute.
void I2S_onTransmit(void(*)(void));
void I2S_onReceive(void(*)(void));


pin_size_t I2S_pinBCLK;
pin_size_t I2S_pinDOUT;
int I2S_bps;
int I2S_freq;
size_t I2S_bufferWords;
int32_t I2S_silenceSample;
bool I2S_isOutput;

bool I2S_running;

bool I2S_hasPeeked;
int32_t I2S_peekSaved;

size_t I2S_writeNatural(int32_t s);
uint32_t I2S_writtenData;
bool I2S_writtenHalf;

int32_t I2S_holdWord = 0;
int I2S_wasHolding = 0;

void (*I2S_cb)();

PIOProgram *I2S_pioprog;
PIO I2S_pio;
int I2S_sm;

#endif // __I2S_H__
