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

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pio_i2s.pio.h"
#include "i2s.h"


void I2S_init(PinMode direction) {
    I2S_running = false;
    I2S_bps = 16;
    I2S_writtenHalf = false;
    I2S_holdWord = 0;
    I2S_wasHolding = 0;
    I2S_isOutput = direction == OUTPUT;
    I2S_pinBCLK = 26;
    I2S_pinDOUT = 28;
#ifdef PIN_I2S_BCLK
    I2S_pinBCLK = PIN_I2S_BCLK;
#endif

#ifdef PIN_I2S_DOUT
    if (I2S_isOutput) {
        I2S_pinDOUT = PIN_I2S_DOUT;
    }
#endif

#ifdef PIN_I2S_DIN
    if (!I2S_isOutput) {
        I2S_pinDOUT = PIN_I2S_DIN;
    }
#endif
    I2S_freq = 48000;
    I2S_cb = NULL;
    I2S_bufferWords = 16;
    I2S_silenceSample = 0;

}

bool I2S_setBCLK(uint pin) {
    if (I2S_running || (pin > 28)) {
        return false;
    }
    I2S_pinBCLK = pin;
    return true;
}

bool I2S_setDATA(uint pin) {
    if (I2S_running || (pin > 29)) {
        return false;
    }
    I2S_pinDOUT = pin;
    return true;
}

bool I2S_setBitsPerSample(int bps) {
    if (I2S_running || ((bps != 8) && (bps != 16) && (bps != 24) && (bps != 32))) {
        return false;
    }
    I2S_bps = bps;
    return true;
}

bool I2S_setBuffers(size_t bufferWords, int32_t silenceSample) {
    if (I2S_running || (bufferWords < 8)) {
        return false;
    }
    I2S_bufferWords = bufferWords;
    I2S_silenceSample = silenceSample;
    return true;
}

bool I2S_setFrequency(int newFreq) {
    I2S_freq = newFreq;
    if (I2S_running) {
        float bitClk = I2S_freq * I2S_bps * 2.0 /* channels */ * 2.0 /* edges per clock */;
        pio_sm_set_clkdiv(I2S_pio, I2S_sm, (float)clock_get_hz(clk_sys) / bitClk);
    }
    return true;
}

void I2S_onTransmit(void(*fn)(void)) {
    if (I2S_isOutput) {
        I2S_cb = fn;
        if (I2S_running) {
            ARB_setCallback(I2S_cb);
        }
    }
}

void I2S_onReceive(void(*fn)(void)) {
    if (!I2S_isOutput) {
        I2S_cb = fn;
        if (I2S_running) {
            ARB_setCallback(I2S_cb);
        }
    }
}

bool I2S_begin() {
    I2S_running = true;
    I2S_hasPeeked = false;
    int off = 0;
    if (I2S_isOutput) {
        pio_i2s_out_program_init(I2S_pio, I2S_sm, off, I2S_pinDOUT, I2S_pinBCLK, I2S_bps);
    } else {
        pio_i2s_in_program_init(I2S_pio, I2S_sm, off, I2S_pinDOUT, I2S_pinBCLK, I2S_bps);
    }
    I2S_setFrequency(I2S_freq);
    if (I2S_bps == 8) {
        uint8_t a = I2S_silenceSample & 0xff;
        I2S_silenceSample = (a << 24) | (a << 16) | (a << 8) | a;
    } else if (I2S_bps == 16) {
        uint16_t a = I2S_silenceSample & 0xffff;
        I2S_silenceSample = (a << 16) | a;
    }
    ARB_init(I2S_bufferWords, I2S_silenceSample, I2S_isOutput ? OUTPUT : INPUT);
    ARB_begin(pio_get_dreq(I2S_pio, I2S_sm, I2S_isOutput), I2S_isOutput ? &I2S_pio->txf[I2S_sm] : (volatile void*)&I2S_pio->rxf[I2S_sm]);
    ARB_setCallback(I2S_cb);
    pio_sm_set_enabled(I2S_pio, I2S_sm, true);

    return true;
}

void I2S_end() {
    I2S_running = false;
}

size_t I2S_write(int32_t val, bool sync) {
    if (!I2S_running || !I2S_isOutput) {
        return 0;
    }
    return ARB_write(val, sync);
}

size_t I2S_write8(int8_t l, int8_t r) {
    if (!I2S_running || !I2S_isOutput) {
        return 0;
    }
    int16_t o = (l << 8) | (r & 0xff);
    return write((int16_t) o);
}

size_t I2S_write16(int16_t l, int16_t r) {
    if (!I2S_running || !I2S_isOutput) {
        return 0;
    }
    int32_t o = (l << 16) | (r & 0xffff);
    return write((int32_t)o, true);
}

size_t I2S_write24(int32_t l, int32_t r) {
    return write32(l, r);
}

size_t I2S_write32(int32_t l, int32_t r) {
    if (!I2S_running || !I2S_isOutput) {
        return 0;
    }
    write((int32_t)l);
    write((int32_t)r);
    return 1;
}

size_t I2S_read(int32_t *val, bool sync) {
    if (!I2S_running || I2S_isOutput) {
        return 0;
    }
    return ARB_read((uint32_t *)val, sync);
}

bool I2S_read8(int8_t *l, int8_t *r) {
    if (!I2S_running || I2S_isOutput) {
        return false;
    }
    if (I2S_wasHolding) {
        *l = (I2S_holdWord >> 8) & 0xff;
        *r = (I2S_holdWord >> 0) & 0xff;
        I2S_wasHolding = 0;
    } else {
        read(&I2S_holdWord, true);
        I2S_wasHolding = 16;
        *l = (I2S_holdWord >> 24) & 0xff;
        *r = (I2S_holdWord >> 16) & 0xff;
    }
    return true;
}

bool I2S_read16(int16_t *l, int16_t *r) {
    if (!I2S_running || I2S_isOutput) {
        return false;
    }
    int32_t o;
    read(&o, true);
    *l = (o >> 16) & 0xffff;
    *r = (o >> 0) & 0xffff;
    return true;
}

bool I2S_read24(int32_t *l, int32_t *r) {
    if (!I2S_running || I2S_isOutput) {
        return false;
    }
    read32(l, r);
    // 24-bit samples are read right-aligned, so left-align them to keep the binary point between 33.32
    *l <<= 8;
    *r <<= 8;
    return true;
}

bool I2S_read32(int32_t *l, int32_t *r) {
    if (!I2S_running || I2S_isOutput) {
        return false;
    }
    read(l, true);
    read(r, true);
    return true;
}

int I2S_availableForWrite() {
    if (!I2S_running || !I2S_isOutput) {
        return 0;
    }
    return ARB_available();
}
