/*
    AudioRingBuffer for Rasperry Pi Pico
    Implements a ring buffer for PIO DMA for I2S read or write

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

#define NARB 8

#pragma once

typedef enum PinMode {INPUT,OUTPUT} PinMode;

void ARB_init(size_t bufferWords, int32_t silenceSample, PinMode direction);
void ARB_deinit();

void ARB_setCallback(void (*fn)());

bool ARB_begin(int dreq, volatile void *pioFIFOAddr);
void ARB_dmaConfig(int channel, int dreq, volatile void *pioFIFOAddr);

bool ARB_write(uint32_t v, bool sync);
bool ARB_read(uint32_t *v, bool sync);
void ARB_flush();

bool ARB_getOverUnderflow();
int ARB_available();

void ARB_dmaIRQ(int channel);
static void ARB_irq();

typedef struct {
    uint32_t *buff;
    volatile bool empty;
} AudioBuffer;

AudioBuffer** ARB_currbuffers;
AudioBuffer* ARB1_buffers[NARB];
volatile int ARB1_curBuffer;
volatile int ARB1_nextBuffer;
AudioBuffer* ARB2_buffers[NARB];
volatile int ARB2_curBuffer;
volatile int ARB2_nextBuffer;

bool ARB_running = false;
size_t ARB_chunkSampleCount;
int ARB_bitsPerSample;
size_t ARB_wordsPerBuffer;
size_t ARB_bufferCount;
bool ARB_isOutput;
int32_t ARB_silenceSample;
int ARB1_channelDMA;
int ARB2_channelDMA;
void (*ARB_callback)();

bool ARB_overunderflow;

// User buffer pointer
int ARB_userBuffer = -1;
size_t ARB_userOff = 0;
