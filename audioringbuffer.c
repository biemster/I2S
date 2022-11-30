/*
    AudioRingBuffer for Raspnerry Pi Pico RP2040
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

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pio_i2s.pio.h"
#include "audioringbuffer.h"

void ARB_init(size_t bufferWords, int32_t silenceSample, PinMode direction) {
    ARB_running = false;
    ARB_silenceSample = silenceSample;
    ARB_wordsPerBuffer = bufferWords;
    ARB_isOutput = (direction == OUTPUT);
    ARB_overunderflow = false;
    ARB_callback = NULL;
    ARB_userBuffer = -1;
    ARB_userOff = 0;
    for (size_t i = 0; i < NARB; i++) {
        ARB1_buffers[i]->buff = malloc(ARB_wordsPerBuffer * sizeof(uint32_t));
        ARB2_buffers[i]->buff = malloc(ARB_wordsPerBuffer * sizeof(uint32_t));
        ARB1_buffers[i]->empty = true;
        ARB2_buffers[i]->empty = true;
    }
}

void ARB_deinit() {
    if (ARB_running) {
        dma_channel_set_irq0_enabled(ARB1_channelDMA, false);
        dma_channel_set_irq0_enabled(ARB2_channelDMA, false);
        dma_channel_unclaim(ARB1_channelDMA);
        dma_channel_unclaim(ARB2_channelDMA);

        irq_set_enabled(DMA_IRQ_0, false);
        // TODO - how can we know if there are no other parts of the core using DMA0 IRQ??
        irq_remove_handler(DMA_IRQ_0, ARB_irq);
    }
}

void ARB_setCallback(void (*fn)()) {
    ARB_callback = fn;
}

bool ARB_begin(int dreq, volatile void *pioFIFOAddr) {
    ARB_running = true;
    // Set all buffers to silence, empty
    for (int i = 0; i < NARB; i++) {
        ARB1_buffers[i]->empty = true;
        ARB2_buffers[i]->empty = true;
        if (ARB_isOutput) {
            for (uint32_t x = 0; x < ARB_wordsPerBuffer; x++) {
                ARB1_buffers[i]->buff[x] = ARB_silenceSample;
                ARB2_buffers[i]->buff[x] = ARB_silenceSample;
            }
        }
    }

    // Get ping and pong DMA channels
    ARB1_channelDMA = dma_claim_unused_channel(true);
    if(ARB1_channelDMA == -1) {
        return false;
    }
    ARB2_channelDMA = dma_claim_unused_channel(true);
    if(ARB2_channelDMA == -1) {
        dma_channel_unclaim(ARB1_channelDMA);
        return false;
    }

    ARB_dmaConfig(ARB1_channelDMA);
    ARB_dmaConfig(ARB2_channelDMA);

    irq_add_shared_handler(DMA_IRQ_0, ARB_irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    ARB1_curBuffer = 0;
    ARB2_curBuffer = 0;
    ARB1_nextBuffer = 2 % ARB_bufferCount;
    ARB2_nextBuffer = 2 % ARB_bufferCount;

    dma_channel_start(ARB1_channelDMA);
    dma_channel_start(ARB2_channelDMA);

    return true;
}

void ARB_dmaConfig(int channel) {
    dma_channel_config c = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32); // 32b transfers into PIO FIFO
    if(ARB_isOutput) {
        channel_config_set_read_increment(&c, true); // Reading incrementing addresses
        channel_config_set_write_increment(&c, false); // Writing to the same FIFO address
    }
    else {
        channel_config_set_read_increment(&c, false); // Reading same FIFO address
        channel_config_set_write_increment(&c, true); // Writing to incrememting buffers
    }
    channel_config_set_dreq(&c, dreq); // Wait for the PIO TX FIFO specified
    channel_config_set_chain_to(&c, (channel == ARB1_channelDMA) ? ARB2_channelDMA : ARB1_channelDMA); // Start other channel when done
    channel_config_set_irq_quiet(&c, false); // Need IRQs

    if(ARB_isOutput) {
        dma_channel_configure(channel, &c, pioFIFOAddr, (channel == ARB1_channelDMA) ? ARB1_buffers[0]->buff : ARB2_buffers[0]->buff, ARB_wordsPerBuffer, false);
    } else {
        dma_channel_configure(channel, &c, (channel == ARB1_channelDMA) ? ARB1_buffers[0]->buff : ARB2_buffers[0]->buff, pioFIFOAddr, ARB_wordsPerBuffer, false);
    }
    dma_channel_set_irq0_enabled(channel, true);
}

bool ARB_write(uint32_t v, bool sync) {
    int ARB_curBuffer = (ARB_currbuffers == ARB1_buffers) ? ARB1_curBuffer : ARB2_curBuffer;
    int ARB_nextBuffer = (ARB_currbuffers == ARB1_buffers) ? ARB1_nextBuffer : ARB2_nextBuffer;
    if (!ARB_running || !ARB_currbuffers || !ARB_isOutput) {
        return false;
    }
    if (ARB_userBuffer == -1) {
        // First write or overflow, pick spot 2 buffers out
        ARB_userBuffer = (ARB_nextBuffer + 2) % ARB_bufferCount;
        ARB_userOff = 0;
    }
    if (!ARB_currbuffers[ARB_userBuffer]->empty) {
        if (!sync) {
            return false;
        } else {
            while (!ARB_currbuffers[ARB_userBuffer]->empty) {
                /* noop busy wait */
            }
        }
    }
    if (ARB_userBuffer == ARB_curBuffer) {
        if (!sync) {
            return false;
        } else {
            while (ARB_userBuffer == ARB_curBuffer) {
                /* noop busy wait */
            }
        }
    }
    ARB_currbuffers[ARB_userBuffer]->buff[ARB_userOff++] = v;
    if (ARB_userOff == ARB_wordsPerBuffer) {
        ARB_currbuffers[ARB_userBuffer]->empty = false;
        ARB_userBuffer = (ARB_userBuffer + 1) % ARB_bufferCount;
        ARB_userOff = 0;
    }
    return true;
}

bool ARB_read(uint32_t *v, bool sync) {
    int ARB_curBuffer = (ARB_currbuffers == ARB1_buffers) ? ARB1_curBuffer : ARB2_curBuffer;
    if (!ARB_running || !ARB_currbuffers || ARB_isOutput) {
        return false;
    }
    if (ARB_userBuffer == -1) {
        // First write or overflow, pick last filled buffer
        ARB_userBuffer = (ARB_curBuffer - 1 + ARB_bufferCount) % ARB_bufferCount;
        ARB_userOff = 0;
    }
    if (ARB_currbuffers[ARB_userBuffer]->empty) {
        if (!sync) {
            return false;
        } else {
            while (ARB_currbuffers[ARB_userBuffer]->empty) {
                /* noop busy wait */
            }
        }
    }
    if (ARB_userBuffer == ARB_curBuffer) {
        if (!sync) {
            return false;
        } else {
            while (ARB_userBuffer == ARB_curBuffer) {
                /* noop busy wait */
            }
        }
    }
    uint32_t* ret = ARB_currbuffers[ARB_userBuffer]->buff[ARB_userOff++];
    if (ARB_userOff == ARB_wordsPerBuffer) {
        ARB_currbuffers[ARB_userBuffer]->empty = true;
        ARB_userBuffer = (ARB_userBuffer + 1) % ARB_bufferCount;
        ARB_userOff = 0;
    }
    *v = ret;
    return true;
}

bool ARB_getOverUnderflow() {
    bool hold = ARB_overunderflow;
    ARB_overunderflow = false;
    return hold;
}

int ARB_available() {
    if (!ARB_running) {
        return 0;
    }
    int avail;
    int ARB_curBuffer = (ARB_currbuffers == ARB1_buffers) ? ARB1_curBuffer : ARB2_curBuffer;
    avail = ARB_wordsPerBuffer - ARB_userOff;
    avail += ((ARB_bufferCount + ARB_curBuffer - ARB_userBuffer) % ARB_bufferCount) * ARB_wordsPerBuffer;
    return avail;
}

void ARB_flush() {
    int ARB_curBuffer = (ARB_currbuffers == ARB1_buffers) ? ARB1_curBuffer : ARB2_curBuffer;
    while (ARB_curBuffer != ARB_userBuffer) {
        // busy wait
    }
}

void __not_in_flash_func(ARB_dmaIRQ)(int channel) {
    ARB_currbuffers = (channel == ARB1_channelDMA) ? ARB1_buffers : ARB2_buffers;
    int ARB_curBuffer = (channel == ARB1_channelDMA) ? ARB1_curBuffer : ARB2_curBuffer;
    int ARB_nextBuffer = (channel == ARB1_channelDMA) ? ARB1_nextBuffer : ARB2_nextBuffer;
    if (ARB_isOutput) {
        for (uint32_t x = 0; x < ARB_wordsPerBuffer; x++) {
            ARB_currbuffers[ARB_curBuffer]->buff[x] = ARB_silenceSample;
        }
        ARB_currbuffers[ARB_curBuffer]->empty = true;
        ARB_overunderflow = ARB_overunderflow | ARB_currbuffers[ARB_nextBuffer]->empty;
        dma_channel_set_read_addr(channel, ARB_currbuffers[ARB_nextBuffer]->buff, false);
    } else {
        ARB_currbuffers[ARB_curBuffer]->empty = false;
        ARB_overunderflow = ARB_overunderflow | !ARB_currbuffers[ARB_nextBuffer]->empty;
        dma_channel_set_write_addr(channel, ARB_currbuffers[ARB_nextBuffer]->buff, false);
    }
    dma_channel_set_trans_count(channel, ARB_wordsPerBuffer, false);

    if(channel == ARB1_channelDMA) {
        ARB1_curBuffer = (ARB1_curBuffer + 1) % ARB_bufferCount;
        ARB1_nextBuffer = (ARB1_nextBuffer + 1) % ARB_bufferCount;
    }
    else {
        ARB2_curBuffer = (ARB1_curBuffer + 1) % ARB_bufferCount;
        ARB2_nextBuffer = (ARB1_nextBuffer + 1) % ARB_bufferCount;
    }

    dma_channel_acknowledge_irq0(channel);
    if (ARB_callback) {
        ARB_callback();
    }
}

void __not_in_flash_func(ARB_irq)() {
    if (dma_channel_get_irq0_status(ARB1_channelDMA)) {
        ARB_dmaIRQ(ARB1_channelDMA);
    }
    if (dma_channel_get_irq0_status(ARB2_channelDMA)) {
        ARB_dmaIRQ(ARB2_channelDMA);
    }
}
