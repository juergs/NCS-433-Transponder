#pragma once

#include <stdint.h>

#define BUFFER_FAIL     0
#define BUFFER_SUCCESS  1

#define BUFFER_SIZE 16 // muss 2^n betragen (8, 16, 32, 64 ...)
#define BUFFER_MASK (BUFFER_SIZE-1) // Klammern auf keinen Fall vergessen

struct Buffer {
    uint64_t data[BUFFER_SIZE];
    uint8_t read; // zeigt auf das Feld mit dem ältesten Inhalt
    uint8_t write; // zeigt immer auf leeres Feld
} buffer = { {}, 0, 0 };

//===============================================================================
// Stellt 1 Byte in den Ringbuffer
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist voll. Es kann kein weiteres Byte gespeichert werden
//     BUFFER_SUCCESS    das Byte wurde gespeichert
//===============================================================================
uint8_t BufferIn(uint64_t item)
{
    uint8_t next = ((buffer.write + 1) & BUFFER_MASK);

    if (buffer.read == next)
        return BUFFER_FAIL; // voll

    printf("Stored: %d\n", buffer.write);

    buffer.data[buffer.write] = item;
    // buffer.data[buffer.write & BUFFER_MASK] = byte; // absolut Sicher
    buffer.write = next;

    return BUFFER_SUCCESS;
}
//===============================================================================
// Holt 1 Byte aus dem Ringbuffer, sofern mindestens eines abholbereit ist
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist leer. Es kann kein Byte geliefert werden.
//     BUFFER_SUCCESS    1 Byte wurde geliefert
//===============================================================================
uint8_t BufferOut(uint64_t *pItem)
{
    if (buffer.read == buffer.write)
        return BUFFER_FAIL;

    printf("buffer.read: [%d] \t", buffer.read); 

    *pItem = buffer.data[buffer.read];    

    buffer.read = (buffer.read + 1) & BUFFER_MASK;

    return BUFFER_SUCCESS;
}
//===============================================================================