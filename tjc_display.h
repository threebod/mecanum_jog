#ifndef MECANUM_JOG_TJC_DISPLAY_H
#define MECANUM_JOG_TJC_DISPLAY_H

#include <stdint.h>

#define TJC_DISPLAY_COMMAND_MAX 80U

/* page0.t0 is the text field used by test/lcd_ttl. The screen project must use UTF-8. */
static uint8_t tjc_display_qr_command(const char *value,
                                      uint8_t command[TJC_DISPLAY_COMMAND_MAX])
{
    static const char prefix[] = "page0.t0.txt=\"";
    uint8_t length = 0U;
    uint8_t index;

    for (index = 0U; prefix[index] != '\0'; ++index)
        command[length++] = (uint8_t)prefix[index];
    while (*value != '\0') {
        uint8_t byte = (uint8_t)*value++;
        if (length >= TJC_DISPLAY_COMMAND_MAX - 4U) return 0U;
        /* Quotes, backslashes and FF can corrupt the screen command. */
        if (byte == '"' || byte == '\\' || byte == 0xFFU) byte = '?';
        command[length++] = byte;
    }
    command[length++] = '"';
    command[length++] = 0xFFU;
    command[length++] = 0xFFU;
    command[length++] = 0xFFU;
    return length;
}

#endif
