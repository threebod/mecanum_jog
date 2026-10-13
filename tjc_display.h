#ifndef MECANUM_JOG_TJC_DISPLAY_H
#define MECANUM_JOG_TJC_DISPLAY_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TJC_DISPLAY_COMMAND_MAX 80U

/* page0.t0/t1/t2 must be text fields. The screen project must use UTF-8. */
static uint8_t tjc_display_text_command(uint8_t field, const char *value,
                                       uint8_t command[TJC_DISPLAY_COMMAND_MAX])
{
    static const char prefix[] = "page0.t0.txt=\"";
    uint8_t length = 0U;
    uint8_t index;

    if (field > 2U) return 0U;
    for (index = 0U; prefix[index] != '\0'; ++index)
        command[length++] = (uint8_t)prefix[index];
    command[7] = (uint8_t)('0' + field);
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

static uint8_t tjc_display_qr_command(const char *value,
                                     uint8_t command[TJC_DISPLAY_COMMAND_MAX])
{
    char display[18];
    if (strlen(value) != 15U || value[3] != '+' ||
        value[7] != '+' || value[11] != '+')
        return tjc_display_text_command(0U, value, command);
    /* Keep the middle '+' at the end of the first line. */
    memcpy(display, value, 8U);
    display[8] = '\r';
    display[9] = '\n';
    memcpy(display + 10U, value + 8U, 8U);
    return tjc_display_text_command(0U, display, command);
}

static uint8_t tjc_display_count_command(uint8_t field, uint16_t count,
                                        uint8_t command[TJC_DISPLAY_COMMAND_MAX])
{
    char value[32];
    /* UTF-8: 正确抓取数： / 正确放置数：, independent of compiler source encoding. */
    const char *label = field == 1U ?
        "\xE6\xAD\xA3\xE7\xA1\xAE\xE6\x8A\x93\xE5\x8F\x96\xE6\x95\xB0\xEF\xBC\x9A" :
        "\xE6\xAD\xA3\xE7\xA1\xAE\xE6\x94\xBE\xE7\xBD\xAE\xE6\x95\xB0\xEF\xBC\x9A";
    if (field != 1U && field != 2U) return 0U;
    snprintf(value, sizeof(value), "%s%u", label, (unsigned)count);
    return tjc_display_text_command(field, value, command);
}

#endif
