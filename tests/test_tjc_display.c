#include <assert.h>
#include <string.h>

#include "../tjc_display.h"

int main(void)
{
    uint8_t command[TJC_DISPLAY_COMMAND_MAX];
    static const uint8_t expected[] = {
        'p','a','g','e','0','.','t','0','.','t','x','t','=','"',
        '4','2','6','"',0xFFU,0xFFU,0xFFU
    };
    uint8_t length = tjc_display_qr_command("426", command);
    assert(length == sizeof(expected));
    assert(memcmp(command, expected, sizeof(expected)) == 0);

    length = tjc_display_qr_command("a\"b\\c", command);
    assert(length == sizeof(expected) + 2U);
    assert(memcmp(command + 14U, "a?b?c", 5U) == 0);
    assert(command[length - 1U] == 0xFFU);

    {
        char task[] = "156+123+516+231";
        static const char twoLines[] = "page0.t0.txt=\"156+123+\r\n516+231\"\xFF\xFF\xFF";
        static const char shortQr[] = "page0.t0.txt=\"123+321\"\xFF\xFF\xFF";
        length = tjc_display_qr_command(task, command);
        assert(length == sizeof(twoLines) - 1U);
        assert(memcmp(command, twoLines, length) == 0);
        assert(strcmp(task, "156+123+516+231") == 0);
        length = tjc_display_qr_command("123+321", command);
        assert(length == sizeof(shortQr) - 1U);
        assert(memcmp(command, shortQr, length) == 0);
    }

    {
        static const char grab[] = "page0.t1.txt=\""
            "\xE6\xAD\xA3\xE7\xA1\xAE\xE6\x8A\x93\xE5\x8F\x96\xE6\x95\xB0\xEF\xBC\x9A"
            "0\"\xFF\xFF\xFF";
        static const char place[] = "page0.t2.txt=\""
            "\xE6\xAD\xA3\xE7\xA1\xAE\xE6\x94\xBE\xE7\xBD\xAE\xE6\x95\xB0\xEF\xBC\x9A"
            "12\"\xFF\xFF\xFF";
        length = tjc_display_count_command(1U, 0U, command);
        assert(length == sizeof(grab) - 1U);
        assert(memcmp(command, grab, length) == 0);
        length = tjc_display_count_command(2U, 12U, command);
        assert(length == sizeof(place) - 1U);
        assert(memcmp(command, place, length) == 0);
        length = tjc_display_count_command(1U, 65535U, command);
        assert(length == sizeof(grab) + 3U);
        assert(memcmp(command + 32U, "65535\"\xFF\xFF\xFF", 9U) == 0);
        assert(tjc_display_count_command(0U, 1U, command) == 0U);
        assert(tjc_display_count_command(3U, 1U, command) == 0U);
    }
    return 0;
}
