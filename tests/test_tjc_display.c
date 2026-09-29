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
    return 0;
}
