#include "app.h"

#include "stdio.h"
#include <stdint.h>
#include <sys/_intsup.h>

#include "board.h"

void app(void)
{
    setSeparator('*', 30);
    printf("%s", name_project);
    setSeparator('=', 30);
    printf("%s", info_project);

    bsp_init();
    bsp_radio_init();

    uint8_t fw_major = 0;
    uint8_t fw_minor = 0;
    bsp_radio_get_version(&fw_major, &fw_minor);

    setSeparator('-', 30);
    printf("Major: 0x%02x\n", fw_major);
    printf("Minor: 0x%02x\n", fw_minor);
    
    while(1)
    {

    }
}

// MyFunc -------------------------------------------------
void setSeparator(char ch, uint8_t qnt)
{
    while(qnt)
    {
        printf("%c",ch);
        qnt--;
    }
    printf("\n");
}