/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program to fix a broken UID changeable MIFARE cards.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/greedyhao/rc522_rtt
 * 
 * This sample shows how to fix a broken UID changeable MIFARE cards that have a corrupted sector 0.
 */

#include "mfrc522.h"

void fix_brickeduid() {
    MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
	PCD_Init(); // Init MFRC522
    rt_kprintf("Warning: this example clears your mifare UID, use with care!\n");

    while (1)
    {
        if ( MIFARE_UnbrickUidSector(false) ) {
            rt_kprintf("Cleared sector 0, set UID to 1234. Card should be responsive again now.\n");
            break;
        }
        rt_thread_mdelay(1000);
    }
    PCD_End();
}
MSH_CMD_EXPORT(fix_brickeduid, "nfc fix bricked uid");
