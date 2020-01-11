/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example to change UID of changeable MIFARE card.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/greedyhao/rc522_rtt
 * 
 * This sample shows how to set the UID on a UID changeable MIFARE card.
 * NOTE: for more informations read the README.rst
 */

#include "mfrc522.h"

/* Set your new UID here! */
#define NEW_UID {0xDE, 0xAD, 0xBE, 0xEF}

static Uid *uid;
static MIFARE_Key key;

void change_uid() {
    MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
    PCD_Init();		// Init MFRC522
    uid = get_uid();
    rt_kprintf("Warning: this example overwrites the UID of your UID changeable card, use with care!\n");

    // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }

    while (1) {
        // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle. And if present, select one.
        if ( ! PICC_IsNewCardPresent() || ! PICC_ReadCardSerial() ) {
            rt_thread_mdelay(50);
            continue;
        }

        // Now a card is selected. The UID and SAK is in uid.

        rt_kprintf("Card UID:");
        for (byte i = 0; i < uid->size; i++) {
            rt_kprintf(uid->uidByte[i] < 0x10 ? " 0" : " ");
            rt_kprintf("%x", uid->uidByte[i]);
        }
        rt_kprintf("\n");

        // // Dump PICC type
        // enum PICC_Type piccType = PICC_GetType(uid.sak);
        // rt_kprintf("PICC type: ");
        // rt_kprintf(PICC_GetTypeName(piccType));
        // rt_kprintf(" (SAK ");
        // rt_kprintf(uid.sak);
        // rt_kprintf(")\r\n");
        // if (    piccType != PICC_TYPE_MIFARE_MINI 
        //     &&  piccType != PICC_TYPE_MIFARE_1K
        //     &&  piccType != PICC_TYPE_MIFARE_4K) {
        //     rt_kprintf("This sample only works with MIFARE Classic cards.\n");
        //     break;
        // }

        // Set new UID
        byte newUid[] = NEW_UID;
        if ( MIFARE_SetUid(newUid, (byte)4, true) ) {
            rt_kprintf("Wrote new UID to card.\n");
        }

        // Halt PICC and re-select it so DumpToSerial doesn't get confused
        PICC_HaltA();
        if ( ! PICC_IsNewCardPresent() || ! PICC_ReadCardSerial() ) {
            return;
        }

        // Dump the new memory contents
        rt_kprintf("New UID and contents:\n");
        PICC_DumpToSerial(uid);

        break;
    }

    PCD_End();
}
MSH_CMD_EXPORT(change_uid, "nfc change uid");
