/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read data from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/greedyhao/rc522_rtt
 * 
 * Example sketch/program showing how to read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the ID/UID, type and any data blocks it can read. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * If your reader supports it, this sketch/program will read all the PICCs presented (that is: multiple tag reading).
 * So if you stack two or more PICCs on top of each other and present them to the reader, it will first output all
 * details of the first and then the next PICC. Note that this may take some time as all data blocks are dumped, so
 * keep the PICCs at reading distance until complete.
 */

#include "mfrc522.h"

static Uid *uid;

void dump_info() {
    MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
    PCD_Init();		// Init MFRC522
    rt_thread_mdelay(4);
    PCD_DumpVersionToSerial();
    rt_kprintf("Scan PICC to see UID, SAK, type, and data blocks...\n");

    uid = get_uid();

    while (1)
	{
        rt_thread_mdelay(500);

        // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle. And if present, select one.
		if ( ! PICC_IsNewCardPresent() || ! PICC_ReadCardSerial())
		{
			rt_thread_mdelay(50);
			continue;
		}

        // Dump debug info about the card; PICC_HaltA() is automatically called
	    PICC_DumpToSerial(uid);
        break;
    }

    PCD_End();
}
MSH_CMD_EXPORT(dump_info, "nfc dump info");
