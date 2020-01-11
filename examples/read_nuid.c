/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read new NUID from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/greedyhao/rc522_rtt
 * 
 * Example sketch/program showing how to the read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the rt-thread SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the type, and the NUID if a new card has been detected. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 */

#include "mfrc522.h"

static MIFARE_Key key;
static Uid *uid;

// Init array that will store new NUID 
byte nuidPICC[4];

void printHex(byte *buffer, byte bufferSize);
void printDec(byte *buffer, byte bufferSize);

void read_nuid() {
	MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
	PCD_Init(); // Init MFRC522
	uid = get_uid();

	for (byte i = 0; i < 6; i++) {
		key.keyByte[i] = 0xFF;
	}

	rt_kprintf("This code scan the MIFARE Classsic NUID.\n");
	rt_kprintf("Using the following key:");
	printHex(key.keyByte, MF_KEY_SIZE);
	rt_kprintf("\n");

	while (1)
	{
		// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle. And if present, select one.
		if ( ! PICC_IsNewCardPresent() || ! PICC_ReadCardSerial())
		{
			rt_thread_mdelay(50);
			continue;
		}

		rt_kprintf("PICC type: ");
		enum PICC_Type piccType = PICC_GetType(uid->sak);
		rt_kprintf("%s\n", PICC_GetTypeName(piccType));

		// Check is the PICC of Classic MIFARE type
		if (piccType != PICC_TYPE_MIFARE_MINI &&  
			piccType != PICC_TYPE_MIFARE_1K &&
			piccType != PICC_TYPE_MIFARE_4K) {
			rt_kprintf("Your tag is not of type MIFARE Classic.\n");
			return;
		}

		if (uid->uidByte[0] != nuidPICC[0] || 
			uid->uidByte[1] != nuidPICC[1] || 
			uid->uidByte[2] != nuidPICC[2] || 
			uid->uidByte[3] != nuidPICC[3] ) {
			rt_kprintf("A new card has been detected.\n");

			// Store NUID into nuidPICC array
			for (byte i = 0; i < 4; i++) {
				nuidPICC[i] = uid->uidByte[i];
			}

			rt_kprintf("The NUID tag is:\n");
			rt_kprintf("In hex: ");
			printHex(uid->uidByte, uid->size);
			rt_kprintf("\n");
			rt_kprintf("In dec: ");
			printDec(uid->uidByte, uid->size);
			rt_kprintf("\n");
		}
		else rt_kprintf("Card read previously.\n");

		// Halt PICC
		PICC_HaltA();

		// Stop encryption on PCD
		PCD_StopCrypto1();
		break;
	}

	PCD_End();
}
MSH_CMD_EXPORT(read_nuid, "nfc read nuid");

/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
void printHex(byte *buffer, byte bufferSize) {
	for (byte i = 0; i < bufferSize; i++) {
		rt_kprintf(buffer[i] < 0x10 ? " 0" : " ");
		rt_kprintf("%x", buffer[i]);
	}
}

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
void printDec(byte *buffer, byte bufferSize) {
	for (byte i = 0; i < bufferSize; i++) {
		rt_kprintf(buffer[i] < 0x10 ? " 0" : " ");
		rt_kprintf("%d", buffer[i]);
	}
}
