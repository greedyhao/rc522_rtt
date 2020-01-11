/**
 * ----------------------------------------------------------------------------
 * This is a MFRC522 library example; see https://github.com/greedyhao/rc522_rtt
 * for further details and other examples.
 * 
 * NOTE: The library file mfrc522.h has a lot of useful info. Please read it.
 * 
 * Released into the public domain.
 * ----------------------------------------------------------------------------
 * This sample shows how to setup blocks on a MIFARE Classic PICC (= card/tag)
 * to be in "Value Block" mode: in this mode the operations Increment/Decrement,
 * Restore and Transfer can be used.
 * 
 * BEWARE: Data will be written to the PICC, in sector #1 (blocks #4 to #7).
 */

#include "mfrc522.h"

static Uid *uid;
static MIFARE_Key key;
static void formatValueBlock(byte blockAddr);
static void dump_byte_array(byte *buffer, byte bufferSize);

void mifare_classic_value_block() {
    MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
	PCD_Init(); // Init MFRC522
	uid = get_uid();

    // Prepare the key (used both as key A and as key B)
    // using FFFFFFFFFFFFh which is the default at chip delivery from the factory
    for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }

    rt_kprintf("Scan a MIFARE Classic PICC to demonstrate Value Block mode.\n");
    rt_kprintf("Using key (for A and B):");
    dump_byte_array(key.keyByte, MF_KEY_SIZE);
    rt_kprintf("\nBEWARE: Data will be written to the PICC, in sector #1\n");

    while (1)
    {
        // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle. And if present, select one.
		if ( ! PICC_IsNewCardPresent() || ! PICC_ReadCardSerial())
		{
			rt_thread_mdelay(50);
			continue;
		}

        // Show some details of the PICC (that is: the tag/card)
        rt_kprintf("Card UID:");
        dump_byte_array(uid->uidByte, uid->size);
        rt_kprintf("\nPICC type: ");
        enum PICC_Type piccType = PICC_GetType(uid->sak);
        rt_kprintf("%s\n", PICC_GetTypeName(piccType));

        // Check for compatibility
        if (    piccType != PICC_TYPE_MIFARE_MINI
            &&  piccType != PICC_TYPE_MIFARE_1K
            &&  piccType != PICC_TYPE_MIFARE_4K) {
            rt_kprintf("This sample only works with MIFARE Classic cards.\n");
            return;
        }

        // In this sample we use the second sector,
        // that is: sector #1, covering block #4 up to and including block #7
        byte sector         = 1;
        byte valueBlockA    = 5;
        byte valueBlockB    = 6;
        byte trailerBlock   = 7;
        enum StatusCode status;
        byte buffer[18];
        byte size = sizeof(buffer);
        int32_t value;

        // Authenticate using key A
        rt_kprintf("Authenticating using key A...\n");
        status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, uid);
        if (status != STATUS_OK) {
            rt_kprintf("PCD_Authenticate() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }

        // Show the whole sector as it currently is
        rt_kprintf("Current data in sector:\n");
        PICC_DumpMifareClassicSectorToSerial(uid, &key, sector);
        rt_kprintf("\n");

        // We need a sector trailer that defines blocks 5 and 6 as Value Blocks and enables key B
        // The last block in a sector (block #3 for Mifare Classic 1K) is the Sector Trailer.
        // See http://www.nxp.com/documents/data_sheet/MF1S503x.pdf sections 8.6 and 8.7:
        //      Bytes 0-5:   Key A
        //      Bytes 6-8:   Access Bits
        //      Bytes 9:     User data
        //      Bytes 10-15: Key B (or user data)
        byte trailerBuffer[] = {
            255, 255, 255, 255, 255, 255,       // Keep default key A
            0, 0, 0,
            0,
            255, 255, 255, 255, 255, 255};      // Keep default key B
        // The access bits are stored in a peculiar fashion.
        // There are four groups:
        //      g[0]    Access bits for block 0 (for sectors 0-31)
        //              or blocks 0-4 (for sectors 32-39)
        //      g[1]    Access bits for block 1 (for sectors 0-31)
        //              or blocks 5-9 (for sectors 32-39)
        //      g[2]    Access bits for block 2 (for sectors 0-31)
        //              or blocks 10-14 (for sectors 32-39)
        //      g[3]    Access bits for the Sector Trailer: block 3 (for sectors 0-31)
        //              or block 15 (for sectors 32-39)
        // Each group has access bits [C1 C2 C3], in this code C1 is MSB and C3 is LSB.
        // Determine the bit pattern needed using MIFARE_SetAccessBits:
        //      g0=0    access bits for block 0 (of this sector) using [0 0 0] = 000b = 0
        //              which means key A|B have r/w for block 0 of this sector
        //              which (in this example) translates to block #4 within sector #1;
        //              this is the transport configuration (at factory delivery).
        //      g1=6    access bits for block 1 (of this sector) using [1 1 0] = 110b = 6
        //              which means block 1 (of this sector) is used as a value block,
        //              which (in this example) translates to block #5 within sector #1;
        //              where key A|B have r, key B has w, key B can increment,
        //              and key A|B can decrement, transfer, and restore.
        //      g2=6    same thing for block 2 (of this sector): set it to a value block;
        //              which (in this example) translates to block #6 within sector #1;
        //      g3=3    access bits for block 3 (of this sector): the Sector Trailer here;
        //              using [0 1 1] = 011b = 3 which means only key B has r/w access
        //              to the Sector Trailer (block 3 of this sector) from now on
        //              which (in this example) translates to block #7 within sector #1;
        MIFARE_SetAccessBits(&trailerBuffer[6], 0, 6, 6, 3);

        // Read the sector trailer as it is currently stored on the PICC
        rt_kprintf("Reading sector trailer...\n");
        status = MIFARE_Read(trailerBlock, buffer, &size);
        if (status != STATUS_OK) {
            rt_kprintf("MIFARE_Read() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }
        // Check if it matches the desired access pattern already;
        // because if it does, we don't need to write it again...
        if (    buffer[6] != trailerBuffer[6]
            ||  buffer[7] != trailerBuffer[7]
            ||  buffer[8] != trailerBuffer[8]) {
            // They don't match (yet), so write it to the PICC
            rt_kprintf("Writing new sector trailer...\n");
            status = MIFARE_Write(trailerBlock, trailerBuffer, 16);
            if (status != STATUS_OK) {
                rt_kprintf("MIFARE_Write() failed: ");
                rt_kprintf("%s\n", GetStatusCodeName(status));
                return;
            }
        }

        // Authenticate using key B
       rt_kprintf("Authenticating again using key B...\n");
        status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &key, uid);
        if (status != STATUS_OK) {
            rt_kprintf("PCD_Authenticate() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }

        // A value block has a 32 bit signed value stored three times
        // and an 8 bit address stored 4 times. Make sure that valueBlockA
        // and valueBlockB have that format (note that it will only format
        // the block when it doesn't comply to the expected format already).
        formatValueBlock(valueBlockA);
        formatValueBlock(valueBlockB);

        // Add 1 to the value of valueBlockA and store the result in valueBlockA.
        rt_kprintf("Adding 1 to value of block %d\n", valueBlockA);
        status = MIFARE_Increment(valueBlockA, 1);
        if (status != STATUS_OK) {
            rt_kprintf("MIFARE_Increment() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }
        status = MIFARE_Transfer(valueBlockA);
        if (status != STATUS_OK) {
            rt_kprintf("MIFARE_Transfer() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }
        // Show the new value of valueBlockA
        status = MIFARE_GetValue(valueBlockA, &value);
        if (status != STATUS_OK) {
            rt_kprintf("mifare_GetValue() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }
        rt_kprintf("New value of value block %d = %d\n", valueBlockA, value);

        // Decrement 10 from the value of valueBlockB and store the result in valueBlockB.
        rt_kprintf("Subtracting 10 from value of block %d\n", valueBlockB);
        status = MIFARE_Decrement(valueBlockB, 10);
        if (status != STATUS_OK) {
            rt_kprintf("MIFARE_Decrement() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }
        status = MIFARE_Transfer(valueBlockB);
        if (status != STATUS_OK) {
            rt_kprintf("MIFARE_Transfer() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }
        // Show the new value of valueBlockB
        status = MIFARE_GetValue(valueBlockB, &value);
        if (status != STATUS_OK) {
            rt_kprintf("mifare_GetValue() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
            return;
        }
        rt_kprintf("New value of value block %d\n", valueBlockB);
        rt_kprintf(" = %d\n", value);
        // Check some boundary...
        if (value <= -100) {
            rt_kprintf("Below -100, so resetting it to 255 = 0xFF just for fun...\n");
            status = MIFARE_SetValue(valueBlockB, 255);
            if (status != STATUS_OK) {
                rt_kprintf("mifare_SetValue() failed: ");
                rt_kprintf("%s\n", GetStatusCodeName(status));
                return;
            }
        }

        // Dump the sector data
        PICC_DumpMifareClassicSectorToSerial(uid, &key, sector);
        rt_kprintf("\n");

        // Halt PICC
        PICC_HaltA();
        // Stop encryption on PCD
        PCD_StopCrypto1();
        break;
    }

    PCD_End();
}
MSH_CMD_EXPORT(mifare_classic_value_block, "nfc mifare classic value block");

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
static void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        rt_kprintf(buffer[i] < 0x10 ? " 0" : " ");
        rt_kprintf("%x", buffer[i]);
    }
}

/**
 * Ensure that a given block is formatted as a Value Block.
 */
static void formatValueBlock(byte blockAddr) {
    byte buffer[18];
    byte size = sizeof(buffer);
    enum StatusCode status;

    rt_kprintf("Reading block %d\n", blockAddr);
    status = MIFARE_Read(blockAddr, buffer, &size);
    if (status != STATUS_OK) {
        rt_kprintf("MIFARE_Read() failed: ");
        rt_kprintf("%s\n", GetStatusCodeName(status));
        return;
    }

    if (    (buffer[0] == (byte)~buffer[4])
        &&  (buffer[1] == (byte)~buffer[5])
        &&  (buffer[2] == (byte)~buffer[6])
        &&  (buffer[3] == (byte)~buffer[7])

        &&  (buffer[0] == buffer[8])
        &&  (buffer[1] == buffer[9])
        &&  (buffer[2] == buffer[10])
        &&  (buffer[3] == buffer[11])

        &&  (buffer[12] == (byte)~buffer[13])
        &&  (buffer[12] ==        buffer[14])
        &&  (buffer[12] == (byte)~buffer[15])) {
       rt_kprintf("Block has correct Value Block format.\n");
    }
    else {
       rt_kprintf("Formatting as Value Block...\n");
        byte valueBlock[] = {
            0, 0, 0, 0,
            255, 255, 255, 255,
            0, 0, 0, 0,
            blockAddr, ~blockAddr, blockAddr, ~blockAddr };
        status = MIFARE_Write(blockAddr, valueBlock, 16);
        if (status != STATUS_OK) {
            rt_kprintf("MIFARE_Write() failed: ");
            rt_kprintf("%s\n", GetStatusCodeName(status));
        }
    }
}
