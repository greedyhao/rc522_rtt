/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program to test your firmware.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * This example test the firmware of your MFRC522 reader module, only known version can be checked. If the test passed
 * it do not mean that your module is faultless! Some modules have bad or broken antennas or the PICC is broken.
 * NOTE: for more informations read the README.rst
 * 
 * @author Rotzbua
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 */

#include "mfrc522.h"

void firmware_check() {
    MFRC522(MFRC522_SS_PIN, MFRC522_RST_PIN);
    PCD_Init();
    rt_thread_mdelay(4);
    rt_kprintf("*****************************\n");
    rt_kprintf("MFRC522 Digital self test\n");
    rt_kprintf("*****************************\n");
    PCD_DumpVersionToSerial();  // Show version of PCD - MFRC522 Card Reader
    rt_kprintf("-----------------------------\n");
    rt_kprintf("Only known versions supported\n");
    rt_kprintf("-----------------------------\n");
    rt_kprintf("Performing test...\n");
    bool result = PCD_PerformSelfTest(); // perform the test
    rt_kprintf("-----------------------------\n");
    rt_kprintf("Result: \n");
    if (result)
        rt_kprintf("OK\n");
    else
        rt_kprintf("DEFECT or UNKNOWN\n");
    rt_kprintf("\n");
    PCD_End();
}
MSH_CMD_EXPORT(firmware_check, "nfc firmware check");
