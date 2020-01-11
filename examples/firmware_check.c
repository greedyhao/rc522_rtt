/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program to test your firmware.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/greedyhao/rc522_rtt
 * 
 * This example test the firmware of your MFRC522 reader module, only known version can be checked. If the test passed
 * it do not mean that your module is faultless! Some modules have bad or broken antennas or the PICC is broken.
 * NOTE: for more informations read the README.rst
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
