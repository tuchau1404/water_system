#ifndef SIMComModuleTc_H //if not define
#define SIMComModuleTc_H
#define TINY_GSM_MODEM_SIM7600       // Modem is SIM7600  TINY_GSM_MODEM_SIM7600g
#include <TinyGsmClient.h> 
#include <ThingSpeak.h>
#include "OLEDModule.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// gsm pins
#define MODEM_PWR_KEY        14
#define MODEM_TX            5
#define MODEM_RX             18

//#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
#define SerialAT Serial1

void initializeSIMCom();
void runSIMCom();
#endif
