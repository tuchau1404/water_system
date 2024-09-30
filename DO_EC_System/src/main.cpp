
/**
 * @file main.cpp
 * @author Bùi Ngọc Long
 * @date 2024-04-17
 * @brief Short description of what this file does.
 *
 * Long description if needed...
 * 
 * @license MIT License
 */

#include <Arduino.h>
#include <OLEDModule.h>
#include <RTCModule.h>
#include <SDCardModule.h>
#include "YosemitechModbus.h"
#include "WifiModule.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ECTDS10_Example.h"
#include "SIMComModule.h"
#include "OLEDModule.h"
#include "esp_sleep.h"

#define uS_TO_S_FACTOR 1000000  // Conversion factor for microseconds to seconds
#define TIME_TO_SLEEP  60      // Time ESP32 will sleep (in seconds)

const long interval = 60000;  // ms
SemaphoreHandle_t   DO_sensor_read_done, 
                    EC_sensor_read_done,
                    upload_data_done,
                    oled_ready,
                    oled_init,
                    data_mutex;

// volatile struct 
// {
//     float DO_doValue;
//     float DO_tempValue;
//     float DO_doPercentValue;
//     float EC_temp;
//     uint16_t EC_salanity,EC_ec,EC_tds;
// }shared_data;
SensorData shared_data;

SoftwareSerial modbus_Serial(16,17);    


void DOSensor(void *pvParameters);
void ECSensor(void *pvParameters);
void RTCandSD(void *pvParameters);
void WIFI(void *pvParameters);
void OLED(void *pvParameters);
void BatteryShow(void *pvParameters);
void SIMCom(void *pvParameters);
void runTasks();
void setupDeepSleep();

void setup() {
    Serial.begin(115200);
    setupDeepSleep();
    runTasks();
    
  
    
 }


void loop() 
{
 
}




void DOSensor(void *pvParameters)
{
    pinMode(2,OUTPUT);//turn on Gate MOSFet
    digitalWrite(2,HIGH);
    vTaskDelay(1000/portTICK_PERIOD_MS);   //delay 4s, 1s for testing
    //initializeDOSensors();
    yosemitech sensor;
    yosemitechModel model =Y504;
    modbus_Serial.begin(9600);
    while(1)
    {
        
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        
        sensor.begin(model, 0x01, &modbus_Serial, -1);
    
        sensor.startMeasurement();
        float doValue, doTemp,doPercent;
        sensor.getValues(doPercent,doTemp,doValue);
        Serial.print("Do Value: ");    Serial.println(doValue);
        Serial.print("Temperature Value: ");    Serial.println(doTemp);
        Serial.println("------------------------");

        shared_data.DO_doValue =doValue;
        shared_data.DO_tempValue=doTemp;

        vTaskDelay(1000/portTICK_PERIOD_MS);
        xSemaphoreGive(data_mutex);
        xSemaphoreGive(DO_sensor_read_done);
    }
    
    vTaskDelay(interval/portTICK_PERIOD_MS);   
    }
}

void ECSensor(void *pvParameters)
{
    ectds10 ecSensor;
    while(1)
    {
        if (xSemaphoreTake(DO_sensor_read_done, portMAX_DELAY) == pdTRUE) 
        {  
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                
               
                bool success = ecSensor.begin(0x03, &modbus_Serial);

                float ec_temp;
                uint16_t ec_salanity,ec_ec,ec_tds;

                ecSensor.getValues(ec_temp,ec_ec,ec_salanity,ec_tds);

                Serial.print("EC Value: ");    Serial.print(ec_ec); Serial.println(" uS/cm");
                Serial.print("EC Temperature Value: ");    Serial.print(ec_temp); Serial.println(" oC");
                Serial.println("------------------------");
                shared_data.EC_ec       = ec_ec;
                shared_data.EC_temp     = ec_temp;
                shared_data.EC_salanity = ec_salanity;
                shared_data.EC_tds      = ec_tds;
                xSemaphoreGive(data_mutex);
                xSemaphoreGive(EC_sensor_read_done);

            }
            
        }
    }
}

void Wifi(void *pvParameters)
{
    initializeWiFi();
    while(1)
    {
        //run after task DO
        if (xSemaphoreTake(EC_sensor_read_done, portMAX_DELAY) == pdTRUE) 
        {
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                
                    connectWifi();
                    SensorData dataThingSpeak;
                    // dataThingSpeak.DO_doValue = shared_data.DO_doValue;
                    // dataThingSpeak.DO_tempValue= shared_data.DO_tempValue;
                    
                    dataThingSpeak.DO_doValue = 10;
                    dataThingSpeak.DO_tempValue= 20;
                    sendDataToThingSpeak(dataThingSpeak);
                    disconnectWifi();
                 
                    xSemaphoreGive(data_mutex);
                    xSemaphoreGive(upload_data_done);
            }
        }
    }
    
}

void RTCandSD(void *pvParameters)
{
    initializeSDCard();
    initializeRTC();
    while(1)
    {
        if (xSemaphoreTake(upload_data_done, portMAX_DELAY) == pdTRUE) 
        {
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                // Get Date Time
                DateTime now = getCurrentDateTime();
                Serial.printf("Date time is : %d-%02d-%02d %02d:%02d:%02d", now.year + 2000, now.month, now.day, now.hour, now.minute, now.second);
                char fileName[50], dataString[50];
                sprintf(fileName, "/%04d-%02d-%02d.txt", now.year+2000, now.month, now.day);
                sprintf(dataString, "%04d-%02d-%02d,%02d-%02d-%02d,%0.02f,%0.02f",
                now.year + 2000, now.month, now.day, now.hour, now.minute, now.second,
                shared_data.DO_doValue, shared_data.DO_tempValue);
                Serial.print("\nFile name is: ");    Serial.println(fileName);
                Serial.print("Data content: ");    Serial.println(dataString); 
                Serial.println("------------------------------------------");
                //  check and create new file if necessary
                checkAndCreateFile(fileName);

                //  save data to file
                appendToFile(fileName, dataString);
                xSemaphoreGive(data_mutex);
                xSemaphoreGive(oled_ready);
            }
        }
       
    
    }
}
void SIMCom(void *pvParameters)
{
    initializeSIMCom();
    while(1)
    {
        if (xSemaphoreTake(EC_sensor_read_done, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) 
            {
                runSIMCom();
                // SensorData dataThingSpeak;
                // dataThingSpeak.DO_doValue = shared_data;
                // dataThingSpeak.DO_tempValue=
                // dataThingSpeak.EC_
                sendDataToThingSpeak(shared_data);
                
                
    
                xSemaphoreGive(data_mutex);
                xSemaphoreGive(upload_data_done);  
            }
        }
        
    }

}

void OLED(void *pvParameters)
{
    initializeOLED();
    xSemaphoreGive(oled_init);
    while(1)
    {
        
        // show OLED
        if (xSemaphoreTake(upload_data_done, portMAX_DELAY) == pdTRUE) 
        {    
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) 
            {
            
                displaySensorData(shared_data);
                //displayBatteryVoltage(1000);
                digitalWrite(2,LOW);
                clearLine(55);
                display.setTextSize(1); 
                display.setCursor(0,55);
                display.print("Going to sleep now...");
                display.display();
                vTaskDelay(2000/portTICK_PERIOD_MS);
                esp_deep_sleep_start();
                xSemaphoreGive(data_mutex); 
            }
        }
        
    
    } 
}

void BatteryShow(void *pvParameters)
{
    vTaskDelay(2000/portTICK_PERIOD_MS);
    uint16_t adcReading=0;
   
    bool oled_init_flag =0;
    if (xSemaphoreTake(oled_init, portMAX_DELAY) == pdTRUE)
        {
            oled_init_flag=1;
            
        } 
    while(1)
    {
        if(oled_init_flag==1)
        {
            adcReading= analogRead(4);
            shared_data.BatteryVoltage= displayBatteryVoltage(adcReading);
            
            //Serial.println(adcReading);
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }
        
    
    }
}


void runTasks()
{
    DO_sensor_read_done     = xSemaphoreCreateBinary();
    EC_sensor_read_done     = xSemaphoreCreateBinary();
    upload_data_done        = xSemaphoreCreateBinary();
    oled_ready              = xSemaphoreCreateBinary();
    oled_init              = xSemaphoreCreateBinary();
    data_mutex              = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(DOSensor,"DOSensor",4096,NULL,5,NULL,1);

    xTaskCreatePinnedToCore(ECSensor,"DOSensor",4096,NULL,5,NULL,1);
     
    //xTaskCreatePinnedToCore(Wifi,"Wifi",4096,NULL,1,NULL,1);
    
    xTaskCreatePinnedToCore(SIMCom,"SIMCom",4096,NULL,1,NULL,1);
    
    //xTaskCreatePinnedToCore(RTCandSD,"RTCandSD",4096,NULL,1,NULL,1);

    xTaskCreatePinnedToCore(OLED,"OLED",4096,NULL,1,NULL,1);

    xTaskCreatePinnedToCore(BatteryShow,"BatteryShow",4096,NULL,6,NULL,1);
}

void setupDeepSleep() {
   

    // Set the wakeup time
    esp_sleep_enable_timer_wakeup((uint64_t)TIME_TO_SLEEP * uS_TO_S_FACTOR);

}
