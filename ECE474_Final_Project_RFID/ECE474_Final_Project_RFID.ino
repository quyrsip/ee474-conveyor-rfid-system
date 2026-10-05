/**
 * @file EE474_Final_project_RFID.ino
 * @brief RFID access-checker: locks in the first scanned card as reference, flags mismatches via LCD, LED, and ESP-NOW alerts to a paired device with a live clock on the LCD.
 *
 * valid_RFID_decider_and_checker task(core 0) implemented using FreeRtos. scans the RFID of a card or badge and decides based on the first scan what is considered a valid ID
 * match_LCD_task(core 0) recieves data from a queue sent by valid_RFID_decider_and_checker and updated state displayed on the LCD accordingly
 * time_LCD_task(core 0) checks the current time using an RTC module and displays it onto the LCD
 * LED_blink(core 0) recieves data from a queue sent by valid_RFID_decider_and_checker and keeps track of how long it's been since new data was put into the queue
 * and makes the LED blink accordingly. also sends a data through esp-now to a peer ESP32-S3 
 * These stated FreeRtos functions are run according to their priority hierarchy until program is stopped manually
 * @author Sadriya Farah, Valentina Aguilar-Hernandez
 * @date 08/21/2026
 * @version 1.0
 */

#include <NiusWireless.h>
#include <LiquidCrystal_I2C.h>
#include <I2C_RTC.h>
#include <Wire.h>
#include <string.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

/**
 * @name RFID assignment
 * @brief RFID address and pin number
**/
#define RC522_I2C_ADDRESS 0x28
#define RC522_RST_PIN     10
#define LEDPIN 19

/**
 * @name real time clock
**/
static DS3231 RTC;



/**
 * @name receiver esp32 address
**/
uint8_t reciever_addr[] = {0x80, 0xb5, 0x4e, 0xe3, 0x22, 0x70};

/**
 * @name matching ID
**/
uint8_t chosenID[10];
uint8_t chosenIDLen=0;
bool IDChosen=false;


NiusRC522 rfid(15, 10, 12, 11, 13);   // CS, RST, SCK, MOSI, MISO
LiquidCrystal_I2C lcd(0x27, 16, 2);


//QueueHandle_t idQueue;
TaskHandle_t checkID_handle;
TaskHandle_t LED_handle;
TaskHandle_t LCDMatch_handle;
TaskHandle_t LCDTime_handle;
QueueHandle_t LCDMatchQueue;
QueueHandle_t LEDMatchQueue;



// prototypes
void match_LCD_task(void* ptr);
void time_LCD_task(void* ptr);
void valid_RFID_decider_and_checker(void* ptr);
void LED_blink(void* ptr);

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    delay(1500);                              // USB-CDC enumerate
    rfid.begin();
    Wire.begin(8,9); //pins connected to SDA and SCL
    lcd.init();
    RTC.begin();


    RTC.setHourMode(CLOCK_H12);


    RTC.startClock();
    RTC.setDay(19);
    RTC.setMonth(8);
    RTC.setYear(2026);

    RTC.setHours(14);
    RTC.setMinutes(30);
    RTC.setSeconds(56);

    //RTC.setWeek(1); // To update the week manually.

	RTC.updateWeek(); // updates the week automatically based on date in the RTC.

    //RTC.setDate(22,07,29);
    //RTC.setTime(22,10,20);
  LCDMatchQueue = xQueueCreate(3, sizeof(bool));
  LEDMatchQueue = xQueueCreate(3, sizeof(bool));
  if(LCDMatchQueue != NULL && LEDMatchQueue != NULL){
     
    xTaskCreatePinnedToCore(
      valid_RFID_decider_and_checker,     		
      "RFID set and checker",   		// Task name
      2048,                	// Stack size (in bytes)
      NULL,                 	// Task input parameter
      4,                    	// 
      &checkID_handle,           // Task handle
      0          //core 0
    );

     xTaskCreatePinnedToCore(
      LED_blink,     		// Task function
      "LED blink",   		// Task name
      2048,                	// Stack size (in bytes)
      NULL,                 	// Task input parameter
      2,                    	// priority
      &LED_handle,           // Task handle
      0         //core 0
    );
    xTaskCreatePinnedToCore(
      match_LCD_task,     		// Task function
      "LCD match check",   		// Task name
      2048,                	// Stack size (in bytes)
      NULL,                 	// Task input parameter
      3,                    	// priority
      &LCDMatch_handle,           // Task handle
      0         //core 0
    );
    xTaskCreatePinnedToCore(
      time_LCD_task,     		// Task function
      "LCD time update",   		// Task name
      4048,                	// Stack size (in bytes)
      NULL,                 	// Task input parameter
      3,                    	// priority
      &LCDTime_handle,           // Task handle
      0         //core 0
    );

  }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
 if(esp_now_init() != ESP_OK){
    Serial.println("ESP NOW init failed");

 }

  esp_now_peer_info_t peer ={};

  memcpy(peer.peer_addr,reciever_addr,6);
  peer.encrypt =false;
  peer.channel=6;
  peer.ifidx =WIFI_IF_STA;
  if(esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("failed to add peer");
    return;
  }

}




/**
 * @brief Registers the first ID scan to occur as valid. All ID's that don't much this ID are considered invalid.
 *
 * This task constantly scans for an ID card to scan. The found ID is locked as the valid ID for the rest of the time this program runs
 * from there onwards it compares all future scans with the valid ID and sends the match through 2 queues one is sent to match_LCD_task
 * the other is sent to LED_blink. This makes sure each check can be processed by both tasks regardless of outside interferences
 * once a card is scanned <tt> rfid.halt() </tt> is called so that each card scan only accounts for one scan.
 * @param ptr Unused FreeRTOS task parameter
 * @return void
 * @see match_LCD_task
 * @see LED_blink
 * @note this function only decides what is considered a valid ID only once.
 */
 void valid_RFID_decider_and_checker(void* ptr){
  bool IDMatch;
  while(1){
    if (!rfid.cardPresent()) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if(!IDChosen){
      chosenIDLen= rfid.uidLen;
      Serial.println("chosen!");
      for(uint8_t i=0; i< chosenIDLen; i++){
        chosenID[i]=rfid.uid[i];
      
      } 
      IDChosen=true;
    }
    IDMatch = (chosenIDLen==rfid.uidLen &&(memcmp(rfid.uid,chosenID,rfid.uidLen)==0));//if its true we have a match!
    xQueueSend(LCDMatchQueue, &IDMatch, portMAX_DELAY);
    xQueueSend(LEDMatchQueue, &IDMatch, portMAX_DELAY);
    rfid.halt();

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * @brief if there is a match this function makes the LCD print MATCH otherwise it will print FAILED MATCH
 *
 * This function checks the queue every 0.5 seconds. if data recieved through the queue indicates a match
 * this functions sends MATCH to the LCD monitor otherwise FAILED MATCH is sent to the LCD
 * @param ptr Unused FreeRTOS task parameter
 * @return void
 * @see valid_RFID_decider_and_checker
 * @note All data sent to the LCD monitor is displayed on the second line.
 */
void match_LCD_task(void* ptr){
  lcd.backlight();
  bool match;
  while(1){
    if(xQueueReceive(LCDMatchQueue, &match, portMAX_DELAY) == pdTRUE){
      if( match==true){
        lcd.setCursor(0, 1);
        lcd.print("MATCH                ");
      }else if( match==false){
        lcd.setCursor(0, 1);
        lcd.print("FAILED MATCH         ");
      }
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }

}

/**
 * @brief this task sends the current time to the LCD every second
 *
 * This task gets the current time from an external RTC module and simply 
 * sends the day and time to be displayed onto the LCD monitor
 * @param ptr Unused FreeRTOS task parameter
 * @return void
 * @note All data sent to the LCD monitor is displayed on the first line.
 */
void time_LCD_task(void* ptr){
  while(1){
    	if(RTC.isRunning()){
        lcd.setCursor(0, 0);
        lcd.print(RTC.getWeekString().substring(0, 3));
        lcd.print(" ");
        lcd.print(RTC.getTimeString());
        lcd.print("    ");
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * @brief sends data to peer using esp_now and blinks the LED if 30 seconds have passed since the last invalid scan.
 *
 * After recieving the data from the queue this function sends a signal to a peer ESP32-S3 using esp_now if the data indicates
 * an invalid scan.
 * Additionally if 30 seconds have passed and the last scan was invalid the LED starts blinking only stopping once a new scan is read.
 * if an invalid scan is proceeded by a valid scan this function will also send the updated data to the peer.
 * @param ptr Unused FreeRTOS task parameter
 * @return void
 * @see valid_RFID_decider_and_checker
 * @note the data sent to the peer is an 8 bit integer.
 */
void LED_blink(void* ptr){
  bool match = true;   // start "matched" so no blink before the first read
  pinMode(LEDPIN, OUTPUT);
  bool ledState = false;
  TickType_t lastReadTick = xTaskGetTickCount();
  const TickType_t halfMinuteTicks = pdMS_TO_TICKS(30000UL);
  uint8_t con_stop=1;

  while(1){
    if(xQueueReceive(LEDMatchQueue, &match, pdMS_TO_TICKS(100)) == pdTRUE){
      lastReadTick = xTaskGetTickCount(); // ANY read resets the clock
      digitalWrite(LEDPIN, LOW);
      ledState = false;
    }
    while(match == false){
      con_stop=0;
      //sending data to esp peer
      esp_err_t result = esp_now_send(
      reciever_addr,
      &con_stop,
      sizeof(uint8_t));
      if(result==ESP_OK){
          Serial.println("Packet queued");
      }else{
          Serial.println("failed to send");
      }
      if((xTaskGetTickCount() - lastReadTick) >= halfMinuteTicks){
        ledState = !ledState;
        digitalWrite(LEDPIN, ledState);
        vTaskDelay(pdMS_TO_TICKS(500));
        
      }
      //if new data has been recieved stop the LED blinking and update the peer ESP32_S3
      if(xQueueReceive(LEDMatchQueue, &match, pdMS_TO_TICKS(10)) == pdTRUE){
        lastReadTick = xTaskGetTickCount();
        digitalWrite(LEDPIN, LOW); // force off immediately, don't wait for next outer pass
        ledState = false;
        if(match==true){
          con_stop=1;//turn it back on
          esp_err_t result = esp_now_send(
            reciever_addr,
            &con_stop,
            sizeof(uint8_t));
      
        if(result==ESP_OK){
          Serial.println("Packet queued");
        }else{
          Serial.println("failed to send");
        }
      }
      }
    }
  }
}



void loop() {
  // put your main code here, to run repeatedly:

}
