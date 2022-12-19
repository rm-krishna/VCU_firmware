#include <Arduino.h>                //Library for using arduino
#include <SPI.h>                //Library for using SPI Communication 
#include <mcp2515.h>            //Library for using CAN Communication
#include <Arduino_FreeRTOS.h>   // Library for using FreeRTOS
#include "task.h"

struct can_frame recieveFrame;
struct can_frame sendFrame;

MCP2515 mcp2515(44);

int button1 = 49; 
int button2 = 23;

int precharge_mosfet = 46;
int discharge_comtactor = 47;


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int lastButtonState = LOW;   // the previous reading from the input pin
int buttonState;
int buttonPushCounter = 1;   // counter for the number of button presses
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds

void Can_read(void *pvParameters);
void Can_write(void *pvParameters);
void neutral(void);
void city(void);
void eco(void);
void sport(void);
void reverse(void);
void Process_data(can_frame *frame);
void precharge(void *pvParameters);
void GPIO_init(void);
void CAN_init(void);
void tasks(void);


unsigned int _150 = 336;
unsigned int _200 = 512;
unsigned int _250 = 592
unsigned int _650 = 1616;
unsigned int _750 = 1872;


unsigned int id_150[8];
unsigned int id_200[8];
unsigned int id_250[8];
unsigned int id_650[8];
unsigned int id_750[8];

char Ride_mode_Actual;
unsigned int Throttle_Out , Battery_current, Motor_Speed;
unsigned int Controller_temperature, Motor_temperature;
unsigned int Temp;

double Current_factor =  0.012695;
double Speed_factor = 0.305176;
double Thr_Ref = 0.019608;
double volt_factor = 0.450980;
double Throttle_Reference,Battery_Voltage;
bool cal_motor_data = true;
int counter = 0;
bool precharge_flag;
double motor_temp_factor = 0.390625;

TaskHandle_t TaskHandle_precharge;                                        // handler for Task2



void setup(void){
    
    while (!Serial);
    Serial.begin(115200);
    SPI.begin();

    GPIO_init();
    tasks(); 
    
    // start scheduler - should never return
    vTaskStartScheduler();
    Serial.println(F("Scheduler Returned"));
}


void loop(){
    // Empty. Things are done in Tasks.
} 


void GPIO_init(void){

    pinMode(precharge_mosfet,OUTPUT);
    pinMode(discharge_comtactor,OUTPUT);
    Serial.println("GPiO_init done");

}


void precharge(void *pvParameters){
    
    digitalWrite(discharge_comtactor, LOW);
    digitalWrite(precharge_mosfet, HIGH);
    //Serial.println("in precharge ");
                                                            // TODO check MCU Voltage
    for(;;){

        if (counter == 2){

            digitalWrite(discharge_comtactor, HIGH);
            //Serial.println("discharge_comtactor");
        }
        if (counter == 3){

            digitalWrite(precharge_mosfet, LOW);
           // Serial.println("precharge done");
        }
        if (counter == 4){
             
             xTaskCreate(Can_write,"Can_write",1024,NULL,2,NULL);   // CAN write task 
             xTaskCreate(Can_read,"Can_read",1024*3,NULL,1,NULL);     // CAN read Task
             
             vTaskDelete(TaskHandle_precharge);
              //Serial.println("Task Deleted");
        }

      vTaskDelay(pdMS_TO_TICKS(100));
      counter++; //
    }
                                                        
}


void CAN_init(void){

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
    mcp2515.setNormalMode();

    //  send Broadcast mode and Boradcasting rate
    sendFrame.can_id  = 0x400;           //CAN id as 0x400
    sendFrame.can_dlc = 2;               //CAN data length as 8
    sendFrame.data[0] = 0x19;
    sendFrame.data[1] = 0x01;
    mcp2515.sendMessage(&sendFrame);     //Sends the CAN message
    Serial.println("Broadcasting");
    
                                                                                    // TODO: Verify all IDs are present.
}


void tasks(){

    xTaskCreate(precharge,"precharge",128,NULL,3,&TaskHandle_precharge);     // CAN read Task
    
}


// * CAN read Task

void Can_read(void *pvParameters){
Serial.println("In read");

    while(1){

        if ( mcp2515.checkReceive() ){

            mcp2515.readMessage(&recieveFrame);
            Process_data((can_frame *)& recieveFrame);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

    }       // ! End Recieving Message

}


void Can_write(void *pvParameters){
    
    CAN_init();

    Serial.println("In sendMessage");
    
    for(;;){

        buttonState = digitalRead(button1);
        buttonState2= digitalRead(button2);
        if((buttonState && buttonState2)==1){
            buttonPushCounter = 1;                              //moves to eco mode
        }

                                                                // compare the buttonState to its previous state
        if (buttonState != lastButtonState) {
                                                                // if the state has changed, increment the counter
            if (buttonState == LOW) {
                                                                // if the current state is HIGH then the button went from off to on:
                buttonPushCounter++;
                pressedTime = millis();
                                                                //reset the counter
                if (buttonPushCounter == 4)
                    buttonPushCounter = 1;
                                                                // exit reverse mode
                if(buttonPushCounter >= 6)
                    buttonPushCounter = 1;

            } else {
                releasedTime = millis();
            }
                                                                // compare the pressedTime and releasedTime
            long pressDuration = releasedTime - pressedTime;

            if( pressDuration > LONG_PRESS_TIME ){
                buttonPushCounter = 5;
            }

             // Delay a little bit to avoid bouncing
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
                                                                // save the current state as the last state, for next time through the loop
        lastButtonState = buttonState;

        // switch mode based on counter

        switch (buttonPushCounter) {
            case 1: 
                //Serial.println("Eco");
                eco();
                break;

            case 2:
                //Serial.println("city");
                city();
                break;

            case 5:
                //Serial.println("reverse");
                reverse();
                break;
            
            case 3:
                //Serial.println("sport");
                sport();
                break;
                
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);

    }

}


if(receiveFrame.can_id == _150 || receiveFrame.can_id == _200 || receiveFrame.can_id == _250){
    void Process_data(can_frame *frame){

    if (recieveFrame.can_id == _150){

        for (int i = 0; i<recieveFrame.can_dlc; i++)  {  

            id_150[i] = recieveFrame.data[i];
        }

        if ( ( (id_150[7] & 0x1C) >> 2) == 0 ){

            //Ride_mode_Actual = ( (id_150[7] & 0x1C) >> 2);
            Serial.println("N");

        }

        if ( ( (id_150[7] & 0x1C) >> 2) == 1 ){

            //Ride_mode_Actual = ( (id_150[7] & 0x1C) >> 2);
            Serial.println("E");

        }

        if ( ( (id_150[7] & 0x1C) >> 2) == 2 ){

            //Ride_mode_Actual = ( (id_150[7] & 0x1C) >> 2);
            Serial.println("C");

        }

        if ( ( (id_150[7] & 0x1C) >> 2) == 3 ){

            //Ride_mode_Actual = ( (id_150[7] & 0x1C) >> 2);
            Serial.println("R");

        }

        if ( ( (id_150[7] & 0x1C) >> 2) == 4 ){

            // Ride_mode_Actual = ( (id_150[7] & 0x1C) >> 2);
            Serial.println("S");

        }


        if(cal_motor_data){

            // * Calclate Motor_Speed 
            Motor_Speed = id_150[1];
            Motor_Speed = Motor_Speed << 8;
            Temp = id_150[0];
            Motor_Speed = Motor_Speed + Temp;
            Motor_Speed = Motor_Speed * 0.305176; 
            //Serial.println(Motor_Speed);

            // * Calclate Battery_current
            Battery_current = id_150[3];
            Battery_current = Battery_current << 8;
            Temp = id_150[2];
            Battery_current = (Battery_current + Temp)*0.012695;
            //Serial.println(Battery_current);

        }                                                                                    //TODO: add function to ptint this.       
    }

    if (recieveFrame.can_id == _200){

        for (int i = 0; i<recieveFrame.can_dlc; i++)  {  
            
            id_200[i] = recieveFrame.data[i];
        }

        // if (cal_motor_data){

        //     // * calculate Throttle Voltage
        //     Throttle_Reference = id_200[1];
        //     Throttle_Reference = Throttle_Reference * Thr_Ref;
        //    // Serial.println(Throttle_Reference);

        //     // * calculate Battery voltage
        //     Battery_Voltage = id_200[7];
        //     Battery_Voltage = Battery_Voltage * volt_factor;
        //    // Serial.println(Battery_Voltage);

        // }
    }
    if(recieveFrame.can_id == _250){
        for (int i = 0; i<recieveFrame.can_dlc; i++)  {  
            
            id_250[i] = recieveFrame.data[i];
        }
        if(cal_motor_data){
            Motor_temperature = id_250[1];
            Motor_temperature = Motor_temperature << 8;
            Temp = id_150[0];
            Motor_temperature = (Motor_temperature + Temp)* motor_temp_factor;

            if (Motor_temperature>160){
            neutral();
            Serial.println("Neutral Mode");
            }
            if(Motor_temperature<160 && Motor_temperature>=151){
            Serial.println("Warning");
            }
            if(Motor_temperature=<-5){
            Serial.println("Under Temperature");
            }
            }
        }
    }

}
else if(recieveFrame.can_id == _650){
    for( int i= 0; i<receiveFrame.can_dlc;i++){
        id_650[i] = recieveFrame.data[i];
    }
}
else if(receiveFrame.can_id == _750){
    for(int i=0; i<receive.can_dlc; i++){
        id_750[i]=recieveFrame.data[i];
    }
}
else {
    Serial.println("Error!")
}



void city(){


    // TODO: add FOR loop to freat frame
    sendFrame.can_id  = 0x100;           //CAN id as 0x100
    sendFrame.can_dlc = 8;               //CAN data length as 8

    sendFrame.data[0] = 0x66;
    sendFrame.data[1] = 0x26;
    sendFrame.data[2] = 0x00;
    sendFrame.data[3] = 0x00;
    sendFrame.data[4] = 0x00;
    sendFrame.data[5] = 0x00;
    sendFrame.data[6] = 0x00;
    sendFrame.data[7] = 0x02; 

    mcp2515.sendMessage(&sendFrame);

}

void eco(){
  sendFrame.can_id  = 0x100;           //CAN id as 0x100

  sendFrame.can_dlc = 8;               //CAN data length as 8

  sendFrame.data[0] = 0x9A;
  sendFrame.data[1] = 0x19;
  sendFrame.data[2] = 0x00;
  sendFrame.data[3] = 0x00;
  sendFrame.data[4] = 0x00;
  sendFrame.data[5] = 0x00;
  sendFrame.data[6] = 0x00;
  sendFrame.data[7] = 0x01;    

  mcp2515.sendMessage(&sendFrame);     //Sends the CAN message

}

void sport(){
  sendFrame.can_id  = 0x100;           //CAN id as 0x100

  sendFrame.can_dlc = 8;               //CAN data length as 8

  sendFrame.data[0] = 0x66;
  sendFrame.data[1] = 0x26;
  sendFrame.data[2] = 0x00;
  sendFrame.data[3] = 0x00;
  sendFrame.data[4] = 0x00;
  sendFrame.data[5] = 0x00;
  sendFrame.data[6] = 0x00;
  sendFrame.data[7] = 0x05;

  mcp2515.sendMessage(&sendFrame);     //Sends the CAN message

}


void neutral(){
  sendFrame.can_id  = 0x100;           //CAN id as 0x100

  sendFrame.can_dlc = 8;               //CAN data length as 8

  sendFrame.data[0] = 0x00;
  sendFrame.data[1] = 0x00;
  sendFrame.data[2] = 0x00;
  sendFrame.data[3] = 0x00;
  sendFrame.data[4] = 0x00;
  sendFrame.data[5] = 0x00;
  sendFrame.data[6] = 0x00;
  sendFrame.data[7] = 0x00;

  mcp2515.sendMessage(&sendFrame);     //Sends the CAN message

}

void reverse(){
  sendFrame.can_id  = 0x100;           //CAN id as 0x100

  sendFrame.can_dlc = 8;               //CAN data length as 8

  sendFrame.data[0] = 0x48;
  sendFrame.data[1] = 0x01;
  sendFrame.data[2] = 0x00;
  sendFrame.data[3] = 0x00;
  sendFrame.data[4] = 0x00;
  sendFrame.data[5] = 0x00;
  sendFrame.data[6] = 0x00;
  sendFrame.data[7] = 0x03;

  mcp2515.sendMessage(&sendFrame);     //Sends the CAN message
  
}














