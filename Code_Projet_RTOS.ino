#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <queue.h> // add the freeRTOS fonctions for queue


struct valeurCapteurs {
    int analogique;
    int numerique;
    double tempsEnMillisecondes;
};

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore = NULL;

//Declare queue 
QueueHandle_t DigitalQueue; // for taskDigitalRead
QueueHandle_t AnalogQueue;  // for taskAnalogRead
QueueHandle_t valeurCapteur; // for taskReceiveValue
QueueHandle_t sensorValue; // for TaskReceiveFromSerialPor


// digital pin 2 has a pushbutton attached to it. Give it a name:
const int pushButton = 3;
const int  pushButton1 = 4;
const int Analogpin = A0;   // Analog pin 


int outputValue; // Analog value to send to task 3
int result;      // Digital value to send to task 3
valeurCapteurs arrayvalue; // declare and array wich contains structure value


// define tasks:
void TaskAnalogRead( void *pvParameters );
void TaskDigitalRead( void *pvParameters );
void TaskReceiveValue( void *pvParameters );
void TaskReceiveFromSerialPort (void *pvParameters);
void TaskTransformTimeData (void *pvParameters);

void setup() {
  // initialize serial communication at 9600 bits per second:
   Serial.begin(9600);

     while (1) {
    ; // wait for serial port to connect.
    }

     if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
    {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
    }

   //queue creation
   AnalogQueue = xQueueCreate(5,sizeof(const int));
   DigitalQueue = xQueueCreate(5,sizeof(const int));
   valeurCapteur = xQueueCreate(5,sizeof(valeurCapteurs));
   sensorValue = xQueueCreate(5,sizeof(valeurCapteurs));

    //tasks creation
     xTaskCreate(TaskAnalogRead,  "AnalogRead",  128  ,  NULL ,  1 ,  NULL ); 
     xTaskCreate(TaskDigitalRead,  "DigitalRead",  128  ,  NULL ,  1 ,  NULL ); 
     xTaskCreate(TaskReceiveValue,  "ReceiveValue",  128  ,  NULL ,  1 ,  NULL ); 
     xTaskCreate(TaskReceiveFromSerialPort,  "ReceiveFromSerialPort",  128  ,  NULL ,  1 ,  NULL ); 
     xTaskCreate(TaskTransformTimeData,  "TransformTimeData",  128  ,  NULL ,  1 ,  NULL ); 
  
}

void loop() {
  // put your main code here, to run repeatedly:

}


 //tasks fonctions 
 void TaskAnalogRead(void *pvParameters)  // This is a task.
{
   int AnalogValue; // value to send the task3
  
  for (;;)
  {
    AnalogValue=analogRead(Analogpin);  // read the input on analog pin:
    xQueueSend(AnalogQueue,&AnalogValue,0); // send the value you read:
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}


void TaskDigitalRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{  
  int DigitalValue1, DigitalValue2; // value received from  the pushbuttons
  
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
  pinMode(pushButton1, INPUT);
  for (;;) // A Task shall never return or exit.
  {
   // read the input pin: 
    DigitalValue1 = digitalRead(pushButton);
    DigitalValue2 =digitalRead(pushButton1);
    result = DigitalValue1 + DigitalValue2;
    xQueueSend(DigitalQueue,&result,0);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}


void TaskReceiveValue(void *pvParameters)
{
  (void) pvParameters;
  for (;;){
    xQueueReceive(AnalogQueue,&outputValue,0);
    xQueueReceive(DigitalQueue,&result,0);
    vTaskDelay(500/portTICK_PERIOD_MS);
      
    // save value read from task1 &2 in the structure
    arrayvalue.analogique = outputValue;
    arrayvalue.numerique = result;
    arrayvalue.tempsEnMillisecondes = millis();
    xQueueSend(valeurCapteur,&arrayvalue,0);
    }
    
    vTaskDelay(100);  
}


 void TaskReceiveFromSerialPort( void *pvParameters __attribute__((unused)) ) 
 { 
  for (;;)
  {  
    xQueueReceive(valeurCapteur,&arrayvalue,0);
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
     // print out the value you read:
      Serial.print("analogique = ");
      Serial.print(arrayvalue.analogique);
      Serial.println("numerique =");
      Serial.print(arrayvalue.numerique);
      Serial.println("temps = " );
      Serial.print(arrayvalue.tempsEnMillisecondes);
      
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    xQueueSend(sensorValue, &arrayvalue, 0);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}


 void TaskTransformTimeData( void *pvParameters __attribute__((unused)) ) 
 { 
  valeurCapteurs timeData;
  for (;;)
  {  
    xQueueReceive(sensorValue,&arrayvalue,0);
      timeData.analogique = arrayvalue.analogique;
      timeData.numerique = arrayvalue.numerique;
      timeData.tempsEnMillisecondes = arrayvalue.tempsEnMillisecondes/6000;
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
     // print out the value you read:
      Serial.print("analogique = ");
      Serial.print(arrayvalue.analogique);
      Serial.println("numerique =");
      Serial.print(arrayvalue.numerique);
      Serial.println("temps = " );
      Serial.print(arrayvalue.tempsEnMillisecondes);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}