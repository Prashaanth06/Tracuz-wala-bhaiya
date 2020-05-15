TaskHandle_t myTask1;
TaskHandle_t myTask2;


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
delay(3000);

 xTaskCreate(task1,"Task1",10000,NULL,2,&myTask1);
  xTaskCreate(task2,"Task2",10000,NULL,5,&myTask2);
 
}

void loop() {
//  task1();
//  task2();
delay(1000);
  
  // put your main code here, to run repeatedly:

}

void task1(void * parameter)
{
  Serial.println("Task one");
   Serial.println(uxTaskPriorityGet(myTask1));
  delay(1000);
}

void task2(void * parameter)
{
  Serial.println("Task two");
    Serial.println(uxTaskPriorityGet(myTask2));

    delay(1000);

}
