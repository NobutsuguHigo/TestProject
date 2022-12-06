#include <esp_now.h>
#include <WiFi.h>
//#include <WiFiUdp.h>

#include <driver/i2s.h>
//const i2s_port_t I2S_PORT = I2S_NUM_0;

#define NUMBER_OF_READ_DATAS 248     //一括読み出しデータ数（バイト数の1/4）

uint8_t rcvDat[248];
//

#define QUEUE_LENGTH 4096
QueueHandle_t xQueue;
BaseType_t ret;

//float pi = 3.14159; //円周率
//float radx=0;
uint8_t tflag=0;

TaskHandle_t th[1];

hw_timer_t * timer0 = NULL;
 
void IRAM_ATTR onTimer0() {
  tflag=1;
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(rcvDat, incomingData, sizeof(rcvDat));
}

void Task1(void *pvParameters) {  
          //char rcvDat[NUMBER_OF_READ_DATAS*2];       //*****一括読み出しデータ個数*2
          int len=0;
  while(1) {

          for(int i=0;i<NUMBER_OF_READ_DATAS;i++){      //*****一括読み出しデータ個数
            //ret=xQueueSend(xQueue,&rcvDat[2*i],portMAX_DELAY);//キューに計算値を送る
            //ret=xQueueSend(xQueue,&rcvDat[2*i+1],portMAX_DELAY);//キューに計算値を送る
            ret=xQueueSend(xQueue,&rcvDat[i],portMAX_DELAY);//キューに計算値を送る
            //rcvDat[i]=127;
          }
            //delay(1);//コア0のwdtリセットのために必要 
  }//while
}//task1

void setup() {
  Serial.begin(115200);
  disableCore0WDT();
  //pinMode(LED_PIN, OUTPUT);
  pinMode(25,OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  xQueue=xQueueCreate(QUEUE_LENGTH,sizeof(uint8_t));  //キュー作成

  //44.1kHz(近似)サンプリングのタイマー割り込み
  timer0 = timerBegin(0, 8, true);//80MHz/8=10MHz(0.1us)
  timerAttachInterrupt(timer0, &onTimer0, true);
  timerAlarmWrite(timer0, 227, true);//0.1*227=22.7us→44.0528（≒44100sampling）
  //timerAlarmWrite(timer0, 232192, true);//0.1*227=22.7us→44.0528（≒44100sampling）
  timerAlarmEnable(timer0);

    //xTaskCreatePinnedToCore(Task1,"Task1", 4096, NULL, 3, &th[0], 0); //Task1実行
    xTaskCreateUniversal(Task1,"Task1", 4096, NULL, 3, &th[0], 0); //Task1実行、core1ではNG
}

void loop() { 
  uint16_t qdatL=0;
  uint16_t qdatH=0;
  uint16_t qdat=0;

  //while(tflag==0){} //なぜかwhileのブロッキングはNG
  if(tflag==1){//サンプリングタイマー割り込みフラグ
    //for(int i;i=0;i<128){
      //ret=xQueueReceive(xQueue,&qdatL,portMAX_DELAY);
      ret=xQueueReceive(xQueue,&qdatH,portMAX_DELAY);
      //Serial.printf("qdatH = %d\n", (uint8_t)qdatH);
      //Serial.printf("qdatL = %d\n", (uint8_t)qdatL);
      //qdat=(((qdatH&0b1111111)<<8)|(qdatL&0b11111111))/256;
      //if(qdatH==127){
        //qdatH=128;
      //}
      //qdat=qdatH<<8;//8だと何も出ない
      //qdat=qdatH<<4;//爆音
      qdat=qdatH;//
      //qdat=qdatH;
      //qdat=((qdatH&0b00001111)<<4)|((qdatL&0b11110000)>>4);//反応がある
      //qdat=qdatH&0b0000000011111111;
      dacWrite(25,(uint8_t)qdat);
      //Serial.printf("qdat = %d\n", (uint8_t)qdat);
      //Serial.printf("uxQueueMessagesWaiting = %d\n", uxQueueMessagesWaiting(xQueue));
    //}
      tflag=0;
  }
      
}
