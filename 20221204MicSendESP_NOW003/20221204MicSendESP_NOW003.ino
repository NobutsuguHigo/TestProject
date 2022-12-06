#include <esp_now.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <math.h>
const i2s_port_t I2S_PORT = I2S_NUM_0;

uint8_t sendDat[248];
float pi=3.14159;

uint8_t tflag=0;

hw_timer_t * timer0 = NULL;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//#define NUMBER_OF_READ_DATAS 512     //一括読み出しデータ数（バイト数の1/4）
#define NUMBER_OF_READ_DATAS 248     //一括読み出しデータ数（バイト数の1/4）


esp_err_t err;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void IRAM_ATTR onTimer0() {
  tflag=1;
}
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  //20220711追加

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));//2.0.2でコンパイルするのに必要2.0.5は不可
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
      .sample_rate = 44100,                         // 44.1KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // could only get it to work with 32bits(org)
      //.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // could only get it to work with 32bits１６だと１オクターブ上がる
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // although the SEL config should be left, it seems to transmit on right all_leftだと1oct↓
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
      //以下の2項目は動作に関係なさそう。4,16でも変わらない
      .dma_buf_count = 32,                           // 32 number of buffers  64が上限？
      .dma_buf_len = 512                              // 8 samples per buffer (minimum) 128
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = 17,   // BCKL
      .ws_io_num = 18,    // LRCL
      .data_out_num = -1, // not used (only for speakers)
      .data_in_num = 5   // DOUT
  };
  
  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  Serial.println("I2S driver installed.");

  //while(1);//ここまでOK

    //44.1kHz(近似)サンプリングのタイマー割り込み
  timer0 = timerBegin(0, 8, true);//80MHz/8=10MHz(0.1us)
  timerAttachInterrupt(timer0, &onTimer0, true);
  timerAlarmWrite(timer0, 227*248, true);//0.1*227*128=22.7us*128→44.0528/128（≒44100/128sampling）
  //timerAlarmWrite(timer0, 232192, true);//0.1*227=22.7us→44.0528（≒44100sampling）
  timerAlarmEnable(timer0);
  
}//setup

void loop() {

if(tflag==1){//サンプリングタイマー割り込みフラグ
  // Read a single sample and log it for the Serial Plotter.
  int32_t samples[NUMBER_OF_READ_DATAS];             //*****一括読み出しデータ個数（128）
  int32_t sample[NUMBER_OF_READ_DATAS];              //*****一括読み出しデータ個数
  uint8_t sampleSend[NUMBER_OF_READ_DATAS*2];          //*****一括読み出しデータ個数*2
  
  size_t _num_bytes_read;
  memset(samples,0,NUMBER_OF_READ_DATAS*4);//512バイトクリア、//*****一括読み出しデータ個数*4
  
  //int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&sample, portMAX_DELAY); // no timeout
  //1024バイト読み→32ビットデータだと256個
  esp_err_t _err=i2s_read(I2S_PORT,(void*)samples,NUMBER_OF_READ_DATAS*4,&_num_bytes_read,portMAX_DELAY);//*****一括読み出しデータ個数*4
  //Serial.printf("_num_bytes_read=%d\n",_num_bytes_read);//1024バイト読み出し確認  
  //if (bytes_read > 0) {
    for(int i=0;i<NUMBER_OF_READ_DATAS;i++){//1024バイトは32ビットだとデータ個数は1/4で256個  //*****一括読み出しデータ個数
      //samples[i]=(samples[i]+231211008)/32768;//符号付き16bit化(/65536)（型は符号付き32bit）//org
      //Serial.println(samples[i]);
      //samples[i]=(samples[i]+231211008)/32768-7000;//符号付き16bit化(/65536)（型は符号付き32bit）//オフセット7000キャンセル（実測）
      //Serial.println(samples[i]);//符号付き16bit化確認OK
      //sample[i]=samples[i]+32768;//符号付き16bit→符号なし16bit化（uint32_t）
      //sample[i]=samples[i]/16777216+127;
      sample[i]=samples[i]>>24+127;
      //Serial.println(sample[i]);
      //Serial.println(sample[i]);//符号なし16bit化（uint32_t）確認OK→再確認OK
      //sendDat[2*i]=sample[i]&0x000000ff;//下位8ビットを8ビット化
      //sendDat[2*i+1]=(sample[i]&0x0000ff00)>>8;//上位8ビットを8ビット化
      //sendDat[i]=(sample[i]&0x0000ff00)>>8;//上位8ビットのみ
      sendDat[i]=sample[i];
      //Serial.println(sendDat[i]);
      //sendDat[i]=i;//test//のこぎり波OK！
      //sendDat[i]=127+64*sin(2*pi*i/63);//正弦波OK！
      //Serial.printf("qdaAll = %d\n", sendDat[2*i+1]*256+sendDat[2*i]);
      //Serial.printf("datL= = %d\n", sendDat[2*i]);
      //Serial.printf("qdaH = %d\n", sendDat[2*i+1]);
    }  
  
    esp_err_t result = esp_now_send(broadcastAddress, sendDat, sizeof(sendDat));

    tflag=0;
}

}
