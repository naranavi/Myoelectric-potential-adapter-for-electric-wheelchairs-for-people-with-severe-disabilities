/***************************************************
  構井くん用YAMAHA JWX インターフェースBOX
  筋電位コントローラ
  2019.10.12

  2019.12.7 非常停止　タッチパネル
  2019.12.5 modeSw mouse

  構成
  TWELITE
  AID:20181110
  ch:18
  pwm:10000

  
  ESP32 Mini D1(HM-665)
   2.8 "tft LCDディスプレイ タッチパネル SPIシリアル240 * 320 ILI9341
  2021.10.7 ford lach画面追加 オンショットに入れた
  2021.9.27 ford lach
  2021.5.22 後進とwait移行に停止してから？秒後の制限を設ける
  2019.11.11 連打
  2019.11.2  マウス速度
  2019.11.1  debug ModeSW,設定後の画面クリア
  2019.10.31 LCDLED ON/OFF TCK
  2019.10.17 onSW機能追加

SENSOR_VP left emg sig  TWELITE PWM1
SENSOR_VN right emg sig TWELITE PWM2
IO34      nc
IO35      電波切れ検出TWELITE DO2
IO32      MODE SW out
IO33      buzer
IO25      left SW out
IO26      緊急停止TWELITE DO1
IO27      forward SW out
IO14      TWELITE Rx
IO12      nc
IO13      tft_LED
SHD/SD2*  nc
SWP/SD3*  nc
SCS/CMD*  nc
SCK/CLK*  nc
SDO/SD0*  nc
SDI/SD1*  nc
IO15      nc
IO2       nc
IO0       nc
IO4       T_CS
IO16      LCD_CS
IO17      LCD_DC
IO5       LCD_RESET
IO18      LCD_SCK
IO19      LCD_SOD
IO21      right SW out
RXD0      TWELITE Tx
TXD0      nc
IO22      reverse SW out
IO2       nc
   
 
 ****************************************************/


#include "SPI.h"
//#include "Adafruit_GFX.h"
//#include "Adafruit_ILI9341.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <EEPROM.h> 
#include "oneSwitch.h"
#include <Ticker.h>
#include <BleMouse.h>

Ticker ticker1bzz;
BleMouse bleMouse;

typedef  uint8_t   u8_t ;
typedef  int8_t    s8_t ;
typedef  uint16_t  u16_t ;
typedef  int16_t   s16_t ;
typedef  uint32_t  u32_t ;
typedef  int32_t   s32_t ;

//従来の車いすに抵抗合成で接続するとき下行を有効にする。
///#define Switch4


// For the Adafruit shield, these are the default.
#define tft_DC 17
#define tft_CS 16
#define CS_PIN  4
#define tft_LED 13
#define TFT_LED_ON digitalWrite(tft_LED,LOW)          //TFTバックライト LED ON
#define TFT_LED_OFF digitalWrite(tft_LED,HIGH)        //TFTバックライト LED OFF

//----------後進チェックパラメータ----------------
#define TBK_STOP    500        // 停止時間これ以上停止していたことが条件
#define TBK_ON        350        // 一発目ON時間これ以下が条件
#define TBK_OFF        350        // 二発目ONまでのOFF時間これ以下が条件
//----------センサチェックリザルト----------------
#define    M_OFF        1
#define    M_ONON        2
#define    M_BACK        3
#define    M_LEFT        4
#define    M_RIGHT        5
#define    M_FRONT        6
#define    M_F_LEFT       7
#define    M_F_RIGHT      8
#define    M_B_LEFT       9
#define    M_B_RIGHT      10

//------------INPUT TWELITE-----------------
#define TWE_STOP    26    //停止信号 DO1
#define TWE_SIG     35    //電波信号 DO2

//------------OUTPUTto Wheel-----------------
#define OUT_FORWARD  27    // 前進SW出力ポート
#define OUT_REVERSE  22    // 後退SW出力ポート
#define OUT_LEFT     25    // 左旋回SW出力ポート
#define OUT_RIGHT    21    // 右旋回SW出力ポート
#define OUT_BZZ      33    // ブザー出力ポート
#define OUT_MODE     32    // YAMAHAモードスイッチ
#define PIN_LED0     2     //LED
//------------PanelPos-----------------
#define P_BAR1X0    10        // バー1のX座標 50
#define P_BAR1Y0    10        // バー1のY座標 30
#define P_BAR2X0    110       // バー2のX座標 150
#define P_BAR2Y0    10        // バー2のY座標 30

#define PD_BAR_WID    40        // バーの幅 40
#define PD_BAR_HIG    100        // バーの高さ 120
#define PD_MARK_WID    10        // マークの横幅 10
#define PD_MARK_HIG    10        // マークの高さ 10 
#define PD_LINE_SIZE 2        // マークラインの幅 1or2

#define P_SWUP_X0    10        // 時間upのボタンX座標 10
#define P_SWUP_Y0    220        // 時間upのボタンY座標 170
#define P_SWDN_X0    115        // 時間downのボタンX座標 10
#define P_SWDN_Y0    220       // 時間downのボタンY座標 270

#define PD_SW_WID    32        // 時間ボタン幅
#define PD_SW_HIG    40        // 時間ボタン高さ
#define PD_SWDN_Y0   200       // 時間downのボタンY座標 270

#define P_TIME_X0    45        // 時間表示X座標 10
#define P_TIME_Y0    220        // 時間表示Y座標 220
#define PD_TIME_WID    70        // 時間表示幅70
#define PD_TIME_HIG    20        // 時間表示高さ40
#define PD_TIME_SWWID  40        // 時間SW表示幅70

#define P_LEDF_X0    80             // 前進LED表示X座標
#define P_LEDF_Y0    128            // 前進LED表示Y座標 200
#define P_LEDB_X0    (P_LEDF_X0)           // 後ろLED表示X座標
#define P_LEDB_Y0    (P_LEDF_Y0+80)       // 後ろLED表示Y座標
#define P_LEDL_X0    (P_LEDF_X0-70)        // 左LED表示X座標
#define P_LEDL_Y0    (P_LEDF_Y0+40)       // 左LED表示Y座標
#define P_LEDR_X0    (P_LEDF_X0+70)        // 右LED表示X座標
#define P_LEDR_Y0    (P_LEDF_Y0+40)        // 右LED表示Y座標


#define PD_LED_R    10        // LED表示のサイズ（半径）
#define P_AUTO_X0    210        // AUTOSW表示X座標
#define P_AUTO_Y0    170        // AUTOSW表示Y座標290
#define PD_AUTO_R    20        // AUTOSW表示のサイズ（半径）

#define SIG_Y0        (P_BAR1Y0+PD_BAR_HIG-1)    // 信号から表示 Y0
#define SIG_MAX        (PD_BAR_HIG-2)            // bar表示最大値
//#define SIG_K        ((1023/(PD_BAR_HIG-2))/2)    // 信号から表示への係数（割る）
//#define SIG_K        ((1500/(PD_BAR_HIG-2))/3)    // 信号から表示への係数（割る）
#define SIG_K        ((1500/(PD_BAR_HIG-2))/6)    // 信号から表示への係数（割る）

//モードボタン
#define P_MODE_X0 240
#define P_MODE_SX  5
#define P_MODE_Y0 10
#define P_MODE_SY  5
#define P_MODE_WID 80
#define P_MODE_DY  34
#define P_MODE_H   30
#define P_MODE_N   7




#define AUTO_LEVEL    ((SIG_MAX * SIG_K *2)/10)    // 20%

#define BAR_BASECOLOR   ILI9341_DARKGREY        // バー表示ベースカラー
#define BAR_SIGCOLOR    ILI9341_YELLOW     // バー表示信号値カラー
#define LINE_COLOR      ILI9341_RED            // 閾値ラインカラー
#define MARK_BASECOLOR  ILI9341_BLUE        // マーク表示ベースカラー
#define CYAN            ILI9341_CYAN
#define TXT_BACE_COLOR  ILI9341_BLUE 
#define TXT_COLOR       ILI9341_WHITE 
#define GRAY1           ILI9341_DARKGREY
#define GRAY2           ILI9341_DARKGREY
#define GR_WHITE        ILI9341_WHITE
#define GR_GREEN        ILI9341_GREEN
#define GR_RED          ILI9341_RED
#define GR_BLACK        ILI9341_BLACK
#define GR_BLUE         ILI9341_BLUE
#define MGREEN          ILI9341_GREEN


// ワンショット時間設定範囲
#define  MIN_ONESHOT    0        // ダイヤル ０－９
#define  MAX_ONESHOT    9        //
#define ONESHOT_FREE       7        //フリー設定の番号

static uint8_t    oneshotTime;   // ワンショット設定時間番号
int oneshottime = 500;           //ワンショット設定値
unsigned long oneshottimer;
#define TC_tweliteC 1000          //電波切れ検知時間
unsigned long tweliteCtimer;      //電波切れ検知タイマー

static uint8_t    modeSw;         //modeSW設定値 0:OFF 1:ON

#define bzzOn    digitalWrite(OUT_BZZ,HIGH)
#define bzzOff    digitalWrite(OUT_BZZ,LOW)
#define modeOn    digitalWrite(OUT_MODE,HIGH)
#define modeOff    digitalWrite(OUT_MODE,LOW)
static boolean bzz_f;
static boolean mode_f;

#define bzz300      bzzOn;ticker1bzz.once_ms(300, bzz_stop)
#define bzz100      bzzOn;ticker1bzz.once_ms(100, bzz_stop)
#define bzz50      bzzOn;ticker1bzz.once_ms(50, bzz_stop)


// LED番号
#define LED_FRONT    1
#define LED_BACK    2
#define LED_LEFT    3
#define LED_RIGHT    4
uint8_t    ledF_flag = 0;            // 前LED状態
uint8_t    ledB_flag = 0;            // 後ろLED状態
uint8_t    ledL_flag = 0;            // 左LED状態
uint8_t    ledR_flag = 0;            // 右LED状態
uint8_t    backflag = 0;            // 後進設定フラグ

int addr = 0;

static uint8_t touchFlag = 0;
static uint16_t touchX = 0;
static uint16_t touchY = 0;

//eeprom
enum eeprom{
  EROM_DUMY,
  EROM_MARK1,               // 設定閾値１
  EROM_MARK2,               // 設定閾値１
  EROM_ONESHOT,             // ワンショット時間番号
  EROM_MODE_SW,             //modeSW 設定値
  EROM_SUM,                // チェックサム

  EROM_ok_lowspeed,                    //低速移動スピード番号
  EROM_ok_hispeed,                     //高速移動スピード番号
  EROM_ok_tchilow,                     //低速高速切替時間番号
  EROM_ok_tclong,                      //長短判定時間番号
  EROM_ok_tcshortoff,                  //短押し終了時間番号
  EROM_ok_tclongoff,                   //長押し終了時間番号
  EROM_SUM2                // チェックサム


};
 


#define SIG1CH    A0            // センサ入力ADCチャネル
#define SIG2CH    A3            // センサ入力ADCチャネル

static unsigned int adc1,adc2;                        // ADCの値
static int emgOnEdge;                                 //EMG新規ONフラグ
static unsigned int sig1,sig1old,sig2,sig2old;        // 表示信号値
static unsigned int mark1,mark1old,mark2,mark2old;    // 表示スレッシュホールドマーク値
static unsigned int thr1now,thr2now;                // 現在のスレッシュホールドレベル
static unsigned int thr1nowHs,thr2nowHs;            // ヒステリシススレッシュホールドレベル
                                                    // AUTO_LEVEL以上の時有効
#define Hysteresis 50                                // [%]
boolean start_lock = true;                          //スタート時ロック動作 true:停止
//

static uint16_t    te0=0;
static uint16_t    te1=0;
static uint16_t    te2=0;            
// 変化時間間隔msec 0が最新
static unsigned long    timer;            // 変化時間検出用時間保持
static unsigned long    touchTimer;        // タッチパネル検出時間間隔用
static unsigned long    barTimer;        // センサー信号表示時間用
static unsigned long    tftledTimer;      //TFTディスプレーバックライトタイマー
#define TC_TFTLED     60000               //自動消灯1分

#define TC_50    50                        // 時間間隔用定数
#define TC_100    100                        // 時間間隔用定数

static uint8_t    swflag;                    // オン・オフ状態
#define SIG1BIT    0                        // sig1 ONレベル保持ビット
#define SIG2BIT    1                        // sig2 ONレベル保持ビット    
#define leftB     (1<<SIG1BIT)
#define rightB    (1<<SIG2BIT)
#define ANDBIT    2                        // 1,2共にONビット


enum fmode {
  Moving,       //走行モード
  Waiting,      //待機モード
  Modesw,       //YAMJAHAモードスイッチ切り替えモード
  Mouseing,     //マウス操作モード
  Buzzing,      //ブザー鳴動
  Setting,      //onkeyパラメータ設定
  EndMode       //最後のモード数 番号
};

char fmodeString[7][8]{
  "Moving",
  "Wait",
  "MS:   ",
  "Mouse",
  "Buzzer",
  "Settin",
  "  --"
};

int nowmode = Waiting;
int newmode;

boolean wait_flag = false;    //走行モードから待機モード移行フラグ 
boolean exit_flag;            //モード変更あり

int modeButton = -1;          //new モードボタンタッチ（ボタン番号）
int now_modeButton = -1;          //new モードボタンタッチ（ボタン番号）


HardwareSerial serial1(1); 



// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_CS, tft_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_CS, tft_DC, tft_MOSI, tft_CLK, tft_RST, tft_MISO);

XPT2046_Touchscreen ts(CS_PIN, 255);  // Param 2 - 255 - No interrupts

//LCD座標系読み出し
TS_Point getPointEx()
{
  TS_Point p = ts.getPoint();
  p.x = ((p.x-230)*10)/109;
  if(p.x<0)p.x = 0;
  p.y = ((p.y-300)*10)/147;
  if(p.y<0)p.y = 0;

  return p;
}


//文字表示
void drawString(const char *s,int x, int y, uint8_t sz, uint16_t cc)
{
  tft.setTextSize(sz);
  tft.setTextColor(cc);
  tft.setCursor(x, y);
  tft.print(s);
}

//LCDモードボタンエリアを除くエリアclear
void clear2(void)
{
  tft.fillRect(0, 0,  P_MODE_X0-16, 240, tft.color565(0, 0, 0));
}


// パラメータをEEPROMに保存
void savePrm(void)
{
    uint8_t sum;

    addr =EROM_MARK1    ;EEPROM.write(addr,(uint8_t)mark1);        // EPROMから　ここに前回の値を
    addr =EROM_MARK2    ;EEPROM.write(addr ,(uint8_t)mark2);        //EEPROMから　ここに前回の値を
    addr =EROM_ONESHOT  ;EEPROM.write(addr ,(uint8_t)oneshotTime);
    addr =EROM_MODE_SW  ;EEPROM.write(addr ,(uint8_t)modeSw);    //modeSW 設定値

    sum = (uint8_t)mark1+(uint8_t)mark2+(uint8_t)oneshotTime+(uint8_t)modeSw;
    addr =EROM_SUM;EEPROM.write(addr ,sum);
    EEPROM.commit();
    Serial.print("eeprom write:");
    Serial.println(sum);
}

//EEROM読み出し
void loadPrm(void){
    mark1 = EEPROM.read(EROM_MARK1);          //しきい値
    mark2 = EEPROM.read(EROM_MARK2);          //しきい値
    oneshotTime =  EEPROM.read(EROM_ONESHOT); //ワンショット設定値
    modeSw = EEPROM.read(EROM_MODE_SW);       //modeSW 設定値
    uint8_t sum =  EEPROM.read(EROM_SUM);

    if (sum != ((mark1+mark2+oneshotTime+modeSw)&0xff)){
        // EEPROM読込エラー
        Serial.print("Prm1 eeprom error ");
        Serial.print(sum);
        Serial.print(" , ");
        Serial.println(mark1);
       
        oneshotTime     = MIN_ONESHOT;
        mark1 = 40;
        mark2 = 40;
        modeSw = 0;
    }
}

void savePrm2(void)
{
  uint8_t sum;

  EEPROM.write(EROM_ok_lowspeed,ok_lowspeed);            //低速移動スピード番号
  EEPROM.write(EROM_ok_hispeed,ok_hispeed);              //高速移動スピード番号
  EEPROM.write(EROM_ok_tchilow,ok_tchilow);              //低速高速切替時間番号
  EEPROM.write(EROM_ok_tclong,ok_tclong);                //長短判定時間番号
  EEPROM.write(EROM_ok_tcshortoff,ok_tcshortoff);        //短押し終了時間番号
  EEPROM.write(EROM_ok_tclongoff,ok_tclongoff);          //長押し終了時間番号

  sum = (uint8_t)ok_lowspeed+(uint8_t)ok_hispeed
        +(uint8_t)ok_tchilow+ (uint8_t)ok_tclong
        +(uint8_t)ok_tcshortoff+ (uint8_t)ok_tclongoff;  
    EEPROM.write(EROM_SUM2 ,sum);
    EEPROM.commit();

}

void loadPrm2(void)
{
  uint8_t sum;
  
  ok_lowspeed = EEPROM.read(EROM_ok_lowspeed);            //低速移動スピード番号
  ok_hispeed = EEPROM.read(EROM_ok_hispeed);             //高速移動スピード番号
  ok_tchilow = EEPROM.read(EROM_ok_tchilow);              //低速高速切替時間番号
  ok_tclong = EEPROM.read(EROM_ok_tclong);                //長短判定時間番号
  ok_tcshortoff = EEPROM.read(EROM_ok_tcshortoff);        //短押し終了時間番号
  ok_tclongoff = EEPROM.read(EROM_ok_tclongoff);          //長押し終了時間番号
  sum = EEPROM.read(EROM_SUM2 );

  if(sum != (uint8_t)ok_lowspeed+(uint8_t)ok_hispeed
        +(uint8_t)ok_tchilow+ (uint8_t)ok_tclong
        +(uint8_t)ok_tcshortoff+ (uint8_t)ok_tclongoff){
          //error デフォルト値設定
          ok_lowspeed = ok_hispeed = ok_tchilow = ok_tclong = ok_tcshortoff = ok_tclongoff = 4;          
          Serial.println("EEPROM2 load Error!!");
        }
}

//=========モードボタンなどの表示================

void modeSW_disp(void){
  //特別表示
  int m = Modesw;
    //mode SW 出力状態ON/OFFの表示と出力処理
    //dispOFF
    drawString("      " ,P_MODE_X0+P_MODE_SX, P_MODE_Y0+P_MODE_SY+P_MODE_DY*m , 2,TXT_COLOR);
    delay(50);
    //ON
    drawString("ModeSW" ,P_MODE_X0+P_MODE_SX, P_MODE_Y0+P_MODE_SY+P_MODE_DY*m , 2,TXT_COLOR);
}

void plot_mode_F(int m)
{

  int i;

  if(m<0){
    //全モードボタン表示
    for(i=0;i<EndMode;i++){
      tft.fillRect(P_MODE_X0, P_MODE_Y0+P_MODE_DY*i ,P_MODE_WID,P_MODE_H ,GR_BLUE);
      tft.drawRect(P_MODE_X0, P_MODE_Y0+P_MODE_DY*i ,P_MODE_WID,P_MODE_H ,TXT_COLOR);
      if(i == Modesw){
        modeSW_disp();
      }else{
        drawString(fmodeString[i] ,P_MODE_X0+P_MODE_SX, P_MODE_Y0+P_MODE_SY+P_MODE_DY*i , 2,TXT_COLOR);
      }
    }
  }else{
      tft.fillRect(P_MODE_X0, P_MODE_Y0+P_MODE_DY*m ,P_MODE_WID,P_MODE_H ,GR_RED);
      tft.drawRect(P_MODE_X0, P_MODE_Y0+P_MODE_DY*m ,P_MODE_WID,P_MODE_H ,TXT_COLOR);
      if(m == Modesw){
        modeSW_disp();
      }else{
        drawString(fmodeString[m] ,P_MODE_X0+P_MODE_SX, P_MODE_Y0+P_MODE_SY+P_MODE_DY*m , 2,TXT_COLOR);
      }
  }
}

//=========ボタンなどの表示================

//---------LED F----------
// c:color
void plot_led_F(uint8_t c)
{
    if (c==0){
        // OFF
          for(int i=0;i<60;i++){
         tft.drawLine(P_LEDF_X0,P_LEDF_Y0,P_LEDF_X0-30+i,P_LEDF_Y0+25,GR_WHITE);//L LED 
        }
        
        //tft.fillCircle(P_LEDF_X0, P_LEDF_Y0, PD_LED_R,WHITE);
    }else if (c==1){
        // ON
       // tft.fillCircle(P_LEDF_X0, P_LEDF_Y0, PD_LED_R-2,BRIGHT_RED);
   for(int i=0;i<50;i++){
         tft.drawLine(P_LEDF_X0,P_LEDF_Y0+2,P_LEDF_X0-25+i,P_LEDF_Y0+23,GR_RED);//L LED 
        }  
  }else{
        // Func
        tft.fillCircle(P_LEDF_X0, P_LEDF_Y0, PD_LED_R-2,GR_GREEN);
    }
}

//---------LED B----------
// c:color
void plot_led_B(uint8_t c)
{
    if (c==0){
        // OFF
          for(int i=0;i<60;i++){
         tft.drawLine(P_LEDB_X0,P_LEDB_Y0,P_LEDB_X0-30+i,P_LEDB_Y0-25,GR_WHITE);//L LED 
        }
    //    tft.fillCircle(P_LEDB_X0, P_LEDB_Y0, PD_LED_R,WHITE);
    }else if (c==1){
        // ON
         for(int i=0;i<50;i++){
         tft.drawLine(P_LEDB_X0,P_LEDB_Y0-2,P_LEDB_X0-25+i,P_LEDB_Y0-23,GR_RED);//L LED 
        }
        //tft.fillCircle(P_LEDB_X0, P_LEDB_Y0, PD_LED_R-2,BRIGHT_RED);
    }
}

//---------LED L----------
// c:color
void plot_led_L(uint8_t c)
{
    if (c==0){
        // OFF
       for(int i=0;i<30;i++){
         tft.drawLine(P_LEDL_X0,P_LEDL_Y0,P_LEDL_X0+40,P_LEDL_Y0-15+i,GR_WHITE);//L LED 
        }
      //  tft.fillCircle(P_LEDL_X0, P_LEDL_Y0, PD_LED_R,WHITE);
    }else if (c==1){
        // ON
        for(int i=0;i<24;i++){
         tft.drawLine(P_LEDL_X0+2,P_LEDL_Y0,P_LEDL_X0+38,P_LEDL_Y0-12+i,GR_RED);//L LED 
        }
      //  tft.fillCircle(P_LEDL_X0, P_LEDL_Y0, PD_LED_R-2,BRIGHT_RED);
    }
}

//---------LED R----------
// c:color
void plot_led_R(uint8_t c)
{
    if (c==0){
        // OFF
            // OFF
       for(int i=0;i<30;i++){
         tft.drawLine(P_LEDR_X0,P_LEDR_Y0,P_LEDR_X0-40,P_LEDR_Y0-15+i,GR_WHITE);//L LED 
        }
        
      //  tft.fillCircle(P_LEDR_X0, P_LEDR_Y0, PD_LED_R,WHITE);
    }else if(c==1) {
        // ON
         for(int i=0;i<24;i++){
         tft.drawLine(P_LEDR_X0-2,P_LEDR_Y0,P_LEDR_X0-38,P_LEDR_Y0-12+i,GR_RED);//L LED 
        }
      //  tft.fillCircle(P_LEDR_X0, P_LEDR_Y0, PD_LED_R-2,BRIGHT_RED);
    }
}


//---------SW---------------
void plot_timeSw(void)
{
    uint16_t i;
    int dy = 20; 
    int dy2 = 2; 
    // 加算のボタン 三角付き
    tft.fillRect(P_SWUP_X0, P_SWUP_Y0, PD_TIME_SWWID,PD_TIME_HIG, GRAY2);
    tft.drawRect(P_SWUP_X0, P_SWUP_Y0, PD_TIME_SWWID, PD_TIME_HIG,GRAY1);

    for(i=0 ; i<8 ; i++){
        tft.drawFastVLine(P_SWUP_X0+i+10, P_SWUP_Y0+dy-i*2,i*2,MGREEN);
        tft.drawFastVLine(P_SWUP_X0-i+26, P_SWUP_Y0+dy-i*2,i*2,MGREEN);
    }
    tft.drawFastVLine(P_SWUP_X0-i+26, P_SWUP_Y0+dy-i*2,i*2,MGREEN);

    // 減算のボタン 三角付き
    tft.fillRect(P_SWDN_X0, P_SWDN_Y0, PD_TIME_SWWID, PD_TIME_HIG, GRAY2);
    tft.drawRect(P_SWDN_X0, P_SWDN_Y0, PD_TIME_SWWID, PD_TIME_HIG, GRAY1);

    for(i=0 ; i<8 ; i++){
        tft.drawFastVLine(P_SWDN_X0+i+10, P_SWDN_Y0+dy2,i*2,MGREEN);
        tft.drawFastVLine(P_SWDN_X0-i+26, P_SWDN_Y0+dy2,i*2,MGREEN);
    }
    tft.drawFastVLine(P_SWDN_X0-i+26, P_SWDN_Y0+dy2,i*2,MGREEN);
    
    // 時間表示まど
    tft.drawRect(P_TIME_X0, P_TIME_Y0, PD_TIME_WID, PD_TIME_HIG, GRAY1);

}

// ワンショット
//設定数値の表示
void dispOneshot(uint8_t d)
{
 char otime0[5]={'0','.','5','s'};
char   otime1[5]={'0','.','6','s'};
char   otime2[5]={'1','.','0','s'};
char   otime3[5]={'1','.','5','s'};
char   otime4[5]={'2','.','0','s'};
char   otime5[5]={'3','.','0','s'};
char   otime6[5]={'5','.','0','s'};
char   otime7[5]={'f','r','e','e'};
char   otime8[5]={'F','F','2','s'};
char   otime9[5]={'F','F','3','s'};
      
    // 時間表示まど
    tft.fillRect(P_TIME_X0, P_TIME_Y0, PD_TIME_WID, PD_TIME_HIG, TXT_BACE_COLOR);
    tft.drawRect(P_TIME_X0, P_TIME_Y0, PD_TIME_WID, PD_TIME_HIG, GRAY1);
 
 switch (d){
          case 0://0.5
              drawString(otime0 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
               
           case 1:
              drawString(otime1 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
           break;//
           case 2://0.5
              drawString(otime2 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
           case 3://0.5
              drawString(otime3 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
           case 4://0.5
              drawString(otime4 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
            case 5://0.5
              drawString(otime5 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
            case 6://0.5
              drawString(otime6 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
            case 7://free
              drawString(otime7 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
            case 8://FF3s
              drawString(otime8 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
            case 9://FF5s
              drawString(otime9 ,P_TIME_X0+5, P_TIME_Y0+5, 2,TXT_COLOR);
            break;
  }

//d = d+'0';



}


//----------Auto sw-----------------------
void plot_autoSw(uint8_t c)
{
    if (c==0){
        tft.fillCircle(P_AUTO_X0, P_AUTO_Y0, PD_AUTO_R,GR_WHITE);
        tft.fillCircle(P_AUTO_X0, P_AUTO_Y0, PD_AUTO_R-3,MGREEN);
    }else{
        // ON
        tft.fillCircle(P_AUTO_X0, P_AUTO_Y0, PD_AUTO_R-3,GR_RED);
    }

}

//----------Bar---------------------------
void plot_barBase(void)
{
    // Mark
    tft.fillRect(P_BAR1X0-10, P_BAR1Y0-10, (P_BAR2X0-P_BAR1X0+PD_BAR_WID+20), PD_BAR_HIG+20, MARK_BASECOLOR);
    tft.drawRect(P_BAR1X0-10, P_BAR1Y0-10, (P_BAR2X0-P_BAR1X0+PD_BAR_WID+20), PD_BAR_HIG+20, TXT_COLOR);

    // bar1
    tft.fillRect(P_BAR1X0, P_BAR1Y0, PD_BAR_WID, PD_BAR_HIG, BAR_BASECOLOR);
    tft.drawRect(P_BAR1X0, P_BAR1Y0 ,PD_BAR_WID, PD_BAR_HIG, TXT_COLOR);

    // bar2
    tft.fillRect(P_BAR2X0, P_BAR2Y0, PD_BAR_WID, PD_BAR_HIG, BAR_BASECOLOR);
    tft.drawRect(P_BAR2X0, P_BAR2Y0, PD_BAR_WID, PD_BAR_HIG, TXT_COLOR);

}

// バー１の表示
// d1:現在表示中のデータ値 d2;今回表示するデータ値
void plot_bar1(uint16_t d1, uint16_t d2)
{
    if (d1>SIG_MAX) d1 = SIG_MAX;
    if (d2>SIG_MAX) d2 = SIG_MAX;

    if (d1<d2){
        // 大なので追加
        tft.fillRect(P_BAR1X0+1, P_BAR1Y0+PD_BAR_HIG-1-d2,
            PD_BAR_WID-2,d2-d1+1,  BAR_SIGCOLOR );
    }else{
        // 小なので背景を追加
        tft.fillRect(P_BAR1X0+1, P_BAR1Y0+PD_BAR_HIG-1-d1,
            PD_BAR_WID-2,d1-d2,  BAR_BASECOLOR);
    }

}


// バー２の表示
// d1:現在表示中のデータ値 d2;今回表示するデータ値
void plot_bar2(uint16_t d1, uint16_t d2)
{

    if (d1>SIG_MAX) d1 = SIG_MAX;
    if (d2>SIG_MAX) d2 = SIG_MAX;

    if (d1<d2){
        // 大なので追加
        tft.fillRect(P_BAR2X0+1, P_BAR2Y0+PD_BAR_HIG-1-d2,
            PD_BAR_WID-2,d2-d1+1,  BAR_SIGCOLOR );
    }else{
        // 小なので背景を追加
        tft.fillRect(P_BAR2X0+1, P_BAR2Y0+PD_BAR_HIG-1-d1,
            PD_BAR_WID-2,d1-d2,  BAR_BASECOLOR);
    }

}

// 閾値マークの表示 左側
// d1:現在表示中のデータ値 d2;今回表示するデータ値
void plot_mark1(uint16_t d1, uint16_t d2)
{
    int i=0;

    // 消去
#if PD_LINE_SIZE == 1
    tft.drawFastHLine(P_BAR1X0+PD_BAR_WID+1,P_BAR1Y0+PD_BAR_HIG-d1-i,
        PD_MARK_WID,BASECOLOR);
    for(i=1;i<PD_MARK_WID/2;i++){
#else
    for(i=0;i<PD_MARK_WID/2;i++){
#endif
        tft.drawFastHLine(P_BAR1X0+PD_BAR_WID+1+i*2,SIG_Y0-d1-i,
            PD_MARK_WID-i*2,MARK_BASECOLOR);
        tft.drawFastHLine(P_BAR1X0+PD_BAR_WID+1+i*2,SIG_Y0-d1+i,
            PD_MARK_WID-i*2,MARK_BASECOLOR);
    }
    
    // 描画
#if PD_LINE_SIZE == 1
    drawFastHLine(P_BAR1X0+PD_BAR_WID+1,P_BAR1Y0+PD_BAR_HIG-d1-i,
        tft.PD_MARK_WID-i*2,MGREEN);
    for(i=1;i<PD_MARK_WID/2;i++){
#else
    for(i=0;i<PD_MARK_WID/2;i++){
#endif
        tft.drawFastHLine(P_BAR1X0+PD_BAR_WID+1+i*2,SIG_Y0-d2-i,
            PD_MARK_WID-i*2,MGREEN);
        tft.drawFastHLine(P_BAR1X0+PD_BAR_WID+1+i*2,SIG_Y0-d2+i,
            PD_MARK_WID-i*2,MGREEN);
    }

}



// 閾値マークの表示 右側
// d1:現在表示中のデータ値 d2;今回表示するデータ値
void plot_mark2(uint16_t d1, uint16_t d2)
{
    int i=0;

    // 消去
#if PD_LINE_SIZE == 1
    tft.drawFastHLine(P_BAR1X0-PD_MARK_WID-1,P_BAR1Y0+PD_BAR_HIG-d1-i,
        PD_MARK_WID,BAR_BASECOLOR);
    for(i=1;i<PD_MARK_WID/2;i++){
#else

    for(i=0;i<PD_MARK_WID/2;i++){
#endif
        tft.drawFastHLine(P_BAR2X0-PD_MARK_WID-1,SIG_Y0-d1-i,
            PD_MARK_WID-i*2,MARK_BASECOLOR);
        tft.drawFastHLine(P_BAR2X0-PD_MARK_WID-1,SIG_Y0-d1+i,
            PD_MARK_WID-i*2,MARK_BASECOLOR);
    }
    
    // 描画
#if PD_LINE_SIZE == 1
    tft.drawFastHLine(P_BAR1X0-PD_MARK_WID-1,P_BAR1Y0+PD_BAR_HIG-d1-i,
        PD_MARK_WID,MGREEN);
    for(i=1;i<PD_MARK_WID/2;i++){
#else
    for(i=0;i<PD_MARK_WID/2;i++){
#endif
        tft.drawFastHLine(P_BAR2X0-PD_MARK_WID-1,SIG_Y0-d2-i,
            PD_MARK_WID-i*2,MGREEN);
        tft.drawFastHLine(P_BAR2X0-PD_MARK_WID-1,SIG_Y0-d2+i,
            PD_MARK_WID-i*2,MGREEN);
    }

}

// 閾値マークLINEの表示 左側
// d:閾値データ値  c:カラー 
void plot_markline1(uint16_t d,uint16_t c)
{

    // 閾値ラインマーク
#if PD_LINE_SIZE == 1
    tft.drawFastHLine(P_BAR1X0+1, P_BAR1Y0+PD_BAR_HIG-1-d,PD_BAR_WID-2,c);
#else
    tft.drawFastHLine(P_BAR1X0+1, SIG_Y0-d,PD_BAR_WID-2,c);
    tft.drawFastHLine(P_BAR1X0+1, SIG_Y0-d+1,PD_BAR_WID-2,c);
#endif
}

// 閾値マークLINEの表示 右側
// d:閾値データ値  c:カラー 
void plot_markline2(uint16_t d,uint16_t c)
{
    // 閾値ラインマーク
#if PD_LINE_SIZE == 1
    tft.drawFastHLine(P_BAR2X0+1, P_BAR2Y0+PD_BAR_HIG-1-d,PD_BAR_WID-2,c);
#else
    tft.drawFastHLine(P_BAR2X0+1, SIG_Y0-d,PD_BAR_WID-2,c);
    tft.drawFastHLine(P_BAR2X0+1, SIG_Y0-d+1,PD_BAR_WID-2,c);
#endif
}

// 閾値１変更
void disp_thr1_now(uint16_t d)
{
Serial.println(d);  

    if (d != mark1){
        // 閾値変更
        thr1now = d * SIG_K;
        // ヒステリシスレベル設定
        if (thr1now>=AUTO_LEVEL){
            thr1nowHs = thr1now-(thr1now*Hysteresis)/100;
        }else{
            thr1nowHs = thr1now;
        }

        mark1old = mark1;
        mark1 = d;
        plot_mark1(mark1old,mark1);                // 三角マーク
        // Line消去
        if (sig1 >= mark1old){
            // 信号エリア
            plot_markline1(mark1old,BAR_SIGCOLOR);    // Line消去表示
        }else if(sig1<(mark1old-1)){
            // 無信号エリア
            plot_markline1(mark1old,BAR_BASECOLOR);    // Line消去表示
        }else{
            // ちょうど境目
            plot_markline1(mark1old,BAR_BASECOLOR);    // Line消去表示
            plot_markline1(mark1old-1,BAR_SIGCOLOR);    // Line消去表示
        }
        // 閾値ライン表示
        plot_markline1(mark1,LINE_COLOR);    
        // 保存
        //savePrm();

    }

}


// 閾値２変更
void disp_thr2_now(uint16_t d)
{
    if (d != mark2){
        // 閾値変更
        thr2now = d * SIG_K;
        // ヒステリシスレベル設定
        if (thr2now>=AUTO_LEVEL){
            thr2nowHs = thr2now-(thr2now*Hysteresis)/100;
        }else{
            thr2nowHs = thr2now;
        }

        mark2old = mark2;
        mark2 = d;
        plot_mark2(mark2old,mark2);                // 三角マーク

        // Line消去
        if (sig2 >= mark2old){
            // 信号エリア
            plot_markline2(mark2old,BAR_SIGCOLOR);    // Line消去表示
        }else if(sig2<(mark2old-1)){
            // 無信号エリア
            plot_markline2(mark2old,BAR_BASECOLOR);    // Line消去表示
        }else{
            // ちょうど境目
            plot_markline2(mark2old,BAR_BASECOLOR);    // Line消去表示
            plot_markline2(mark2old-1,BAR_SIGCOLOR);    // Line消去表示
        }
        // 閾値ライン表示
        plot_markline2(mark2,LINE_COLOR);    
        
        // 保存
        //savePrm();
    }

}


// Bar表示処理
void disp_bar_now(void)
{

    sig1old = sig1;
    sig1 = adc1/SIG_K;
    if (sig1old != sig1){
        // 書き換え
        plot_bar1(sig1old,sig1);
        plot_markline1(mark1,LINE_COLOR);
    }

    sig2old = sig2;
    sig2 = adc2/SIG_K;
    if (sig2old != sig2){
        // 書き換え
        plot_bar2(sig2old,sig2);
        plot_markline2(mark2,LINE_COLOR);
    }
}

/// ワンショット値読み出し
long  oneshotget(uint8_t d)
{
  //ワンショット時間ms
  long Tb[] = {500,600,1000,1500,2000,3000,5000,500000,502000,503000};//500000:free 
                                                                      //503000:2秒継続 自動走行開始
                                                                      //505000:3秒継続 自動走行開始
  return Tb[d];
}
// ワンショット出力設定
void  oneshotWrite(uint8_t d)
{
  oneshottime = oneshotget(d);
}


// EMGセンサーチェック
// >>状態
uint8_t inEmg(void)
{

    uint8_t m;
    uint8_t c=0;
    static boolean f = true;
    static uint8_t cb=0;
    
    adc1 = analogRead(SIG1CH)/2;
    adc2 = analogRead(SIG2CH)/2;
//    adc1 = analogRead(SIG1CH)/8;
//    adc2 = analogRead(SIG2CH)/8;
      
    // 左センサレベル
    if(adc1>=thr1now){
        // ON
        c |= (1<<SIG1BIT);
    }else{
        // ヒステリシスチェック 前進モード時のみ有効
        f = false;
        if ((swflag & (1<<ANDBIT)) == (1<<ANDBIT) && backflag==0 && adc1>=thr1nowHs){
            c |= (1<<SIG1BIT);
        }

    }

    // 右センサレベル
    if(adc2>=thr2now){
        // ON
        c |= (1<<SIG2BIT);
    }else{
        // ヒステリシスチェック 前進モード時のみ有効
        if ((swflag & (1<<ANDBIT)) == (1<<ANDBIT) && backflag==0  && adc2>=thr2nowHs){
            c |= (1<<SIG2BIT);
        }

    }
    //c = 0; //////////
    ///test
    if(digitalRead(2)==LOW){
      c |= rightB;
    }
    c |= testSw();

    emgOnEdge = (cb ^ c) & c;   //立ち上がりエッジ
    cb = c;
    if(f){
      return 0;
    }else{
      return c;
    }
}

//5回連続チェック
void rendachek(uint8_t c)
{
 #define Wait_limtTc 2500      //ms以内
 #define Wait_MinilimtTc 100      //ms以上
 #define  Wait_Cn 5           //5回
  static unsigned long t1,t2,t3,t4,t5;
  static int   wait_cnt;
  static boolean f;

    //モード移行チェック 5回カウント
    if((c & leftB)==0 ){
      if ( (millis()-t1)>Wait_limtTc){
        //時間たちすぎ
        wait_cnt = 0;
      }
      f = false;
    }
    
    if(f==false && (c & leftB)>0){
      //new on
      f = true;
      if(millis()-t1 >Wait_MinilimtTc){
        wait_cnt++;
          Serial.print("wait_cnt:");Serial.print(wait_cnt);
          Serial.print(" time::");Serial.println(millis()-t5);
        if(wait_cnt >= Wait_Cn && (millis()-t4)<=Wait_limtTc ){
          //移行発生
          Serial.print("wait_cnted:");Serial.println(wait_cnt);
          wait_flag = true;
          wait_cnt = 0;
          bzzOn;
          ticker1bzz.once_ms(300, bzz_stop);
        }
        t5=t4; t4=t3; t3=t2; t2=t1; t1=millis();      
      }
    }
}

// センサーチェック
// >>状態
uint8_t inproc(void)
{
    uint8_t c=0;
    static int tweliteC;
    static boolean frontFlag;
    static boolean backFlag;
    
    // EMGセンサーチェック>>on bit
    c = inEmg();
    if (c==0){
      start_lock = false;                 //スタート時ロック動作 true:停止
    }
    if (start_lock){
      //スタート時ロック動作 true:停止
      c = 0;
    }


    //モード移行チェック 5回カウント
    rendachek(c);

    //緊急停止チェック
    if(digitalRead(TWE_STOP) == LOW){
      //緊急停止!!
      wait_flag = true;
      tft.setCursor(P_LEDF_X0-35, P_LEDF_Y0+40-4);   
      tft.setTextColor(ILI9341_RED);   
      tft.setTextSize(1);
      tft.println("  EMG.STOP");
      return M_OFF;
    }

    //電波切れチェック
    if (tweliteC != digitalRead(TWE_SIG)){
      //変化あり
      tweliteC = digitalRead(TWE_SIG);
      tweliteCtimer = millis()+TC_tweliteC;
    }else{
      if(tweliteCtimer < millis()){
        //電波切れ発生
        wait_flag = true;
        tft.setCursor(P_LEDF_X0-35, P_LEDF_Y0+40-4);   
        tft.setTextColor(ILI9341_RED);   
        tft.setTextSize(1);
        tft.println("  NO RADIO");
        return M_OFF;
      }
    }
    
    // 両ともセンサレベル
    if ((c & ((1<<SIG1BIT)|(1<<SIG2BIT))) == ((1<<SIG1BIT)|(1<<SIG2BIT))){
        // 両方ON
        c |= (1<<ANDBIT);
    }

    // 両センサ共に変化の時間間隔記録
    if( (swflag ^ c)&(1<<ANDBIT)){
        // 変化あり
        te2 = te1;
        te1 = te0;
        te0=millis()-timer;
        timer = millis();
    }


    // 後進チェック
    if(((swflag ^ c) & c) &(1<<ANDBIT)){
        // 立ち上がり
        tft.fillCircle(P_LEDF_X0, P_LEDF_Y0+40, 10,GR_RED);

        if (te2>=TBK_STOP && te1 <TBK_ON && te0 <TBK_OFF){
            // 後進開始
            swflag = c;
            frontFlag = false;
            backFlag = true;
            tft.fillCircle(P_LEDF_X0, P_LEDF_Y0+40, 10,GR_BLACK);
            return M_BACK;
        }else{
 tft.fillCircle(P_LEDF_X0, P_LEDF_Y0+40, 10,GR_BLACK);
        }

    }
    swflag = c;

    // 両センサともOFFチェック
    if (c==0){
        // OFF 停止
        frontFlag = false;
        backFlag = false;
        return M_OFF;
    }

    // 両ON前進or後進
    if(c &(1<<ANDBIT)){
        // 両センサーON
        if(backFlag == false){
          frontFlag = true;
        }
        return M_ONON;
        
    }else if (c &(1<<SIG1BIT)){
        // 左センサON
        if(frontFlag) return M_F_LEFT;
        if(backFlag)  return M_B_LEFT;
        return M_LEFT;

    }else if (c &(1<<SIG2BIT)){
        // 右センサON
        if(frontFlag) return M_F_RIGHT;
        if(backFlag)  return M_B_RIGHT;
        return M_RIGHT;
        
    }else if(backFlag){
        return M_BACK;
    }


    // 一応停止
    return M_OFF;

}

// LCDパネルのLEDの表示
void ledWrite(uint8_t m, uint8_t f)
{
    switch (m){
        case LED_FRONT:
            if (ledF_flag != f){
                // 変更が必要
                plot_led_F(f);
                ledF_flag = f;
            }
            break;
        case LED_BACK:
            if (ledB_flag != f){
                // 変更が必要
                plot_led_B(f);
                ledB_flag = f;
            }
            break;
        case LED_LEFT:
            if (ledL_flag != f){
                // 変更が必要
                plot_led_L(f);
                ledL_flag = f;
            }
            break;
        case LED_RIGHT:
            if (ledR_flag != f){
                // 変更が必要
                plot_led_R(f);
                ledR_flag = f;
            }
            break;
    }

}


// ジョイコン動作
void joyset(uint8_t f)
{


#ifdef Switch4
    if(f == M_F_RIGHT || f == M_B_RIGHT){
      f = M_RIGHT;
    }
    if(f == M_F_LEFT || f == M_B_LEFT){
      f = M_LEFT;
    }
#endif
    
    switch (f){
        case M_OFF:            // stop
            digitalWrite( OUT_FORWARD,0);
            digitalWrite( OUT_REVERSE,0);
            digitalWrite( OUT_LEFT,0);
            digitalWrite( OUT_RIGHT,0);
            ledWrite(LED_FRONT ,0);
            ledWrite(LED_BACK ,0);
            ledWrite(LED_LEFT ,0);
            ledWrite(LED_RIGHT ,0);
            break;
        case M_FRONT:
            digitalWrite( OUT_FORWARD,1);
            digitalWrite( OUT_REVERSE,0);
            digitalWrite( OUT_LEFT,0);
            digitalWrite( OUT_RIGHT,0);
            ledWrite(LED_FRONT ,1);
            ledWrite(LED_BACK ,0);
            ledWrite(LED_LEFT ,0);
            ledWrite(LED_RIGHT ,0);
            break;
        case M_BACK:
            digitalWrite( OUT_FORWARD,0);
            digitalWrite( OUT_REVERSE,1);
            digitalWrite( OUT_LEFT,0);
            digitalWrite( OUT_RIGHT,0);
            ledWrite(LED_FRONT ,0);
            ledWrite(LED_BACK ,1);
            ledWrite(LED_LEFT ,0);
            ledWrite(LED_RIGHT ,0);
            break;
        case M_LEFT:
            digitalWrite( OUT_FORWARD,0);
            digitalWrite( OUT_REVERSE,0);
            digitalWrite( OUT_LEFT,1);
            digitalWrite( OUT_RIGHT,0);
            ledWrite(LED_FRONT ,0);
            ledWrite(LED_BACK ,0);
            ledWrite(LED_LEFT ,1);
            ledWrite(LED_RIGHT ,0);
            break;
        case M_RIGHT:
            digitalWrite( OUT_FORWARD,0);
            digitalWrite( OUT_REVERSE,0);
            digitalWrite( OUT_LEFT,0);
            digitalWrite( OUT_RIGHT,1);
            ledWrite(LED_FRONT ,0);
            ledWrite(LED_BACK ,0);
            ledWrite(LED_LEFT ,0);
            ledWrite(LED_RIGHT ,1);
            break;
        case M_F_LEFT:
            digitalWrite( OUT_FORWARD,1);
            digitalWrite( OUT_REVERSE,0);
            digitalWrite( OUT_LEFT,1);
            digitalWrite( OUT_RIGHT,0);
            ledWrite(LED_FRONT ,1);
            ledWrite(LED_BACK ,0);
            ledWrite(LED_LEFT ,1);
            ledWrite(LED_RIGHT ,0);
            break;
        case M_B_LEFT:
            digitalWrite( OUT_FORWARD,0);
            digitalWrite( OUT_REVERSE,1);
            digitalWrite( OUT_LEFT,1);
            digitalWrite( OUT_RIGHT,0);
            ledWrite(LED_FRONT ,0);
            ledWrite(LED_BACK ,1);
            ledWrite(LED_LEFT ,1);
            ledWrite(LED_RIGHT ,0);
            break;
        case M_F_RIGHT:
            digitalWrite( OUT_FORWARD,1);
            digitalWrite( OUT_REVERSE,0);
            digitalWrite( OUT_LEFT,0);
            digitalWrite( OUT_RIGHT,1);
            ledWrite(LED_FRONT ,1);
            ledWrite(LED_BACK ,0);
            ledWrite(LED_LEFT ,0);
            ledWrite(LED_RIGHT ,1);
            break;
        case M_B_RIGHT:
            digitalWrite( OUT_FORWARD,0);
            digitalWrite( OUT_REVERSE,1);
            digitalWrite( OUT_LEFT,0);
            digitalWrite( OUT_RIGHT,1);
            ledWrite(LED_FRONT ,0);
            ledWrite(LED_BACK ,1);
            ledWrite(LED_LEFT ,0);
            ledWrite(LED_RIGHT ,1);
            break;

    }


}

// オート閾値設定
// 現在の信号値の3/4を設定値とする、ただし信号値が低いと設定しない。
void autoSet(void)
{
    if (adc1>=AUTO_LEVEL && adc2>=AUTO_LEVEL){
        // オート設定
        thr1now = (adc1*3)/4;            // 3/4
        thr2now = (adc2*3)/4;
        disp_thr1_now(thr1now/SIG_K);        // 閾値表示
        disp_thr2_now(thr2now/SIG_K);        // 閾値表示
        savePrm();
    }
}

//emgエミュレートタッチ
uint8_t testSw(void)
{
  uint8_t c=0;
  if(ts.touched()){
    TS_Point p = getPointEx();
    if((p.x>P_LEDL_X0 && p.x<P_LEDL_X0+40) && (p.y>P_LEDL_Y0-20 && p.y<P_LEDL_Y0+20)){
      c = leftB;
       
    }
    if((p.x>P_LEDR_X0-40 && p.x<(P_LEDR_X0)) && (p.y>(P_LEDR_Y0-20) && p.y<(P_LEDR_Y0+20))){
      c |= rightB;
      Serial.print("c:");
      Serial.println(c);
    }
  }
  return c;
}

// タッチパネル処理
void intouch(void)
{
#define TOUCH_Z_LOW 5        //　タッチ有効 LOWサイド 
#define TOUCH_Z_HI  4000    //　タッチ有効 Hiサイド 

static    uint8_t ttflsg=0;        // 連続防止タッチフラグ
//Serial.println("intouch(");
    if (millis()>touchTimer){
        // 処理時間
        touchTimer = millis() +TC_50;    // 次の処理時間

        if(ts.touched()){
          uint8_t f = 0;                    // タッチ有効フラグ
          uint16_t xx,yy;
  
          TS_Point p = getPointEx();
  Serial.print(p.x);
  Serial.print(",");
  Serial.print(p.y);
  Serial.print(",");
  Serial.println(p.z);
  
          if(touchFlag == 0){
              // 新規ON
              touchFlag = f = 1;
              xx = p.x;
              yy = p.y;
              touchX = xx;
              touchY = yy;
          }else{
              if (abs(touchX - xx)>10 || abs(touchY - yy)>10){
                  // タッチしながら移動
                  touchFlag = f = 1;
                  xx = p.x;
                  yy = p.y;
                  touchX = xx;
                  touchY = yy;
              }
          }

   Serial.print(xx);
  Serial.print(",");
  Serial.print(yy);
  Serial.print(",");
  Serial.println(f);
  
          if (f){
  
              // タッチ解析実行
              // modeボタン
              if(xx>P_MODE_X0){
                int wy = (yy-P_MODE_Y0)/P_MODE_DY;  //モード番号
                if (now_modeButton != wy){
                  //新しくボタンが押された
                  modeButton = wy;
                  now_modeButton = wy;          //new モードボタンタッチ（ボタン番号）

                }
                if(newmode != wy){    //同じモードには反応しない
                  newmode = wy;
                  Serial.print("SW:");
                  Serial.println(wy);
                  
                  exit_flag = true;
                  return;
                }else{
                  //同じ番号でBzzとModeSWは反応してみる
                  if(wy==Buzzing){      //ブザー鳴動
                    bzz_f = true;
                  }else if(wy == Modesw){
                    mode_f = true;
                  }
                }
              }else{
                modeButton = -1;
              }
              
              // 閾値１設定？
              if(xx < (P_BAR1X0+PD_BAR_WID) && P_BAR1Y0<yy && (P_BAR1Y0+PD_BAR_HIG)>yy){
                  // 閾値1変更
                  disp_thr1_now(SIG_Y0 - yy);
                  savePrm();

              }else if(xx > (P_BAR2X0) && xx < (P_BAR2X0+PD_BAR_WID+PD_BAR_WID/2)  && P_BAR2Y0<yy && (P_BAR2Y0+PD_BAR_HIG)>yy){
                  // 閾値2変更
                  disp_thr2_now(SIG_Y0 - yy);
                  savePrm();
              }else if(xx > P_SWUP_X0 && xx < (P_SWUP_X0+PD_SW_WID) 
                  && yy > PD_SWDN_Y0 && yy < (PD_SWDN_Y0+PD_SW_HIG)  && ttflsg==0){
                      // 時間upボタンON
                      ttflsg=1;    // タッチフラグ
                      oneshotTime++;
                      if (oneshotTime > MAX_ONESHOT){
                          oneshotTime = MAX_ONESHOT;
                      }
                      dispOneshot(oneshotTime);        // LCD表示
                      oneshotWrite(oneshotTime);        // 設定出力
                      // 保存
                      savePrm();
                      
              }else if(xx > P_SWDN_X0 && xx < (P_SWDN_X0+PD_SW_WID) 
                  && yy > PD_SWDN_Y0 && yy < (PD_SWDN_Y0+PD_SW_HIG) && ttflsg==0){
                      // 時間downボタンON
                      ttflsg=1;    // タッチフラグ
                      if (oneshotTime > MIN_ONESHOT){
                          oneshotTime--;
                      }
                      dispOneshot(oneshotTime);        // LCD表示        
                      oneshotWrite(oneshotTime);        // 設定出力
                      // 保存
                      savePrm();
  
  
              }else if(xx > P_AUTO_X0 && xx < (P_AUTO_X0+PD_AUTO_R*2) 
                  && yy > (P_AUTO_Y0-PD_AUTO_R) && yy < (P_AUTO_Y0+PD_AUTO_R)){
                      // オートSW ON
                      plot_autoSw(1);            // オート閾値設定のボタンON表示
                      delay(200);
                      autoSet();
                      plot_autoSw(0);            // オート閾値設定のボタンOFF表示
  
              }else if(xx >174){
                //停止候補
                if (newmode == Moving){
                  //走行モード
                  newmode++;
                  Serial.println("exit touch!!");
                  exit_flag = true;
                  return;
                }
              }
          }
      }else{
        ttflsg=touchFlag = 0;
        now_modeButton = -1;
      }
    }
    delay(1);
}


// センサレベル表示 
void disprefresh(void)
{
    if (millis()>barTimer){
        // 処理時間
        barTimer = millis() +TC_50;    // 次の処理時間
        disp_bar_now();                // 信号バー表示
    }
}

//mode切り替えか 
int inEmgExit(void)
{
  static unsigned long t = millis();
  
  int c = inEmg();
  if(c & leftB){
    //exit?
    if(millis()>t+500){
      exit_flag = true;
      newmode++;
      bzzOn;
      ticker1bzz.once_ms(100, bzz_stop);
      return c;
    }
  }else{
    t = millis();
  }
  return c;
}

//走行画面表示
void moving_base_disp(void)
{

    clear2();
    //tft.fillRect(0, 0, (P_BAR2X0-P_BAR1X0+PD_BAR_WID+20), 239,GR_BLACK  );

    plot_barBase();           // 信号バーの表示
    plot_timeSw();            // 時間設定スイッチの表示
    plot_led_F(0);            // LEDを表示
    plot_led_B(0);            // LEDを表示
    plot_led_L(0);            // LEDを表示
    plot_led_R(0);            // LEDを表示
    //plot_autoSw(0);            // オート閾値設定のボタン表示
    // 閾値表示
    int d = mark1;
    mark1 = 0;              //違う値でないと描画しないので
    disp_thr1_now(d);
    d = mark2;
    mark2 = 0;
    disp_thr2_now(d);

    // ワンショット設定数値の表示
    dispOneshot(oneshotTime);    
    oneshotWrite(oneshotTime);        // 設定出力
  
}

//通常走行モード初期化
void m_moving_ini(void)
{
    plot_mode_F(-1);   //ボタン表示  
    plot_mode_F(Moving);      //ボタン

    moving_base_disp();       //ベース画像表示
    drawString("shot time",100,204,1,TXT_COLOR);

    tweliteCtimer = millis()+TC_tweliteC;
    wait_flag = false;
    start_lock = true;                          //スタート時ロック動作 true:停止


}

//通常走行モード
void m_moving(void)
{
  int c;
  static boolean new_on;
  int old_c = M_OFF;
  unsigned long autoRunTimer = 0; //自動前進タイマー
  boolean  autoRunFlag = false;  //自動前進フラグ
  boolean  releaseFlag = false;  //自動前進フラグ
  const int NON = 0;
  const int OFF = 1;
  const int TIMEWEIT = 2;
  const int RUN =3;
  const int RELEASE = 4;
  const int RELEASEWAIT = 5;
  int autoRunSt = NON;
  
  for(;;){
    c = inproc();
  
    if (wait_flag || exit_flag){
      //モード移行要求あり
      Serial.print("wait:");Serial.print(wait_flag );
      Serial.print(" exit:");Serial.print(exit_flag );
      if (wait_flag) newmode = Waiting;
      backflag = 0;
      joyset(M_OFF);
      new_on = false;
      return;
    }
    if(c != M_OFF && new_on == false){
      //ワンショット開始
      new_on = true;
      oneshottimer = oneshottime + millis();      
    }else if(c == M_OFF){
      new_on = false;
    }
    if(oneshottimer<millis()){
      //ワンショットで停止
      c = M_OFF;
    }

    //自動前進処理
    if( oneshotTime>ONESHOT_FREE){
      if(autoRunSt == NON){
        autoRunSt = OFF;
      }
    }else{
      autoRunSt = NON;
    }
    
    if( c == M_OFF && autoRunSt == RUN){
      //力をぬいた
      autoRunSt = RELEASE;
    }

    if(autoRunSt == RELEASEWAIT && c == M_OFF){
      //自動完全終了
      autoRunSt = NON;
    }
    if( autoRunSt == RELEASE && c == M_ONON ){
      //自動前進解除
      joyset(M_OFF);
      autoRunSt = RELEASEWAIT;
      c = M_OFF;
      Serial.println("autoRun Stop");
      bzz50;
    }
 
    if(autoRunSt == RELEASEWAIT && c == M_ONON){
      //自動完全終了待ち
      joyset(M_OFF);
      c = M_OFF;
    }


    if(c == M_ONON && old_c != M_ONON && backflag == 0){
      //前進オート計測開始
      Serial.println("new FF");
      if(autoRunSt == OFF){
        autoRunSt = TIMEWEIT;
        autoRunTimer = oneshotget(oneshotTime)-oneshotget(ONESHOT_FREE)+millis();
        Serial.println("autoRun timer start");
      }else{
        autoRunTimer = 0;
      }
    }

    if(c == M_ONON && autoRunSt == TIMEWEIT && autoRunTimer<millis()){
      if(backflag){
        autoRunSt = OFF;
      }else{
      //前進オート開始
        autoRunSt = RUN;
        bzz300;
        Serial.println("autoRun start!!");
      }
    }
    
    old_c = c;
    Serial.print("oneshotTime=");Serial.println(oneshotTime);
      
    if(autoRunSt == RUN || autoRunSt == RELEASE  ){
        //自動前進中
        if(c == M_OFF ){
            c = M_FRONT;
           // Serial.println("Convert FF");


        }else if(c == M_LEFT){
            c = M_F_LEFT;
        }else if(c == M_RIGHT){
            c = M_F_RIGHT;
        }
    }else{
      if(c != M_ONON){
        autoRunTimer = 0;
      }
    }




    if (c == M_BACK){
        // 後進設定
        backflag = 1;
        joyset(M_BACK);
        c = M_BACK;
    }else if(c== M_ONON){
        if (backflag){
            joyset(M_BACK);
        }else{
            joyset(M_FRONT);
        }
    }else if(c==M_OFF){
        backflag = 0;
        joyset(M_OFF);
    }else{
        joyset(c);
    }

    // タッチパネル処理
    intouch();
    // 表示リフレッシュ（バー）
    disprefresh();

  }
}

//待機初期化
void m_waiting_ini(void)
{
  plot_mode_F(Waiting);   //ボタン表示
  wait_flag = false;
  tftledTimer = millis()+TC_TFTLED;      //TFTディスプレーバックライトタイマー
  TFT_LED_ON;
  delay(100);
  while(inEmg()){
    delay(10);
  }
}


//待機
//右連打5回か左長噛みで走行モード、左短で次のファンクション
void m_waiting(void)
{
  static unsigned long t;
  for(;;){
    // タッチパネル処理
    intouch();
    if(ts.touched()){
      tftledTimer = millis()+TC_TFTLED;      //TFTディスプレーバックライトタイマー
      TFT_LED_ON;
    }
    // 表示リフレッシュ（バー）
    disprefresh();
    if(exit_flag){
      TFT_LED_ON;
      return;
    }

    uint8_t c =  inEmgExit();
    rendachek(c);
    if( wait_flag){
      //モード移行要求あり 移動モード
      newmode = Moving; 
    }
    if(exit_flag){
      //長噛み
       TFT_LED_ON;
       return;
    }
    if (tftledTimer<millis()){
      tftledTimer=millis()+3600000;
      TFT_LED_OFF;
    }
    delay(10);
  }
}

//YAMAHA モードSW
//on/offのワンショット動作
void m_modesw_ini(void)
{
  plot_mode_F(Modesw);   //ボタン表示  
}
void m_modesw(void)
{
  for(;;){
    // タッチパネル処理
    mode_f = false;
    intouch();
    // 表示リフレッシュ（バー）
    disprefresh();
    if(exit_flag){
      return;
    }

    int c = inEmgExit();
    if(exit_flag){
      //exit
      return;
    }
    if(emgOnEdge & rightB || modeButton == Modesw){
      //設定SW切り替えON
      modeButton = -1;
      modeOn;
      bzz100;
      modeSW_disp();
      Serial.print("modeButton:");Serial.println(modeButton);
      delay(200);
      modeOff;
      plot_mode_F(Modesw);   //ボタン表示  
      //savePrm();
     }
     
    delay(100);
  }  
}

///--------MOUSE button
void m_mouseing_ini(void)
{
  plot_mode_F(Mouseing);   //ボタン表示  
}
void m_mouseing(void)
{
  onekeymose();
}

//--------------Buzzer button
void m_buzzing_ini(void)
{
  plot_mode_F(Buzzing);   //ボタン表示  
}
void m_buzzing(void)
{
  for(;;){
    // タッチパネル処理
    bzz_f = false;          
    intouch();
    // 表示リフレッシュ（バー）
    disprefresh();
    int c = inEmgExit();
    if(exit_flag){
      //exit
      return;
    }
    if(c & rightB || now_modeButton == Buzzing){
      //ON
      bzzOn;
    }else{
      bzzOff;
    }
    delay(10);
  }
}

//値を設定
void setting_set(int f, int d)
{
  switch(f){
    case 0:
      ok_lowspeed = d;
      break;

    case 1:
      ok_hispeed = d;
      break;

    case 2:
      ok_tchilow = d;
      break;

    case 3:
      ok_tclong = d;
      break;

    case 4:
      ok_tcshortoff = d;
      break;

    case 5:
      ok_tclongoff = d;
      break;   
  }
}

void setting_disp(int f)
{
  #define ST_X0 4 
  #define ST_Y0 20
  #define ST_GX0 ((8*8)+ST_X0)    //グラフ位置
  #define ST_DX   16              //横間隔
  #define ST_DY   24              //縦間隔
  #define ST_NN   6               //項目数
  
  int d;
  tft.setTextColor(TXT_COLOR );
  tft.setTextSize(1);
  tft.setCursor(ST_X0, ST_Y0+ST_DY*f);

  switch(f){
    case 0:
      tft.println("Low Speed");
      d = ok_lowspeed;
      break;

    case 1:
      tft.println("Hi Speed");
      d = ok_hispeed;
      break;

    case 2:
      tft.println("Low to Hi");
      tft.setCursor(ST_X0, tft.getCursorY());
      tft.println("     Time");
      d = ok_tchilow;
      break;

    case 3:
      tft.println("Detect ");
      tft.setCursor(ST_X0, tft.getCursorY());
      tft.println("Long Time");
      d = ok_tclong;
      break;

    case 4:
      tft.println("Short");
      tft.setCursor(ST_X0, tft.getCursorY());
      tft.println("OFF Time");
      d = ok_tcshortoff;
      break;

    case 5:
      tft.println("Long");
      tft.setCursor(ST_X0, tft.getCursorY());
      tft.println("OFF Time");
      d = ok_tclongoff;
      break;   
  }

  for(int j=0;j<8;j++){
    tft.fillRect(ST_GX0+ST_DX*j, ST_Y0+ST_DY*f, 15, 15, GR_BLUE);
    if(j==d){
      tft.fillRect(ST_GX0+ST_DX*j, ST_Y0+ST_DY*f, 15, 15, GR_RED);
    }
    tft.drawRect(ST_GX0+ST_DX*j, ST_Y0+ST_DY*f, 15, 15, GR_WHITE);
  }  
}


// セッテイングタッチパネル処理
void setting_intouch(void)
{
#define TOUCH_Z_LOW 5        //　タッチ有効 LOWサイド 
#define TOUCH_Z_HI  4000    //　タッチ有効 Hiサイド 

static uint8_t ttflsg=0;        // 連続防止タッチフラグ
int xx,yy,f;

    if (millis()>touchTimer){
        // 処理時間
        touchTimer = millis() +TC_50;    // 次の処理時間
        if(ts.touched()==0){
          touchFlag = 0;
        }else{        
          if(touchFlag == 0){        
            //new touch    
            TS_Point p = getPointEx();
            touchFlag = 1;
            xx = p.x;
            yy = p.y;
  
            // タッチ解析実行
            // modeボタン
            if(xx>P_MODE_X0){
              int wy = (yy-P_MODE_Y0)/P_MODE_DY;  //モード番号
              newmode = wy;                 
              exit_flag = true;
              return;
            }
            //設定チェック
            f = ((yy-ST_Y0)/ST_DY);   //項目 no
            if(f>=0 && f<ST_NN){
              int n = (xx-ST_GX0)/ST_DX;   //設定番号　0-7           
              if (n>=0 && n<8){
                //pint
                setting_set(f,n);
                setting_disp(f);
              }           
            }
          }
        }
    }
}


//--------------Setting button
void m_setting_ini(void)
{

  plot_mode_F(Setting);   //ボタン表示  
  //エリアclear
  tft.fillRect(0, 0,  P_MODE_X0-16, 240, tft.color565(0, 0, 0));
  yield();
  for(int i=0;i<ST_NN;i++){
    setting_disp(i);
  }
}

void m_setting(void)
{
  for(;;){
    // タッチパネル処理
    bzz_f = false;          
    
    setting_intouch();

    int c = inEmgExit();
    if(exit_flag == true ){
      //exit
      savePrm2();
      tft.fillRect(0, 0,  P_MODE_X0-16, 240, tft.color565(0, 0, 0));
      moving_base_disp();       //移動ベース画像表示
      return;
    }
    delay(10);
  }
}




void setup()
{
    uint8_t sum =0;
    uint8_t k1,k2;

  Serial.begin(115200);
  Serial.println("ILI9341 Test!"); 
  //serial1.begin(115200,SERIAL_8N1,14,13); //bps,config,Rx,Tx
  serial1.begin(115200,SERIAL_8N1,14,12); //bps,config,Rx,Tx
                                          //begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms)

  
  //タッチパネル
  ts.begin();
  ts.setRotation(3);

  tft.begin();
  tft.setRotation(1); //横置き コネクタ右側

  EEPROM.begin(128);  // EEPROM開始（サイズ指定）
 
  bleMouse.begin();     //ble start

    timer = millis();            // 変化時間検出用時間保持
    touchTimer = millis();        // タッチパネル検出時間間隔用
    delay(5);
    barTimer = millis() ;        // センサー信号表示時間用

    pinMode( OUT_FORWARD,OUTPUT);        // 方向
    pinMode( OUT_REVERSE,OUTPUT);
    pinMode( OUT_LEFT,OUTPUT);
    pinMode( OUT_RIGHT,OUTPUT);
    pinMode( OUT_BZZ,OUTPUT);
    pinMode( OUT_MODE,OUTPUT);
    pinMode( PIN_LED0,OUTPUT);
    pinMode( tft_LED,OUTPUT);
    TFT_LED_ON;
    
    digitalWrite(OUT_FORWARD,LOW);
    digitalWrite(OUT_REVERSE,LOW);
    digitalWrite(OUT_LEFT,LOW);
    digitalWrite(OUT_RIGHT,LOW);
    digitalWrite(OUT_BZZ,LOW);
    digitalWrite(OUT_BZZ,LOW);

    pinMode( TWE_STOP,INPUT_PULLUP);
    pinMode( TWE_SIG,INPUT_PULLUP);
   // pinMode( 3,INPUT_PULLUP);

    digitalWrite(OUT_BZZ,HIGH);
    delay(500);
    digitalWrite(OUT_BZZ,LOW);

    // パネル描画
    tft.fillScreen(ILI9341_BLACK);
    plot_barBase();           // 信号バーの表示
    plot_timeSw();            // 時間設定スイッチの表示
    plot_led_F(0);            // LEDを表示
    plot_led_B(0);            // LEDを表示
    plot_led_L(0);            // LEDを表示
    plot_led_R(0);            // LEDを表示
    //plot_autoSw(0);            // オート閾値設定のボタン表示


    // Bar表示処理
    mark1 = mark2 = mark1old = mark2old = SIG_MAX;
    loadPrm();

    //第2パラメータ読み込み
    loadPrm2();
    
    // 閾値表示
    disp_thr1_now(mark1);
    disp_thr2_now(mark2);

    // ワンショット設定数値の表示
    dispOneshot(oneshotTime);    
    oneshotWrite(oneshotTime);        // 設定出力
    
    sig1old = 0;
    sig1 = 1;
    sig2old = 0;
    sig2 = 1;
    disp_bar_now();

/*  Demo of 
    void drawCircle(int poX, int poY, int r,unsigned int color) and
    void fillCircle(int poX, int poY, int r,unsigned int color);
tft.drawCircle(100, 100, 30,YELLOW);
tft.drawCircle(100, 200, 40,CYAN);
tft.fillCircle(200, 100, 30,RED);
tft.fillCircle(200, 200, 30,BLUE);

*/

    pinMode( PIN_LED0, OUTPUT );
    digitalWrite( PIN_LED0,1);
    
      drawString("Templer system",228,228,1,TXT_COLOR);
}

//--------------main loop--------------
void loop()
{
    int i=0;
    uint16_t xx,yy;
    uint8_t c = M_OFF;
    unsigned long t;
/*
    pinMode( 15,INPUT);
    pinMode( 17,INPUT);
    pinMode( 14,OUTPUT);
    pinMode( 16,OUTPUT);
    digitalWrite( 14,0);
    digitalWrite( 16,1);
 */
    newmode = Waiting;
    nowmode = Moving;
    plot_mode_F(-1);
    moving_base_disp();
    
    while(1){
      
         exit_flag = false;

        if(newmode >= EndMode){
          newmode = Moving;
        }
        Serial.print("mode:");
        Serial.println(newmode);
        
        if(nowmode != newmode){
          plot_mode_F(-1);
          nowmode = newmode;
          if(nowmode == Moving){       //走行モード
            m_moving_ini();
          }else if(nowmode == Waiting){      //待機モード
            m_waiting_ini();
          }else if(nowmode == Modesw){       //YAMJAHAモードスイッチ切り替えモード
            m_modesw_ini();
          }else if(nowmode == Mouseing){     //マウス操作モード
            m_mouseing_ini();
          }else if(nowmode == Buzzing){      //ブザー鳴動
            m_buzzing_ini();
          }else if(nowmode == Setting){      //パラメータ設定
            m_setting_ini();
          }
        }
        //mode
        //左長押しチェック
        t = millis()+500;
        while (leftB & inEmg()){
          if(t<millis()){
            nowmode = Moving; //長押し走行モードへ
    plot_mode_F(-1);   //ボタン表示  
    plot_mode_F(Moving);      //ボタン
            
            bzzOn;
            ticker1bzz.once_ms(200, bzz_stop);
            break;
          }
          delay(5);
        }
        exit_flag = false;
        while (leftB & inEmg());
        //タッチOFF待ち
        while(ts.touched()){
          delay(10);
        }
        //mode実行
        if(nowmode == Moving){       //走行モード
          m_moving_ini();
          while(inEmg());
          m_moving();
        }else if(nowmode == Waiting){      //待機モード
          m_waiting();
        }else if(nowmode == Modesw){       //YAMJAHAモードスイッチ切り替えモード
          m_modesw();
        }else if(nowmode == Mouseing){     //マウス操作モード
          m_mouseing();
        }else if(nowmode == Buzzing){      //ブザー鳴動
          m_buzzing();
          }else if(nowmode == Setting){      //パラメータ設定
            m_setting();
        }



        
        // タッチパネル処理
       // intouch();

        // 表示リフレッシュ（バー）
       // disprefresh();

        if(serial1.available()){
          Serial.write(serial1.read());
        }
    }

}
















unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.fillScreen(ILI9341_RED);
  yield();
  tft.fillScreen(ILI9341_GREEN);
  yield();
  tft.fillScreen(ILI9341_BLUE);
  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(10, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");

  return micros() - start;


  
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  yield();
  
  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i*10, i*10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i*10, i*10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=6) {
    i2 = i / 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()); i>20; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}
