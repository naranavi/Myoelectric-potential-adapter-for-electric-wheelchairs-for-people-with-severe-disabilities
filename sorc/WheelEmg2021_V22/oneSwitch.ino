

/*
 * ワンキーマウスのようにスイッチの長短を解析します。
 * 
 * 短のみのとき(2)0b10
 * 長のみのとき(3)0b11
 * 短長のとき (5)0b101
 */
//#include <Ticker.h>
//#include <Arduino.h>
//#include "oneSwitch.h"

void bzzLong(void);
boolean getSw(void);
int swProc(void);


// 長短判定時間>>ms  1    2   3   4   5   6   7   8   
int OK_tclong[] = {800,700,600,500,450,400,350, 300};
#define OK_tclongs ((sizeof OK_tclong) / (sizeof OK_tclong[0]))

// 短押し終了時間>>ms   1    2   3   4   5   6   7   8   
int OK_tcshortoff[] = {800,700,600,500,450,400,350,300};
#define OK_tcshortoffs ((sizeof OK_tcshortoff) / (sizeof OK_tcshortoff[0]))

// 長押し終了時間>>ms   1    2   3   4   5   6   7   8   
int OK_tclongoff[] = {800,700,600,500,450,400,350,300};
#define OK_tclongoffs ((sizeof OK_tclongoff) / (sizeof OK_tclongoff[0]))

// マウス高速定値>>ms  1   2   3   4   5   6   7   8   
///int OK_hispeed[] =    {70,60,50, 40,30, 20 ,10 ,5};
int OK_hispeed[] =    {30,15,13, 12,10, 10 ,10 ,10};
#define OK_hispeeds ((sizeof OK_hispeed) / (sizeof OK_hispeed[0]))
// マウス高速移動時の移動幅
///int Dot_hispeed[] =  {2,2,3,3,3,4,4 ,4};
int Dot_hispeed[] =  {3,3,4,5,5,6,7 ,8};

// マウス低速定値>>ms  1   2   3   4   5   6   7   8   
///int OK_lowspeed[] =  {350,300,250,200,150,100, 75 ,50};
int OK_lowspeed[] =  {200,100,50,33,50,40, 33 ,50};
#define OK_lowspeeds ((sizeof OK_lowspeed) / (sizeof OK_lowspeed[0]))
// マウス低速移動時の移動幅
int Dot_lowspeed[] =  {1,1,1,1,2,2,2 ,3};
#define Dots_lowlen 80          //low speed 移動距離
// マウス低速高速切替時間定値>>ms  1    2    3    4    5    6    7    8   
///int OK_tchilow[] =              {500,1000,1500,2000,2500,3000,3500 ,4000};
// 移動距離
int OK_tchilow[] = {40,60,80,100,120,140,160 ,180};
#define OK_tchilows ((sizeof OK_tchilow) / (sizeof OK_tchilow[0]))



int mouse_dir;                      //移動方向 0:停止
int mouse_speedtime;                //マウス移動速度 コマンド送信間隔ms
int mouse_speed;                    //マウス移動速度 コマンド送信、移動距離

/*
int ok_lowspeed;                    //低速移動スピード番号
int ok_hispeed;                     //高速移動スピード番号
int ok_tchilow;                     //低速高速切替時間番号
int ok_tclong;                      //長短判定時間番号
int ok_tcshortoff;                 //短押し終了時間番号
int ok_tclongoff;                  //長押し終了時間番号
*/

unsigned long mouse_speedtimer;     //マウス移動速度タイマー
unsigned long mouse_lowhichangtimer; //マウス移動速度切り替えタイマー

int mouse_tclong;                   //長短判定時間ms
int mouse_tcshortoff;               //短押し終了時間ms
int mouse_tclongoff;                //長押し終了時間ms
int mouse_hispeed;                  // マウス高速定値>>ms
int mouse_lowspeed;                 // マウス低速定値>>ms
int mouse_tchilow;                 //マウス低速高速切替時間定値>>ms


//Ticker ticker1bzz;

//Serial Serial(0); 

int Bzz;       //3番がブザー出力とする
int TcLONG;    //長検出時間[ms]
//int TcEND;    //操作終了時間[ms]
int thLevel;  //ON/OFF判定閾値
int thLevelLow;//ヒステリシスON/OFF判定閾値



void bzz_stop() {
  digitalWrite(OUT_BZZ,LOW);
  //Serial.print("stop: ");
  //Serial.println(millis());

}

//ok sound
void  setTon()   
{
  digitalWrite(OUT_BZZ,HIGH);
  delay(70);
  digitalWrite(OUT_BZZ,LOW);
  delay(30);
  digitalWrite(OUT_BZZ,HIGH);
  delay(30);
  digitalWrite(OUT_BZZ,LOW);
    
}


//スイッチがGNDに接続されたときONとします。
boolean getSw(void)
{
  static boolean hysflag;

  //return (digitalRead(2)==LOW);       ///

  int adc2 = analogRead(A3)/8;    //A3:right  A0:left

  if ( adc2 >= thr2now){
    //ON
    thLevelLow =  thr2now/2;
    hysflag = true;
    return true;
  }else{
    if(hysflag){
      if(adc2 >= thLevelLow){
        //on
        return true;
      }
    }
    hysflag = false;
    return false;
  }
}


//マウス・ワンキー系設定値読み出し
void getMousPrm(void)
{
  //長短判定時間
  mouse_tclong = OK_tclong[ok_tclong];

  // 短押し終了時間>>ms
  mouse_tcshortoff = OK_tcshortoff[ok_tcshortoff];

  // 長押し終了時間>>ms
  mouse_tclongoff = OK_tclongoff[ok_tclongoff];

  // マウス高速定値>>ms
  mouse_hispeed = OK_hispeed[ok_hispeed];

  // マウス低速定値>>ms
  mouse_lowspeed = OK_lowspeed[ok_lowspeed];

  // マウス低速高速切替時間定値>>ms  
  mouse_tchilow = OK_tchilow[ok_tchilow];

}


void oneSwitch_ini(void)
{
  // serial
  pinMode(OUT_BZZ, OUTPUT);
  digitalWrite(OUT_BZZ, LOW);
  
}

//ワンスイッチ入力
//dir
int oneSwitch(int dir){
  int d=0;
  
  
  if(getSw()){
    Serial.println("sw on!");
    
    if(dir){
      //マウス移動中のため一旦停止
      bleMouse_move(0,0);
    }
    
     d = swProc();
     Serial.print("onkey hex:"); Serial.println(d,HEX);
  }
  return d;
}









//sw 処理
int swProc(void)
{
  unsigned long ts,te;
  int f=1;

  
  for(;;){
    f = f << 1;
    ts = millis()+mouse_tclong;
    boolean fLong = false;  //長検出済みフラグ
    bzzShort();

    while(getSw()){
      if (millis()>=ts && fLong == false){
        //長検出 
        fLong = true;
        bzzLong();
        f = f | 1;
      }
    }
    delay(10);
    //SW off
    if(fLong){
      //長
      te = millis()+mouse_tclongoff;
    }else{
      //短
      te = millis()+mouse_tcshortoff;
    }

   
    while(getSw()== false){
      if (millis()>te){
        //入力終了
        return f;
        } 
     }    
  }
    
}

void bzzShort(void)
{
  digitalWrite(OUT_BZZ,HIGH);
  Serial.println(millis());
  ticker1bzz.once_ms(20, bzz_stop);
  //delay(20);
  //digitalWrite(Bzz,LOW);
  
}

void bzzLong(void)
{
  digitalWrite(OUT_BZZ,HIGH);
  Serial.println(millis());
  ticker1bzz.once_ms(80, bzz_stop);
  //delay(80);
  //digitalWrite(Bzz,LOW);
  
}


//ble mouse move
void bleMouse_move(signed char x, signed char y, signed char wheel , signed char hWheel )
{

  Serial.print("mouse move x:");
  Serial.print(x);
  Serial.print(" y:");
  Serial.print(y);
  Serial.print(" wheel:");
  Serial.print(wheel);
  Serial.print(" hWheel:");
  Serial.println(hWheel);

  bleMouse.move(x,  y,  wheel, hWheel);
}

//ble mousue move no.
void bleMouse_move_no(int n)
{
  if(n == 1){
    bleMouse_move(mouse_speed,0); //right
  }if(n == 2){
    bleMouse_move(0,mouse_speed); //down
  }if(n == 3){
    bleMouse_move(-mouse_speed,0); //left
  }if(n == 4){
    bleMouse_move(0,-mouse_speed); //up      
  }
} 

// ble mouse clic
void bleMouse_click(uint8_t b = MOUSE_LEFT)
{
  Serial.print("mouse click:");
  Serial.println(b);

  bleMouse.click(b);
}

//ble mouse prss
void bleMouse_press(uint8_t b = MOUSE_LEFT)
{
  Serial.print("mouse press:");
  Serial.print(b);

  bleMouse.press(b);

}

//方向適正化
int mouse_dir_chk(int d)
{
  if (d>4){
    d = d -4;
  }
  return d;
}

//スピードLow 設定
void speedLow()
{
    mouse_lowhichangtimer = millis()+OK_tchilow[ok_tchilow]; 
    mouse_speedtime = OK_lowspeed[ok_lowspeed];
    mouse_speed = Dot_lowspeed[ok_lowspeed];
    mouse_lowhichangtimer = millis()+(OK_tchilow[ok_tchilow]*mouse_speedtime)/mouse_speed; 

    
}


void onekeymose(void)
{
  int d;
  int c;
////int  d = oneSwitch(111, 22, 33, 4);
  
  //設定値ロード
  getMousPrm();
  mouse_dir = 0;

  pinMode(2,INPUT_PULLUP);

  while(getSw()){
    //offまで待つ
    // タッチパネル処理
    intouch();
    // 表示リフレッシュ（バー）
    disprefresh();
    if(exit_flag){
      return;
    }
    c = inEmgExit();
    if(exit_flag){
      //exit
      return;
    }
    if(c & rightB){
      //ON
      bzzOn;
    }else{
      bzzOff;
    }
    delay(10);
    
  }

  
  for(;;){

    d = oneSwitch(mouse_dir);
    //Serial.print("d=");Serial.println(d,HEX);

    if(d!=0 && mouse_dir !=0){
      //移動中
      if(d == 0b10){
        //方向変更１
         mouse_dir+=1;
         mouse_dir = mouse_dir_chk(mouse_dir);
         speedLow();  //スピードLow 設定
         bleMouse_move_no(mouse_dir);
         d = 0;
      }else if(d == 0b100){
        //方向変更２
        mouse_dir+=2;
         mouse_dir = mouse_dir_chk(mouse_dir);
         speedLow();  //スピードLow 設定
         bleMouse_move_no(mouse_dir);
         d = 0;
      }else if(d == 0b1000){
        //方向変更３
        mouse_dir+=3;
         mouse_dir = mouse_dir_chk(mouse_dir);
         speedLow();  //スピードLow 設定
         bleMouse_move_no(mouse_dir);
         d = 0;
      }else if(d !=0b10){
        //stop
        mouse_dir = 0;
        d = 0;
        bleMouse_move(0,0);
      }
    }
    if(d == 0){
      //継続動作
      if(mouse_dir){
        //移動中
        if (millis()>mouse_lowhichangtimer){
          //lowspeed-->hispeed
          Serial.print("mouse speed hi!!");
          mouse_speedtime = OK_hispeed[ok_hispeed];
          mouse_speedtimer = mouse_speedtime + millis();
          mouse_lowhichangtimer = millis()+3600000;
          mouse_speed = Dot_hispeed[ok_hispeed];
        }
        if(millis()>mouse_speedtimer){
          //mouse move
          mouse_speedtimer = mouse_speedtime + millis();
          bleMouse_move_no(mouse_dir);
        }

      }
    }else if(d == 0b10){
      //短点
      if (mouse_dir == 0){
        //mouse stop
        mouse_dir = 1;    //start
        //スピードLow
        speedLow();  //スピードLow 設定
        bleMouse_move(mouse_speed,0); //right
        
      }else{
        mouse_dir++;     //方向転換
        if(mouse_dir>4)mouse_dir = 1;
        //スピードLow
         speedLow();  //スピードLow 設定
        bleMouse_move_no(mouse_dir);
      }
    }else if(d == 0b11){
      //長点 左クリック
        bleMouse_click(MOUSE_LEFT);
        delay(200);
    
    }else if(d == 0b111){
      //ダブルクリック
      bleMouse_click(MOUSE_LEFT);
      delay(170);
      bleMouse_click(MOUSE_LEFT);
      delay(100);
 
    
    }else if(d == 0b1111){
      //右クリック
      bleMouse_click(MOUSE_RIGHT);
    
    }else if(d == 0b110){
      //左ドラッグ低速移動開始
      mouse_dir = 1;    //start
      speedLow();  //スピードLow 設定
      bleMouse_press(MOUSE_LEFT);
      delay(20);
      bleMouse_move(mouse_speed,0); //right
    
    }else if(d == 0b101){
      //低速移動開始
      mouse_dir = 1;    //start
      mouse_lowhichangtimer = millis()+30000; 
      mouse_speedtimer = OK_lowspeed[ok_lowspeed]*2+millis();
      mouse_speed = Dot_lowspeed[ok_lowspeed];
      bleMouse_move(mouse_speed,0); //right
    
    }else if(d == 0b100){
      //移動開始下
      mouse_dir = 2;    //start
      speedLow();  //スピードLow 設定
      bleMouse_move(0,mouse_speed); //down

    }else if(d == 0b1000){
      //移動開始左
      mouse_dir = 3;    //start
      speedLow();  //スピードLow 設定
      bleMouse_move(-mouse_speed,0); //left

    }else if(d == 0b10000){
      //移動開始上
      mouse_dir = 4;    //start
      speedLow();  //スピードLow 設定
      bleMouse_move(0,-mouse_speed); //up

    }else if(d == 0b11110){
      //右ドラッグ移動開始右
      mouse_dir = 1;    //start
      speedLow();  //スピードLow 設定
      bleMouse_press(MOUSE_RIGHT);
      delay(20);
      bleMouse_move(mouse_speed,0); //right
      
    }else if(d == 0b11111){
        //スクロールボタン
        bleMouse_click(MOUSE_MIDDLE);

    }else if(d == 0b1101){
        //スクロール上ボタン
        bleMouse_move(0,0,1);;

    }else if(d == 0b11010){
        //スクロール下ボタン
        bleMouse_move(0,0,-1);

    //設定
    }else if(d == 0b11101){
        //スピードアップ
        ok_hispeed++;
        if(ok_hispeed>=OK_hispeeds)ok_hispeed = OK_hispeeds-1;
        mouse_hispeed = OK_hispeed[ok_hispeed];
        Serial.print("hi speed:");Serial.print(ok_hispeed);Serial.print(" time:");Serial.print(mouse_hispeed);
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存
        
    }else if(d == 0b11011){
        //スピードダウン
        ok_hispeed--;
        if(ok_hispeed<0)ok_hispeed = 0;
        mouse_hispeed = OK_hispeed[ok_hispeed];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存
        
    }else if(d == 0b11101){
        //低速スピードアップ
        ok_lowspeed++;
        if(ok_lowspeed>=OK_lowspeeds)ok_lowspeed = OK_lowspeeds-1;
        mouse_lowspeed = OK_lowspeed[ok_hispeed];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }else if(d == 0b110100){
        //低速移動時間短く
        ok_tchilow--;
        if(ok_tchilow<0)ok_tchilow = 0;
        mouse_tchilow = OK_tchilow[ok_tchilow];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }else if(d == 0b110101){
        //低速移動時間長く
        ok_tchilow++;
        if(ok_tchilow>=OK_tchilows)ok_tchilow = OK_tchilows-1;
        mouse_tchilow = OK_tchilow[ok_tchilow];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }else if(d == 0b11011){
        //低速スピードダウン
        ok_lowspeed--;
        if(ok_lowspeed<0)ok_lowspeed = 0;
        mouse_lowspeed = OK_lowspeed[ok_hispeed];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存
        
        
    }else if(d == 0b111011){
        //長短判定時間短く
        ok_tclong++;
        if(ok_tclong>=OK_tclongs)ok_tclong = OK_tclongs-1;
        mouse_tclong = OK_tclong[ok_tclong];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }else if(d == 0b110111){
        //長短判定時間長く
        ok_tclong--;
        if(ok_tclong<0)ok_tclong = 0;
        mouse_tclong = OK_tclong[ok_tclong];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }else if(d == 0b111001){
        //短押し終了時間短く
        ok_tcshortoff++;
        if(ok_tcshortoff>=OK_tcshortoffs)ok_tcshortoff = OK_tcshortoffs-1;
        mouse_tcshortoff = OK_tcshortoff[ok_tcshortoff];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }else if(d == 0b110001){
        //短押し終了時間時間長く
        ok_tcshortoff--;
        if(ok_tcshortoff<0)ok_tcshortoff = 0;
        mouse_tcshortoff = OK_tcshortoff[ok_tcshortoff];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

     }else if(d == 0b101001){
        //長押し終了時間短く
        ok_tclongoff++;
        if(ok_tclongoff>=OK_tclongoffs)ok_tclongoff = OK_tclongoffs-1;
        mouse_tclongoff = OK_tclongoff[ok_tclongoff];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }else if(d == 0b100001){
        //長押し終了時間時間長く
        ok_tclongoff--;
        if(ok_tclongoff<0)ok_tclongoff = 0;
        mouse_tclongoff = OK_tclongoff[ok_tclongoff];
        setTon();     //ok sound
        savePrm2();   //EEPROM パラメータ保存

    }






    // タッチパネル処理
    intouch();
    // 表示リフレッシュ（バー）
    disprefresh();
    if(exit_flag){
      return;
    }
    c = inEmgExit();
    if(exit_flag){
      //exit
      return;
    }
    if(c & leftB){
      //ON
      bzzOn;
    }else{
      bzzOff;
    }
    ///delay(10);
  }
  



}
