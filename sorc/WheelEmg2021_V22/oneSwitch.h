/*
 * ワンキーマウスのようにスイッチの長短を解析します。
 * 
 * 短のみのとき(2)0b10
 * 長のみのとき(3)0b11
 * 短長のとき (5)0b101
 */
 int oneSwitch(int th, int longtc, int endtc, int bzz);
 void onekeymose(void);
 void bleMouse_move(signed char x, signed char y, signed char wheel = 0, signed char hWheel = 0);
int ok_lowspeed;                    //低速移動スピード番号
int ok_hispeed;                     //高速移動スピード番号
int ok_tchilow;                     //低速高速切替時間番号
int ok_tclong;                      //長短判定時間番号
int ok_tcshortoff;                 //短押し終了時間番号
int ok_tclongoff;                  //長押し終了時間番号
