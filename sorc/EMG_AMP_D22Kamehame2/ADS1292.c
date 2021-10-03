//=========================================================
// ADS1292 Project
//=========================================================
// File Name : ADS1292.c
// Function  : ADS1292 Control
// ADS1292 Texas Instruments
//  �����d�́A2 �`���l���A24 �r�b�g�A�A�i���O�E�t�����g�G���h�A�����d�C�M���̑���p
//  �o�[�W�����Q���
//---------------------------------------------------------
// 2013.11.19 Yoshimi Sugimoto
// 

#include <avr/io.h>
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "ADS1x9x.h"
#include "integer.h"
#include "main.h"
#include "serialLed.h"
#include "softserial.h"

// ADS1292 �����N���b�N�Z���N�g�iCLKSEL=Hi�j
#define DELAY_COUNT 2


//
// port
#define ADS_DDR 	DDRD	// �����ݒ�|�[�g
#define ADS_PORT	PORTD	// �|�[�g
#define ADS_PORTB	PORTB	// �|�[�g
#define ADS_PIN		PIND	// �|�[�g
#define ADS_PINB	PINB	// �|�[�g

#define ADS_RXD 	PORTD0	// SPI �f�[�^����
#define ADS_TXD 	PORTD1	// SPI �f�[�^�o��
#define ADS_XCK 	PORTD4	// SPI clock
#define ADS_CSN 	PORTD2	// ADS CS# �Z���N�g
//#define ADS_DRDY 	PORTD5	// ADS /DRDY
//#define ADS_DRDY_INT PCINT21	// /DRDY���荞�݃|�[�g
//#define ADS_PCMSK	PCMSK2	// PCINT21 �}�X�N���W�X�^
#define ADS_DRDY 	PORTB0	// ADS /DRDY
#define ADS_DRDY_INT PCINT0	// /DRDY���荞�݃|�[�g
#define ADS_PCMSK	PCMSK0	// PCINT0 �}�X�N���W�X�^
#define ADS_PCIF	PCIF0	// �s���ω����荞�ݗv���t���O
#define	ADS_PCIE	PCIE0

#define ADS_START 	PORTD6	// ADS START
#define ADS_RESN 	PORTD7	// ADS RES# ���Z�b�g

#define ADS_CS_ON	ADS_PORT &=~(1<<ADS_CSN)	// ADS �I��
#define ADS_CS_OFF	ADS_PORT |=(1<<ADS_CSN)		// ADS ��I��
#define ADS_RES_ON	ADS_PORT &=~(1<<ADS_RESN)	// ADS RESET���s
#define ADS_RES_OFF	ADS_PORT |=(1<<ADS_RESN)	// ADS REST����
#define ADS_DRDY_UP	ADS_PORTB |=(1<<ADS_DRDY)	// ADS DRDY �v���A�b�v

#define ads_drdy	(~ADS_PINB & (1<<ADS_DRDY))		// �ϊ��ς݃f�[�^�����邩�e�X�g
#define ADS_START_ON	ADS_PORT |=(1<<ADS_START)	// ADS�A���J�n�n�[�h�i��
#define ADS_START_OFF	ADS_PORT &=~(1<<ADS_START)	// ADS�A����~�n�[�h�i��


#define ADS_TXD_ON	ADS_PORT |=(1<<ADS_TXD)		// ADS TX Hi
#define ADS_TXD_OFF	ADS_PORT &=~(1<<ADS_TXD)	// ADS TX Low
#define ADS_XCK_ON	ADS_PORT |=(1<<ADS_XCK)		// ADS CK Hi
#define ADS_XCK_OFF	ADS_PORT &=~(1<<ADS_XCK)	// ADS CK Low



#define INIPORT ADS_DDR |= (1<<ADS_TXD)|(1<<ADS_XCK)|(1<<ADS_CSN)|(1<<ADS_RESN)|(1<<ADS_START) \
				;ADS_CS_OFF \
				;ADS_RES_OFF \
				;ADS_TXD_OFF \
				;ADS_XCK_OFF \
				;ADS_START_OFF \
				;ADS_DRDY_UP


// Global Variables										                                  											  *

unsigned char SPI_Tx_buf[10];
volatile unsigned char SPI_Rx_Data_Flag = 0,  SPI_Rx_buf[12], SPI_Rx_Count=0, SPI_Rx_exp_Count=0 ;

volatile BYTE tx_cnt=0;

volatile BYTE val_flag = 0;
volatile BYTE stat1=0;
volatile BYTE stat2=0;
volatile BYTE stat3=0;
volatile BYTE timer1=0;
volatile BYTE timer2=0;

volatile WORD ch_A = 0;
volatile WORD ch_B = 0;

//FIR
volatile long ch1_60_s1=0, ch1_60_s2=0; 
volatile long ch1_50_s1=0, ch1_50_s2=0; 
volatile long ch2_60_s1=0, ch2_60_s2=0; 
volatile long ch2_50_s1=0, ch2_50_s2=0; 

volatile long w1=0, w2=0, w3=0, w4=0; 
volatile long ch1w=0, ch2w=0; 
volatile ULONG uw1=0; 

volatile WORD ww2=0;
volatile int wi2=0; 
volatile int dt1=0,dt2=0,dt2w,dat2=0;
long dc2=200; 

//#define SMP_MAX 50		// �ړ����ϐ�
#define SMP_MAX 10		// �ړ����ϐ�
int  ch1_smp=0;
long ch1_sm[SMP_MAX];			// �ړ����� DC���o�p
int  ch2_smp=0;
long ch2_sm[SMP_MAX];			// �ړ����� DC���o�p

#define SMPW_MAX32 32	// �ړ����ϗ�
#define SMPW_MAX 64	// �ړ����ϗ�
#define SMPW_DVK 4	// PWM�W��

long ch1_smpw[SMPW_MAX];		// �p���[�̈ړ�����
int  ch1_smpwp=0;
long ch2_smpw[SMPW_MAX];		// �p���[�̈ړ�����
int  ch2_smpwp=0;

char sss[64];					// tx buff

/* ADS1x9x Register values*/
unsigned char 	ADS1x9xRegVal[16] = {

	//0:Device ID read Ony
	 0x00,
   	//1:CONFIG1
	 0x02,
    //2:CONFIG2
     0xE0,
    //3:LOFF
     0xF0,
	//4:CH1SET (PGA gain = 6)
     0x00,
	//5:CH2SET (PGA gain = 6)
     00,
	//6:RLD_SENS (default)
	 0x2C,
	//7:LOFF_SENS (default)
	 0x0F,    
    //8:LOFF_STAT
     0x00,
    //9:RESP1
     0xEA,
	//a:RESP2
	 0x03,
	//b:GPIO
     0x0C 
};		
unsigned char 	ADS1x9x_Default_Register_Settings[15] = {

	//00h Device ID read Ony
	 0x00,
   	//01h CONFIG1 500SPS
	 0x02,
    //02h CONFIG2 Lead-off:en Vref:ON Vr:2.42V +CLK_out
		//0xA8,			// vref:on 42.42V clckout test no
		0xe8,			// vref:on 42.42V clckout test no
    //03h LOFF    70%-30% 6nA DC
	 	//0x10,			// 95% 6nA DC
		0x50,			// 90%
	//04h 		PD1 GAIN1_2 GAIN1_1 GAIN1_0 MUX1_3 MUX1_2 MUX1_1 MUX1_0
	 	0x10,			// CH1SET (PGA gain = 1)
	 //05h 		PD2 GAIN2_2 GAIN2_1 GAIN2_0 MUX2_3 MUX2_2 MUX2_1 MUX2_0
 		0x10,			// CH2SET (PGA gain = 1)
 	 //06h RLD_SENS (default) RLDbuff:ON IN2N:�ڑ� IN2P:�ڑ�
	 0x20,				// RLD buffer ON ���ԓd�ʂ����邽��
	 //0x22,
	 //07h LOFF_SENS (default)�@lead-off detection:en
	 //0x00,
	 //0x3f,					// FLIP1,2 enable
	 0x0F,				// LOFFxx enable ����ƊO�ꂽ���ɏo�͂��O�ɂȂ�   
    //08h LOFF_STAT
     0x00,
    //09h RESP1
    0x02,
	//0Ah RESP2 RLDREF_INT:1(AVDD/2)
	0x03,
	 //0x03,
	//0Bh GPIO GPIO input
     0x0C 
};		

//---------------------------------------------------------
// �s���ω����荞��
//---------------------------------------------------------
ISR(PCINT2_vect)
{
}
ISR(PCINT1_vect)
{
}

// DRDY �ω����荞��
ISR(PCINT0_vect)
{

	if (ads_drdy>0){
		// DRDY�L���Ȃ̂�SIO�J�n
		//SPI_Rx_Count = UDR0;					// RXC0�t���O�N���A
		//SPI_Rx_Count = UDR0;
		
		ADS_CS_ON;
		UCSR0B |= (1<<RXCIE0);      // enable RX interrupt
		SPI_Rx_Count = 0;
		LED_ON;
  		// ���̎�M�̂��ߑ��M
		UDR0 = 0; 					// To get Next byte.
	}
//	reti();
}


// SCI��M���荞��
ISR(USART_RX_vect)
{
		int i;
       	
      	SPI_Rx_buf[SPI_Rx_Count] = UDR0;		// in data
      	SPI_Rx_Count++;
      	if ( SPI_Rx_Count == SPI_Rx_exp_Count){
			// �w�萔��M����
			UCSR0B &= ~(1<<RXCIE0);                 // Disable RX interrupt
			ADS_CS_OFF;
			stat1 = SPI_Rx_buf[0];
			stat2 = SPI_Rx_buf[1];
			stat3 = SPI_Rx_buf[2];
			timer1++;
			timer2++;
			timer++;



//ch1�@�E��

			uw1 = (ULONG)SPI_Rx_buf[3]<<16 | (ULONG)SPI_Rx_buf[4]<<8 |(ULONG)SPI_Rx_buf[5];
		//	uw1 =0xffc00001;
			if (SPI_Rx_buf[3] & 0x80){
				w1 = uw1 | 0xff000000;
			}else{
				w1 = uw1;
			}
			w4=w1;

			w1 = w1-ch1w;
			ch1w = w4;

/*
			// �o�C�A�X�Z�o
			ch1_sm[ch1_smp++] = w1;
			if (ch1_smp>=SMP_MAX)ch1_smp=0;
			w3 = 0;
			for(i=0;i<SMP_MAX;i++) w3 +=ch1_sm[i];
			w3 = w3/SMP_MAX;			// ���� DC�o�C�A�X

			w1 = (w1 - w3);		// �o�C�A�X����data


			// FIR 60Hz�m�b�`
			w3 = w1 +ch1_60_s1;	// �t�B���^��̃f�[�^
			w2 = w1 * 458;		//60Hz
			ch1_60_s1 = -w1- w2/1000 + ch1_60_s2;
			ch1_60_s2 = w1;
*/




			// FIR 60Hz�m�b�`
			w3 = w1 +ch1_60_s1;	// �t�B���^��̃f�[�^
			w2 = w1 * 458;		//60Hz
			ch1_60_s1 = -w1- w2/1000 + ch1_60_s2;
			ch1_60_s2 = w1;
			

/*
			// �o�C�A�X�Z�o
			w1=w3;
			ch1_sm[ch1_smp++] = w1;
			if (ch1_smp>=SMP_MAX)ch1_smp=0;
			w3 = 0;
			for(i=0;i<SMP_MAX;i++) w3 +=ch1_sm[i];
			w3 = w3/SMP_MAX;			// ���� DC�o�C�A�X

			w3 = (w1 - w3);		// �o�C�A�X����data
*/



/*
			//�e�X�g�o��
			i = (int)(w3/256/16);
			//i=PWM_MAX/4;
			if (i<0){
				//(-)
				 ch_A =(WORD) (PWM_MAX/2+i);
			}else{
				//(+)
				ch_A =(WORD) (i+ PWM_MAX/2);
			}
*/

/*
			// FIR 50Hz�m�b�`
//@@			w1=w3;
			w3 = w1 +ch1_50_s1;	// �t�B���^��̃f�[�^
			w2 = w1 * 635;   // 50hz
			ch1_50_s1 = -w1- w2/1000 + ch1_50_s2;
			ch1_50_s2 = w1;
*/
/*
			//i=PWM_MAX/4;
			if (w4<0){
				//(-)
					i =(int)( w4/32);
				 ch_B = (PWM_MAX/2+i);
			}else{
				//(+)
					i =(int)( w4/32);
				ch_B =(i+ PWM_MAX/2);
			}
*/
			
			// ch1�p���[
			ch1_smpw[ch1_smpwp++]=labs(w3);		// filter on
//			ch1_smpw[ch1_smpwp++]=labs(w1);		// no filter
			if (ch1_smpwp >= SMPW_MAX) ch1_smpwp=0;

			w1=0;
			for(i=0;i<SMPW_MAX;i++){
				w1 += ch1_smpw[i];
			}
			ch_A = (int)(w1/SMPW_MAX/SMPW_DVK);			// 	

#ifdef AYASE
			ch_A = ch_A/2;	
			if (ch_A >PWM_MAX_ayase) ch_A = PWM_MAX_ayase;
#else
			ch_A = ch_A/2;	// DegiBeet
			if (ch_A >PWM_MAX) ch_A = PWM_MAX;
#endif

//			ch_A = (int)(w1/SMPW_MAX/SMPW_DVK/2);			// 	20130820	
//			ch_A = (int)(w1/256);			// 		

		//ch2

//			uw1 = (ULONG)SPI_Rx_buf[3]<<16 | (ULONG)SPI_Rx_buf[4]<<8 |(ULONG)SPI_Rx_buf[5];
//			if (SPI_Rx_buf[3] & 0x80){
			uw1 = (ULONG)SPI_Rx_buf[6]<<16 | (ULONG)SPI_Rx_buf[7]<<8 |(ULONG)SPI_Rx_buf[8];
			if (SPI_Rx_buf[6] & 0x80){
				w1 = uw1 | 0xff000000;
			}else{
				w1 = uw1;
			}

			w4=w1;
			w1 = w1-ch2w;
			ch2w = w4;


/*
			// �o�C�A�X�Z�o
			ch2_sm[ch2_smp++] = w1;
			if (ch2_smp>=SMP_MAX)ch2_smp=0;
			w3 = 0;
			for(i=0;i<SMP_MAX;i++) w3 +=ch2_sm[i];
			w3 = w3/SMP_MAX;			// ���� DC�o�C�A�X

			w1 = (w1 - w3);		// data
*/



			// FIR 60Hz�m�b�`
			w3 = w1 +ch2_60_s1;	// �t�B���^��̃f�[�^
			w2 = w1 * 458;		//60Hz
			ch2_60_s1 = -w1- w2/1000 + ch2_60_s2;
			ch2_60_s2 = w1;
			
			/*
			//�e�X�g�o��
			i = (int)(w3/256/16);
			//i=PWM_MAX/4;
			if (i<0){
				//(-)
				 ch_A =(WORD) (PWM_MAX/2+i);
			}else{
				//(+)
				ch_A =(WORD) (i+ PWM_MAX/2);
			}
			*/


/*
			// FIR 50Hz�m�b�`
			w1=w3;
			w3 = w1 +ch2_50_s1;	// �t�B���^��̃f�[�^
			w2 = w1 * 635;   // 50hz
			ch2_50_s1 = -w1- w2/1000 + ch2_50_s2;
			ch2_50_s2 = w1;
*/
			
			// ch2�p���[
			ch2_smpw[ch2_smpwp++]=labs(w3);
			if (ch2_smpwp >= SMPW_MAX) ch2_smpwp=0;


			w1=0;
			for(i=0;i<SMPW_MAX;i++){
				w1 += ch2_smpw[i];
			}
			ch_B = (int)(w1/SMPW_MAX/SMPW_DVK);			//
			 
#ifdef AYASE
			ch_B = ch_B/2;	
			if (ch_B >PWM_MAX_ayase) ch_B = PWM_MAX_ayase;
#else
			ch_B = ch_B/2;	// DegiBeet
			if (ch_B >PWM_MAX) ch_B = PWM_MAX;
#endif
			
				
//			ch_B = (int)(w1/SMPW_MAX/SMPW_DVK/2);			// 	20130820	
//			ch_B = (int)(w1/256);			// 		
			
//			LED_OFF;

			//ch_B = SPI_Rx_buf[6]<<5 | SPI_Rx_buf[7]>>3;

//			ch_A = SPI_Rx_buf[4]<<8 | SPI_Rx_buf[5];
//			ch_B = SPI_Rx_buf[7]<<8 | SPI_Rx_buf[8];
			val_flag  = 1;


			// ���M�����^�C�~���O�J�E���^
			//2msec���Ƃɏ��������̂�*10��20msec�ňꏄ
			tx_cnt++;
			if (tx_cnt==10)tx_cnt=0;

			if(tx_cnt==1){
				// bluetooth TX
				sprintf(sss,"S%03d,%03d",ch_A,ch_B);
				i=0;
				while(sss[i] != 0){
					softserial(sss[i++]);
				}
				//for(i=0;i<7;i++){
				//	softserial(sss[i]);
				//}
				softserial(0xd);

			}

			if(tx_cnt==3){

				if (seq=SQ_STOP){
					//STOP SW��
					if (sw_old == 0 && swin!=0){
						//new ON
						sw_old = 1;

				// Disp �V���A��LED
				// �M�����x�ŗ΂�LED�̖��邳�𐧌�
				// �d�ɂ͂��ꎞ��3���キ
				// �M�����x�Ō��𐧌� >200:5   >150:3  >100:2  <100�F5���キ���x��50  
				if ((stat1 & (1<<0)) + (stat2 & 1<<7)){
					// �d�ɊO��
					for (i=0;i<N_SLEDS;i++){
						sleds[i][SLED_GREEN]=0;
						sleds[i][SLED_RED]=25;
						sleds[i][SLED_BLUE]=0;
					}
				}else{
					for (i=0;i<N_SLEDS;i++){
						sleds[i][SLED_RED]=0;
						sleds[i][SLED_BLUE]=0;
					}
					if(ch_A<=50){
						// �Œ჌�x��
						for (i=0;i<N_SLEDS;i++){
							sleds[i][SLED_GREEN]=(BYTE)ch_A;
						}
					}else if (ch_A>200){
						for (i=0;i<N_SLEDS;i++){
							sleds[i][SLED_GREEN]=(BYTE)ch_A;
						}
					}else if (ch_A>150){
						sleds[1][SLED_GREEN]=(BYTE)ch_A;
						sleds[2][SLED_GREEN]=(BYTE)ch_A;
						sleds[3][SLED_GREEN]=(BYTE)ch_A;
					}else{
						sleds[1][SLED_GREEN]=(BYTE)ch_A;
						sleds[2][SLED_GREEN]=(BYTE)ch_A;
					}
				}
				disp_sled();

			}

      	}else{
      		// ���̎�M�̂��ߑ��M
			UDR0 = 0; 					// To get Next byte.
      	}
//		LED_BLU_OFF;
	//	reti();
 }

//--------------------------
// delay n*10ms
void Wait_N_Ticks(uint16_t t)
{
	
	for( ; t>0 ; t--){
	 	_delay_ms(10);
	}
}


//--------------------------
// ���M
void SPI_TxData(uint16_t  d)
{
	uint16_t w;

	ADS_CS_ON;

	w = (d<<7);			// bit8 �� MSB��
//	w=d;
	UCSR0A = (1<<TXC0);						// ���M�I���t���O�N���A
	while ( !(UCSR0A & (1<<UDRE0)) );	 	// ���M�ޯ̧�󂫑ҋ@ 
	UDR0 = (w >>8);							// �ް����M(���M�J�n)

//	while ( !(UCSR0A & (1<<UDRE0)) );	 	// ���M�ޯ̧�󂫑ҋ@ 

	ADS_XCK_OFF;
//	ADS_DDR |= (1<<ADS_TXD)|(1<<ADS_XCK);

	if(d & 1){
		// LSB ON
		ADS_TXD_ON;
	}else{
		// LSB OFF
		ADS_TXD_OFF;
	}



	while ( !(UCSR0A & (1<<TXC0)) );	 	// ���M�����ҋ@ 


	UCSR0B = (0<<TXEN0);	 				// ���M�s���� 
	UCSR0C = (0<<UMSEL01)|(0<<UMSEL00)|(0<<UDORD0)|(0<<UCPHA0)|(0<<UCPOL0);


	ADS_XCK_ON;
	ADS_XCK_OFF;

	// SPI��L����
	UCSR0C = (1<<UMSEL01)|(1<<UMSEL00)|(0<<UDORD0)|(0<<UCPHA0)|(0<<UCPOL0);
	UCSR0B = (1<<TXEN0);	 	// ���M���� 

	ADS_CS_OFF;
}

//--------------------------
// SPI���M �\�t�g�^�C�v
void xxSPI_TxData(uint16_t  d)
{
//	uint16_t w;
	uint16_t p;
	
	p = 0x100;

	ADS_CS_ON;		// ���ON�ł����삷�邪...

	while(p>0){
		if(d & p){
			// hi
			ADS_TXD_ON;
		}else{
			// Low
			ADS_TXD_OFF;
		}
		ADS_XCK_ON;
		p = p >> 1;
		ADS_XCK_OFF;
		
	}

	ADS_CS_OFF;



}



/**********************************************************************************************************
* ADS1x9x_Reset			 			                                         						  	  *
**********************************************************************************************************/
void ADS1x9x_Reset(void)
  {
	    ADS_RES_OFF;					// Set High
	    /* Provide suficient dealy*/
	    _delay_ms(1);						// Wait 1 mSec
	    ADS_RES_ON;						// Set to low
	    _delay_ms(1);						// Wait 1 mSec
	    ADS_RES_OFF;					// Set High
	    _delay_ms(7);						// Wait 7 mSec
    }
  
/**********************************************************************************************************
* ADS1x9x_Disable_Start						                                          					  *
**********************************************************************************************************/
void ADS1x9x_Disable_Start(void)
{
    ADS_CS_ON;				// Set to LOW
	_delay_ms(7);			// Wait 7 mSec
}
/*********************************************************************************************************/
/**********************************************************************************************************
* ADS1x9x_Disable_Start						                                          					  *
**********************************************************************************************************/
void ADS1x9x_Enable_Start(void)
{
    ADS_CS_OFF;				// Set to High
	_delay_ms(100);			// Wait 10 mSec
}
/*********************************************************************************************************/
/**********************************************************************************************************
* Set_ADS1x9x_Chip_Enable																                  *
**********************************************************************************************************/
void Set_ADS1x9x_Chip_Enable (void)
{
	/* ADS1x9x CS is Active low*/
	ADS_CS_ON;		// Set to LOW
}
/**********************************************************************************************************
* Clear_ADS1x9x_Chip_Enable						                                          			  *
**********************************************************************************************************/

void Clear_ADS1x9x_Chip_Enable (void)
  {
  	
  	_delay_us(2);	// 1.5uS (at 1MHz)
	/* ADS1x9x CS is Active low*/
	ADS_CS_OFF;		// Set to High
  }
/**********************************************************************************************************
* Init_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
void Init_ADS1x9x_DRDY_Interrupt (void)
{
	
	ADS_DDR &= ~(1<<ADS_DRDY);		// input
	ADS_PORT |= (1<<ADS_DRDY);     	// pull-Up resistance
	PCIFR |= (1<<ADS_PCIF);			// irt flag clear
	ADS_PCMSK &= ~(1<<ADS_DRDY_INT);	// DRDY interrupt disabled

	
}
/**********************************************************************************************************
* Enable_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
void Enable_ADS1x9x_DRDY_Interrupt (void)
{
	PCIFR |= (1<<ADS_PCIF);			// irt flag clear
 	PCICR |= (1<<ADS_PCIE);			// �s���ω����荞�݃O���[�v�Q�ݒ�
	ADS_PCMSK |= (1<<ADS_DRDY_INT);		// �s���ω����荞��DRDY�ݒ�


}
/**********************************************************************************************************
* Disable_ADS1x9x_DRDY_Interrupt												                                          *
**********************************************************************************************************/
void Disable_ADS1x9x_DRDY_Interrupt (void)
{
	PCIFR |= (1<<ADS_PCIF);			// irt flag clear
 	//PCICR &= ~(1<<ADS_PCIE);			// �s���ω����荞�݃O���[�v�Q�֎~�ݒ�
	ADS_PCMSK &= ~(1<<ADS_DRDY_INT);	// �s���ω����荞��DRDY�֎~�ݒ�
}

/**********************************************************************************************************
* Set_GPIO														                                          *
**********************************************************************************************************/
void Set_GPIO(void)
{
	INIPORT;

}  

/**********************************************************************************************************
* Set_UCB0_SPI													                                          *
**********************************************************************************************************/
void Set_UCB0_SPI(void)
{

	UBRR0 = (F_CPU /(2*1000000))-1;	// Clock
		
	// SPI����AMSB���瑗�M�A�N���b�N��[�A�x�[�XLow
	UCSR0C = (1<<UMSEL01)|(1<<UMSEL00)|(0<<UDORD0)|(1<<UCPHA0)|(0<<UCPOL0); 

	// ����M�\�ݒ� ���荞�݋֎~ ��M�A���M��
	UCSR0B = (0<<RXCIE0)|(0<<TXCIE0)|(1<<RXEN0)|(1<<TXEN0);
}  

/**********************************************************************************************************
* ADS1x9x_SPI_Command_Data						                                          *
**********************************************************************************************************/
void ADS1x9x_SPI_Command_Data(unsigned char Data)
{
	unsigned char delayVar;
//	Set_ADS1x9x_Chip_Enable();
//	for (delayVar = 0; delayVar < 50; delayVar++);
//	Clear_ADS1x9x_Chip_Enable();

	Set_ADS1x9x_Chip_Enable();
	
	UDR0 = Data;                                     // Send the data sitting at the pointer DATA to the TX Buffer
 	while ( !(UCSR0A & (1<<RXC0)) );

	delayVar = UDR0;

	for (delayVar = 0; delayVar < 150; delayVar++);

	Clear_ADS1x9x_Chip_Enable();

}

/**********************************************************************************************************
* Init_ADS1x9x_Resource						                                          *
**********************************************************************************************************/

void Init_ADS1x9x_Resource(void)
{
    Set_GPIO();										// Initializes ADS1x9x's input control lines
    Set_UCB0_SPI();									// Initialize SPI regs.
    //Set_DMA_SPI();	   								// Initialize DMA regs for SPI.
    
}


/**********************************************************************************************************
*	        ADS1x9x Control Registers      				                                  *
**********************************************************************************************************/
/**********************************************************************************************************
* Wake_Up_ADS1x9x						                                          						  *
**********************************************************************************************************/
void Wake_Up_ADS1x9x (void)
  { 
    ADS1x9x_SPI_Command_Data (WAKEUP);                   // Send 0x02 to the ADS1x9x                                                      
  }

/**********************************************************************************************************
* Put_ADS1x9x_In_Sleep						                                          					  *
**********************************************************************************************************/
void Put_ADS1x9x_In_Sleep (void)
  {
    ADS1x9x_SPI_Command_Data (STANDBY);                 // Send 0x04 to the ADS1x9x
  }
/**********************************************************************************************************
* Soft_Reset_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Soft_Reset_ADS1x9x (void)
  {
    ADS1x9x_SPI_Command_Data (RESET);                   // Send 0x06 to the ADS1x9x
  }
/**********************************************************************************************************
* Soft_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/

void Soft_Start_ReStart_ADS1x9x (void)
  {
    ADS1x9x_SPI_Command_Data (START);                  // Send 0x08 to the ADS1x9x
    Clear_ADS1x9x_Chip_Enable ();                                                       
  }
/**********************************************************************************************************
* Hard_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/

void Hard_Start_ReStart_ADS1x9x(void)
  {
	//P8OUT |= (enum PORT8_ADC_CONTROL)ADC_START;			// Set Start pin to High
  }

/**********************************************************************************************************
* Soft_Start_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Soft_Start_ADS1x9x (void)
  {
    ADS1x9x_SPI_Command_Data (START);                   // Send 0x0A to the ADS1x9x
  }

/**********************************************************************************************************
* Soft_Stop_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Soft_Stop_ADS1x9x (void)
  {
    ADS1x9x_SPI_Command_Data (STOP);                   // Send 0x0A to the ADS1x9x
  }

/**********************************************************************************************************
* Hard_Stop_ADS1x9x					                                          						  *
**********************************************************************************************************/

void Hard_Stop_ADS1x9x (void)
  {
  	unsigned short i, j;
    ADS_START_OFF;		// Set Start pin to Low
    for (j = 0; j < DELAY_COUNT; j++)
    {
    	for ( i=0; i < 35000; i++);
    }
  }

/**********************************************************************************************************
* Soft_Start_ReStart_ADS1x9x			                                          						  *
**********************************************************************************************************/

void Stop_Read_Data_Continuous (void)
  {
    ADS1x9x_SPI_Command_Data(SDATAC);					// Send Stop Read Data Continuously mode
  }

/**********************************************************************************************************
* Start_Read_Data_Continuous			                                          						  *
**********************************************************************************************************/

void Start_Read_Data_Continuous (void)
  {
    ADS1x9x_SPI_Command_Data (RDATAC);					// Send Enable Read Data Continuous mode.
  }


/**********************************************************************************************************
* Start_Data_Conv_Command			                                          						  *
**********************************************************************************************************/

void Start_Data_Conv_Command (void)
  {
    ADS1x9x_SPI_Command_Data (START);					// Send 0x08 to the ADS1x9x
  }

/**********************************************************************************************************
* Initialize ADS1x9x						                                          *
**********************************************************************************************************/
void Init_ADS1x9x (void)
{
	ADS1x9x_Reset();
	ADS1x9x_Disable_Start();
	ADS1x9x_Enable_Start();
}
/*********************************************************************************************************/

/**********************************************************************************************************
* enable_ADS1x9x_Conversion													                          *
**********************************************************************************************************/
void enable_ADS1x9x_Conversion (void)
  {
    Start_Read_Data_Continuous ();		//RDATAC command
    
    Hard_Start_ReStart_ADS1x9x();

  }
/*********************************************************************************************************/

/*********************************************************************************************************
* ADS1x9x_Reg_Write																	                 *
**********************************************************************************************************/

void ADS1x9x_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
  { 
  	short i;
  	switch (READ_WRITE_ADDRESS)
  	{
  		case 1:
  			DATA = DATA & 0x87;
  		break;
  		case 2:
  			DATA = DATA & 0xFB;
  			DATA |= 0x80;
  			
  		break;
  		case 3:
  			DATA = DATA & 0xFD;
  			DATA |= 0x10;
  			
  		break;
  		case 7:
  			DATA = DATA & 0x3F;
  		break;
  		case 8:
  			DATA = DATA & 0x5F;
  		break;
  		case 9:
  			DATA |= 0x02;
  		break;
  		case 10:
  			DATA = DATA & 0x87;
  			DATA |= 0x01;
  		break;
  		case 11:
  			DATA = DATA & 0x0F;
  		break;
  		
  		default:
  		
  		break;
  		
  	}
	SPI_Tx_buf[0] = READ_WRITE_ADDRESS | WREG;
	SPI_Tx_buf[1] = 0;						// Write Single byte
	SPI_Tx_buf[2] = DATA;					// Write Single byte
	Set_ADS1x9x_Chip_Enable();
	
	for ( i =0; i < 50;i++);

	UDR0 = SPI_Tx_buf[0];              // Send the first data to the TX Buffer
 	while ( !(UCSR0A & (1<<RXC0)) );			// USCI_B0 TX buffer ready?
	i = UDR0;							// Read Rx buf

	UDR0 = SPI_Tx_buf[1];              // Send the first data to the TX Buffer
	while ( !(UCSR0A & (1<<RXC0)) );			// USCI_B0 TX buffer ready?
	i = UDR0;
	UDR0 = SPI_Tx_buf[2];              // Send the first data to the TX Buffer
	while ( !(UCSR0A & (1<<RXC0)) );			// USCI_B0 TX buffer ready?
	i = UDR0;

  }
/*********************************************************************************************************
* ADS1x9x_Reg_Read																	                 *
**********************************************************************************************************/
  unsigned char ADS1x9x_Reg_Read(unsigned char Reg_address)
  {
  		unsigned char retVal;
		SPI_Tx_buf[0] = Reg_address | RREG;
		SPI_Tx_buf[1] = 0;							// Read number of bytes - 1
		
		Set_ADS1x9x_Chip_Enable();					// Set chip select to low
		
		UDR0 = SPI_Tx_buf[0];                  // Send the first data to the TX Buffer
		while ( !(UCSR0A & (1<<RXC0)) );				// USCI_B0 TX buffer ready?
		UDR0 = SPI_Tx_buf[1];                  // Send the first data to the TX Buffer
		while ( !(UCSR0A & (1<<RXC0)) );				// USCI_B0 TX buffer ready?
		retVal = UDR0;							// Read RX buff
		UDR0 = 0x00;                           // Send the first data to the TX Buffer
		while ( !(UCSR0A & (1<<RXC0)) );				// USCI_B0 TX buffer ready?
		retVal = UDR0;							// Read RX buff

		Clear_ADS1x9x_Chip_Enable();				// Disable chip select
		_delay_us(4);
		return 	retVal;
  }


/**********************************************************************************************************
*	        ADS1x9x �L�����u���[�V�������s          				                  					  *
**********************************************************************************************************/
void Calib_Run()
{
	BYTE d;

	// 0Ah:CALIB_ON <-1
	d = (1<<CALIB_ON) | ADS1x9x_Default_Register_Settings[ADS1x9x_REG_RESP2];
	ADS1x9x_Reg_Write(ADS1x9x_REG_RESP2,d);

	// OFFSETCAL �R�}���h
    ADS1x9x_SPI_Command_Data (OFFSETCAL);					// Send 0x1A to the ADS1x9x
 	 _delay_ms(10);


	// 0Ah:CALIB_ON <-0 �߂�
	d &= ~(1<<CALIB_ON);
	ADS1x9x_Reg_Write(ADS1x9x_REG_RESP2,d);


}


/**********************************************************************************************************
*	        ADS1x9x default Initialization          				                  					  *
**********************************************************************************************************/

void ADS1x9x_Default_Reg_Init(void)
{

	unsigned char Reg_Init_i;
	Set_ADS1x9x_Chip_Enable();
	for ( Reg_Init_i =0; Reg_Init_i <100;Reg_Init_i++);
	Clear_ADS1x9x_Chip_Enable();

	for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
	{
		ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9x_Default_Register_Settings[Reg_Init_i]);
	}

}

/**********************************************************************************************************
*	        ADS1x9x_Read_All_Regs          				                  					  *
**********************************************************************************************************/

void ADS1x9x_Read_All_Regs(unsigned char ADS1x9xeg_buf[])
{
	unsigned char Regs_i;
/*	Set_ADS1x9x_Chip_Enable();
	for ( Regs_i =0; Regs_i <200;Regs_i++);
	Clear_ADS1x9x_Chip_Enable();
	_delay_us(10);
*/
	for ( Regs_i = 0; Regs_i < 12; Regs_i++)
	{
		ADS1x9xeg_buf[Regs_i] = ADS1x9x_Reg_Read(Regs_i);
	//	_delay_us(10);

	}

}
/*********************************************************************************************************/
/**********************************************************************************************************
*	        ADS1x9x_PowerOn_Init          				                  					  			  *
***********************************************************************************************************/
void ADS1x9x_PowerOn_Init(void)
{
#define P PWM_MAX/100



   WORD d1,d2;

	d1=d2=0;


   Init_ADS1x9x_Resource();		// MPU��SPI & i/o�ݒ�
   _delay_ms(40);

   ADS1x9x_Reset();				// ADC reset

   //Init_ADS1x9x_DRDY_Interrupt();
  sled_ini();

    _delay_ms(2);


//   Hard_Stop_ADS1x9x();

	 Wake_Up_ADS1x9x ();			// SPI_Command_Data (WAKEUP);
   
   Start_Data_Conv_Command();		// start command
   

   Soft_Stop_ADS1x9x();				// stop command


   _delay_ms(2);


   	Stop_Read_Data_Continuous();					// SDATAC command(Stop Read Data Continuously mode)

 	 _delay_ms(2);

	ADS1x9x_Default_Reg_Init();				// ���W�X�^�[�����l�ݒ�

//	Calib_Run();							// 	�L�����u���[�V����


	Start_Read_Data_Continuous();			// �A���ϊ�

 	
	//ADS1292_24BIT:
	SPI_Rx_exp_Count=9;		// 3 byte status + 3 bytes ch1 data + 3 bytes CH0 data
	ADS_CS_OFF;

	Soft_Start_ADS1x9x ();					// start
	// ��M�J�n
	Enable_ADS1x9x_DRDY_Interrupt();

	sei();


	for(;;){
		
		pwm_A(0);
		pwm_B(0);

		sdisp = OFFG;
		while(swin==0);
		//new ON
		sdisp = STARTG;
		_deley_ms(20);
		while(swin);
		timer = 0;
		power =0;
		sdisp = LEVELG;

		pwm_A(LOW_LEVEL);
		pwm_B(LOW_LEVEL);
		
		// �s�[�N���o
		while (timer < 3000/2){
			if (peek < ch_A){
				peek = ch_A;
			}
			while(val_flag==0);
			val_flag=0;
		}

		//���ˑ҂�
		while (peek/2 > ch_A){
			while(val_flag==0);
			val_flag=0;
			if (timer > 10000/2){
				// 10�b�𒴂���Ǝ�������
				break;
			}
		}
		
		sdisp = FULLG;
		
		a = power
		pwm_A(ch_A);
		pwm_B(ch_B);




//		LED_RED_ON;
		while(val_flag==0);
		val_flag=0;
		pwm_A(ch_A);
		pwm_B(ch_B);

		if ((stat1 & (1<<0)) + (stat2 & 1<<7)){
			// �d�ɊO�� �_��
			if(timer1 & 0x20){
				LED_LEFT_ON;
			}else{
				LED_LEFT_OFF;
			}
		}else{
			LED_LEFT_PWM = ch_A;
		}

		if (stat1 & (1<<2|1<<1)){
			// �d�ɊO�� �_��
			if(timer1 & 0x20){
				LED_RIGHT_ON;
			}else{
				LED_RIGHT_OFF;
			}
		}else{
			LED_RIGHT_PWM = ch_B;
		}


	}

	for(;;){

   ADS1x9x_Read_All_Regs(ADS1x9xRegVal);

  _delay_ms(8);

	}

   ADS1x9x_Default_Reg_Init();
   ADS1x9x_Read_All_Regs(ADS1x9xRegVal);

	//LED_RED_ON;
    for(;;);
	
}




