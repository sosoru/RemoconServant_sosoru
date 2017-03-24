// USB IR REMOCON KIT
//	ver 2.1	2013�D03�D26 DAIKIN 33kHz�Ή�
//	ver 2	2013�D03�D08 �G�A�R���Ή�
//	ver 1	����


/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�E for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS�ECONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
********************************************************************/

#ifndef IRREMOCONMAIN_C
#define IRREMOCONMAIN_C

/** INCLUDES *******************************************************/
#include <adc.h>
#include <timers.h>
#include <pwm.h>
#include <string.h>
#include "./USB/usb.h"
#include "HardwareProfile.h"
#include "./USB/usb_function_hid.h"
#include "eeprom.h"

/** CONFIGURATION **************************************************/
#pragma config CPUDIV = NOCLKDIV
#pragma config USBDIV = OFF
#pragma config FOSC   = HS
#pragma config PLLEN  = ON
#pragma config FCMEN  = OFF
#pragma config IESO   = OFF
#pragma config PWRTEN = OFF
#pragma config BOREN  = OFF
#pragma config BORV   = 30
#pragma config WDTEN  = OFF
#pragma config WDTPS  = 32768
#pragma config MCLRE  = OFF
#pragma config HFOFST = OFF
#pragma config STVREN = ON
#pragma config LVP    = OFF
#pragma config XINST  = OFF
#pragma config BBSIZ  = OFF
#pragma config CP0    = OFF
#pragma config CP1    = OFF
#pragma config CPB    = OFF
#pragma config WRT0   = OFF
#pragma config WRT1   = OFF
#pragma config WRTB   = OFF
#pragma config WRTC   = OFF
#pragma config EBTR0  = OFF
#pragma config EBTR1  = OFF
#pragma config EBTRB  = OFF


/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
int RemoconReceiveData(void);
int RemoconOutData(void);

/** DECLARATIONS ***************************************************/
#define MODE_MOUSE		0
#define MODE_KEYBOARD	1
#define	MODE_VOLUME		2
//#define	MODE_JOYSTICK	2
#define EEPROM_DATA_NUM			20	/* EEPROM�i�[�f�[�^�� */
#define EEPROM_DATA_SIZE		10	/* EEPROM�i�[�P�f�[�^�̃T�C�Y */
#define EEPROM_DATA_TOTAL_SIZE		EEPROM_DATA_NUM*EEPROM_DATA_SIZE	/* EEPROM�i�[�f�[�^�̃g�[�^���T�C�Y */
#define	EEPROM_DATA_MODE		0	//	0:���[�h
#define	EEPROM_DATA_VALUE		1	//	1:�l
#define	EEPROM_DATA_MODIFIER	2	//	2:Modifier�i�L�[�{�[�h�p�j
#define EEPROM_DATA_READERCODE	3	/* �ԊO�����[�_�R�[�h�i�[�ʒu */
#define EEPROM_DATA_DATACODE	4	/* �ԊO���f�[�^�R�[�h�i�[�ʒu */

#define MASTER_MOUSE_SPEED		50	//	Mouse�̈ړ����x�̒����l
#define MASTER_WHEEL_SPEED		1000

#define TX_BUFFER_SIZE 64			/* USB���M�o�b�t�@�T�C�Y */
#define RX_BUFFER_SIZE 64			/* USB��M�o�b�t�@�T�C�Y */

#define MOVE_OFF	0
#define MOVE_ON		1

#define NUM_OF_PINS		12

#define PIN1	PORTCbits.RC5
#define	PIN2	PORTCbits.RC4
#define	PIN3	PORTCbits.RC3
#define PIN4	PORTCbits.RC6
#define	PIN5	PORTCbits.RC7
#define	PIN6	PORTBbits.RB7
#define	PIN7	PORTBbits.RB6
#define	PIN8	PORTBbits.RB5
#define	PIN9	PORTBbits.RB4	
#define	PIN10	PORTCbits.RC2
#define PIN11	PORTCbits.RC1
#define	PIN12	PORTCbits.RC0

/* Timer0 0.1ms */
/* 100us / ( 0.021us x 4 ) = 1200  48MHz = 1/48us = 0.021us */
/* 1200 / 8 = 150    �v���X�P�[��=8*/
/* 0x10000 - 0x96 = 0xFF88    16bit�J�E���^*/
#define WRITE_TIMER0_COUNT        0xFF6A        //Timer0�̎��� 

//����38kHz=26us
// 38kHz = ( 78 + 1 ) * 4 * 0.0208us(48MHz) * 4(�v���X�P�[��) = 26.2912us
#define PWM_PERIOD	0x4E 	/* Period 78 = 0x4E */
#define	PWM_OFF		0
#define PWM_50		0x9E	/* PWM Duty 50% = (Period + 1) * 4 * 0.5 */
#define PWM_100		0x13C	/* PWM Duty 100% = (Period + 1) * 4 * 1.0 */

//����33kHz=30us
// 33kHz = ( 89 + 1 ) * 4 * 0.0208us(48MHz) * 4(�v���X�P�[��) = 29.952us
#define PWM_PERIOD_33k	0x59 	/* Period 89 = 0x59 */
#define	PWM_OFF_33k		0
#define PWM_50_33k		0xB4	/* PWM Duty 50% = (Period + 1) * 4 * 0.5 */
#define PWM_100_33k		0x168	/* PWM Duty 100% = (Period + 1) * 4 * 1.0 */



// �ԊO�������R���p
#define ON		1
#define OFF		0
#define R_ON	0
#define R_OFF	1
#define READSTATUS_READERCODE_WAIT		0x00	/* ��M��ԁ@���[�_�R�[�h�҂� */
#define READSTATUS_DATA_WAIT			0x01	/* ��M��ԁ@�f�[�^�҂� */
#define READSTATUS_READ_END				0x02	/* ��M��ԁ@�f�[�^�I�� */
#define READSTATUS_NEXT_DATA_WAIT		0x04	/* ��M��ԁ@���f�[�^�҂� */
#define READSTATUS_2ND_READERCODE_WAIT	0x08	/* ��M��ԁ@2nd���[�_�R�[�h�҂� */
#define READSTATUS_2ND_DATA_WAIT		0x10	/* ��M��ԁ@2nd�f�[�^�҂� */
#define READBUFFER_READER_SIZE			1		/* ��M�o�b�t�@�T�C�Y�@���[�_�R�[�h(1byte) */
#define READBUFFER_DATA_SIZE			6		/* ��M�o�b�t�@�T�C�Y�@�f�[�^(6byte) */
#define READBUFFER_SIZE					READBUFFER_READER_SIZE+READBUFFER_DATA_SIZE	/* ��M�o�b�t�@�T�C�Y�@���[�_�R�[�h(1byte) + �f�[�^(6byte) */
#define READBUFFER_FORMAT_SIZE_EX		1		/* �g�� ��M�o�b�t�@�T�C�Y�@�t�H�[�}�b�g�R�[�h(1byte) */
#define READBUFFER_DATA_SIZE_EX			2		/* �g�� ��M�o�b�t�@�T�C�Y�@�f�[�^�T�C�Y�i�P��bit�j(1byte) */
#define READBUFFER_DATA_AREA_SIZE_EX	32		/* �g�� ��M�o�b�t�@�T�C�Y�@�f�[�^�G���A�T�C�Y(32byte) */
#define READBUFFER_SIZE_EX				READBUFFER_FORMAT_SIZE_EX+READBUFFER_DATA_SIZE_EX+READBUFFER_DATA_AREA_SIZE_EX	/* �g�� ��M�o�b�t�@�T�C�Y�@�t�H�[�}�b�g�R�[�h(1byte) + �f�[�^�T�C�Y(1byte) + �f�[�^�G���A(32byte) */
#define OUTSTATUS_OUTPUT_WAIT			0x00	/* �o�͏�ԁ@�o�͑҂� */
#define OUTSTATUS_READER_CODE			0x01	/* �o�͏�ԁ@���[�_�R�[�h */
#define OUTSTATUS_DATA_CODE				0x02	/* �o�͏�ԁ@�f�[�^�R�[�h */
#define OUTSTATUS_END					0x04	/* �o�͏�ԁ@�I�� */
#define OUTSTATUS_2ND_READER_CODE		0x08	/* �o�͏�ԁ@2nd���[�_�R�[�h */
#define OUTSTATUS_2ND_DATA_CODE			0x10	/* �o�͏�ԁ@2nd�f�[�^�R�[�h */
#define OUTBUFFER_READER_SIZE			1		/* �o�̓o�b�t�@�T�C�Y�@���[�_�R�[�h(1byte) */
#define OUTBUFFER_DATA_SIZE				6		/* �o�̓o�b�t�@�T�C�Y�@�f�[�^(6byte) */
#define OUTBUFFER_SIZE					OUTBUFFER_READER_SIZE+OUTBUFFER_DATA_SIZE	/* �o�̓o�b�t�@�T�C�Y�@���[�_�R�[�h(1byte) + �f�[�^(6byte) */
#define OUTBUFFER_FORMAT_SIZE_EX		1		/* �g�� �o�̓o�b�t�@�T�C�Y�@�t�H�[�}�b�g�R�[�h(1byte) */
#define OUTBUFFER_DATA_SIZE_EX			2		/* �g�� �o�̓o�b�t�@�T�C�Y�@�f�[�^�T�C�Y�i�P��bit�j(1byte) */
#define OUTBUFFER_DATA_AREA_SIZE_EX		32		/* �g�� �o�̓o�b�t�@�T�C�Y�@�f�[�^�G���A�T�C�Y(32byte) */
#define OUTBUFFER_SIZE_EX				OUTBUFFER_FORMAT_SIZE_EX+OUTBUFFER_DATA_SIZE_EX+OUTBUFFER_DATA_AREA_SIZE_EX	/* �g�� �o�̓o�b�t�@�T�C�Y�@���[�_�R�[�h(1byte) + �f�[�^(32byte) */
#define FORMATCODE_UNKNOWN				0		/* �ԊO���ʐM�t�H�[�}�b�g �s�� */
#define FORMATCODE_KADEN				1		/* �ԊO���ʐM�t�H�[�}�b�g �Ɠd�� */
#define FORMATCODE_NEC					2		/* �ԊO���ʐM�t�H�[�}�b�g NEC */
#define FORMATCODE_SONY					3		/* �ԊO���ʐM�t�H�[�}�b�g SONY */
#define FORMATCODE_MITSU				4		/* �ԊO���ʐM�t�H�[�}�b�g MITSUBISHI */
#define FORMATCODE_DAIKIN				5		/* �ԊO���ʐM�t�H�[�}�b�g DAIKIN */
#define FORMATCODE_DAIKIN2				6		/* �ԊO���ʐM�t�H�[�}�b�g DAIKIN2 */
#define RECEIVE_WAIT_MODE_NONE			0		/* PC���ԊO���R�[�h��M�҂����[�h�@NONE */
#define RECEIVE_WAIT_MODE_WAIT			1		/* PC���ԊO���R�[�h��M�҂����[�h�@WAIT */

#define READTIMING						100		/* �ǂݍ��ݎ���[usec] */
#define READCODE_END_CNT				200 	/* ���̉�OFF����������I�� 1000 = 100[ms] / 0.1[ms] */
#define SENDCODE_END_CNT				200 	/* �I���R�[�h���̉�OFF�𑱂��� 1000 = 100[ms] / 0.1[ms] */
#define NEXT_DATA_WAIT					500		/* ���̃f�[�^�̓ǂݍ��݂����̎���OFF���p��������J�n���� 2000 = 200[ms] / 0.1[ms] */
#define DATA_CODE_INTERVAL_MIN_CNT		40 		/* �f�[�^���Q�ɂ킩��Ă���R�[�h�̃f�[�^�ԍŏ��Ԋu 40 = 4[ms] / 0.1[ms] */
#define DATA_CODE_INTERVAL_MAX_CNT		500 	/* �f�[�^���Q�ɂ킩��Ă���R�[�h�̃f�[�^�ԍő�Ԋu 500 = 50[ms] / 0.1[ms] */
#define DATA_CODE_INTERVAL_SEND_CNT		260 	/* �f�[�^���Q�ɂ킩��Ă���R�[�h�̑��M�f�[�^�Ԋu 300 = 30[ms] / 0.1[ms] */
#define DATA_MAX_BITS					0xFF	/* �f�[�^�R�[�h�̍ő�r�b�g�� */

/* �Ɠd�� */
#define READERCODE_ON_KADEN			32		/* ���[�_�R�[�h ON���� 3.2ms / 0.1ms */
#define READERCODE_OFF_KADEN		16		/* ���[�_�R�[�h OFF���� 1.6ms / 0.1ms */
#define READERCODE_ON_T_KADEN		2		/* ���[�_�R�[�h ON���� 2T */
#define READERCODE_ON_MIN_KADEN		24		/* ���[�_�R�[�h ON���ԍŏ� 24 = MIN40 / 12T * 8T */
#define READERCODE_OFF_MIN_KADEN	12		/* ���[�_�R�[�h OFF���ԍŏ� 12 = MIN40 / 12T * 4T */
//#define READERCODE_OFF_KADEN		1600	/* ���[�_�R�[�h OFF���� [usec] */
#define READERCODE_MIN_KADEN		40		/* ���[�_�R�[�h�� ((3.2 + 1.6) - 0.8) / 0.1 [msec] */
#define READERCODE_MAX_KADEN		56		/* ���[�_�R�[�h�� ((3.2 + 1.6) + 0.8) / 0.1 [msec] */
#define DATACODE_DATA1_KADEN		8		/* �f�[�^�R�[�h0/1���f� OFF�̒���������ȏ�Ȃ�1 */
#define DATACODE_MIN_KADEN			4		/* �f�[�^�R�[�h���ŏ� data0 ((0.4 + 0.4) - 0.4) / 0.1 [msec] */
#define DATACODE_MAX_KADEN			20		/* �f�[�^�R�[�h���ő� data1 ((0.4 + 1.2) + 0.4) / 0.1 [msec] */
#define DATA0_ON_KADEN				4		/* �f�[�^�O ON���� 0.4ms / 0.1ms */
#define DATA0_OFF_KADEN				4		/* �f�[�^�O OFF���� 0.4ms / 0.1ms */
#define DATA1_ON_KADEN				4		/* �f�[�^�P ON���� 0.4ms / 0.1ms */
#define DATA1_OFF_KADEN				12		/* �f�[�^�P OFF���� 1.2ms / 0.1ms */
/* NEC */
#define READERCODE_ON_NEC			90		/* ���[�_�R�[�h ON���� 9.0ms / 0.1ms */
#define READERCODE_OFF_NEC			45		/* ���[�_�R�[�h OFF���� 4.5ms / 0.1ms */
#define READERCODE_ON_T_NEC			2		/* ���[�_�R�[�h ON���� 2T */
#define READERCODE_ON_MIN_NEC		80		/* ���[�_�R�[�h ON���ԍŏ� 80 = MIN125 / 24T * 16T */
#define READERCODE_OFF_MIN_NEC		40		/* ���[�_�R�[�h OFF���ԍŏ� 40 = MIN125 / 24T * 8T */
//#define READERCODE_OFF_NEC			4500	/* ���[�_�R�[�h OFF���� [usec] */
#define READERCODE_MIN_NEC			90		/* ���[�_�R�[�h�� ((9.0 + 4.5) - 4.5) / 0.1 [msec] */
#define READERCODE_MAX_NEC			180		/* ���[�_�R�[�h�� ((9.0 + 4.5) + 4.5) / 0.1 [msec] */
#define DATACODE_DATA1_NEC			12		/* �f�[�^�R�[�h0/1���f� OFF�̒���������ȏ�Ȃ�1 */
#define DATACODE_MIN_NEC			5		/* �f�[�^�R�[�h���ŏ� data0 ((0.56 + 0.56) - 0.56) / 0.1 [msec] */
#define DATACODE_MAX_NEC			28		/* �f�[�^�R�[�h���ő� data1 ((0.56 + 1.68) + 0.56) / 0.1 [msec] */
#define DATA0_ON_NEC				5		/* �f�[�^�O ON���� 0.56ms / 0.1ms */
#define DATA0_OFF_NEC				5		/* �f�[�^�O OFF���� 0.56ms / 0.1ms */
#define DATA1_ON_NEC				5		/* �f�[�^�P ON���� 0.56ms / 0.1ms */
#define DATA1_OFF_NEC				16		/* �f�[�^�P OFF���� 1.68ms / 0.1ms */
/* SONY */
#define READERCODE_ON_SONY			24		/* ���[�_�R�[�h ON���� 2.4ms / 0.1ms */
#define READERCODE_OFF_SONY			6		/* ���[�_�R�[�h OFF���� 0.6ms / 0.1ms */
#define READERCODE_ON_T_SONY		4		/* ���[�_�R�[�h ON���� 4T */
#define READERCODE_ON_MIN_SONY		16		/* ���[�_�R�[�h ON���ԍŏ� 16 = MIN24 / 5T * 4T */
#define READERCODE_OFF_MIN_SONY		4		/* ���[�_�R�[�h OFF���ԍŏ� 4 = MIN24 / 5T * 1T */
//#define READERCODE_OFF_SONY			600		/* ���[�_�R�[�h OFF���� [usec] */
#define READERCODE_MIN_SONY			24		/* ���[�_�R�[�h�� ((2.4 + 0.6) - 0.6) / 0.1 [msec] */
#define READERCODE_MAX_SONY			36		/* ���[�_�R�[�h�� ((2.4 + 0.6) + 0.6) / 0.1 [msec] */
#define DATACODE_DATA1_SONY			9		/* �f�[�^�R�[�h0/1���f� ON�̒���������ȏ�Ȃ�1 */
#define DATACODE_MIN_SONY			6		/* �f�[�^�R�[�h���ŏ� data0 ((0.6 + 0.6) - 0.6) / 0.1 [msec] */
#define DATACODE_MAX_SONY			24		/* �f�[�^�R�[�h���ő� data1 ((1.2 + 0.6) + 0.6) / 0.1 [msec] */
#define DATA0_ON_SONY				6		/* �f�[�^�O ON���� 0.6ms / 0.1ms */
#define DATA0_OFF_SONY				6		/* �f�[�^�O OFF���� 0.6ms / 0.1ms */
#define DATA1_ON_SONY				12		/* �f�[�^�P ON���� 1.2ms / 0.1ms */
#define DATA1_OFF_SONY				6		/* �f�[�^�P OFF���� 0.6ms / 0.1ms */
/* MITSUBISHI ���[�_�R�[�h�Ȃ��A�����Ȃ�f�[�^ */
#define DATACODE_DATA1_M			14		/* �f�[�^�R�[�h0/1���f� OFF�̒���������ȏ�Ȃ�1 */
#define DATACODE_MIN_M				8		/* �f�[�^�R�[�h���ŏ� data0 ((0.4 + 0.8) - 0.4) / 0.1 [msec] */
#define DATACODE_MAX_M				28		/* �f�[�^�R�[�h���ő� data1 ((0.4 + 2.0) + 0.4) / 0.1 [msec] */
#define DATA0_ON_M					4		/* �f�[�^�O ON���� 0.4ms / 0.1ms */
#define DATA0_OFF_M					8		/* �f�[�^�O OFF���� 0.8ms / 0.1ms */
#define DATA1_ON_M					4		/* �f�[�^�P ON���� 0.4ms / 0.1ms */
#define DATA1_OFF_M					20		/* �f�[�^�P OFF���� 2.0ms / 0.1ms */
/* DAIKIN 1 */
#if 1
#define READERCODE_ON_DAI			44		/* ���[�_�R�[�h ON���� 4.4ms / 0.1ms */
#define READERCODE_OFF_DAI			22		/* ���[�_�R�[�h OFF���� 2.2ms / 0.1ms */
#define READERCODE_ON_T_DAI			2		/* ���[�_�R�[�h ON���� 2T */
#define READERCODE_ON_MIN_DAI		29		/* ���[�_�R�[�h ON���ԍŏ� 29 = MIN44 / 15T * 10T */
#define READERCODE_OFF_MIN_DAI		14		/* ���[�_�R�[�h OFF���ԍŏ� 14 = MIN44 / 15T * 5T */
//#define READERCODE_OFF_DAI		1600	/* ���[�_�R�[�h OFF���� [usec] */
#define READERCODE_MIN_DAI			44		/* ���[�_�R�[�h�� ((4.4 + 2.2) - 2.2) / 0.1 [msec] */
#define READERCODE_MAX_DAI			88		/* ���[�_�R�[�h�� ((4.4 + 2.2) + 2.2) / 0.1 [msec] */
#define DATACODE_DATA1_DAI			7		/* �f�[�^�R�[�h0/1���f� OFF�̒���������ȏ�Ȃ�1 */
#define DATACODE_MIN_DAI			4		/* �f�[�^�R�[�h���ŏ� data0 ((0.4 + 0.4) - 0.4) / 0.1 [msec] */
#define DATACODE_MAX_DAI			23		/* �f�[�^�R�[�h���ő� data1 ((0.4 + 1.3) + 0.4) / 0.1 [msec] */
#define DATA0_ON_DAI				4		/* �f�[�^�O ON���� 0.4ms / 0.1ms */
#define DATA0_OFF_DAI				4		/* �f�[�^�O OFF���� 0.5ms / 0.1ms */
#define DATA1_ON_DAI				4		/* �f�[�^�P ON���� 0.4ms / 0.1ms */
#define DATA1_OFF_DAI				13		/* �f�[�^�P OFF���� 1.3ms / 0.1ms */
#endif
#if 0
#define READERCODE_ON_DAI			50		/* ���[�_�R�[�h ON���� 5.0ms / 0.1ms */
#define READERCODE_OFF_DAI			25		/* ���[�_�R�[�h OFF���� 2.5ms / 0.1ms */
#define READERCODE_ON_T_DAI			2		/* ���[�_�R�[�h ON���� 2T */
#define READERCODE_ON_MIN_DAI		33		/* ���[�_�R�[�h ON���ԍŏ� 33 = MIN50 / 15T * 10T */
#define READERCODE_OFF_MIN_DAI		16		/* ���[�_�R�[�h OFF���ԍŏ� 16 = MIN50 / 15T * 5T */
//#define READERCODE_OFF_DAI		1600	/* ���[�_�R�[�h OFF���� [usec] */
#define READERCODE_MIN_DAI			50		/* ���[�_�R�[�h�� ((5.0 + 2.5) - 2.5) / 0.1 [msec] */
#define READERCODE_MAX_DAI			100		/* ���[�_�R�[�h�� ((5.0 + 2.5) + 2.5) / 0.1 [msec] */
#define DATACODE_DATA1_DAI			7		/* �f�[�^�R�[�h0/1���f� OFF�̒���������ȏ�Ȃ�1 */
#define DATACODE_MIN_DAI			5		/* �f�[�^�R�[�h���ŏ� data0 ((0.5 + 0.5) - 0.5) / 0.1 [msec] */
#define DATACODE_MAX_DAI			25		/* �f�[�^�R�[�h���ő� data1 ((0.5 + 1.5) + 0.5) / 0.1 [msec] */
#define DATA0_ON_DAI				5		/* �f�[�^�O ON���� 0.5ms / 0.1ms */
#define DATA0_OFF_DAI				5		/* �f�[�^�O OFF���� 0.5ms / 0.1ms */
#define DATA1_ON_DAI				5		/* �f�[�^�P ON���� 0.5ms / 0.1ms */
#define DATA1_OFF_DAI				15		/* �f�[�^�P OFF���� 1.5ms / 0.1ms */
#endif
/* DAIKIN 2 */
#define READERCODE_ON_DAI2			30		/* ���[�_�R�[�h ON���� 3.0ms / 0.1ms */
#define READERCODE_OFF_DAI2			90		/* ���[�_�R�[�h OFF���� 9.0ms / 0.1ms */
#define READERCODE_OFF_T_DAI2		3		/* ���[�_�R�[�h OFF���� 3T */
#define READERCODE_ON_MIN_DAI2		22		/* ���[�_�R�[�h ON���ԍŏ� 22 = MIN90 / 12T * 3T */
#define READERCODE_OFF_MIN_DAI2		67		/* ���[�_�R�[�h OFF���ԍŏ� 67 = MIN90 / 12T * 9T */
//#define READERCODE_OFF_DAI		1600	/* ���[�_�R�[�h OFF���� [usec] */
#define READERCODE_MIN_DAI2			90		/* ���[�_�R�[�h�� ((3.0 + 9.0) - 3.0) / 0.1 [msec] */
#define READERCODE_MAX_DAI2			150		/* ���[�_�R�[�h�� ((3.0 + 9.0) + 3.0) / 0.1 [msec] */
// �f�[�^����DAIKIN1�Ɠ���
//#define DATACODE_DATA1_DAI2			7		/* �f�[�^�R�[�h0/1���f� OFF�̒���������ȏ�Ȃ�1 */
//#define DATACODE_MIN_DAI2			5		/* �f�[�^�R�[�h���ŏ� data0 ((0.5 + 0.5) - 0.5) / 0.1 [msec] */
//#define DATACODE_MAX_DAI2			25		/* �f�[�^�R�[�h���ő� data1 ((0.5 + 1.5) + 0.5) / 0.1 [msec] */
//#define DATA0_ON_DAI2				5		/* �f�[�^�O ON���� 0.5ms / 0.1ms */
//#define DATA0_OFF_DAI2				5		/* �f�[�^�O OFF���� 0.5ms / 0.1ms */
//#define DATA1_ON_DAI2				5		/* �f�[�^�P ON���� 0.5ms / 0.1ms */
//#define DATA1_OFF_DAI2				15		/* �f�[�^�P OFF���� 1.5ms / 0.1ms */




#define CODE_TYPE_STANDARD	0	// �R�[�h�^�C�v�@�W��
#define CODE_TYPE_EXTENSION	1	// �R�[�h�^�C�v�@�g��

/** ROMS ******************************************************/
#pragma romdata

const ROM BYTE configCleanerRemoteCode_Kyou[]={0x01, 0x38, 0x00, 0x01, 0x10, 0x00, 0xB0, 0x4F, 0x0A, 0xF5};
const ROM BYTE configCleanerRemoteCode_Jutan[]={0x01, 0x38, 0x00, 0x01, 0x10, 0x00, 0xB0, 0x4F, 0x02, 0xFD};
const ROM BYTE configCleanerRemoteCode_YukaTatami[]={0x01, 0x38, 0x00, 0x01, 0x10, 0x00, 0xB0, 0x4F, 0x09, 0xF6};
const ROM BYTE configCleanerRemoteCode_Off[]={0x01, 0x38, 0x00, 0x01, 0x10, 0x00, 0xB0, 0x4F, 0x01, 0xFE};


/** VARIABLES ******************************************************/
#pragma udata
char	c_version[]="2.1.0";
BYTE mouse_buffer[4] = {0};
BYTE volume_buffer[1] = {0};
BYTE keyboard_buffer[8]; 
USB_HANDLE lastTransmission;
USB_HANDLE lastTransmission2;
USB_HANDLE lastINTransmissionKeyboard;
USB_HANDLE lastOUTTransmissionKeyboard;
USB_HANDLE USBOutHandle = 0;
USB_HANDLE USBInHandle = 0;

unsigned char result_button_press_set = 0;
unsigned char result_button_press_set2 = 0;

// �ԊO���f�[�^��0�o�C�g�ڂ��i�[����ϐ� 20�f�[�^��20�o�C�g
unsigned char eeprom_check_data[EEPROM_DATA_NUM] = {0};
// �ԊO���f�[�^�P�f�[�^���@10�o�C�g
unsigned char eeprom_1data[EEPROM_DATA_SIZE];

// �ԊO�������R���f�[�^��M�p�ϐ�
//unsigned char uc_read_buff[READBUFFER_SIZE_EX] = {0};			/* �ǂݍ��ݒ��̃f�[�^��ۑ� */
//unsigned char uc_fix_read_buff[READBUFFER_SIZE_EX] = {0};		/* �ǂݍ��݊����f�[�^��ۑ� */
//unsigned char send_read_data_flag = 0;							/* �ǂݍ��݊����f�[�^��PC�ɑ��M����Ƃ��Ƀt���O�𗧂Ă� */
//unsigned char process_read_data_flag = 0;							/* �ǂݍ��݊����f�[�^���������邷��Ƃ��Ƀt���O�𗧂Ă� */
//unsigned char uc_process_read_data[READBUFFER_SIZE_EX];			/* �ǂݍ��݃f�[�^���������邷��Ƃ��ɕۑ� */
//unsigned int ui_data_pos = 0;
//unsigned char uc_read_status = READSTATUS_READERCODE_WAIT;
//unsigned int ui_off_count = 0;
//unsigned int ui_on_count = 0;
//unsigned char uc_tick_base = 0;
//unsigned char uc_format_type = FORMATCODE_UNKNOWN;
//unsigned char uc_receive_wait_mode = RECEIVE_WAIT_MODE_NONE;/* PC�����ԊO���f�[�^��M�҂��̂Ƃ�1 */
//unsigned int ui_next_data_wait_cnt = 0;						/* ���̃f�[�^�܂ł�OFF�̌p�����ԃJ�E���^ */

unsigned char uc_now_signal = 0;
unsigned char uc_pre_signal = 0;

// �ԊO�������R���f�[�^���M�p�ϐ�
unsigned int ui_out_pos = 0;
unsigned char uc_out_status = OUTSTATUS_OUTPUT_WAIT;
unsigned char uc_out_buff[OUTBUFFER_SIZE_EX];
unsigned int ui_out_on_time = 0;
unsigned int ui_out_off_time = 0;
unsigned char uc_out_data_len = 0;
unsigned char uc_out_data0_on = 0;
unsigned char uc_out_data0_off = 0;
unsigned char uc_out_data1_on = 0;
unsigned char uc_out_data1_off = 0;
unsigned char uc_send_bit_size = 0;
unsigned char uc_send_bit_size_2nd_data = 0;

unsigned char uc_first_data_bit = 0;

unsigned char uc_read_code_type = CODE_TYPE_STANDARD;
unsigned char uc_out_code_type = CODE_TYPE_STANDARD;

unsigned char uc_end_count = 0;

unsigned int ui_PWM50_Set_Val = PWM_50;

// �f�o�b�O
unsigned char debug_ary[4] = {0};


// usb�ł̑��M�Ɏg���o�b�t�@�͂����Ő錾
#pragma udata usbram2

BYTE volume_input[1];
BYTE mouse_input[4];
BYTE hid_report[8];
unsigned char volume_input_out_flag = 0;
unsigned char mouse_input_out_flag = 0;
unsigned char hid_report_out_flag = 0;
unsigned char ReceivedDataBuffer[RX_BUFFER_SIZE];
unsigned char ToSendDataBuffer[TX_BUFFER_SIZE];

#pragma udata

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
        #if defined(USB_INTERRUPT)
	        USBDeviceTasks();
        #endif

	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		unsigned char fi;
		unsigned char _portB;

		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.	
		
		if(INTCONbits.TMR0IF == 1)
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(WRITE_TIMER0_COUNT);

			// �f�[�^�o�͒��̎��͎�M���Ȃ�
			if( uc_out_status == OUTSTATUS_OUTPUT_WAIT )
			{
				/* �ԊO�������R���̃f�[�^��M */
				//RemoconReceiveData();
			}	

			/* �ԊO�������R���̃f�[�^�o�� */
			RemoconOutData();
		}

		//�{�^���������ꂽ��ԊO�������R������f�[�^���o�͂���

		if(INTCONbits.RABIF == 1)
		{
			_portB = PORTB;

			/* �o�͏�Ԃ������甲���� */
			if(uc_out_status != OUTSTATUS_OUTPUT_WAIT || 0 > ui_out_on_time || 0 > ui_out_off_time){
				goto ISR_RAB_END;
 			}
			
			debug_ary[0] = _portB;

			//RB4, RB5, RB6, RB7���{�^���ɑ����D��������R�}���h�f�[�^��ROM����R�s�[				 
			if((_portB & 0x10) == 0) memcpypgm2ram( uc_out_buff, (const far rom BYTE *)configCleanerRemoteCode_Kyou, 10);
			else if ((_portB & 0x20) == 0) memcpypgm2ram( uc_out_buff, (const far rom BYTE *)configCleanerRemoteCode_Jutan, 10);
			else if ((_portB & 0x40) == 0) memcpypgm2ram( uc_out_buff, (const far rom BYTE *)configCleanerRemoteCode_YukaTatami, 10);
			else if ((_portB & 0x80) == 0) memcpypgm2ram( uc_out_buff, (const far rom BYTE *)configCleanerRemoteCode_Off, 10);
			else goto ISR_RAB_END;

			for(fi = 10; fi < OUTBUFFER_SIZE_EX; fi++){
				uc_out_buff[fi] = 0; //�c��̗̈��0�Ŗ��߂�
			}
			uc_out_code_type = CODE_TYPE_EXTENSION;

			RemoconOutData();
ISR_RAB_END:			
			// clear interrupt flag finally
			INTCONbits.RABIF = 0; 
		}

	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif

#pragma code

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void main(void)
{   
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

//	�^�C�}0�̐ݒ�
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8);
	WriteTimer0(WRITE_TIMER0_COUNT);
//	CCP���W���[����PWM�ݒ�
	OpenPWM1(PWM_PERIOD);
//	�^�C�}2�̐ݒ� �v���X�P�[��1:4
	OpenTimer2(TIMER_INT_OFF & T2_PS_1_4 & T2_POST_1_1 );
	SetDCPWM1(PWM_OFF);
//	SetDCPWM1(PWM_50);

//	���荞�݊J�n
	RCONbits.IPEN = 1;	//���荞�ݗD��t����
	INTCON2bits.TMR0IP = 0;	//�^�C�}0���ʃ��x�����荞�݂ɐݒ�
	INTCONbits.TMR0IE = 1;	//�^�C�}0���荞�݋���

	IOCB = 0b11110000; 		//�|�[�gB���荞�݋���(�ǉ�)
	INTCON2bits.RABIP = 0;	//��ʃ��x�����荞�݂Ƃ��Đݒ�
	INTCONbits.RABIE = 1;	//�|�[�g�ω��̊��荞�݋���(�ǉ�)

	INTCONbits.GIEL = 1;	//��ʃ��x�����荞�݂̋���
//	INTCONbits.GIEH = 1;	//���ʃ��x�����荞�݂̋���
	INTCONbits.GIE = 1;



    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // frequently (such as once about every 100 microseconds) at any
        				  // time that a SETUP packet might reasonably be expected to
        				  // be sent by the host to your device.  In most cases, the
        				  // USBDeviceTasks() function does not take very long to
        				  // execute (~50 instruction cycles) before it returns.
        #endif
    				  

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    ADCON1 |= 0x0F;                 // Default all pins to digital

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
	int fi,fj;
	char mode_changedp = 1;

	TRISB = 0b11110000;	//RB5����� b7:RB7 b6:RB6 b5:RB5 b4:RB4
	TRISC = 0x80;	//RC���o��  b7:RC7 b6:RC6 b5:RC5 b4:RC4 b3:RC3 b2:RC2 b1:RC1 bit0:RC0

	LATC = 0xff;

	ANSEL = 0x00;
	ANSELH = 0x00;	//�S�ăf�W�^��

	INTCON2bits.RABPU = 0;	//�����v���A�b�v���g����悤�ɂ���
	WPUBbits.WPUB4 = 1;
	WPUBbits.WPUB5 = 1;
	WPUBbits.WPUB6 = 1;
	WPUBbits.WPUB7 = 1;

    //initialize the variable holding the handle for the last
	
    for(fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
    {
	    uc_out_buff[fi] = 0;
	}
//	for(fi = 0; fi < READBUFFER_SIZE_EX; fi++)
//	{
//		uc_process_read_data[fi] = 0;
//	}
	for(fi = 0; fi < EEPROM_DATA_SIZE; fi++)
	{
		eeprom_1data[fi] = 0;
	}

	
    // transmission
    lastTransmission = 0;
    lastTransmission2 = 0;
    lastINTransmissionKeyboard = 0;
    lastOUTTransmissionKeyboard = 0;
    USBOutHandle = 0;
    USBInHandle = 0;

	volume_input[0] = 0;

    mouse_input[0] = 
    mouse_input[1] = 
    mouse_input[2] = 0;

	hid_report_in[0] = 
	hid_report_in[1] = 
	hid_report_in[2] = 
	hid_report_in[3] = 
	hid_report_in[4] = 
	hid_report_in[5] = 
	hid_report_in[6] = 
	hid_report_in[7] = 0;

	mouse_buffer[0] =
	mouse_buffer[1] =
	mouse_buffer[2] =
	mouse_buffer[3] = 0;

	keyboard_buffer[0] = 
	keyboard_buffer[1] = 
	keyboard_buffer[2] = 
	keyboard_buffer[3] = 
	keyboard_buffer[4] = 
	keyboard_buffer[5] = 
	keyboard_buffer[6] = 
	keyboard_buffer[7] = 0;

	volume_buffer[0] = 0;

	// EEPROM�̐ԊO���f�[�^��0�o�C�g�ڂ��i�[����
	for( fi = 0, fj = EEPROM_DATA_DATACODE; fi < EEPROM_DATA_NUM; fi++, fj+=EEPROM_DATA_SIZE )
	{
		eeprom_check_data[fi] = ReadEEPROM( fi * EEPROM_DATA_SIZE + EEPROM_DATA_DATACODE);
	}
}//end UserInit


/********************************************************************
 * Function:        void ProcessIO(void)
 *	
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{
	int fi,fj;
	int pressed_keys;
	char result_button_press;
	char tmp;
	char result;
	char code_match_flag = 0;
	unsigned char uc_1data = 0;


    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

	/* �ԊO����M�f�[�^���R�s�[���� */
	/* ��U�t���O���N���A */
//	process_read_data_flag = 0;
//	/* �ԊO���R�[�h�ǂݍ��ݍς� */
//	if( uc_fix_read_buff[0] != FORMATCODE_UNKNOWN )
//	{
//		/* PC���ԊO���R�[�h��M�҂� */
//		if( uc_receive_wait_mode == RECEIVE_WAIT_MODE_WAIT )
//		{
//			send_read_data_flag = 1;
//			for( fi = 0; fi < READBUFFER_SIZE_EX; fi++ )
//			{
//				uc_process_read_data[fi] = uc_fix_read_buff[fi];
//				uc_fix_read_buff[fi] = 0;
//			}
//		}
//		else
//		{
//			send_read_data_flag = 0;
//			process_read_data_flag = 1;
//			for( fi = 0; fi < READBUFFER_SIZE; fi++ )
//			{
//				uc_process_read_data[fi] = uc_fix_read_buff[fi];
//			}
//			for( fi = 0; fi < READBUFFER_SIZE_EX; fi++ )
//			{
//				uc_fix_read_buff[fi] = 0;
//			}
//		}
//	}
//	else
//	{	/* PC���ԊO���R�[�h��M�҂��łȂ��Ƃ��̓o�b�t�@���N���A */
//		if( uc_receive_wait_mode != RECEIVE_WAIT_MODE_WAIT )
//		{
//			send_read_data_flag = 0;
//			for( fi = 0; fi < (READBUFFER_SIZE_EX - READBUFFER_DATA_AREA_SIZE_EX); fi++ )
//			{
//				uc_process_read_data[fi] = 0;
//			}
//		}
//	}	
//
//	code_match_flag = 0;
//	pressed_keys = 2;
//	/* �ԊO���R�[�h��M�H�@���[�_�R�[�h���O�ȊO�Ŏ�M�����ƌ��Ȃ� */
//	if(process_read_data_flag == 1)
//	{
//		/* �f�[�^�������[�v */
//		for(fi = 0; fi < EEPROM_DATA_NUM; fi++ )
//		{
//			code_match_flag = 0;
//			/* 0�o�C�g�̂ݔ�r���Ĉ�v���Ă�����1�o�C�g�ڈȍ~��EEPROM����ǂݏo����r���� */
//			if(eeprom_check_data[fi] == uc_process_read_data[1])
//			{
//				code_match_flag++;	//��v
//				eeprom_1data[EEPROM_DATA_DATACODE] = eeprom_check_data[fi];
//
//				uc_1data = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_READERCODE);
//				/* ���[�_�R�[�h��r */
//				if( uc_1data == uc_process_read_data[0] )
//				{	/*���[�_�R�[�h��v*/
//					code_match_flag++;	//��v
//					eeprom_1data[EEPROM_DATA_READERCODE] = uc_1data;
//
//					/* 0�o�C�g�ڂ���v���Ă����̂�1�o�C�g�ȍ~��ǂݏo����r���� */
//					for( fj = 1; fj < READBUFFER_DATA_SIZE; fj++ )
//					{
//						uc_1data = ReadEEPROM((unsigned char)fi * (unsigned char)EEPROM_DATA_SIZE + (unsigned char)EEPROM_DATA_DATACODE + (unsigned char)fj );
//						if(uc_1data == uc_process_read_data[1+fj])
//						{
//							code_match_flag++;	//��v
//							eeprom_1data[EEPROM_DATA_DATACODE+fj] = uc_1data;
//						}
//					}
//
//					// �ԊO���R�[�h�S�Ĉ�v
//					if(code_match_flag == (READBUFFER_READER_SIZE + READBUFFER_DATA_SIZE))
//					{
//						eeprom_1data[EEPROM_DATA_MODE] = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_MODE);
//						eeprom_1data[EEPROM_DATA_VALUE] = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_VALUE);
//						eeprom_1data[EEPROM_DATA_MODIFIER] = ReadEEPROM(fi * EEPROM_DATA_SIZE + EEPROM_DATA_MODIFIER);
//						/* ��M�����ԊO���R�[�h����v�����̂Őݒ肳��Ă���R�[�h�𑗐M���� */
//
//						switch(eeprom_1data[EEPROM_DATA_MODE])
//						{
//							case MODE_MOUSE:
//								if(eeprom_1data[EEPROM_DATA_VALUE] == 0){  // ���N���b�N
//									mouse_buffer[0] |= 1;
//								}	
//								else if(eeprom_1data[EEPROM_DATA_VALUE] == 1){  // �E�N���b�N
//									mouse_buffer[0] |= 0x02;
//								}	
//								else if(eeprom_1data[EEPROM_DATA_VALUE] == 2){  // �z�C�[���N���b�N
//									mouse_buffer[0] |= 0x04;
//								}
//								mouse_input_out_flag = 5;
//								break;
//							case MODE_KEYBOARD:
//								if(pressed_keys != 8)
//								{
//									keyboard_buffer[0] |= eeprom_1data[EEPROM_DATA_MODIFIER];
//									keyboard_buffer[pressed_keys] = eeprom_1data[EEPROM_DATA_VALUE];
//									pressed_keys++;
//								}
//								hid_report_out_flag = 5;
//								break;
//							case MODE_VOLUME:
//							    switch(eeprom_1data[EEPROM_DATA_VALUE])
//							    {
//								    case 0: volume_buffer[0] |= 0x01; break;
//								    case 1: volume_buffer[0] |= 0x02; break;
//								    case 2: volume_buffer[0] |= 0x04; break;
//								}
//								volume_input_out_flag = 5;
//								break;
//							default:
//								break;
//						}
//						/* ��M�����ԊO���R�[�h����v�����̂Őݒ肳��Ă���R�[�h�𑗐M���� �����܂� */
//					}	
//				}
//			}	
//		}	
//	}	

//---------------------------------------------------------------------
//	USB�f�[�^���M��
    if(!HIDTxHandleBusy(lastTransmission))
    {
        //copy over the data to the HID buffer
        //�}�E�X�f�[�^�̑��M
        mouse_input[0] = mouse_buffer[0];
        mouse_input[1] = mouse_buffer[1];
        mouse_input[2] = mouse_buffer[2];
        mouse_input[3] = mouse_buffer[3];

		mouse_buffer[0] =0;
		mouse_buffer[1] =0;
		mouse_buffer[2] =0;
		mouse_buffer[3] =0;

		if( mouse_input_out_flag > 0 )
		{
        	//Send the 8 byte packet over USB to the host.
        	lastTransmission = HIDTxPacket(HID_EP, (BYTE*)&mouse_input, sizeof(mouse_input));
        	mouse_input_out_flag--;
 		}      	
    }
    if(!HIDTxHandleBusy(lastINTransmissionKeyboard))
    {	       	//Load the HID buffer
    	hid_report_in[0] = keyboard_buffer[0];
    	hid_report_in[1] = keyboard_buffer[1];
		hid_report_in[2] = keyboard_buffer[2];
	    hid_report_in[3] = keyboard_buffer[3];
    	hid_report_in[4] = keyboard_buffer[4];
    	hid_report_in[5] = keyboard_buffer[5];
    	hid_report_in[6] = keyboard_buffer[6];
    	hid_report_in[7] = keyboard_buffer[7];

		keyboard_buffer[0] =
		keyboard_buffer[2] =
		keyboard_buffer[3] =
		keyboard_buffer[4] =
		keyboard_buffer[5] =
		keyboard_buffer[6] =
		keyboard_buffer[7] = 0;

		if( hid_report_out_flag > 0 )
		{
    		//Send the 8 byte packet over USB to the host.
			lastINTransmissionKeyboard = HIDTxPacket(HID_EP3, (BYTE*)hid_report_in, 0x08);
			hid_report_out_flag--;
		}	
	}
   if(!HIDTxHandleBusy(lastTransmission2))
    {
        volume_input[0] = volume_buffer[0];
		volume_buffer[0] = 0;

		if( volume_input_out_flag > 0 )
		{
        	lastTransmission2 = HIDTxPacket(HID_EP2, (BYTE*)&volume_input, sizeof(volume_input));
        	volume_input_out_flag--;
 		}      	
    }
//---------------------------------------------------------------------
//	USB�f�[�^�ʐM��
    if(!HIDRxHandleBusy(USBOutHandle))				//Check if data was received from the host.
    {
	    // ���M�o�b�t�@�N���A
	    for( fi = 0; fi < TX_BUFFER_SIZE; fi++)
	    {
		    ToSendDataBuffer[fi] = 0;
		}
		
        switch(ReceivedDataBuffer[0])
        {
            case 0x56: // V=0x56 Get Firmware version
                ToSendDataBuffer[0] = 0x56;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
				tmp = strlen(c_version);
				if( 0 < tmp && tmp <= (64-2) )
				{
					for( fi = 0; fi < tmp; fi++ )
					{
						ToSendDataBuffer[fi+1] = c_version[fi];
					}
					// �Ō��NULL������ݒ�
					ToSendDataBuffer[fi+1] = 0x00;
				}
				else
				{
					//�o�[�W����������ُ�
					ToSendDataBuffer[1] = 0x00;
				}				
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x80:  // EEPROM�Ƀf�[�^��ݒ� [0x80,�擪�A�h���X,�T�C�Y,data1, ... ,data61]
            	/* �p�����[�^�`�F�b�N �擪�A�h���X(0-199)+�f�[�^�T�C�Y(1-61)<=200 �T�C�Y=1-61 */
            	if((ReceivedDataBuffer[1]+ReceivedDataBuffer[2]) <= EEPROM_DATA_TOTAL_SIZE 
            		&& 1 <= ReceivedDataBuffer[2] && ReceivedDataBuffer[2] <= 61)
            	{
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	WriteEEPROM(ReceivedDataBuffer[1]+fi,ReceivedDataBuffer[3+fi]);
		            }
		            // OK�A���T
		            ToSendDataBuffer[1] = 0x00;
	           	}
	           	else
	           	{
		           	// NG�A���T
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                ToSendDataBuffer[0] = 0x80;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x81:  // EEPROM�Ƀf�[�^��ݒ� [0x81,�f�[�^No,�T�C�Y,data1, ... ,data10]
            	/* �p�����[�^�`�F�b�N �f�[�^NO=1-20 �T�C�Y=10 */
            	if( 1 <= ReceivedDataBuffer[1] && ReceivedDataBuffer[1] <= EEPROM_DATA_NUM && ReceivedDataBuffer[2] == EEPROM_DATA_SIZE)
            	{
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	WriteEEPROM((ReceivedDataBuffer[1]-1)*EEPROM_DATA_SIZE+fi,ReceivedDataBuffer[3+fi]);
		            }
		            // OK�A���T
		            ToSendDataBuffer[1] = 0x00;
		            
		        	// 0�o�C�g�ڂ�ǂݍ���ł���o�b�t�@���X�V
					eeprom_check_data[ReceivedDataBuffer[1]-1] = ReceivedDataBuffer[7];
	           	}
	           	else
	           	{
		           	// NG�A���T
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                ToSendDataBuffer[0] = 0x81;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x82:  // EEPROM����f�[�^�ǂݏo�� [0x82,�擪�A�h���X,�T�C�Y] -> �A���T[0x82,�T�C�Y,data1, ... , data61]
				ToSendDataBuffer[0] = 0x82;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
            	/* �p�����[�^�`�F�b�N �擪�A�h���X(0-199)+�f�[�^�T�C�Y(1-61)<=200 �T�C�Y=1-61 */
            	if((ReceivedDataBuffer[1]+ReceivedDataBuffer[2]) <= EEPROM_DATA_TOTAL_SIZE 
            		&& 1 <= ReceivedDataBuffer[2] && ReceivedDataBuffer[2] <= 61)
            	{
	            	ToSendDataBuffer[1] = ReceivedDataBuffer[2];
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	ToSendDataBuffer[2+fi] = ReadEEPROM(ReceivedDataBuffer[1]+fi);
		            }
	            }
	           	else
	           	{
		           	// NG�A���T
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x83:  // EEPROM����f�[�^�ǂݏo�� [0x83,�f�[�^No,�T�C�Y] -> �A���T[0x83,�T�C�Y,data1, ... , data10]
				ToSendDataBuffer[0] = 0x83;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
            	/* �p�����[�^�`�F�b�N �f�[�^NO=1-20 �T�C�Y=10 */
            	if( 1 <= ReceivedDataBuffer[1] && ReceivedDataBuffer[1] <= EEPROM_DATA_NUM && ReceivedDataBuffer[2] == EEPROM_DATA_SIZE)
            	{
	            	ToSendDataBuffer[1] = ReceivedDataBuffer[2];
	            	for( fi = 0; fi < ReceivedDataBuffer[2]; fi++)
	            	{
		            	ToSendDataBuffer[2+fi] = ReadEEPROM((ReceivedDataBuffer[1]-1)*EEPROM_DATA_SIZE+fi);
		            }
	           	}
	           	else
	           	{
		           	// NG�A���T
		           	ToSendDataBuffer[1] = 0xFF;
		        }
                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
#if 1	//DEBUG
            case 0x40:
                ToSendDataBuffer[0] = 0x40;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Pushbutton State command.
				ToSendDataBuffer[1] = 0x00;// (unsigned char)(ui_off_count & 0xff);
				ToSendDataBuffer[2] = 0x00;// (unsigned char)(ui_on_count & 0xff);
				ToSendDataBuffer[3] = debug_ary[0];
				ToSendDataBuffer[4] = debug_ary[1];
				ToSendDataBuffer[5] = debug_ary[2];
				ToSendDataBuffer[6] = debug_ary[3];

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x41:
                ToSendDataBuffer[0] = 0x41;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
#endif	//DEBUG
            case 0x50:	/* �ԊO����M�f�[�^�𑗐M */
                ToSendDataBuffer[0] = 0x50;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

//				if(send_read_data_flag == 1)
//				{
//					for(fi = 0; fi < READBUFFER_SIZE; fi++)
//					{
//						ToSendDataBuffer[fi+1] = uc_process_read_data[fi];
//					}
//				}
//				else
//				{
					for(fi = 0; fi < READBUFFER_SIZE; fi++)
					{
						ToSendDataBuffer[fi+1] = 0x00;
					}
//				}

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x51:	/* �ԊO����M�҂����[�h�ݒ� */
	            uc_read_code_type = CODE_TYPE_STANDARD;
//            	if( RECEIVE_WAIT_MODE_WAIT == ReceivedDataBuffer[1] )
//            	{	/* ��M�҂����[�h�ݒ� */
//	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_WAIT;
//	            }
//	            else
//	            {	/* ��M�҂����[�h���� */
//	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_NONE;
//	         	}
                ToSendDataBuffer[0] = 0x51;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x52:	/* �ԊO����M�f�[�^�𑗐M �g���� */
                ToSendDataBuffer[0] = 0x52;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.
//
//				if(send_read_data_flag == 1)
//				{
//					for(fi = 0; fi < READBUFFER_SIZE_EX; fi++)
//					{
//						ToSendDataBuffer[fi+1] = uc_process_read_data[fi];
//					}
//				}
//				else
//				{
					for(fi = 0; fi < READBUFFER_SIZE_EX; fi++)
					{
						ToSendDataBuffer[fi+1] = 0x00;
					}
//				}

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
            case 0x53:	/* �ԊO����M�҂����[�h�ݒ� �g���� */
//            	if( RECEIVE_WAIT_MODE_WAIT == ReceivedDataBuffer[1] )
//            	{	/* ��M�҂����[�h�ݒ� */
//	            	uc_read_code_type = CODE_TYPE_EXTENSION;
//	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_WAIT;
//	            }
//	            else
//	            {	/* ��M�҂����[�h���� */
//	            	uc_receive_wait_mode = RECEIVE_WAIT_MODE_NONE;
//	            	uc_read_code_type = CODE_TYPE_STANDARD;
//	         	}
                ToSendDataBuffer[0] = 0x53;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.

                if(!HIDTxHandleBusy(USBInHandle))
                {
                    USBInHandle = HIDTxPacket(HID_EP4,(BYTE*)&ToSendDataBuffer[0],64);
                }
                break;
                
            case 0x60:	/* �ԊO���o�̓f�[�^����M���� */
                ToSendDataBuffer[0] = 0x60;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.
				
				uc_out_code_type = CODE_TYPE_STANDARD;
				for(fi = 0; fi < OUTBUFFER_SIZE; fi++){
					uc_out_buff[fi] = ReceivedDataBuffer[fi+1];
				}
                break;
            case 0x61:	/* �ԊO���o�̓f�[�^����M���� �g���� */
                ToSendDataBuffer[0] = 0x61;				//Echo back to the host PC the command we are fulfilling in the first byte.  In this case, the Get Remocon Data command.
				
				uc_out_code_type = CODE_TYPE_EXTENSION;
				for(fi = 0; fi < OUTBUFFER_SIZE_EX; fi++){
					uc_out_buff[fi] = ReceivedDataBuffer[fi+1];
				}
                break;
        }
         //Re-arm the OUT endpoint for the next packet
        USBOutHandle = HIDRxPacket(HID_EP4,(BYTE*)&ReceivedDataBuffer,64);
    }
}




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {       
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(HID_EP2,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(HID_EP3,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(HID_EP4,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }      
    return TRUE; 
}

/*******************************************************************
 * Function:        int RemoconOutData(
 *                        void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          int - A processing result is returned. 0 is normal.
 *
 * Side Effects:    None
 *
 * Overview:        ���̊֐��́A�ԊO�������R���̃f�[�^���o�͂��܂��B    
 *                  �o�̓o�b�t�@[0]�̃t�H�[�}�b�g�^�C�v�]���A[1]�ȍ~�̃f�[�^���o�͂��܂��B 
 *                  0 is returned at the time of normal processing.
 *
 * Note:            None
 *******************************************************************/
int RemoconOutData(void)
{
	int ret_code = 0;
	int fi = 0;
	unsigned int byte_pos = 0;
	unsigned int bit_pos = 0;
	unsigned char send_bit = 0;
	unsigned char format_type;
	unsigned char out_buffer_offset;
	
	/* �o�͏�ԂłȂ� */
	if( 0 >= ui_out_on_time && 0 >= ui_out_off_time )
	{
		/* �o�͑҂� */
		if( uc_out_status == OUTSTATUS_OUTPUT_WAIT )
		{

			/* ������ */
			ui_out_pos = 0;	/* �o�͈ʒu */
            //�I�����o�J�E���g������
            uc_end_count = 0;
			
			// �t�H�[�}�b�g�^�C�v�Z�b�g
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// �g���^�� 0 �o�C�g��
				format_type = uc_out_buff[0];
			}
			else
			{	// �W���^�� 0�o�C�g�ڂ̉���4bit
				format_type = (uc_out_buff[0] & 0x0F);
			}
			
			/* �o�̓o�b�t�@�ɏo�̓f�[�^������H */
			if( format_type != FORMATCODE_UNKNOWN )
			{
				/* ���̏�Ԃ� */
				uc_out_status = OUTSTATUS_DATA_CODE;
				/* �f�[�^�T�C�Y�ݒ�(bit) */
				if(uc_out_code_type == CODE_TYPE_EXTENSION)
				{	// �g���^�� 1�o�C�g�ڂ�2�o�C�g��
					uc_send_bit_size = uc_out_buff[1];
					uc_send_bit_size_2nd_data = uc_out_buff[2];
				}
				else
				{	// �W���^�� 0�o�C�g�ڂ̏��4bit	
					/* uc_out_buff[0]�̏��4�r�b�g�ɏo�̓f�[�^��/4�̒l�������Ă��� */
					/* (uc_out_buff[0] >> 4) * 4 = uc_out_buff[0] >> 2 */
					uc_send_bit_size = uc_out_buff[0];
					uc_send_bit_size = ((uc_send_bit_size&0xF0)>>2);
					uc_send_bit_size_2nd_data = 0;
				}
				
				// PWM Duty Set
				ClosePWM1();
				if(format_type == FORMATCODE_DAIKIN && uc_send_bit_size == 0x38)
				{	// 33kHz
					ui_PWM50_Set_Val = PWM_50_33k;
					OpenPWM1(PWM_PERIOD_33k);
				}
				else
				{	// 38kHz
					ui_PWM50_Set_Val = PWM_50;
					OpenPWM1(PWM_PERIOD);
				}
					
				
				/* ON��OFF���Ԃ��Z�b�g*/
				/* �Ɠd���t�H�[�}�b�g */
				if( format_type == FORMATCODE_KADEN )
				{
					ui_out_on_time = READERCODE_ON_KADEN;
					ui_out_off_time = READERCODE_OFF_KADEN;
					
					uc_out_data0_on = DATA0_ON_KADEN;
					uc_out_data0_off = DATA0_OFF_KADEN;
					uc_out_data1_on = DATA1_ON_KADEN;
					uc_out_data1_off = DATA1_OFF_KADEN;
				}
				/* NEC�t�H�[�}�b�g */
				else if( format_type == FORMATCODE_NEC )
				{
					ui_out_on_time = READERCODE_ON_NEC;
					ui_out_off_time = READERCODE_OFF_NEC;
					
					uc_out_data0_on = DATA0_ON_NEC;
					uc_out_data0_off = DATA0_OFF_NEC;
					uc_out_data1_on = DATA1_ON_NEC;
					uc_out_data1_off = DATA1_OFF_NEC;
				}
				/* SONY�t�H�[�}�b�g */
				else if( format_type == FORMATCODE_SONY )
				{
					ui_out_on_time = READERCODE_ON_SONY;
					ui_out_off_time = READERCODE_OFF_SONY;
					
					uc_out_data0_on = DATA0_ON_SONY;
					uc_out_data0_off = DATA0_OFF_SONY;
					uc_out_data1_on = DATA1_ON_SONY;
					uc_out_data1_off = DATA1_OFF_SONY;
				}
				/* MITSUBISHI�t�H�[�}�b�g */
				else if( format_type == FORMATCODE_MITSU )
				{
					/* ���[�_�R�[�h�Ȃ� */
					ui_out_on_time = 0;
					ui_out_off_time = 0;
					
					uc_out_data0_on = DATA0_ON_M;
					uc_out_data0_off = DATA0_OFF_M;
					uc_out_data1_on = DATA1_ON_M;
					uc_out_data1_off = DATA1_OFF_M;
				}
				/* DAIKIN�t�H�[�}�b�g */
				else if( format_type == FORMATCODE_DAIKIN )
				{
					ui_out_on_time = READERCODE_ON_DAI;
					ui_out_off_time = READERCODE_OFF_DAI;
					
					uc_out_data0_on = DATA0_ON_DAI;
					uc_out_data0_off = DATA0_OFF_DAI;
					uc_out_data1_on = DATA1_ON_DAI;
					uc_out_data1_off = DATA1_OFF_DAI;
				}
				/* DAIKIN2�t�H�[�}�b�g */
				else if( format_type == FORMATCODE_DAIKIN2 )
				{
					ui_out_on_time = READERCODE_ON_DAI2;
					ui_out_off_time = READERCODE_OFF_DAI2;
					
					uc_out_data0_on = DATA0_ON_DAI;
					uc_out_data0_off = DATA0_OFF_DAI;
					uc_out_data1_on = DATA1_ON_DAI;
					uc_out_data1_off = DATA1_OFF_DAI;
				}
				/* �t�H�[�}�b�g�s�� */
				else
				{
					/* �o�̓o�b�t�@�N���A */
					for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
					{
						uc_out_buff[fi] = 0;
					}
					uc_out_status = OUTSTATUS_OUTPUT_WAIT;
					uc_send_bit_size = 0;
					uc_send_bit_size_2nd_data = 0;
					format_type = 0;
				}
			}
		}
		else if( uc_out_status == OUTSTATUS_DATA_CODE )
		{
			// �t�H�[�}�b�g�^�C�v�Z�b�g
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// �g���^�� 0 �o�C�g��
				format_type = uc_out_buff[0];
				out_buffer_offset = OUTBUFFER_SIZE_EX - OUTBUFFER_DATA_AREA_SIZE_EX;
			}
			else
			{	// �W���^�� 0�o�C�g�ڂ̉���4bit
				format_type = (uc_out_buff[0] & 0x0F);
				out_buffer_offset = OUTBUFFER_SIZE - OUTBUFFER_DATA_SIZE;
			}

			byte_pos = ui_out_pos / 8;
			bit_pos = ui_out_pos % 8;
			
			if ( ( ui_out_pos == (unsigned int)uc_send_bit_size && uc_send_bit_size_2nd_data == 0 ) || ui_out_pos > DATA_MAX_BITS )
			{	/* �I���R�[�h */
				send_bit = 0xF0;
                
                //�I�����o�J�E���g+1
                uc_end_count++;
			}
			else if(ui_out_pos == (unsigned int)uc_send_bit_size && uc_send_bit_size_2nd_data > 0)
			{	/* 2nd�f�[�^���� */
				send_bit = 3;
			}	
			else if( 0 <= (byte_pos+out_buffer_offset) && (byte_pos+out_buffer_offset) < OUTBUFFER_SIZE_EX )
			{
				send_bit = (uc_out_buff[byte_pos+out_buffer_offset] >> bit_pos ) & 0x01;
			}
			else
			{	/* �G���[ */
				send_bit = 0xFF;
			}	

			if( send_bit == 0 )
			{	/* DATA0�o�� */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = uc_out_data0_off;
			}
			else if ( send_bit == 1 )
			{	/* DATA1�o�� */
				ui_out_on_time = uc_out_data1_on;
				ui_out_off_time = uc_out_data1_off;
			}
			else if ( send_bit == 0xF0 )
			{	/* �I���R�[�h�o�� */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = SENDCODE_END_CNT;
			}
			else if ( send_bit == 3 )
			{	/* ���̃f�[�^�܂ł̊Ԋu���󂯂� */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = DATA_CODE_INTERVAL_SEND_CNT;
				
				// ���[�_�R�[�h���M��Ԃ�
				uc_out_status = OUTSTATUS_2ND_READER_CODE;
			}
			else if ( send_bit == 0xFF )
			{	/* �G���[ */
				ui_out_on_time = 0;
				ui_out_off_time = 0;
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}
			
			/* SONY�t�H�[�}�b�g�̓g���[���[�R�[�h�Ȃ� */
			if( format_type == FORMATCODE_SONY )
			{
				//�ŏI�f�[�^���M����OFF�𒷂�����
				if ( ui_out_pos == (unsigned int)(uc_send_bit_size - 1) )
				{	/* �I���R�[�h */
					ui_out_off_time = SENDCODE_END_CNT;
				}	
			}

			/* �Ɠd�t�H�[�}�b�g�̏ꍇ��end trail�p���X�̉񐔂ŕ��� */
			if( format_type == FORMATCODE_KADEN )
			{
				//�ŏI�f�[�^���M��
				if ( ui_out_pos == (unsigned int)(uc_send_bit_size) )
				{	
					ui_out_off_time = (uc_end_count == 1) ? SENDCODE_END_CNT / 3 : SENDCODE_END_CNT;
				}	
			}
				
            /* �Ɠd�t�H�[�}�b�g�̏ꍇ��end trail�p���X��2�񑗏o����(�|���@�΍�) */
			if( (format_type != FORMATCODE_KADEN || uc_end_count >= 2) && send_bit >= 0xF0 )
//if ( send_bit >= 0xF0)
			{	/* ���̑��M�f�[�^�Ȃ� */
				/* �o�̓o�b�t�@�N���A */
				for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
				{
					uc_out_buff[fi] = 0;
				}
				/* �o�͑҂� */
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}
			else if(send_bit == 0 || send_bit == 1)
			{
				/* �o�̓f�[�^�ʒu���C���N�������g */
				ui_out_pos++;
			}
		}
		else if( uc_out_status == OUTSTATUS_2ND_READER_CODE )
		{	// 2nd Reader Code�o��

			uc_out_status = OUTSTATUS_2ND_DATA_CODE;
			
			// �t�H�[�}�b�g�^�C�v�Z�b�g
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// �g���^�� 0 �o�C�g��
				format_type = uc_out_buff[0];
				out_buffer_offset = OUTBUFFER_SIZE_EX - OUTBUFFER_DATA_AREA_SIZE_EX;
			}
			else
			{	// �W���^�� 0�o�C�g�ڂ̉���4bit
				format_type = (uc_out_buff[0] & 0x0F);
				out_buffer_offset = OUTBUFFER_SIZE - OUTBUFFER_DATA_SIZE;
			}
			
			/* �Ɠd���t�H�[�}�b�g */
			if( format_type == FORMATCODE_KADEN )
			{
				ui_out_on_time = READERCODE_ON_KADEN;
				ui_out_off_time = READERCODE_OFF_KADEN;
			}
			/* NEC�t�H�[�}�b�g */
			else if( format_type == FORMATCODE_NEC )
			{
				ui_out_on_time = READERCODE_ON_NEC;
				ui_out_off_time = READERCODE_OFF_NEC;
			}
#if 0
			/* SONY�t�H�[�}�b�g */
			else if( format_type == FORMATCODE_SONY )
			{
				ui_out_on_time = READERCODE_ON_SONY;
				ui_out_off_time = READERCODE_OFF_SONY;
			}
			/* MITSUBISHI�t�H�[�}�b�g */
			else if( format_type == FORMATCODE_MITSU )
			{
				/* ���[�_�R�[�h�Ȃ� */
				ui_out_on_time = 0;
				ui_out_off_time = 0;
			}
#endif
			/* DAIKIN�t�H�[�}�b�g */
			else if( format_type == FORMATCODE_DAIKIN )
			{
				ui_out_on_time = READERCODE_ON_DAI;
				ui_out_off_time = READERCODE_OFF_DAI;
			}
			/* DAIKIN2�t�H�[�}�b�g */
			else if( format_type == FORMATCODE_DAIKIN2 )
			{
				ui_out_on_time = READERCODE_ON_DAI2;
				ui_out_off_time = READERCODE_OFF_DAI2;
			}
			/* �t�H�[�}�b�g�s�� */
			else
			{
				/* �o�̓o�b�t�@�N���A */
				for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
				{
					uc_out_buff[fi] = 0;
				}
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
				uc_send_bit_size = 0;
				uc_send_bit_size_2nd_data = 0;
				format_type = 0;
			}
		}
		else if( uc_out_status == OUTSTATUS_2ND_DATA_CODE )
		{	// 2nd Data Code�o��

			// �t�H�[�}�b�g�^�C�v�Z�b�g
			if(uc_out_code_type == CODE_TYPE_EXTENSION)
			{	// �g���^�� 0 �o�C�g��
				format_type = uc_out_buff[0];
				out_buffer_offset = OUTBUFFER_SIZE_EX - OUTBUFFER_DATA_AREA_SIZE_EX;
			}
			else
			{	// �W���^�� 0�o�C�g�ڂ̉���4bit
				format_type = (uc_out_buff[0] & 0x0F);
				out_buffer_offset = OUTBUFFER_SIZE - OUTBUFFER_DATA_SIZE;
			}
				
			byte_pos = ui_out_pos / 8;
			bit_pos = ui_out_pos % 8;
			
			if ( ui_out_pos == (unsigned int)(uc_send_bit_size + uc_send_bit_size_2nd_data) || ui_out_pos > DATA_MAX_BITS )
			{	/* �I���R�[�h */
				send_bit = 0xF0;
			}
			else if( 0 <= (byte_pos+out_buffer_offset) && (byte_pos+out_buffer_offset) < OUTBUFFER_SIZE_EX )
			{
				send_bit = (uc_out_buff[byte_pos+out_buffer_offset] >> bit_pos ) & 0x01;
			}
			else
			{	/* �G���[ */
				send_bit = 0xFF;
			}	

			if( send_bit == 0 )
			{	/* DATA0�o�� */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = uc_out_data0_off;
			}
			else if ( send_bit == 1 )
			{	/* DATA1�o�� */
				ui_out_on_time = uc_out_data1_on;
				ui_out_off_time = uc_out_data1_off;
			}
			else if ( send_bit == 0xF0 )
			{	/* �I���R�[�h�o�� */
				ui_out_on_time = uc_out_data0_on;
				ui_out_off_time = SENDCODE_END_CNT;
			}
			else if ( send_bit == 0xFF )
			{	/* �G���[ */
				ui_out_on_time = 0;
				ui_out_off_time = 0;
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}

			if( send_bit >= 0xF0 )
			{	/* ���̑��M�f�[�^�Ȃ� */
				/* �o�̓o�b�t�@�N���A */
				for( fi = 0; fi < OUTBUFFER_SIZE_EX; fi++ )
				{
					uc_out_buff[fi] = 0;
				}
				/* �o�͑҂� */
				uc_out_status = OUTSTATUS_OUTPUT_WAIT;
			}
			/* �o�̓f�[�^�ʒu���C���N�������g */
			ui_out_pos++;
		}				
	}

	/* �o��ON�� */
	if( 0 < ui_out_on_time )
	{
		SetDCPWM1(ui_PWM50_Set_Val);
		//SetDCPWM1(PWM_50);
		ui_out_on_time--;
	}
	/* �o��OFF�� */
	else if( 0 < ui_out_off_time )
	{
		SetDCPWM1(PWM_OFF);
		ui_out_off_time--;
	}
	else
	{
		SetDCPWM1(PWM_OFF);
	}

    return ret_code; 
}

/*******************************************************************
 * Function:        int RemoconReceiveData(
 *                        void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          int - A processing result is returned. 0 is normal.
 *
 * Side Effects:    None
 *
 * Overview:        This function receives the data of infrared    
 *                  remote control. The received data is stored 
 *                  in a receiving buffer. 
 *                  0 is returned at the time of normal processing.
 *
 * Note:            None
 *******************************************************************/
int RemoconReceiveData(void)
{
//	
//	int ret_code = 0;
//	int ret = 0;
//	int fi = 0;
//	unsigned char set_size_data = 0;
//	unsigned char set_2nd_size_data = 0;
//	unsigned char set_bit_data = 0;
//	unsigned int byte_pos = 0;
//	unsigned int bit_pos = 0;
//	unsigned char bit_mask_data = 1;
//	unsigned char first_bit = 0;
//	unsigned char second_bit = 0;
//	unsigned char samae_bit_count = 0;
//
//	// ���i���PIN5  ������PIN8
////	if( !PIN8 == ON ){
//	if( !PIN5 == ON ){
//		uc_now_signal = ON;
//	}
//	else
//	{
//		uc_now_signal = OFF;
//	}
//
//	/* ON/OFF�J�E���g */
//	if( uc_now_signal == ON )
//	{
//		ui_on_count++;
//	}
//	else
//	{
//		ui_off_count++;
//	}
//	
//	/* �O��OFF�@�����@����ON �����@�J�E���g�� �̂Ƃ� */
//	if( (uc_pre_signal == OFF && uc_now_signal == ON && ui_on_count > 0 && ui_off_count > 0)
//		|| ( ui_on_count > 0 && ui_off_count > READCODE_END_CNT))
//	{
//		/* ���[�^�R�[�h�҂��H */
//		if ( uc_read_status == READSTATUS_READERCODE_WAIT ){
//
//			set_bit_data = 0xFF;
//
//			/* ���[�_�R�[�h��M */
//			/* ���� */
//			/* �Ɠd������ */
//			if( uc_format_type == FORMATCODE_UNKNOWN )
//			{
//				if(READERCODE_MIN_KADEN <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_KADEN)
//				{
//					if( (ui_off_count*(READERCODE_ON_T_KADEN-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_KADEN+1)))
//					{
//						uc_format_type = FORMATCODE_KADEN;
//					}
//				}
//			}
//			/* NEC���� */
//			if( uc_format_type == FORMATCODE_UNKNOWN )
//			{
//				if(READERCODE_MIN_NEC <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_NEC)
//				{
//					if( (ui_off_count*(READERCODE_ON_T_NEC-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_NEC+1)))
//					{
//						uc_format_type = FORMATCODE_NEC;
//					}
//				}	
//			}
//			/* SONY���� */
//			if( uc_format_type == FORMATCODE_UNKNOWN )
//			{
//				if(READERCODE_MIN_SONY <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_SONY)
//				{
//					if( (ui_off_count*(READERCODE_ON_T_SONY-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_SONY+1)))
//					{
//						uc_format_type = FORMATCODE_SONY;
//					}
//				}
//			}
//			/* DAIKIN���� */
//			if( uc_format_type == FORMATCODE_UNKNOWN )
//			{
//				if(READERCODE_MIN_DAI <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI)
//				{
//					if( (ui_off_count*(READERCODE_ON_T_DAI-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_DAI+1)))
//					{
//						uc_format_type = FORMATCODE_DAIKIN;
//					}
//				}	
//			}
//			/* DAIKIN2���� */
//			if( uc_format_type == FORMATCODE_UNKNOWN )
//			{
//				if(READERCODE_MIN_DAI2 <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI2)
//				{
//					if( (ui_on_count*(READERCODE_OFF_T_DAI2-1)) < ui_off_count && ui_off_count < (ui_on_count*(READERCODE_OFF_T_DAI2+1)))
//					{
//						uc_format_type = FORMATCODE_DAIKIN2;
//					}
//				}
//			}
//			/* ���[�_�R�[�h�����̂��ߍŌ�ɔ��肷�邱�� */
//			/* MITSUBISHI���� ���[�_�R�[�h�Ȃ��̂��߁A�����Ȃ�f�[�^���� */
//			if( uc_format_type == FORMATCODE_UNKNOWN )
//			{
//				if(DATACODE_MIN_M <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_M)
//				{
//					if( 2 < ui_on_count && ui_on_count < 6)
//					{
//						uc_format_type = FORMATCODE_MITSU;
//
//						if( ui_off_count >= DATACODE_DATA1_M )
//						{	/* DATA 1 */
//							set_bit_data = ON;
//						}
//						else
//						{	/*DATA 0*/
//							set_bit_data = OFF;
//						}
//					}
//				}
//			}
//		
//			/* �J�E���^���Z�b�g */
//			ui_on_count = 1;	/* ����ON�Ȃ̂�0�ł͂Ȃ�1���Z�b�g */
//			ui_off_count = 0;
//
//			if( uc_format_type != FORMATCODE_UNKNOWN )
//			{	
//				for( fi = 0 ; fi < READBUFFER_SIZE_EX ; fi++ ){
//					uc_read_buff[fi] = 0;
//				}
//
//				/* ���[�_�R�[�h�Z�b�g */
//				uc_read_buff[0] = uc_format_type;
//	
//				/* ��M��ԃC���N�������g */
//				uc_read_status = READSTATUS_DATA_WAIT;
//			}
//			//���[�_�R�[�h�Ȃ��ŁA�����Ȃ�f�[�^�̏ꍇ
//			if( set_bit_data == ON || set_bit_data == OFF )
//			{
//				if( set_bit_data == ON )
//				{
//					uc_read_buff[1] = 0x01;
//				}
//				else if ( set_bit_data == OFF )
//				{
//					uc_read_buff[1] = 0x00;
//				}		
//				ui_data_pos = 1;
//			}	
//		}
//		/* �f�[�^��M�҂��H */
//		else if ( uc_read_status == READSTATUS_DATA_WAIT ){
//			
//			/* �f�[�^�R�[�h��M */
//			/*  */
//			set_bit_data = 0xFF;
//		
//			/* ���� */
//			/* �Ɠd������ */
//			if( uc_format_type == FORMATCODE_KADEN )
//			{
//				if(DATACODE_MIN_KADEN <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_KADEN)
//				{
//					if( ui_off_count >= DATACODE_DATA1_KADEN )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//			/* NEC���� */
//			else if( uc_format_type == FORMATCODE_NEC )
//			{
//				if(DATACODE_MIN_NEC <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_NEC)
//				{
//					if( ui_off_count >= DATACODE_DATA1_NEC )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//			/* SONY���� */
//			else if( uc_format_type == FORMATCODE_SONY )
//			{
//				// SONY�̓g���[���[�R�[�h�Ȃ��̂��߁A�I���R�[�h���ɂ��f�[�^���肷��
//				if((DATACODE_MIN_SONY <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_SONY)
//					|| ( ui_on_count > 0 && ui_off_count > READCODE_END_CNT))
//				{
//					if( ui_on_count >= DATACODE_DATA1_SONY )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//			/* DAIKIN or DAIKIN2 ���� */
//			else if( uc_format_type == FORMATCODE_DAIKIN || uc_format_type == FORMATCODE_DAIKIN2 )
//			{
//				if(DATACODE_MIN_DAI <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_DAI)
//				{
//					if( ui_off_count >= DATACODE_DATA1_DAI )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//			/* MITSUBISHI���� */
//			else if( uc_format_type == FORMATCODE_MITSU )
//			{
//				if(DATACODE_MIN_M <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_M)
//				{
//					if( ui_off_count >= DATACODE_DATA1_M )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//
//			if( set_bit_data == ON || set_bit_data == OFF )
//			{
//				byte_pos = ui_data_pos / 8;
//				bit_pos = ui_data_pos % 8;
//				
//				if(uc_read_code_type == CODE_TYPE_EXTENSION)
//				{	// �g���̎�
//					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_AREA_SIZE_EX )
//					{
//						if( set_bit_data == ON )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] | (bit_mask_data << bit_pos);
//						}
//						else if ( set_bit_data == OFF )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] & ~(bit_mask_data << bit_pos);
//						}		
//					}
//				}
//				else
//				{	// �W���̂Ƃ���
//					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_SIZE )
//					{
//						if( set_bit_data == ON )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] | (bit_mask_data << bit_pos);
//						}
//						else if ( set_bit_data == OFF )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] & ~(bit_mask_data << bit_pos);
//						}		
//					}
//				}
//				
//				ui_data_pos++;
//			}
//
//			if(uc_read_code_type == CODE_TYPE_EXTENSION)
//			{	// �g��
//				// �f�[�^�����Q�ɕ�����Ă���ꍇ
//				if((uc_format_type == FORMATCODE_KADEN || uc_format_type == FORMATCODE_NEC || uc_format_type == FORMATCODE_DAIKIN || uc_format_type == FORMATCODE_DAIKIN2) 
//					&& DATA_CODE_INTERVAL_MIN_CNT <= ui_off_count && ui_off_count <= DATA_CODE_INTERVAL_MAX_CNT )
//				{
//					/* 2nd�f�[�^�̃��[�_�R�[�h�҂��� */
//					uc_read_status = READSTATUS_2ND_READERCODE_WAIT;
//					/* 1st�f�[�^�̃f�[�^�����Z�b�g */
//					uc_first_data_bit = (unsigned char)(ui_data_pos & 0xFF);
//				}
//				if(ui_off_count > DATA_CODE_INTERVAL_MAX_CNT || ui_data_pos == DATA_MAX_BITS )
//				{
//					/* ��M��ԃC���N�������g */
//					uc_read_status = READSTATUS_READ_END;
//					/* 1st�f�[�^�̃f�[�^�����Z�b�g */
//					uc_first_data_bit = (unsigned char)(ui_data_pos & 0xFF);
//				}
//			}
//			else
//			{	// �W��
//				if(ui_off_count > READCODE_END_CNT || ui_data_pos == DATA_MAX_BITS )
//				{
//					/* ��M��ԃC���N�������g */
//					uc_read_status = READSTATUS_READ_END;
//					/* 1st�f�[�^�̃f�[�^�����Z�b�g */
//					uc_first_data_bit = (unsigned char)(ui_data_pos & 0xFF);
//				}
//			}		
//	
//
//			/* �J�E���^���Z�b�g */
//			ui_on_count = 1;	/* ����ON�Ȃ̂�0�ł͂Ȃ�1���Z�b�g */
//			ui_off_count = 0;
//		}
//		/* �f�[�^��M�҂��H */
//		else if ( uc_read_status == READSTATUS_2ND_READERCODE_WAIT )
//		{
//			/* �Ɠd������ */
//			if( uc_format_type == FORMATCODE_KADEN )
//			{
//				if(READERCODE_MIN_KADEN <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_KADEN)
//				{
////					if( (ui_off_count*(READERCODE_ON_T_KADEN-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_KADEN+1)))
////					{
////						uc_format_type = FORMATCODE_KADEN;
////					}
//					
//					/* �J�E���^���Z�b�g */
//					ui_on_count = 1;	/* ����ON�Ȃ̂�0�ł͂Ȃ�1���Z�b�g */
//					ui_off_count = 0;
//
//					/* ��M��ԃC���N�������g��2nd�f�[�^��M�҂��� */
//					uc_read_status = READSTATUS_2ND_DATA_WAIT;
//				}
//			}
//			/* NEC���� */
//			else if( uc_format_type == FORMATCODE_NEC )
//			{
//				if(READERCODE_MIN_NEC <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_NEC)
//				{
////					if( (ui_off_count*(READERCODE_ON_T_NEC-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_NEC+1)))
////					{
////						uc_format_type = FORMATCODE_NEC;
////					}
//					
//					/* �J�E���^���Z�b�g */
//					ui_on_count = 1;	/* ����ON�Ȃ̂�0�ł͂Ȃ�1���Z�b�g */
//					ui_off_count = 0;
//
//					/* ��M��ԃC���N�������g��2nd�f�[�^��M�҂��� */
//					uc_read_status = READSTATUS_2ND_DATA_WAIT;
//				}	
//			}
//			/* DAIKIN���� */
//			else if( uc_format_type == FORMATCODE_DAIKIN )
//			{
//				if(READERCODE_MIN_DAI <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI)
//				{
////					if( (ui_off_count*(READERCODE_ON_T_DAI-1)) < ui_on_count && ui_on_count < (ui_off_count*(READERCODE_ON_T_DAI+1)))
////					{
////						uc_format_type = FORMATCODE_DAIKIN;
////					}
//
//					/* �J�E���^���Z�b�g */
//					ui_on_count = 1;	/* ����ON�Ȃ̂�0�ł͂Ȃ�1���Z�b�g */
//					ui_off_count = 0;
//
//					/* ��M��ԃC���N�������g��2nd�f�[�^��M�҂��� */
//					uc_read_status = READSTATUS_2ND_DATA_WAIT;
//				}	
//			}
//			/* DAIKIN2���� */
//			else if( uc_format_type == FORMATCODE_DAIKIN2 )
//			{
//				if(READERCODE_MIN_DAI2 <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= READERCODE_MAX_DAI2)
//				{
////					if( (ui_on_count*(READERCODE_OFF_T_DAI2-1)) < ui_off_count && ui_off_count < (ui_on_count*(READERCODE_OFF_T_DAI2+1)))
////					{
////						uc_format_type = FORMATCODE_DAIKIN2;
////					}
//					
//					/* �J�E���^���Z�b�g */
//					ui_on_count = 1;	/* ����ON�Ȃ̂�0�ł͂Ȃ�1���Z�b�g */
//					ui_off_count = 0;
//
//					/* ��M��ԃC���N�������g��2nd�f�[�^��M�҂��� */
//					uc_read_status = READSTATUS_2ND_DATA_WAIT;
//				}
//			}
//
//
//			if( ui_off_count > READCODE_END_CNT || ui_data_pos == DATA_MAX_BITS )
//			{
//				/* ��M��ԃC���N�������g */
//				uc_read_status = READSTATUS_READ_END;
//			}
//		}
//		/* �f�[�^��M�҂��H */
//		else if ( uc_read_status == READSTATUS_2ND_DATA_WAIT )
//		{
//			set_bit_data = 0xFF;
//
//			/* �Ɠd������ */
//			if( uc_format_type == FORMATCODE_KADEN )
//			{
//				if(DATACODE_MIN_KADEN <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_KADEN)
//				{
//					if( ui_off_count >= DATACODE_DATA1_KADEN )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//			/* NEC���� */
//			else if( uc_format_type == FORMATCODE_NEC )
//			{
//				if(DATACODE_MIN_NEC <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_NEC)
//				{
//					if( ui_off_count >= DATACODE_DATA1_NEC )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//			/* DAIKIN���� */
//			else if( uc_format_type == FORMATCODE_DAIKIN || uc_format_type == FORMATCODE_DAIKIN2 )
//			{
//				if(DATACODE_MIN_DAI <= (ui_on_count + ui_off_count)
//					&& (ui_on_count + ui_off_count) <= DATACODE_MAX_DAI)
//				{
//					if( ui_off_count >= DATACODE_DATA1_DAI )
//					{	/* DATA 1 */
//						set_bit_data = ON;
//					}
//					else
//					{	/*DATA 0*/
//						set_bit_data = OFF;
//					}
//				}
//			}
//
//
//			if( set_bit_data == ON || set_bit_data == OFF )
//			{
//				byte_pos = ui_data_pos / 8;
//				bit_pos = ui_data_pos % 8;
//				
//				if(uc_read_code_type == CODE_TYPE_EXTENSION)
//				{	// �g���̎�
//					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_AREA_SIZE_EX )
//					{
//						if( set_bit_data == ON )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] | (bit_mask_data << bit_pos);
//						}
//						else if ( set_bit_data == OFF )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] = uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] & ~(bit_mask_data << bit_pos);
//						}		
//					}
//				}
//				else
//				{	// �W���̂Ƃ���
//					if( 0 <= byte_pos && byte_pos < READBUFFER_DATA_SIZE )
//					{
//						if( set_bit_data == ON )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] | (bit_mask_data << bit_pos);
//						}
//						else if ( set_bit_data == OFF )
//						{
//							uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] = uc_read_buff[byte_pos+(READBUFFER_SIZE-READBUFFER_DATA_SIZE)] & ~(bit_mask_data << bit_pos);
//						}		
//					}
//				}
//				
//				ui_data_pos++;
//			}
//
//			if( ui_off_count > READCODE_END_CNT || ui_data_pos == DATA_MAX_BITS )
//			{
//				/* ��M��ԃC���N�������g */
//				uc_read_status = READSTATUS_READ_END;
//			}
//
//			/* �J�E���^���Z�b�g */
//			ui_on_count = 1;	/* ����ON�Ȃ̂�0�ł͂Ȃ�1���Z�b�g */
//			ui_off_count = 0;
//		}
//	}
//
//	/* �f�[�^�ǂݍ��݊����H */
//	if ( uc_read_status == READSTATUS_READ_END ){
//		/* �f�[�^�m�� */
//		if(uc_read_code_type == CODE_TYPE_EXTENSION)
//		{	// �g���̎��͂��̂܂�
//
//			if(uc_first_data_bit == (unsigned char)(ui_data_pos & 0xFF))
//			{
//				set_size_data = uc_first_data_bit;
//				set_2nd_size_data = 0;
//			}
//			else
//			{
//				set_size_data = uc_first_data_bit;
//				set_2nd_size_data = (unsigned char)ui_data_pos - uc_first_data_bit;
//				
//#if 0
//				// First Data��Second Data������v���Ă���ꍇ�͓����f�[�^�����`�F�b�N
//				if(set_size_data == set_2nd_size_data )
//				{
//					samae_bit_count = 0;
//					for(fi = 0; fi < set_size_data; fi++)
//					{
//						byte_pos = fi / 8;
//						bit_pos = fi % 8;
//						first_bit = (uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] >> bit_pos) & 0x01;
//						byte_pos = (set_size_data + fi) / 8;
//						bit_pos = (set_size_data + fi) % 8;
//						second_bit = (uc_read_buff[byte_pos+(READBUFFER_SIZE_EX-READBUFFER_DATA_AREA_SIZE_EX)] >> bit_pos) & 0x01;
//						if(first_bit != second_bit)
//						{
//							break;
//						}
//						else
//						{
//							samae_bit_count++;
//						}
//					}
//					if(set_size_data == samae_bit_count)
//					{
//						set_2nd_size_data = 0;
//					}		
//				}
//#endif
//			}
//			// 8bit�����̓m�C�Y�Ƃ��Ė���
//			if(set_size_data < 8)
//			{
//				set_size_data = 0;
//				set_2nd_size_data = 0;
//			}
//		}
//		else
//		{	// �W���̂Ƃ��́A1/4
//			set_size_data = (unsigned char)((ui_data_pos>>2) & 0xFF);
//		}
//		
//		/* �f�[�^����̏ꍇ�i���[�_�R�[�h�Ȃ��f�[�^�̏ꍇ�Ƀm�C�Y���Ђ낤���Ƃ�����j */
//		if( set_size_data > 0 )
//		{
//			for( fi = 0 ; fi < READBUFFER_SIZE_EX ; fi++ ){
//				uc_fix_read_buff[fi] = uc_read_buff[fi];
//				uc_read_buff[fi] = 0;
//			}
//
//			if(uc_read_code_type == CODE_TYPE_EXTENSION)
//			{	// �g���̂Ƃ��� 1�o�C�g�ڂ�2�o�C�g�ڂɃf�[�^�����Z�b�g
//				uc_fix_read_buff[1] = set_size_data;
//				uc_fix_read_buff[2] = set_2nd_size_data;
//			}
//			else
//			{	// �W���̂Ƃ�
//				/* [0]�̏��4byte�Ƀf�[�^�����Z�b�g */
//				uc_fix_read_buff[0] |= (unsigned char)((set_size_data << 4) & 0xFF);
//			}
//		}
//		else
//		{	/* �m�C�Y���Ђ�����ꍇ�́A�f�[�^�N���A���čēǂݍ��� */
//			for( fi = 0 ; fi < READBUFFER_SIZE_EX ; fi++ ){
//				uc_read_buff[fi] = 0;
//			}
//		}
//
//		/* ������ */
//		uc_read_status = READSTATUS_NEXT_DATA_WAIT;
//		ui_next_data_wait_cnt = 0;
//		ui_data_pos = 0;
//		uc_first_data_bit = 0;
//		uc_format_type = FORMATCODE_UNKNOWN;
//	}
//	/* ���f�[�^�҂� */
//	if ( uc_read_status == READSTATUS_NEXT_DATA_WAIT ){
//		
//		/* OFF����莞�Ԍp�������玟�̃f�[�^�ǂݍ��� */
//		if ( uc_now_signal == OFF )
//		{
//			ui_next_data_wait_cnt++;
//		}
//		else
//		{
//			ui_next_data_wait_cnt = 0;
//		}	
//		/* OFF��莞�Ԍp���H */
//		if( ui_next_data_wait_cnt >= NEXT_DATA_WAIT )
//		{
//			/* ���[�_�R�[�h�҂��� */
//			uc_read_status = READSTATUS_READERCODE_WAIT;
//			/* �J�E���^���Z�b�g */
//			ui_on_count = 0;
//			ui_off_count = 0;
//		}
//	}
//		
//	/* ����̐M���l��ۑ� */	
//	uc_pre_signal = uc_now_signal;
//			
//    return ret_code; 
}
/** EOF main.c *************************************************/
#endif
