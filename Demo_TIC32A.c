/*
 * Demo_TIC32A.c
 *
 */ 



///////////////////////////////////////////////////////////////////////////////
// ���������������� ������ ��� ������������ ���������� TIC32A (� ����� TIC270A)
// � ����������� 128x32 �� ���� ����������� PCF8531. ���������� �����������
// �������������� � ������� ���������� I2C.
// �������� ������������ � ������� ������������� AVRISP mkII.
// ����������� ���������� � Arduino PRO MINI 3.3V ������������ � �����
// TIC32A-ARDUINO_PRO_MINI_3.3V.pdf
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// FUSES: EXT=0xFD, HIGH=0xDA, LOW=0xFF // ��� ������ �� �������� ������
//----------------------------------------------------------------------------
// BODLEVEL: 2V7
// RSTDISBL: -
// DWEN: -
// SPIEN: +
// WDTON: -
// EESAVE: -
// BOOTSZ: 1024W_3C00
// BOOTRST: +
// CKDIV8: -
// CKOUT: -
// SUT_CKSEL: EXTXOSC_8MHZ_XX_16KCK_14CK_65MS
///////////////////////////////////////////////////////////////////////////////




#include "Config.h"
#include "Mt.h"
#include "I2C.h"
#include "Font.h"
#include "TIC32A.h"



// ������ ������� �������������� ������
#define STR_INFO_SIZE	100

// ������� �������������� ������, ������� ����� ���������� � ������� �����
// ����������
char StrInfo[STR_INFO_SIZE];

static uint8_t fLcdReady = FALSE;




///////////////////////////////////////////////////////////////////////////////
// ���������� ��������� ������ ����� Chr
///////////////////////////////////////////////////////////////////////////////
uint8_t IncChar(uint8_t Chr)
{
	Chr++;
	if (Chr == 0x80) Chr = 0xC0;
	if (Chr == 0) Chr = 0x20;
	return Chr;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ Task_Info.
// ������� ������� ������ � ������� ����� ���������� ������� 6x8.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Info(struct pt *Context))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������
	static char Buf[30];
	static uint8_t size;
	static uint8_t pos1, pos2;
	static uint8_t i;

	PT_BEGIN(Context); // ������ �����������
	
	I2C_InitMaster(); // ������������� ������ I2C

	// ������������� ����������
	PT_SPAWN(Context, &ContextChild, LCD_Init(&ContextChild));

	// ������� ���������
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 0,
		LCD_X_RES - 1, LCD_Y_RES - 1));
	
	// �������� ������ ������ � ���, ��� ��������� ����� � ������
	fLcdReady = TRUE;

	// �������������� �������������� ������
	strcpy_P(StrInfo, PSTR("   ����� 6x8: 0123456789���������������������������������������������������������������"));

	// ��������� ���������� �������� �������������� ������, ��������� ��
	// ���������
	size = LCD_X_RES / FONT_Width(FONT_6x8) + 1;

	Buf[size] = '\0';
	pos1 = 0; // �������� � ������ ������� ������������� �������

	while (TRUE)
	{
		pos2 = pos1++;
		if (pos1 >= strlen(StrInfo)) pos1 = 0;
		
		// �������� ����� �������������� ������ � ��������� �����
		for (i = 0; i < size; i++)
		{
			Buf[i] = StrInfo[pos2++];
			if (pos2 >= strlen(StrInfo)) pos2 = 0;
		}

		// ������� �������������� ������
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 0, 0, FONT_6x8,
			Buf, 0));
		
		MT_SleepMs(Context, 900); // ���� 900��				
	}

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ Task_Fonts.
// ������������ ������� � ����������� �������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Fonts(struct pt *Context))
{
	const int CNT_FONTS = 7; // ���������� ��������������� �������
	static struct pt ContextChild; // �������� ��� ��������� �����������
	static char Buf[30];
	static uint8_t chr1, chr2;
	static uint8_t x1, y1, x2, y2;
	static uint8_t size;
	static uint8_t i;
	static uint8_t cnt;
	static uint8_t curFont, idFont;

	PT_BEGIN(Context); // ������ �����������

	// ����, ���� �� ���������� ������������� ����������
	PT_WAIT_UNTIL(Context, fLcdReady);

	BeginDemo:


	// ������������ �������
	///////////////////////////////////////////////////////////////////////////


	// �������� � �������������� ������ ����� �������� ��� ������������
	// ������ 6x8
	strcpy_P(StrInfo, PSTR("   ����� 6x8: 0123456789���������������������������������������������������������������"));

	// ������������ ������� 8x8 � 8x8n
	///////////////////////////////////////////////////////////////////////////

	// ������ ������ ������� ���������� (��� ����� � ������������ Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	size = LCD_X_RES / FONT_Width(FONT_8x8) - 5 + 1;
	Buf[size] = '\0';

	// ����� �������� ��������������� �������
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 12, FONT_6x8,
		PSTR("8x8: "), 0));
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 22, FONT_6x8,
		PSTR("8x8n:"), 0));

	chr1 = 0x20; // �������� � ������ ������� ������������� �������

	// ��������� �������� 80���
	MT_TimeoutMs(TIMEOUT_DEMO, 80000UL);

	while (TRUE)
	{
		// ��������� ������ ��� ������		
		chr2 = chr1;
		for (i = 0; i < size; i++)
		{
			Buf[i] = chr2;
			chr2 = IncChar(chr2);
		}
		chr1 = IncChar(chr1);
		
		// ������� ������� ������ 8x8
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 30, 12,
			FONT_8x8, Buf, 0));

		PT_YIELD(Context);
		
		// ������� ������� ������ 8x8n
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 30, 22,
			FONT_8x8n, Buf, 0));

		MT_SleepMs(Context, 500); // ���� 500��

		// ���� ������� 80��� ����������, �� ����� �� �����
		if (MT_TimeoutGet(TIMEOUT_DEMO) == 0) break;
	}


	// ������������ ������� 8x16, 10x14, 12x16 �
	// ������ 6x8 � ��������� ������� � �������.
	///////////////////////////////////////////////////////////////////////////

	// ������ ������ ������� ���������� (��� ����� � ������������ Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	// ����� �������� ����� ������
	MT_TimeoutSet(TIMEOUT_FONT, 0);

	size = LCD_X_RES / FONT_Width(FONT_8x16) - 5 + 1;
	Buf[size] = '\0';

	curFont = CNT_FONTS - 1;

	chr1 = 0x20; // �������� � ������ ������� ������� ������
	
	// ��������� �������� 100���
	MT_TimeoutMs(TIMEOUT_DEMO, 100000UL);

	while (TRUE)
	{	
		// ��������� ������ ��� ������
		chr2 = chr1;
		for (i = 0; i < size; i++)
		{
			Buf[i] = chr2;
			chr2 = IncChar(chr2);
		}
		chr1 = IncChar(chr1);

		if (MT_TimeoutGet(TIMEOUT_FONT) == 0)
		{
			// ��������� �������� ��� ��������� ����� ������
			MT_TimeoutMs(TIMEOUT_FONT, 5000);

			curFont++;
			if (curFont >= CNT_FONTS) curFont = 0;

			// ������� �������� ���������������� ������
			switch (curFont)
			{
			case 0:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("8x16:  "), 0));
				break;
			case 1:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("8x16n: "), 0));
				break;
			case 2:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("8x16t: "), 0));
				break;
			case 3:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("8x16g: "), 0));
				break;
			case 4:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("10x14: "), 0));
				break;
			case 5:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("12x16: "), 0));
				break;
			// ����� 6x8 � ��������� �� ������ � ������
			default:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("6x8WH: "), 0));
				break;
			}
			
			// ����� ���������� ����� ������ � �������
			PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
		}

		switch (curFont)
		{
		case 0: idFont = FONT_8x16; break;
		case 1: idFont = FONT_8x16n; break;
		case 2: idFont = FONT_8x16t; break;
		case 3: idFont = FONT_8x16g; break;
		case 4: idFont = FONT_10x14; break;
		case 5: idFont = FONT_12x16; break;
		default: idFont = FONT_6x8; break;
		}

		if (curFont >= 6)
			// ����� 6x8 � ��������� �� ������ � ������
			PT_WAIT_UNTIL(Context, LCD_StrBuf(42, 12, idFont, Buf,
				LCD_TWICE_WIDTH | LCD_TWICE_HEIGHT));
		else
			PT_WAIT_UNTIL(Context, LCD_StrBuf(42, 12, idFont, Buf, 0));
		
		// ����� ���������� ����� ������ � �������
		PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));

		MT_SleepMs(Context, 500); // ���� 500��
		
		// ���� ������ �����, �� ����� �� �����
		if (MT_TimeoutGet(TIMEOUT_DEMO) == 0) break;
	}


	// ������������ ������ �����
	///////////////////////////////////////////////////////////////////////////

	// ��������� ���������� �������������� ������
	strcpy_P(StrInfo, PSTR("    ����� �����"));

	// ������ ������ ������� ���������� (��� ����� � ������������ Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	MT_SleepMs(Context, 500); // ���� 500��

	for (cnt = 0; cnt < 3; cnt++)
	{
		// ������������� ����� ������ �������
		switch (cnt)
		{
		case 0: LCD_DrawMode(LCD_OR); break;
		case 1: LCD_DrawMode(LCD_AND); break;
		case 2: LCD_DrawMode(LCD_XOR); break;
		}

		// ��������� �������� 5000��
		MT_TimeoutMs(TIMEOUT_DEMO, 5000);

		i = 0;

		// �������� � �����, ���� �� ���������� �������
		while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
		{
			// ��������� ��������� ���������� x1 � ��������� 0...127
			x1 = random() % LCD_X_RES;

			// ��������� ��������� ���������� y1 � ��������� 9...31
			y1 = (random() % (LCD_Y_RES - 9)) + 9;

			// ������ ����� �� ���������� �����������
			PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, x1, y1));

			i++;
			
			// ����� ������ 8-�� �������� ����� ������ ���������� � �����������
			if ((i % 8) == 0) PT_YIELD(Context);
		}		
	}


	// ������������ ������ �����
	///////////////////////////////////////////////////////////////////////////

	// ��������� ���������� �������������� ������
	strcpy_P(StrInfo, PSTR("    ����� �����"));

	for (cnt = 0; cnt < 4; cnt++)
	{
		switch (cnt)
		{
		case 0:
			// ������ ������ ������� ����������
			// (��� ����� � ������������ Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_OR); // ��������� ������ ������ �������
			break;
		case 1:
			// ����������� ������ ������� ����������
			// (����� � ������������ Y >= 9)
			PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild, 0, 9,
				LCD_X_RES - 1, LCD_Y_RES - 1, TRUE));
			LCD_DrawMode(LCD_AND); // ��������� ������ ������ �������
			break;
		default:
			// ������ ������ ������� ����������
			// (��� ����� � ������������ Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_XOR); // ��������� ������ ������ �������
			break;
		}

		if (cnt < 3)
		{
			// ����� ����� �� ��������� �����������

			// ��������� ��������� ���������� x2 � ��������� 0...127
			x2 = random() % LCD_X_RES;

			// ��������� ��������� ���������� y2 � ��������� 9...31
			y2 = (random() % (LCD_Y_RES - 9)) + 9;

			// ��������� �������� 4000��
			MT_TimeoutMs(TIMEOUT_DEMO, 4000);

			// �������� � �����, ���� �� ���������� �������
			while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
			{
				x1 = x2;
				y1 = y2;
				
				// ��������� ��������� ���������� x2 � ��������� 0...127
				x2 = random() % LCD_X_RES;

				// ��������� ��������� ���������� y2 � ��������� 9...31
				y2 = (random() % (LCD_Y_RES - 9)) + 9;

				// ������ ����� �� ���������� �����������
				PT_SPAWN(Context, &ContextChild, LCD_Line(&ContextChild,
					x1, y1, x2, y2));

				MT_SleepMs(Context, 100); // ���� 100��

				if (cnt >= 2)
					// ������� ����� (� ������ LCD_XOR)
					PT_SPAWN(Context, &ContextChild, LCD_Line(&ContextChild,
						x1, y1, x2, y2));
			}
		}
	}


	// ������������ ����������� ���������������
	///////////////////////////////////////////////////////////////////////////

	// ��������� ���������� �������������� ������
	strcpy_P(StrInfo, PSTR("    ����������� ��������������"));

	LCD_DrawMode(LCD_XOR); // ��������� ������ ������ �������

	// ������ ������ ������� ���������� (��� ����� � ������������ Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	// ��������� �������� 5000��
	MT_TimeoutMs(TIMEOUT_DEMO, 5000);

	// �������� � �����, ���� �� ���������� �������
	while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
	{
		// ��������� ��������� ���������� x1 � ��������� 0...127
		x1 = random() % LCD_X_RES;

		// ��������� ��������� ���������� y1 � ��������� 9...31
		y1 = (random() % (LCD_Y_RES - 9)) + 9;

		// ��������� ��������� ���������� x2 � ��������� 0...127
		x2 = random() % LCD_X_RES;

		// ��������� ��������� ���������� y2 � ��������� 9...31
		y2 = (random() % (LCD_Y_RES - 9)) + 9;

		// ������ �������������
		PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild,
			x1, y1, x2, y2, TRUE));

		MT_SleepMs(Context, 200); // ���� 200��
	}

	// ������ ������ ������� ���������� (��� ����� � ������������ Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	// ��������� ������ ������ �������
	LCD_DrawMode(LCD_OR);

	for (i = 0; i < 12; i++)
	{
		// ������ �������������
		PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild,
			64 - i * 5, 20 - i, 64 + i * 5, 20 + i, TRUE));

		MT_SleepMs(Context, 200); // ���� 200��
	}

	MT_SleepMs(Context, 1000); // ���� 1000��


	// ������������ �����������
	///////////////////////////////////////////////////////////////////////////

	// ��������� ���������� �������������� ������
	strcpy_P(StrInfo, PSTR("    ����������"));
	
	// ������ ������ ������� ���������� (��� ����� � ������������ Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	LCD_DrawMode(LCD_OR); // ��������� ������ ������ �������

	for (i = 0; i < 5; i++)
	{
		// ������ ����������
		PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild,
			2 + i * 22, 20, 2 + i * 2));

		MT_SleepMs(Context, 500); // ���� 500��
	}

	MT_SleepMs(Context, 1000); // ���� 1000��


	for (cnt = 0; cnt < 2; cnt++)
	{
		switch (cnt)
		{
		case 0: 
			// ������ ������ ������� ����������
			// (��� ����� � ������������ Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_OR); // ��������� ������ ������ �������
			break;
		default:
			// ������ ������ ������� ����������
			// (��� ����� � ������������ Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_XOR); // ��������� ������ ������ �������
		}

		// ��������� �������� 5000��
		MT_TimeoutMs(TIMEOUT_DEMO, 5000);

		// �������� � �����, ���� �� ���������� �������
		while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
		{
			// ��������� ��������� ���������� x1 � ��������� 0...127
			x1 = random() % LCD_X_RES;

			// ��������� ��������� ���������� y1 � ��������� 10...30
			y1 = (random() % (LCD_Y_RES - 11)) + 10;

			// ��������� ��������� ������ � ��������� 0...14
			size = random() % 15;

			if (size > (y1 - 9)) size = size % (y1 - 9);
			if (size <= 1) size++;

			// ������ ����������
			PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild, x1, y1,
				size));

			MT_SleepMs(Context, 100); // ���� 100��

			if (cnt > 0)
				// ������� ���������� (� ������ LCD_XOR)
				PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild,
					x1, y1, size));
		}
	}

	goto BeginDemo;

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ Task_Blink.
// ���������� �����������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Blink(struct pt *Context))
{
	static uint8_t i;

	PT_BEGIN(Context); // ������ �����������

	// ��������� ������, � �������� ��������� ��������� ��� ������ � ��������
	// ������
	DRIVER(HL, OUT);

	while (TRUE)
	{
		for (i = 0; i < 2; i++)
		{
			ON(HL); // ��������� ����������
			MT_SleepMs(Context, 50); // ���� 50��
			OFF(HL); // ���������� ����������
			MT_SleepMs(Context, 150); // ���� 150��			
		}

		MT_SleepMs(Context, 600); // ���� 600vc
	}

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// main()
///////////////////////////////////////////////////////////////////////////////
int main(void)
{
	// ��������� �������� �������

	#if (DIVIDER_OSC == 1)
	clock_prescale_set(clock_div_1); // ������������� �������� 1/1
	#elif (DIVIDER_OSC == 2)
	clock_prescale_set(clock_div_2); // ������������� �������� 1/2
	#elif (DIVIDER_OSC == 4)
	clock_prescale_set(clock_div_4); // ������������� �������� 1/4
	#elif (DIVIDER_OSC == 8)
	clock_prescale_set(clock_div_8); // ������������� �������� 1/8
	#elif (DIVIDER_OSC == 16)
	clock_prescale_set(clock_div_16); // ������������� �������� 1/16
	#elif (DIVIDER_OSC == 32)
	clock_prescale_set(clock_div_32); // ������������� �������� 1/32
	#elif (DIVIDER_OSC == 64)
	clock_prescale_set(clock_div_64); // ������������� �������� 1/64
	#elif (DIVIDER_OSC == 128)
	clock_prescale_set(clock_div_128); // ������������� �������� 1/128
	#elif (DIVIDER_OSC == 256)
	clock_prescale_set(clock_div_256); // ������������� �������� 1/256
	#else
	#error Unknown divider for MAIN_OSC
	#endif


	sei();

	// ��������� ��������� �����
	MT_Init();

	// ������ �����
	MT_TaskInit(Task_Info, TASK_ACTIVE);
	MT_TaskInit(Task_Fonts, TASK_ACTIVE);
	MT_TaskInit(Task_Blink, TASK_ACTIVE);
	

    while (TRUE)
    {
	    MT_DISPATCH(); // ���� ������ ��������� �����
    }
}
///////////////////////////////////////////////////////////////////////////////
