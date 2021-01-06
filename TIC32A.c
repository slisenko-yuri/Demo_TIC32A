#include "Config.h"
#include "Mt.h"
#include "I2C.h"
#include "Font.h"
#include "TIC32A.h"

///////////////////////////////////////////////////////////////////////////////
// ���������� ��� ��������� ���������� TIC32A (� ����� TIC270A) �� ����
// ����������� PCF8531.
// ��������� ������������ � Arduino PRO MINI 3.3V � ������� �����������.
///////////////////////////////////////////////////////////////////////////////



/*
���������� TIC32A - ������ IDC16
================================
TIC32A	IDC16			������
--------------------------------
1(VLCD)	13		C100n	GND
2(RES/)	14		R10k	VCC
3(VDD2)	1		-VD+		VCC
4(VDD1)	2				VCC
5(SDA)	16				SDA
6(GND)	15				GND
7(SCL)	4				SCL

*����� ��������� 1 ���������� (VLCD) � GND ���������� ����������� 100n
*����� ��������� 2 ���������� (RES/) � VCC ���������� �������� 10k
*�� ������� 3 ���������� (VDD2) ���������� VCC=5V �������� ����� ���
 ��������������� ����������� ����� (��� 4148) ��� ��������� ���������� ��
 ����� 4.5�.
 ���� �� VCC=3.3V, �� ������ ������ ��������������� ���������.
*������� SCL � SDA ������ ���� ��������� � VCC � ������� ���������� 4.7k
*����� VCC � GND ���������� ����������� 100n
*/




#define TIC32A_ADDRESS	0x3C // ����� ���������� TIC32A �� ���� I2C

// ������ ��� � �������� �������
///////////////////////////////////////////////////////////////////////////////

#define NUM_Co		7 // ���� ����������, �� ��������� ���� ������ 1 ����.
						// ���� �������, �� ������ ���� ��������� ������

#define NUM_RS		6 // ���� ����������, �� �������� ���� - ��� ����(�)
						// ������.  ���� �������, �� ��������� ���� - ���
						// �������

// H[1:0]=00 (function and RAM command page)

#define NUM_H1		1 // ����� ���� H1 � ������� instruction set
#define NUM_H0		0 // ����� ���� H0 � ������� instruction set

#define NUM_PD		2 // ����� ���� PD � ������� function set
#define NUM_V		1 // ����� ���� V � ������� function set

// H[1:0]=01 (display setting command page)

#define NUM_M1		1 // ����� ���� M1 � ������� multiplex rate
#define NUM_M0		0 // ����� ���� M0 � ������� multiplex rate

#define NUM_D		2 // ����� ���� D � ������� display control
#define NUM_IM		1 // ����� ���� IM � ������� display control
#define NUM_E		0 // ����� ���� E � ������� display control

#define	NUM_BS2		2 // ����� ���� BS2 � ������� bias system
#define	NUM_BS1		1 // ����� ���� BS1 � ������� bias system
#define	NUM_BS0		0 // ����� ���� BS0 � ������� bias system

// H[1:0]=10 (HV-gen command page)

#define NUM_PRS		1 // ����� ���� PRS � ������� HV-gen control
#define NUM_HVE		0 // ����� ���� HVE � ������� HV-gen control

#define NUM_S1		1 // ����� ���� S1 � ������� HV-gen configuration
#define NUM_S0		0 // ����� ���� S0 � ������� HV-gen configuration

#define NUM_TC2		2 // ����� ���� TC2 � ������� temperature control
#define NUM_TC1		1 // ����� ���� TC1 � ������� temperature control
#define NUM_TC0		0 // ����� ���� TC0 � ������� temperature control



// ������� ���������� TIC32A
///////////////////////////////////////////////////////////////////////////////

// H[1:0]=XX
///////////////////////////////////////////////////////////////////////////////
#define CMD_DEF_H		0x01	// set default H[1:0] (select H[1:0]=0 ->
								// function and RAM command page)

// H[1:0]=00 (function and RAM command page)
///////////////////////////////////////////////////////////////////////////////
#define	CMD_INST_SET	0x08 // instruction set (select command page ->
							// H1(DB1),H0(DB0))
#define CMD_FUN_SET		0x20 // function set (power-down control,
							// entry mode -> PD(DB2),V(DB1))
#define CMD_ADR_Y		0x40 // set Y address of RAM (Y=0..5)
							 //(��� ���������� TIC32A ������������ Y=0..3)
#define CMD_ADR_X		0x80 // set X address of RAM (X=0..127)

// H[1:0]=01 (display setting command page)
///////////////////////////////////////////////////////////////////////////////
#define	CMD_MUL_RATE	0x04 // multiplex rate (set multiplex rate ->
							// M1(DB1), M0(DB0))
#define CMD_DISP_CONT	0x08 // display control (set display
							// configuration -> D(DB2),IM(DB1),E{DB0))
#define CMD_BIAS_SYS	0x10 // bias system (set bias system ->
							// BS2(DB2),BS1(DB1),BS0(DB0))

// H[1:0]=10 (HV-gen command page)
///////////////////////////////////////////////////////////////////////////////
#define CMD_HV_CONT		0x04 // HV-gen control (set Vlcd programming
							// range -> PRS(DB1),HVE(DB0))
#define CMD_HV_CONF		0x08 // HV-gen configuration (set voltage
							// multiplication factor -> S1(DB1),S0(DB0))
#define CMD_TEMP_CONT	0x20 // temperature control (set temperature
							// coefficient -> TC2(DB2),TC1(DB1),TC0(DB0))
#define CMD_VLCD_CONT	0x80 // Vlcd control (set Vlcd register ->
							// (VOP6(DB6)...VOP0(DB0)))




#define STR_FLASH	TRUE
#define STR_RAM		FALSE




// ������ ���������� ����� ����� �������� ���������� x � y
#define SWAP(x, y)	{uint8_t temp; temp = x; x = y; y = temp;}




union Union32
{
	uint8_t byte[4];
	uint32_t value;
};




static uint8_t	Buf[LCD_Y_RES / 8][LCD_X_RES]; // ����� ��� �������

// ���������� ���������� ������� ������, ������� ��� �� �������� � ���������
static uint8_t xlChanged;
static uint8_t xhChanged;
static uint8_t ylChanged;
static uint8_t yhChanged;

// ���������� ������� ������, ������� � ������ ������ ���������� � ���������
static uint8_t xlTransfer;
static uint8_t xhTransfer;
static uint8_t ylTransfer;
static uint8_t yhTransfer;

static uint8_t DrawMode; // ����� ������ �������




// ������, ����������� ��� ������ �������� ��������� ������
static const uint8_t Bit4to8[] PROGMEM =
{
	0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
	0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF
};




///////////////////////////////////////////////////////////////////////////////
// �� ���� �������� ���������� ������������.
///////////////////////////////////////////////////////////////////////////////
static uint16_t Max(uint16_t a, uint16_t b)
{
	if (a > b) return a;
	return b;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// �� ���� �������� ���������� �����������.
///////////////////////////////////////////////////////////////////////////////
static uint16_t Min(uint16_t a, uint16_t b)
{
	if (a < b) return a;
	return b;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ ������� � ��������� TIC32A.
///////////////////////////////////////////////////////////////////////////////
static uint8_t SendCmd(uint8_t Cmd)
{
	uint8_t Buf[2];

	if (!I2C_WaitReady(0)) return 0;
	
	Buf[0] = (1 << NUM_Co) | (0 << NUM_RS); // ����������
	Buf[1] = Cmd; // �������
	
	I2C_Send(TIC32A_ADDRESS, Buf, 2);
	return 1;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ ������ � ��������� TIC32A.
///////////////////////////////////////////////////////////////////////////////
static uint8_t SendData(uint8_t *Data, uint8_t Size)
{
	uint8_t Instr;

	if (!I2C_WaitReady(0)) return 0;
	
	Instr = (0 << NUM_Co) | (1 << NUM_RS); // ����������
	
	I2C_SendSendPtr(TIC32A_ADDRESS, &Instr, 1, Data, Size);
	return 1;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������ � ������, ������������� � ���.
// ���������:
// X - ��������� ���������� �� ����������� (0...127)
// Y - ��������� ���������� �� ��������� (0...31)
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Str - ����� ������ �� FLASH, ���� � RAM
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// fFLASH - ����, ����������� ��������������� ������, ��������� ����������
//          Str. ���� ���� ����� TRUE, �� ��� ��������, ��� ������
//          ��������� �� FLASH, ����� � RAM.
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
///////////////////////////////////////////////////////////////////////////////
static uint8_t StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, const char *Str,
	 uint8_t Settings, uint8_t fFLASH)
{
	uint8_t	col, row, j, num, m, data;
	uint8_t xl, xh, yl, yh;
	uint8_t len;
	uint8_t chr;
	uint16_t tmp;
	uint8_t widthFont; // ������ ������� � ��������
	uint8_t heightFont; // ������ ������� � ��������
	uint8_t cntByteChangeCol; // ���������� ���������� ������ ������ ��� ������
								// ������� �������
	uint8_t cntByteSymCol; // ���������� ���� � ����� ������� �������
	union Union32 mask; // ����� ��� �������� �������
	union Union32 column; // ������� �������
	uint8_t const *addrFont; // ����� ������
	uint8_t twiceW = 0; // 1 = �������� ������ �������
	uint8_t twiceH = 0; // 1 = �������� ������ �������
	uint8_t inv = 0; // 1 = ��������� ����������� �������

	widthFont = FONT_Width(idFont); // ������ �������� ������
	heightFont = FONT_Height(idFont); // ������ �������� ������

	if (Settings & LCD_TWICE_WIDTH)
		twiceW = 1; // ��������� ������

	if (Settings & LCD_TWICE_HEIGHT)
		twiceH = 1; // ��������� ������

	if (Settings & LCD_INVERSION)
		inv = 1; // ��������� ����������� ��������

	if (fFLASH)
		tmp = strlen_P(Str); //��������� ����� ������ �� FLASH
	else
		tmp = strlen(Str); // ��������� ����� ������ � RAM

	if (tmp < 255) len = tmp; else len = 255;

	// ���� � ������ ������ ���� �������� ������ � ���������,
	// �� ����� ���������, �� ���������� �� ������������ �������
	// ����� ���������� �������

	// ��������� ����� ���������� ����� ���������� ������� ������ � ������
	// ����������� ��� ������
	xl = Min(X, xlChanged);
	tmp = Min(LCD_X_RES, X + len * (widthFont << twiceW));
	xh = Max(tmp, xhChanged);

	yl = Min(Y, ylChanged);
	tmp = Min(LCD_Y_RES, Y + (heightFont << twiceH));
	yh = Max(tmp, yhChanged);

	// ���� � ������ ������ ���� �������� � ���������, �� ���������
	// ����������� ����� ���������� ������� ������ � ������������ ��������
	if (LCD_Busy())
	{
		// ���� ���������� ������� ������������ � ������������, �� �������
		if (!(xh <= xlTransfer) || (xhTransfer <= xl) || 
			(yh <= ylTransfer) || (yhTransfer <= yl)) return FALSE;
	}

	// ��������� ����� ���������� ����� ���������� ������� ������
	xlChanged = xl;
	xhChanged = xh;
	ylChanged = yl;
	yhChanged = yh;

	// �������� ����� ������ (������� ������, ��������������� ����� �����������
	// ������������� ��������)
	addrFont = FONT_Addr(idFont) + 2; 
	
	// ��������� �����, ������� ����� ������������ ��� �������� �������.
	// � ���� ����� ���������� ������ ����� ������ ���������� ������.
	mask.value = 1;
	mask.value <<= (heightFont << twiceH);
	mask.value--; // �������� ����� � ���� ���� ������ ��� �������� �����
					// ������� �������
	
	#if defined(LCD_ROTATE)
	// �������� �����, ��������� � ������� ��������.
	mask.value <<= (sizeof(mask) * 8 - (heightFont << twiceH)) - Y % 8;
	#else
	mask.value <<= Y % 8; // �������� ����� � ������ ���������� Y.
	#endif
	
	// ��������� ���������� ����, ��������� ��� ������ ������� �������
	// (��� ����� �������� �� ������ � ������).
	cntByteSymCol = (heightFont - 1) / 8 + 1;

	// �.�. ������ ����� ������������� ������� � ����� ����������, � �� ������
	// � ������� ������� ����� (��������, ���� ������� ������� � ������� 8
	// ����� ����� ������������� � ���� ������), �� ���������� ���������
	// ���������� ���������� ������ � ������ ��� ������ ������� �������.

	// ������������ ���������� ���� ��� ������ ������� �������, ������� �����
	// ����������� ���������.
		
	cntByteChangeCol = 0;

	#if defined(LCD_ROTATE)
	for (num = sizeof(mask); num > 0; num--)
	{
		if (mask.byte[num - 1] != 0) cntByteChangeCol++;
		else break;
	}
	#else
	for (num = 0; num < sizeof(mask); num++)
	{
		if (mask.byte[num] != 0) cntByteChangeCol++;
		else break;
	}
	#endif
	
	mask.value = ~mask.value; // ����������� �����.

	// ���� �� ������� ������� ������
	///////////////////////////////////////////////////////////////////////////
	for (j = 0; j < len; j++)
	{
		if (X >= LCD_X_RES) break;
		
		if (fFLASH)
			// ���� ������ ������ ��������� �� FLASH
			chr = pgm_read_byte(&Str[j]);
		else
			// ���� ������ ������ ��������� � ���
			chr = Str[j];

		// ������������ ��� ������� � ������������ � ����������
		///////////////////////////////////////////////////////////////////////

		if ((chr >= 0x20) && (chr <= 0x7F))
		{
			// �������� � ������� ��� �������� ASCII[0x20-0x7F]
			chr -= 0x20; //chr -= 32;
		}
		else if (chr >= 0xC0)
		{
			// �������� � ������� ��� �������� CP1251[0xC0-0xFF]
			chr -= 0x60; //chr -= 96;
		}
		else
		{
			// ��������� ���������� (�� ��� � ������� ��� �������� ������)
			chr = 0; // ������
		}

		// ���� �� ���� �������� �������
		///////////////////////////////////////////////////////////////////////
		for (col = 0; col < widthFont; col++)
		{
			if (X >= LCD_X_RES) break;

			column.value = 0;

			// ������ ����� ��� ������ ���������� ������� �������
			///////////////////////////////////////////////////////////////////
			for (num = 0; num < cntByteSymCol; num++)
			{
				if (twiceH) // ���� ��������� ������ �������
				{
					m = pgm_read_byte(&(addrFont[chr * widthFont * 
						cntByteSymCol + col * cntByteSymCol + num]));

					#if defined(LCD_ROTATE)
					column.byte[(sizeof(column) - 1) - (num * 2)] =
						pgm_read_byte(&(Bit4to8[m >> 4]));
					column.byte[(sizeof(column) - 1) - (num * 2 + 1)] =
						pgm_read_byte(&(Bit4to8[m & 0xF]));
					#else
					column.byte[num * 2] = pgm_read_byte(&(Bit4to8[m & 0xF]));
					column.byte[num * 2 + 1] = pgm_read_byte(&(Bit4to8[m >> 4]));
					#endif
				}
				else
				{
					#if defined(LCD_ROTATE)
					column.byte[(sizeof(column) - 1) - num] =
						pgm_read_byte(&(addrFont[chr * widthFont * cntByteSymCol +
							col * cntByteSymCol + num]));
					#else
					column.byte[num] =
						pgm_read_byte(&(addrFont[chr * widthFont * cntByteSymCol +
							col * cntByteSymCol + num]));
					#endif
				}
			}

			// �������� ������� � ������ ���������� Y.
			#if defined(LCD_ROTATE)
			column.value >>= Y % 8;
			#else
			column.value <<= Y % 8;
			#endif
			
			// ���� ��������� �����������, �� ����������� ������� �������
			if (inv)
			{
				column.value ^= ~mask.value;
			}
			
			// ����� ������� � �����
			for (m = 0; m <= twiceW; m++)
			{
				if (X < LCD_X_RES)
				{
					// ���� �� ���������� ������ ������ ��� ������ �������
					// �������
					for (num = 0; num < cntByteChangeCol; num++)
					{
						row = Y + num * 8;
						if (row < LCD_Y_RES)
						{
							#if defined(LCD_ROTATE)
							row = (LCD_Y_RES / 8 - 1) - row / 8;
							data = Buf[row][(LCD_X_RES - 1) - X];
							data &= mask.byte[(sizeof(mask) - 1) - num];
							data |= column.byte[(sizeof(column) - 1) - num];
							Buf[row][(LCD_X_RES - 1) - X] = data;
							#else
							row /= 8;
							data = Buf[row][X];
							data &= mask.byte[num];
							data |= column.byte[num];
							Buf[row][X] = data;
							#endif
						}
					}
					X++;
				}
				else
				{
					X = LCD_X_RES;
					break;
				}
			}
		}
	}
	if (X > LCD_X_RES) X = LCD_X_RES;
	
	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������������� ����������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Init(struct pt *Context))
{
	PT_BEGIN(Context); // ������ �����������
 
	MT_MutexWait(Context, MUTEX_LCD); // ����������� �������

	// ������������� function and RAM command page
	PT_WAIT_UNTIL(Context, SendCmd(CMD_DEF_H));

	// PD=0 - chip is active
	// V=0 - horizontal addressing
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUN_SET | (0<<NUM_PD) | (0<<NUM_V)));
	
	// Y=0
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | 0));
	
	// X=0
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | 0));	

	// ��������� � display setting command page
	PT_WAIT_UNTIL(Context, SendCmd(CMD_INST_SET | 1));
	
	// D=1,E=0 - normal mode
	// (D=0,E=0 - display blank, D=0,E=1 - all display segments, D=0,E=1 - inverse video mode)
	//  IM=0 - normal mode; full display + icons
	PT_WAIT_UNTIL(Context, SendCmd(CMD_DISP_CONT | (1<<NUM_D) | (0<<NUM_IM) | (0<<NUM_E)));
	
	// M[1:0]=01 - Multiplex rate = 1:34 (00->1:17, 10->1:26)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_MUL_RATE | 0x1)); // 1:34 (ok)

	// BS[2:0]=100 - Recommended for multiplex rate 1:34
	// BS[2:0]=101 - Recommended for multiplex rate 1:26
	// BS[2:0]=110 - Recommended for multiplex rate 1:17
	PT_WAIT_UNTIL(Context, SendCmd(CMD_BIAS_SYS | 0x4)); // for 1:34 (ok)

	// ������������� function and RAM command page
	PT_WAIT_UNTIL(Context, SendCmd(CMD_DEF_H));

	// ��������� � HV-gen command page
	PT_WAIT_UNTIL(Context, SendCmd(CMD_INST_SET | 2));

	// TC[2:0]=000 - temperature coefficient TC0
	PT_WAIT_UNTIL(Context, SendCmd(CMD_TEMP_CONT | 0)); // ok
	
	// S[1:0]=10 - 4 x voltage multiplier (00->2x, 01->3x, 11->5x)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_HV_CONF | 0x2)); //  4x (ok,3.3V)
	
	// PRS=0 - Vlcd programming range LOW (1->Vlcd programming range HIGH)
	// HVE=1 - voltage multiplier enabled
	PT_WAIT_UNTIL(Context, SendCmd(CMD_HV_CONT | (0<<NUM_PRS) | (1<<NUM_HVE))); // ok
	
	// VOP[6:0]=0x44
	PT_WAIT_UNTIL(Context, SendCmd(CMD_VLCD_CONT | 0x5C)); // ok
	
	// ������������� function and RAM command page
	PT_WAIT_UNTIL(Context, SendCmd(CMD_DEF_H));

	DrawMode = LCD_OR;

	// ������������� ��������� ��������� ������� ������
	xlChanged = LCD_X_RES;
	xhChanged = 0;
	ylChanged = LCD_Y_RES;
	yhChanged = 0;

	// ������������� ��������� ������� ������, ������������ � ���������
	xlTransfer = 0;
	xhTransfer = 0;
	ylTransfer = 0;
	yhTransfer = 0;

	MT_MutexFree(MUTEX_LCD); // ����������� �������
	
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ���������� TRUE, ���� ��������� �����, �.�. � ������ ������ ����������
// �������� �� ������ � ���������.
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_Busy(void)
{
	if (yhTransfer > ylTransfer)
		return TRUE;

	return FALSE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ���������� ���������� ������� ������ � ���������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Update(struct pt *Context))
{
	PT_BEGIN(Context); // ������ �����������
	
	MT_MutexWait(Context, MUTEX_LCD); // ������ ��������

	// ���� � ������ ���������� ����������, �� �� ����� �������� � ���������
	if ((xhChanged > xlChanged) && (yhChanged > ylChanged))
	{
		// ���������� ���������� ������������ �������
		xlTransfer = xlChanged;
		xhTransfer = xhChanged;
		ylTransfer = ylChanged;
		yhTransfer = yhChanged;
		
		// ����������� ���������� ���������� ������� ������
		xlChanged = LCD_X_RES;
		xhChanged = 0;
		ylChanged = LCD_Y_RES;
		yhChanged = 0;

		while (yhTransfer > ylTransfer)
		{
			#if defined(LCD_ROTATE)
			// ������������� ��������� ����� �� X
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X |
				(LCD_X_RES - xhTransfer)));

			// ������������� ��������� ����� �� Y
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y |
				((LCD_Y_RES - 1 - ylTransfer) / 8)));

			// �������� ������ � ���������
			PT_WAIT_UNTIL(Context, SendData(&Buf[(LCD_Y_RES - 1 - ylTransfer)
				/ 8][LCD_X_RES - xhTransfer], xhTransfer - xlTransfer));
			#else
			// ������������� ��������� ����� �� X
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | xlTransfer));

			// ������������� ��������� ����� �� Y
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | (ylTransfer / 8)));

			// �������� ������ � ���������
			PT_WAIT_UNTIL(Context, SendData(&Buf[ylTransfer / 8][xlTransfer],
				xhTransfer - xlTransfer));
			#endif

			// ������������ ���������� ������������ ������� ������ �
			// ������ ���������� ������ ������
			ylTransfer += (8 - (ylTransfer % 8));
		}
		
		// ����������� ���������� ������������ �������
		xlTransfer = 0;
		xhTransfer = 0;
		ylTransfer = 0;
		yhTransfer = 0;
	}

	MT_MutexFree(MUTEX_LCD); // ����������� �������

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ������� ������ � ���������� ������������.
// ���������:
// X1 - ���������� �������� ������ ���� ��������� ������� �� �����������
//      (0...127),
// Y1 - ���������� �������� ������ ���� ��������� ������� �� ���������
//      (0...31),
// X2 - ���������� ������� ������� ���� ��������� ������� �� �����������
//      (0...127),
// Y1 - ���������� ������� ������� ���� ��������� ������� �� ���������
//      (0...31).
//
// ������:
// // �������� ���� �����
// PT_WAIT_UNTIL(Context, LCD_ClearBuf(0, 0, 127, 31));
// // ����� ���������� ����� ������ � ���������
// PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_ClearBuf(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2)
{
	uint8_t sizeY;
	union Union32 mask;
	uint8_t cntByteChangeCol;
	uint8_t xl, xh, yl, yh;
	uint8_t x, num;

	// ����������� �� �������� �����������

	// ���� ����� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (X1 > X2) SWAP(X1, X2)
	
	// ���� ������� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (Y1 > Y2) SWAP(Y1, Y2)

	if ((X1 >= LCD_X_RES) || (Y1 >= LCD_Y_RES)) return TRUE;
	if (X2 >= LCD_X_RES) X2 = LCD_X_RES - 1;
	if (Y2 >= LCD_Y_RES) Y2 = LCD_Y_RES - 1;
	X2++;
	Y2++;

	// ��������� ����� ���������� ��� ���������� ������� ������ 
	xl = Min(X1, xlChanged);
	xh = Max(X2, xhChanged);
	yl = Min(Y1, ylChanged);
	yh = Max(Y2, yhChanged);

	// ���� � ������ ������ ���� �������� � ���������, �� ���������
	// ����������� ������������ ������� � ����������
	if (LCD_Busy())
	{
		// ���� ���������� ������� ������������ � ������������, �� �������
		if (!(xh <= xlTransfer) || (xhTransfer <= xl) || (yh <= ylTransfer) ||
			(yhTransfer <= yl))
			return FALSE;
	}

	// ������������� ����� ���������� ���������� ������� ������
	xlChanged = xl;
	xhChanged = xh;
	ylChanged = yl;
	yhChanged = yh;
	
	// ��������� ������ ��������� ������� �� ���������
	sizeY = Y2 - Y1;

	// ��������� �����, ������� ����� ������������ ��� ������� ��������.
	// � ���� ����� ���������� ������ ����� ������ ��������� �������
	mask.value = 1;
	mask.value <<= sizeY;
	mask.value--; // �������� ����� � ���� ���� ������ ��� �������� �����
					// �������� ��������� �������
	#if defined(LCD_ROTATE)
	// �������� �����, ��������� � ������� ��������
	mask.value <<= sizeof(mask) * 8 - sizeY - Y1 % 8;
	#else
	mask.value <<= Y1 % 8;
	#endif

	// ������������ ���������� ���� ��� ������ ������� ��������� �������,
	// ������� ����� ����������� ���������.

	cntByteChangeCol = 0;

	#if defined(LCD_ROTATE)
	for (num = sizeof(mask); num > 0; num--)
	{
		if (mask.byte[num - 1] != 0) cntByteChangeCol++;
		else break;
	}
	#else
	for (num = 0; num < sizeof(mask); num++)
	{
		if (mask.byte[num] != 0) cntByteChangeCol++;
		else break;
	}
	#endif

	mask.value = ~mask.value; // ����������� �����

	// ���� �� ������� ������� ��������� �������
	for (x = X1; x < X2; x++)
	{
		for (num = 0; num < cntByteChangeCol; num++)
		{
			#if defined(LCD_ROTATE)
			Buf[(LCD_Y_RES - 1 - (Y1 + num * 8)) / 8][LCD_X_RES - 1 - x] &=
				mask.byte[sizeof(mask) - 1 - num];
			#else
			Buf[(Y1 + num * 8) / 8][x] &= mask.byte[num];
			#endif
		}
	}

	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ������� ���������� � ���������� ������������.
// ���������:
// X1 - ���������� �������� ������ ���� ��������� ������� �� �����������
//      (0...127),
// Y1 - ���������� �������� ������ ���� ��������� ������� �� ���������
//      (0...31),
// X2 - ���������� ������� ������� ���� ��������� ������� �� �����������
//      (0...127),
// Y1 - ���������� ������� ������� ���� ��������� ������� �� ���������
//      (0...31).
//
// ������:
// PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 0, 127, 31));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Clear(struct pt *Context, uint8_t X1, uint8_t Y1, uint8_t X2,
	uint8_t Y2))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������

	PT_BEGIN(Context); // ������ �����������

	PT_WAIT_UNTIL(Context, LCD_ClearBuf(X1, Y1, X2, Y2));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));

	PT_END(Context); // ���������� �����������
}
//////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������ � ������, ������������� � ��� �� �����������,
// ��������� ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...127).
// Y - ��������� ���������� �� ��������� (0...31).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h.
// Str - ����� ������ � ���.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_WAIT_UNTIL(Context, LCD_StrBuf(42, 12, FONT_6x8, Msg,
//   LCD_TWICE_WIDTH | LCD_TWICE_HEIGHT));
///////////////////////////////////////////////////////////////////////////////
uint8_t	LCD_StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char *Str,
	uint8_t Settings)
{
	return StrBuf(X, Y, idFont, Str, Settings, STR_RAM);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ����������� ������ �� ��������� �� �����������, ���������
// ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...127).
// Y - ��������� ���������� �� ��������� (0...31).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Str - ����� ������ � ���.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_SPAWN(Context, &ContextChild,
//   LCD_Str(&ContextChild, 0, 0, FONT_6x8, Msg, 0));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Str(struct pt *Context, uint8_t X, uint8_t Y, uint8_t idFont,
	char *Str, uint8_t Settings))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������

	PT_BEGIN(Context); // ������ �����������
	PT_WAIT_UNTIL(Context, LCD_StrBuf(X, Y, idFont, Str, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������ � ������, ������������� � ��� �� �����������,
// ��������� ����������� X � Y. 
// ���������:
// X - ��������� ���������� �� ����������� (0...127).
// Y - ��������� ���������� �� ��������� (0...31).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h.
// Str_P - ����� ������ �� FLASH.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12, FONT_6x8, PSTR("MSG"), 0));
///////////////////////////////////////////////////////////////////////////////
uint8_t	LCD_StrBuf_P(uint8_t X, uint8_t Y, uint8_t idFont, const char *Str_P,
	uint8_t Settings)
{
	return StrBuf(X, Y, idFont, Str_P, Settings, STR_FLASH);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ����������� ������ �� ��������� �� �����������, ���������
// ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...127).
// Y - ��������� ���������� �� ��������� (0...31).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Str_P - ����� ������ �� FLASH.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������1:
// PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 3, 12, FONT_6x8,
//   PSTR("������"), 0));
//
// ������2:
// // ����� ������ c ��������� ������� � �������.
// const char Str_P[] PROGMEM = "������"; // ���������� ������
// PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 0, FONT_8x16,
//   Str_P, LCD_TWICE_WIDTH | LCD_TWICE_HEIGHT));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Str_P(struct pt *Context, uint8_t X, uint8_t Y, uint8_t idFont,
	const char *Str_P, uint8_t Settings))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������

	PT_BEGIN(Context); // ������ �����������
	PT_WAIT_UNTIL(Context, LCD_StrBuf_P(X, Y, idFont, Str_P, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������� � ������, ������������� � ��� �� �����������,
// ��������� ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...127).
// Y - ��������� ���������� �� ��������� (0...31).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h.
// Chr - ��� �������.
// Settings - ����� ����������� �������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������)
//        LCD_TWICE_HEIGHT (�������� ������ �������)
//        LCD_INVERSION (��������� ����������� �������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_WAIT_UNTIL(Context, LCD_ChrBuf(42, 12, FONT_6x8, 'A', 0));
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_ChrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char Chr,
	uint8_t Settings)
{
	char str[2];

	str[0] = Chr;
	str[1] = '\0';
	
	return StrBuf(X, Y, idFont, str, Settings, STR_RAM);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ����������� ������� �� ��������� �� �����������, ���������
// ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...127).
// Y - ��������� ���������� �� ��������� (0...31).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Chr - ��� �������.
// Settings - ����� ����������� �������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������)
//        LCD_TWICE_HEIGHT (�������� ������ �������)
//        LCD_INVERSION (��������� ����������� �������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_SPAWN(Context, &ContextChild,
//                             LCD_Chr(&ContextChild, 0, 0, FONT_6x8, 'A', 0));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Chr(struct pt *Context, uint8_t X, uint8_t Y, uint8_t idFont,
	char Chr, uint8_t Settings))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������

	PT_BEGIN(Context); // ������ �����������
	PT_WAIT_UNTIL(Context, LCD_ChrBuf(X, Y, idFont, Chr, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // ������ �����������
}
///////////////////////////////////////////////////////////////////////////////



// �������
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ������ ��������� �������.
// ��������:
// Mode - ����� ��������� ���� �� ��������� ��������:
//        LCD_OR,
//        LCD_AND,
//        LCD_XOR.
///////////////////////////////////////////////////////////////////////////////
void LCD_DrawMode(uint8_t Mode)
{
	DrawMode = Mode;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ���� ������ �� ���������.
// ���������:
// X - ���������� �� ����������� (0...127).
// Y - ���������� �� ��������� (0...31).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Pixel(struct pt *Context, uint8_t X, uint8_t Y))
{
	uint8_t mask;
	uint8_t data;
	static uint8_t row;

	PT_BEGIN(Context); // ������ �����������

	if ((X >= LCD_X_RES) || (Y >= LCD_Y_RES))
		PT_EXIT(Context);

	// ���� �������, ����� ����� ����� �������� ������ � ������
	while (LCD_Busy())
	{
		// ���� ���������� ������� �� ������������ � ������������, �� �������
		// �� �����.
		// (�.�. ���� ������������ ����� �� ����� � ������� ������, ������� �
		// ������ ������ ���������� � ���������)
		if (!(X < xlTransfer) || (xhTransfer <= X) ||
			(Y < ylTransfer) || (yhTransfer <= Y))
			PT_YIELD(Context);
		else break;
	}

	#if defined(LCD_ROTATE)
	mask = 0x80 >> (Y % 8);
	row = (LCD_Y_RES - 1 - Y) / 8;
	data = Buf[row][LCD_X_RES - 1 - X];
	#else
	mask = 1 << (Y % 8);
	row = Y / 8;
	data = Buf[row][X];
	#endif

	switch (DrawMode)
	{
	case LCD_OR: data |= mask; break;
	case LCD_AND: data &= ~mask; break;
	case LCD_XOR: data ^= mask; break;
	}
	
	#if defined(LCD_ROTATE)
	Buf[row][LCD_X_RES - 1 - X] = data;
	#else
	Buf[row][X] = data;
	#endif

	// ������ ����� �������� ���������� ���� � ���������
	
	MT_MutexWait(Context, MUTEX_LCD); // ����������� �������
		
	#if defined(LCD_ROTATE)
	// ������������� ��������� ����� �� X
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | (LCD_X_RES - 1 - X)));
	
	// ������������� ��������� ����� �� Y
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | row));

	// �������� ������ � ���������
	PT_WAIT_UNTIL(Context, SendData(&Buf[row][LCD_X_RES - 1 - X], 1));
	#else
	// ������������� ��������� ����� �� X
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | X));
			
	// ������������� ��������� ����� �� Y
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | row));

	// �������� ������ � ���������
	PT_WAIT_UNTIL(Context, SendData(&Buf[row][X], 1));
	#endif
	
	MT_MutexFree(MUTEX_LCD); // ����������� �������
	
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ ����� ����� ����� ������� �� ���������� (�������� ����������).
// ���������:
// X1 - ���������� ������ ����� �� ����������� (0...127),
// Y1 - ���������� ������ ����� �� ��������� (0...31),
// X2 - ���������� ��������� ����� �� ����������� (0...127),
// Y2 - ���������� ��������� ����� �� ��������� (0...31).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Line(struct pt *Context, uint8_t X1, uint8_t Y1, uint8_t X2,
	uint8_t Y2))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������
    static int16_t dX, dY;
	static int16_t stepX, stepY;
	static int16_t fraction;
	static uint8_t x, y;

	PT_BEGIN(Context); // ������ �����������

	x = X1;
	y = Y1;

	// dY   Y2 - Y1
	// -- = -------
	// dX   X2 - X1

	dY = Y2 - Y1;
	dX = X2 - X1;

	if (dY < 0)
	{
		// dY �������������
		dY = -dY;
		stepY = -1;
	}
	else
	{
		// dY �������������
		stepY = 1;
	}

	if (dX < 0)
	{
		// dX �������������
		dX = -dX;
		stepX = -1;
	}
	else
	{
		// dX �������������
		stepX = 1;
	}

	dX <<= 1;
	dY <<= 1;

	PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, X1, Y1));

	// ������ ��������� ����� �� �����
	if (dX > dY)
	{
		fraction = dY - (dX >> 1);
		while (x != X2)
		{
			if (fraction >= 0)
			{
				y += stepY;
				fraction -= dX;
			}
			x += stepX;
			fraction += dY;

			PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, x, y));
		}
	}
	else
	{
		fraction = dX - (dY >> 1);
		while (y != Y2)
		{
			if (fraction >= 0)
			{
				x += stepX;
				fraction -= dY;
			}
			y += stepY;
			fraction += dX;

			PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, x, y));
		}
	}

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ ���������� � �������, ���������� �������� ������� ����������� X � Y �
// �������� R.
// ���������:
// X - ���������� ������ ���������� �� ����������� (0...127),
// Y - ���������� ������ ���������� �� ��������� (0...31),
// R - ������ ���������� (0...127).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Circle(struct pt *Context, uint8_t X, uint8_t Y, uint8_t R))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������
	static int16_t d;
	static int8_t xc, yc;

	PT_BEGIN(Context); // ������ �����������

	yc = R;
	d = 3 - ((int16_t)R << 1);
	xc = 0;
	
	while (xc <= yc)
	{
		PT_SPAWN(Context, &ContextChild,
			LCD_Pixel(&ContextChild, xc + X, yc + Y));

		if (yc != 0)
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, xc + X, -yc + Y));

		if ((xc != 0) && (yc != 0))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -xc + X, -yc + Y));

		if (xc != 0)
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -xc + X, yc + Y));

		if (xc != yc)
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, yc + X, xc + Y));

		if ((xc != 0) && (xc != yc))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, yc + X, -xc + Y));

		if ((xc != 0) && (yc != 0) && (xc != yc))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -yc + X, -xc + Y));

		if ((yc != 0) && (xc != yc))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -yc + X, xc + Y));
		
		if (d < 0)
		{
			d = d + 4 * xc + 6;
		}
		else
		{
			d = d + 4 * (xc - yc) + 10;
			yc--;
		}
		xc++;
	};
	
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ��������������.
// ���������:
// X1 - ���������� �������� ������ ���� �������������� �� ����������� (0..127),
// Y1 - ���������� �������� ������ ���� �������������� �� ��������� (0..31),
// X2 - ���������� ������� ������� ���� �������������� �� ����������� (0..127),
// Y1 - ���������� ������� ������� ���� �������������� �� ��������� (0..31),
// Fill - ���� ���������������, ���� �����, ����� ���������� �������
//        �������������� ���� ���������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Rect(struct pt *Context, uint8_t X1, uint8_t Y1, uint8_t X2,
	uint8_t Y2, uint8_t Fill))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������
	static uint8_t sX1, sY1, sX2, sY2;
	static uint8_t xl, xh, yl, yh;
	uint8_t sizeY;
	uint8_t cntByteChangeCol;
	union Union32 mask;
	uint8_t x, num;
	uint8_t data;
	#if defined(LCD_ROTATE)
	uint8_t y;
	#endif

	PT_BEGIN(Context); // ������ �����������

	// ����������� �� �������� �����������

	// ���� ����� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (X1 > X2) SWAP(X1, X2)
	
	// ���� ������� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (Y1 > Y2) SWAP(Y1, Y2)

	if (Fill)
	{
		if ((X1 >= LCD_X_RES) || (Y1 >= LCD_Y_RES))
			PT_EXIT(Context);
		
		if (X2 >= LCD_X_RES) X2 = LCD_X_RES - 1;
		if (Y2 >= LCD_Y_RES) Y2 = LCD_Y_RES - 1;
		X2++;
		Y2++;

		// ��������� ����� ���������� ��� ���������� ������� ������
		xl = Min(X1, xlChanged);
		xh = Max(X2, xhChanged);
		yl = Min(Y1, ylChanged);
		yh = Max(Y2, yhChanged);
		
		sX1 = X1; sY1 = Y1; sX2 = X2; sY2 = Y2;

		// ���� � ������ ������ ���� �������� � ���������, �� ���������
		// ����������� � ������������ ��������, �, ���� ��� ���, �� ����,
		// ���� �� ���������� ��������
		while (LCD_Busy()) 
		{
			// ���� ���������� ������� �� ������������ � ������������, ��
			// ������� �� �����
			if (!(xh <= xlTransfer) || (xhTransfer <= xl) ||
				(yh <= ylTransfer) || (yhTransfer <= yl))
				PT_YIELD(Context);
			else break;
		}
		
		// ��������� ����� ���������� ���������� �������
		xlChanged = xl;
		xhChanged = xh;
		ylChanged = yl;
		yhChanged = yh;
	
		// ��������� ������ �������������� �� ���������
		sizeY = sY2 - sY1;

		// ��������� �����, ������� ����� ������������ ��� �����������
		// ��������������.
		// � ���� ����� ���������� ������ ����� ������ ��������������.
		mask.value = 1;
		mask.value <<= sizeY;
		mask.value--; // �������� ����� � ���� ���� ������ ��� ��������
						// ����� �������� ��������������

		#if defined(LCD_ROTATE)
		// �������� �����, ��������� � ������� ��������
		mask.value <<= sizeof(mask) * 8 - sizeY - sY1 % 8;
		#else
		mask.value <<= sY1 % 8;
		#endif

		// ������������ ���������� ���� ��� ������ ������� ��������������,
		// ������� ����� ����������� ���������.

		cntByteChangeCol = 0;

		#if defined(LCD_ROTATE)
		for (num = sizeof(mask); num > 0; num--)
		{
			if (mask.byte[num - 1] != 0) cntByteChangeCol++;
			else break;
		}
		#else
		for (num = 0; num < sizeof(mask); num++)
		{
			if (mask.byte[num] != 0) cntByteChangeCol++;
			else break;
		}
		#endif

		if (DrawMode == LCD_AND) mask.value = ~mask.value;

		// ���� �� ������� ������� ��������������
		for (x = sX1; x < sX2; x++)
		{
			for (num = 0; num < cntByteChangeCol; num++)
			{
				#if defined(LCD_ROTATE)
				y = (LCD_Y_RES - 1 - (sY1 + num * 8)) / 8;
				data = Buf[y][LCD_X_RES - 1 - x];
				if (DrawMode == LCD_OR)
					data |= mask.byte[sizeof(mask) - 1 - num];
				else if (DrawMode == LCD_XOR)
					data ^= mask.byte[sizeof(mask) - 1 - num];
				else if (DrawMode == LCD_AND)
					data &= mask.byte[sizeof(mask) - 1 - num];
				Buf[y][LCD_X_RES - 1 - x] = data;
				#else
				data = Buf[(sY1 + num * 8) / 8][x];
				if (DrawMode == LCD_OR) data |= mask.byte[num];
				else if (DrawMode == LCD_XOR) data ^= mask.byte[num];
				else if (DrawMode == LCD_AND) data &= mask.byte[num];
				Buf[(sY1 + num * 8) / 8][x] = data;
				#endif
			}
		}
		
		// ����� ���������� ������� ������ � ���������
		PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	}
	else
	{
		sX1 = X1; sY1 = Y1; sX2 = X2; sY2 = Y2;

		PT_SPAWN(Context, &ContextChild,
			LCD_Line(&ContextChild, sX1, sY1, sX2, sY1));

		if (sY2 > sY1)
			PT_SPAWN(Context, &ContextChild,
				LCD_Line(&ContextChild, sX1, sY2, sX2, sY2));

		if ((sY2 - sY1) > 1)
			PT_SPAWN(Context, &ContextChild,
				LCD_Line(&ContextChild, sX1, sY1 + 1, sX1, sY2 - 1));

		if ((sX2 > sX1) && ((sY2 - sY1) > 1))
			PT_SPAWN(Context, &ContextChild,
				LCD_Line(&ContextChild, sX2, sY1 + 1, sX2, sY2 - 1));
	}

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////
