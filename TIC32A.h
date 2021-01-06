#if !defined(_TIC32A_H_)
#define _TIC32A_H_




#define LCD_PWR_3V3		// ������������ ��� ������, ����� ��������������� �
						// ��������� ������� �� 3.3�

#define	LCD_X_RES			128	// ���������� �� �����������
#define	LCD_Y_RES			32	// ���������� �� ���������

// ������ ������ ������
#define LCD_TWICE_WIDTH		0x1 // �������� ������ ��������
#define LCD_TWICE_HEIGHT	0x2 // �������� ������ ��������
#define LCD_INVERSION		0x4 // ��������� ����������� ��������

// ������ ������ �������
#define LCD_OR   0 // ������� �������������� �������� ���������
#define LCD_AND  1 // ������� �������������� ��������� ��������
#define LCD_XOR  2 // ������� �������������� ��������� ��������



// ������������� ����������
extern PT_THREAD(LCD_Init(struct pt *Context));

extern uint8_t LCD_Busy(void);

// ������� ���������� ���������� ������� ������ � ���������
extern PT_THREAD(LCD_Update(struct pt *Context));

// ������� ������� ������ � ���������� ������������
extern uint8_t LCD_ClearBuf(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2);

// ������� ������� ���������� � ���������� ������������
extern PT_THREAD(LCD_Clear(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));
	
// ��������� ����������� ������ � ������, ������������� � ���
extern uint8_t LCD_StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char *Str,
	uint8_t Settings);

// ������� ����������� ������ �� ���������
extern PT_THREAD(LCD_Str(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char *Str, uint8_t Settings));

// ��������� ����������� ������ � ������, ������������� � ���.
// �������� ������ ��� ���� ������������� �� FLASH.
extern uint8_t LCD_StrBuf_P(uint8_t X, uint8_t Y, uint8_t idFont,
	const char *Str_P, uint8_t Settings);

// ������� ����������� ������ � ���������.
// �������� ������ ��� ���� ������������� �� FLASH.
extern PT_THREAD(LCD_Str_P(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, const char *Str_P, uint8_t Settings));

// ��������� ����������� ������� � ������, ������������� � ���
extern uint8_t LCD_ChrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char Chr,
	uint8_t Settings);

// ������� ����������� ������� �� ���������
extern PT_THREAD(LCD_Chr(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char Chr, uint8_t Settings));


// �������
///////////////////////////////////////////////////////////////////////////////

// ��������� ������ ��������� �������
extern void LCD_DrawMode(uint8_t Mode);

// ������� ���� ������ �� ���������
extern PT_THREAD(LCD_Pixel(struct pt *Context, uint8_t X, uint8_t Y));

// ������ ����� ����� ����� ������� �� ����������
extern PT_THREAD(LCD_Line(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));

// ������ ����������
extern PT_THREAD(LCD_Circle(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t R));

// ��������� ��������������
extern PT_THREAD(LCD_Rect(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2, uint8_t Fill));

#endif
