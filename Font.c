#include "Config.h"
#include "Font.h"




// ������
///////////////////////////////////////////////////////////////////////////////

// ������ ������������ ����� ������� ��� ������������ ����������� 160 ��������
// � ������ 0x20...0x7F � 0xC0...0xFF.

#if defined(FONT_5x12)
// ����� 5x12.
static const uint8_t Font5x12[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "DOSAPP_05_5x12(r).c"
	#else
	#include "DOSAPP_05_5x12.c"
	#endif
};
#endif

#if defined(FONT_6x8)
// ����� 6x8.
static const uint8_t Font6x8[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "Font6x8(r).c"
	#else
	#include "Font6x8.c"
	#endif
};
#endif

#if defined(FONT_6x12)
// ����� 6x12.
static const uint8_t Font6x12[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "ter-u12n_ibm866_01_6x12(r).c"
	#else
	#include "ter-u12n_ibm866_01_6x12.c"
	#endif
};
#endif

#if defined(FONT_8x8)
// ����� 8x8.
static const uint8_t Font8x8[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "VIKTORIJa-EGA866-8x8(r).c"
	#else
	#include "VIKTORIJa-EGA866-8x8.c"
	#endif
};
#endif

#if defined(FONT_8x8t)
// ������ 8x8 (Theclaw).
static const uint8_t Font8x8t[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "THECLAW8(r).c"
	#else
	#include "THECLAW8.c"
	#endif
};
#endif

#if defined(FONT_8x8n)
// ����� 8x8 (n)
static const uint8_t Font8x8n[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "LVSCYR-VerySoft-8x8(r).c"
	#else
	#include "LVSCYR-VerySoft-8x8.c"
	#endif
};
#endif

#if defined(FONT_8x12)
// ����� 8x12.
static const uint8_t Font8x12[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "8x12_terminal(r).c"
	#else
	#include "8x12_terminal.c"
	#endif
};
#endif

#if defined(FONT_8x13n)
// ����� 8x13.
static const uint8_t Font8x13n[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "8x13_courer(r).c"
	#else
	#include "8x13_courer.c"
	#endif
};
#endif

#if defined(FONT_8x14)
// ����� 8x14.
static const uint8_t Font8x14[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "VIKTORIJa-EGA866-8x14(r).c"
	#else
	#include "VIKTORIJa-EGA866-8x14.c"
	#endif
};
#endif

#if defined(FONT_8x14t)
// ����� 8x14 (Theclaw).
static const uint8_t Font8x14t[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "THECLW14(r).c"
	#else
	#include "THECLW14.c"
	#endif
};
#endif

#if defined(FONT_8x14n)
// ����� 8x14 (n).
static const uint8_t Font8x14n[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "VGA-Golub-8x14(r).c"
	#else
	#include "VGA-Golub-8x14.c"
	#endif
};
#endif

#if defined(FONT_8x16)
// ����� 8x16.
static const uint8_t Font8x16[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "8x16_MODERN(r).c"
	#else
	#include "8x16_MODERN.c"
	#endif
};
#endif

#if defined(FONT_8x16n)
// ����� 8x16 (n).
static const uint8_t Font8x16n[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "tdrvses-2B-8x16_t(r).c"
	#else
	#include "tdrvses-2B-8x16_t.c"
	#endif
};
#endif

#if defined(FONT_8x16g)
// ����� 8x16 (g).
static const uint8_t Font8x16g[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "VGA-Golub-8x16(r).c"
	#else
	#include "VGA-Golub-8x16.c"
	#endif
};
#endif

#if defined(FONT_8x16t)
// ����� 8x16 (Theclaw).
static const uint8_t Font8x16t[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "8x16_THECLAW(r).c"
	#else
	#include "8x16_THECLAW.c"
	#endif
};
#endif

#if defined(FONT_10x14)
// ����� 10x14.
static const uint8_t Font10x14[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "10x14_ROMIK(r).c"
	#else
	#include "10x14_ROMIK.c"
	#endif
};
#endif

#if defined(FONT_12x16)
// ����� 12x16.
static const uint8_t Font12x16[] PROGMEM=
{
	#if defined(LCD_ROTATE)
	#include "DOS12X16(r).c"
	#else
	#include "DOS12X16.c"
	#endif
};
#endif




///////////////////////////////////////////////////////////////////////////////
// ������� ���������� ����� ������� ��� ������, ������������� ��������
// ������ ���������� idFont.
///////////////////////////////////////////////////////////////////////////////
uint8_t const *FONT_Addr(uint8_t idFont)
{
	#if defined(FONT_5x12)
	if (idFont == FONT_5x12) return Font5x12;
	#endif

	#if defined(FONT_6x8)
	if (idFont == FONT_6x8) return Font6x8;
	#endif
	
	#if defined(FONT_6x12)
	if (idFont == FONT_6x12) return Font6x12;
	#endif

	#if defined(FONT_8x8)
	if (idFont == FONT_8x8) return Font8x8;
	#endif

	#if defined(FONT_8x8t)
	if (idFont == FONT_8x8t) return Font8x8t;
	#endif

	#if defined(FONT_8x8n)
	if (idFont == FONT_8x8n) return Font8x8n;
	#endif
	
	#if defined(FONT_8x12)
	if (idFont == FONT_8x12) return Font8x12;
	#endif

	#if defined(FONT_8x13)
	if (idFont == FONT_8x13) return Font8x13;
	#endif

	#if defined(FONT_8x14)
	if (idFont == FONT_8x14) return Font8x14;
	#endif

	#if defined(FONT_8x14t)
	if (idFont == FONT_8x14t) return Font8x14t;
	#endif

	#if defined(FONT_8x14n)
	if (idFont == FONT_8x14n) return Font8x14n;
	#endif

	#if defined(FONT_8x16)
	if (idFont == FONT_8x16) return Font8x16;
	#endif

	#if defined(FONT_8x16n)
	if (idFont == FONT_8x16n) return Font8x16n;
	#endif

	#if defined(FONT_8x16g)
	if (idFont == FONT_8x16g) return Font8x16g;
	#endif

	#if defined(FONT_8x16t)
	if (idFont == FONT_8x16t) return Font8x16t;
	#endif

	#if defined(FONT_10x14)
	if (idFont == FONT_10x14) return Font10x14;
	#endif

	#if defined(FONT_12x16)
	if (idFont == FONT_12x16) return Font12x16;
	#endif

	return NULL;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ���������� ������ �������� ��� ������, ������������� ��������
// ������ ���������� idFont.
///////////////////////////////////////////////////////////////////////////////
uint8_t FONT_Width(uint8_t idFont)
{
	return pgm_read_byte(FONT_Addr(idFont) + 0);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ���������� ������ �������� ��� ������, ������������� ��������
// ������ ���������� idFont.
///////////////////////////////////////////////////////////////////////////////
uint8_t FONT_Height(uint8_t idFont)
{
	return pgm_read_byte(FONT_Addr(idFont) + 1);
}
///////////////////////////////////////////////////////////////////////////////
