#if !defined(_TIC32A_H_)
#define _TIC32A_H_




#define LCD_PWR_3V3		// Определяется для случая, когда микроконтроллер и
						// индикатор запитан от 3.3В

#define	LCD_X_RES			128	// Разрешение по горизонтали
#define	LCD_Y_RES			32	// Разрешение по вертикали

// Режимы вывода текста
#define LCD_TWICE_WIDTH		0x1 // Удвоение ширины символов
#define LCD_TWICE_HEIGHT	0x2 // Удвоение высоты символов
#define LCD_INVERSION		0x4 // Инверсное отображение символов

// Режимы вывода графики
#define LCD_OR   0 // Графика отрисовывается обычными пикселами
#define LCD_AND  1 // Графика отрисовывается стиранием пикселов
#define LCD_XOR  2 // Графика отрисовывается инверсией пикселов



// Инициализация индикатора
extern PT_THREAD(LCD_Init(struct pt *Context));

extern uint8_t LCD_Busy(void);

// Выводит содержимое измененной области буфера в индикатор
extern PT_THREAD(LCD_Update(struct pt *Context));

// Очищает область буфера с указанными координатами
extern uint8_t LCD_ClearBuf(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2);

// Очищает область индикатора с указанными координатами
extern PT_THREAD(LCD_Clear(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));
	
// Формирует изображение строки в буфере, расположенном в ОЗУ
extern uint8_t LCD_StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char *Str,
	uint8_t Settings);

// Выводит изображение строки на индикатор
extern PT_THREAD(LCD_Str(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char *Str, uint8_t Settings));

// Формирует изображение строки в буфере, расположенном в ОЗУ.
// Исходная строка при этом располагается во FLASH.
extern uint8_t LCD_StrBuf_P(uint8_t X, uint8_t Y, uint8_t idFont,
	const char *Str_P, uint8_t Settings);

// Выводит изображение строки в индикатор.
// Исходная строка при этом располагается во FLASH.
extern PT_THREAD(LCD_Str_P(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, const char *Str_P, uint8_t Settings));

// Формирует изображение символа в буфере, расположенном в ОЗУ
extern uint8_t LCD_ChrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char Chr,
	uint8_t Settings);

// Выводит изображение символа на индикатор
extern PT_THREAD(LCD_Chr(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char Chr, uint8_t Settings));


// ГРАФИКА
///////////////////////////////////////////////////////////////////////////////

// Установка режима отрисовки графики
extern void LCD_DrawMode(uint8_t Mode);

// Выводит один пиксел на индикатор
extern PT_THREAD(LCD_Pixel(struct pt *Context, uint8_t X, uint8_t Y));

// Рисует линию между двумя точками на индикаторе
extern PT_THREAD(LCD_Line(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));

// Рисует окружность
extern PT_THREAD(LCD_Circle(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t R));

// Рисование прямоугольника
extern PT_THREAD(LCD_Rect(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2, uint8_t Fill));

#endif
