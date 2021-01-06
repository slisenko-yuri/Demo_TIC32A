/*
 * Demo_TIC32A.c
 *
 */ 



///////////////////////////////////////////////////////////////////////////////
// Демонстрационный пример для графического индикатора TIC32A (а также TIC270A)
// с разрешением 128x32 на базе контроллера PCF8531. Управление индикатором
// осуществляется с помощью интерфейса I2C.
// Прошивка производится с помощью программатора AVRISP mkII.
// Подключение индикатора к Arduino PRO MINI 3.3V представлено в файле
// TIC32A-ARDUINO_PRO_MINI_3.3V.pdf
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// FUSES: EXT=0xFD, HIGH=0xDA, LOW=0xFF // Для работы от внешнего кварца
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



// Размер бегущей информационной строки
#define STR_INFO_SIZE	100

// Бегущая информационная строка, которая будет выводиться в верхней части
// индикатора
char StrInfo[STR_INFO_SIZE];

static uint8_t fLcdReady = FALSE;




///////////////////////////////////////////////////////////////////////////////
// Возвращает следующий символ после Chr
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
// Задача Task_Info.
// Выводит бегущую строку в верхней части индикатора шрифтом 6x8.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Info(struct pt *Context))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока
	static char Buf[30];
	static uint8_t size;
	static uint8_t pos1, pos2;
	static uint8_t i;

	PT_BEGIN(Context); // Начало протопотока
	
	I2C_InitMaster(); // Инициализация подуля I2C

	// Инициализация индикатора
	PT_SPAWN(Context, &ContextChild, LCD_Init(&ContextChild));

	// Очищаем индикатор
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 0,
		LCD_X_RES - 1, LCD_Y_RES - 1));
	
	// Извещаем другие потоки о том, что индикатор готов к работе
	fLcdReady = TRUE;

	// Инициализируем информационную строку
	strcpy_P(StrInfo, PSTR("   Шрифт 6x8: 0123456789АБВГДЕЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдежзийклмнопртуфхцчшщъыьэюя"));

	// Вычисляем количество символов информационной строки, выводимых на
	// индикатор
	size = LCD_X_RES / FONT_Width(FONT_6x8) + 1;

	Buf[size] = '\0';
	pos1 = 0; // Начинаем с самого первого отображаемого символа

	while (TRUE)
	{
		pos2 = pos1++;
		if (pos1 >= strlen(StrInfo)) pos1 = 0;
		
		// Копируем часть информационной строки в локальный буфер
		for (i = 0; i < size; i++)
		{
			Buf[i] = StrInfo[pos2++];
			if (pos2 >= strlen(StrInfo)) pos2 = 0;
		}

		// Выводим информационную строку
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 0, 0, FONT_6x8,
			Buf, 0));
		
		MT_SleepMs(Context, 900); // Спим 900мс				
	}

	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Задача Task_Fonts.
// Демонстрация шрифтов и графических функций.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Fonts(struct pt *Context))
{
	const int CNT_FONTS = 7; // Количество демонстрируемых шрифтов
	static struct pt ContextChild; // Контекст для дочернего протопотока
	static char Buf[30];
	static uint8_t chr1, chr2;
	static uint8_t x1, y1, x2, y2;
	static uint8_t size;
	static uint8_t i;
	static uint8_t cnt;
	static uint8_t curFont, idFont;

	PT_BEGIN(Context); // Начало протопотока

	// Ждем, пока не завершится инициализация индикатора
	PT_WAIT_UNTIL(Context, fLcdReady);

	BeginDemo:


	// Демонстрация шрифтов
	///////////////////////////////////////////////////////////////////////////


	// Копируем в информационную строку набор символов для демонстрации
	// шрифта 6x8
	strcpy_P(StrInfo, PSTR("   Шрифт 6x8: 0123456789АБВГДЕЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдежзийклмнопртуфхцчшщъыьэюя"));

	// Демонстрация шрифтов 8x8 и 8x8n
	///////////////////////////////////////////////////////////////////////////

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	size = LCD_X_RES / FONT_Width(FONT_8x8) - 5 + 1;
	Buf[size] = '\0';

	// Вывод названий демонстрируемых шрифтов
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 12, FONT_6x8,
		PSTR("8x8: "), 0));
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 22, FONT_6x8,
		PSTR("8x8n:"), 0));

	chr1 = 0x20; // Начинаем с самого первого отображаемого символа

	// Установка таймаута 80сек
	MT_TimeoutMs(TIMEOUT_DEMO, 80000UL);

	while (TRUE)
	{
		// Формируем строку для вывода		
		chr2 = chr1;
		for (i = 0; i < size; i++)
		{
			Buf[i] = chr2;
			chr2 = IncChar(chr2);
		}
		chr1 = IncChar(chr1);
		
		// Выводим символы шрифта 8x8
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 30, 12,
			FONT_8x8, Buf, 0));

		PT_YIELD(Context);
		
		// Выводим символы шрифта 8x8n
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 30, 22,
			FONT_8x8n, Buf, 0));

		MT_SleepMs(Context, 500); // Спим 500мс

		// Если таймаут 80сек закончился, то выход из цикла
		if (MT_TimeoutGet(TIMEOUT_DEMO) == 0) break;
	}


	// Демонстрация шрифтов 8x16, 10x14, 12x16 и
	// шрифта 6x8 с удвоенной шириной и высотой.
	///////////////////////////////////////////////////////////////////////////

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	// Сброс таймаута смены шрифта
	MT_TimeoutSet(TIMEOUT_FONT, 0);

	size = LCD_X_RES / FONT_Width(FONT_8x16) - 5 + 1;
	Buf[size] = '\0';

	curFont = CNT_FONTS - 1;

	chr1 = 0x20; // Начинаем с самого первого символа шрифта
	
	// Установка таймаута 100сек
	MT_TimeoutMs(TIMEOUT_DEMO, 100000UL);

	while (TRUE)
	{	
		// Формируем строку для вывода
		chr2 = chr1;
		for (i = 0; i < size; i++)
		{
			Buf[i] = chr2;
			chr2 = IncChar(chr2);
		}
		chr1 = IncChar(chr1);

		if (MT_TimeoutGet(TIMEOUT_FONT) == 0)
		{
			// Установка таймаута для следующей смены шрифта
			MT_TimeoutMs(TIMEOUT_FONT, 5000);

			curFont++;
			if (curFont >= CNT_FONTS) curFont = 0;

			// Выводим название демонстрируемого шрифта
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
			// Шрифт 6x8 с удвоением по ширине и высоте
			default:
				PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12 + 4, FONT_6x8,
					PSTR("6x8WH: "), 0));
				break;
			}
			
			// Вывод измененной части буфера в дисплей
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
			// Шрифт 6x8 с удвоением по ширине и высоте
			PT_WAIT_UNTIL(Context, LCD_StrBuf(42, 12, idFont, Buf,
				LCD_TWICE_WIDTH | LCD_TWICE_HEIGHT));
		else
			PT_WAIT_UNTIL(Context, LCD_StrBuf(42, 12, idFont, Buf, 0));
		
		// Вывод измененной части буфера в дисплей
		PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));

		MT_SleepMs(Context, 500); // Спим 500мс
		
		// Если тайаут истек, то выход из цикла
		if (MT_TimeoutGet(TIMEOUT_DEMO) == 0) break;
	}


	// Демонстрация вывода точек
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Вывод точек"));

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	MT_SleepMs(Context, 500); // Спим 500мс

	for (cnt = 0; cnt < 3; cnt++)
	{
		// Устанавливаем режим вывода графики
		switch (cnt)
		{
		case 0: LCD_DrawMode(LCD_OR); break;
		case 1: LCD_DrawMode(LCD_AND); break;
		case 2: LCD_DrawMode(LCD_XOR); break;
		}

		// Установка таймаута 5000мс
		MT_TimeoutMs(TIMEOUT_DEMO, 5000);

		i = 0;

		// Крутимся в цикле, пока не завершится таймаут
		while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
		{
			// Вычисляем случайную координату x1 в диапазоне 0...127
			x1 = random() % LCD_X_RES;

			// Вычисляем случайную координату y1 в диапазоне 9...31
			y1 = (random() % (LCD_Y_RES - 9)) + 9;

			// Рисуем точку по полученным координатам
			PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, x1, y1));

			i++;
			
			// После каждых 8-ми проходов цикла отдаем управление в планировщик
			if ((i % 8) == 0) PT_YIELD(Context);
		}		
	}


	// Демонстрация вывода линий
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Вывод линий"));

	for (cnt = 0; cnt < 4; cnt++)
	{
		switch (cnt)
		{
		case 0:
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_OR); // Установка режима вывода графики
			break;
		case 1:
			// Закрашиваем нижнюю область индикатора
			// (точки с координатами Y >= 9)
			PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild, 0, 9,
				LCD_X_RES - 1, LCD_Y_RES - 1, TRUE));
			LCD_DrawMode(LCD_AND); // Установка режима вывода графики
			break;
		default:
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_XOR); // Установка режима вывода графики
			break;
		}

		if (cnt < 3)
		{
			// Вывод линий по случайным координатам

			// Вычисляем случайную координату x2 в диапазоне 0...127
			x2 = random() % LCD_X_RES;

			// Вычисляем случайную координату y2 в диапазоне 9...31
			y2 = (random() % (LCD_Y_RES - 9)) + 9;

			// Установка таймаута 4000мс
			MT_TimeoutMs(TIMEOUT_DEMO, 4000);

			// Крутимся в цикле, пока не завершится таймаут
			while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
			{
				x1 = x2;
				y1 = y2;
				
				// Вычисляем случайную координату x2 в диапазоне 0...127
				x2 = random() % LCD_X_RES;

				// Вычисляем случайную координату y2 в диапазоне 9...31
				y2 = (random() % (LCD_Y_RES - 9)) + 9;

				// Рисуем линию по полученным координатам
				PT_SPAWN(Context, &ContextChild, LCD_Line(&ContextChild,
					x1, y1, x2, y2));

				MT_SleepMs(Context, 100); // Спим 100мс

				if (cnt >= 2)
					// Стираем линию (в режиме LCD_XOR)
					PT_SPAWN(Context, &ContextChild, LCD_Line(&ContextChild,
						x1, y1, x2, y2));
			}
		}
	}


	// Демонстрация закрашенных прямоугольников
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Закрашенные прямоугольники"));

	LCD_DrawMode(LCD_XOR); // Установка режима вывода графики

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	// Установка таймаута 5000мс
	MT_TimeoutMs(TIMEOUT_DEMO, 5000);

	// Крутимся в цикле, пока не завершится таймаут
	while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
	{
		// Вычисляем случайную координату x1 в диапазоне 0...127
		x1 = random() % LCD_X_RES;

		// Вычисляем случайную координату y1 в диапазоне 9...31
		y1 = (random() % (LCD_Y_RES - 9)) + 9;

		// Вычисляем случайную координату x2 в диапазоне 0...127
		x2 = random() % LCD_X_RES;

		// Вычисляем случайную координату y2 в диапазоне 9...31
		y2 = (random() % (LCD_Y_RES - 9)) + 9;

		// Рисуем прямоугольник
		PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild,
			x1, y1, x2, y2, TRUE));

		MT_SleepMs(Context, 200); // Спим 200мс
	}

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	// Установка режима вывода графики
	LCD_DrawMode(LCD_OR);

	for (i = 0; i < 12; i++)
	{
		// Рисуем прямоугольник
		PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild,
			64 - i * 5, 20 - i, 64 + i * 5, 20 + i, TRUE));

		MT_SleepMs(Context, 200); // Спим 200мс
	}

	MT_SleepMs(Context, 1000); // Спим 1000мс


	// Демонстрация окружностей
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Окружности"));
	
	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	LCD_DrawMode(LCD_OR); // Установка режима вывода графики

	for (i = 0; i < 5; i++)
	{
		// Рисуем окружность
		PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild,
			2 + i * 22, 20, 2 + i * 2));

		MT_SleepMs(Context, 500); // Спим 500мс
	}

	MT_SleepMs(Context, 1000); // Спим 1000мс


	for (cnt = 0; cnt < 2; cnt++)
	{
		switch (cnt)
		{
		case 0: 
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_OR); // Установка режима вывода графики
			break;
		default:
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_XOR); // Установка режима вывода графики
		}

		// Установка таймаута 5000мс
		MT_TimeoutMs(TIMEOUT_DEMO, 5000);

		// Крутимся в цикле, пока не завершится таймаут
		while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
		{
			// Вычисляем случайную координату x1 в диапазоне 0...127
			x1 = random() % LCD_X_RES;

			// Вычисляем случайную координату y1 в диапазоне 10...30
			y1 = (random() % (LCD_Y_RES - 11)) + 10;

			// Вычисляем случайный радиус в диапазоне 0...14
			size = random() % 15;

			if (size > (y1 - 9)) size = size % (y1 - 9);
			if (size <= 1) size++;

			// Рисуем окружность
			PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild, x1, y1,
				size));

			MT_SleepMs(Context, 100); // Спим 100мс

			if (cnt > 0)
				// Стираем окружность (в режиме LCD_XOR)
				PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild,
					x1, y1, size));
		}
	}

	goto BeginDemo;

	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Задача Task_Blink.
// Управление светодиодом.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Blink(struct pt *Context))
{
	static uint8_t i;

	PT_BEGIN(Context); // Начало протопотока

	// Настройка вывода, к которому подключен светодиод для работы в качестве
	// выхода
	DRIVER(HL, OUT);

	while (TRUE)
	{
		for (i = 0; i < 2; i++)
		{
			ON(HL); // Включение светодиода
			MT_SleepMs(Context, 50); // Спим 50мс
			OFF(HL); // Выключение светодиода
			MT_SleepMs(Context, 150); // Спим 150мс			
		}

		MT_SleepMs(Context, 600); // Спим 600vc
	}

	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// main()
///////////////////////////////////////////////////////////////////////////////
int main(void)
{
	// Настройка тактовой частоты

	#if (DIVIDER_OSC == 1)
	clock_prescale_set(clock_div_1); // Устанавливаем делитель 1/1
	#elif (DIVIDER_OSC == 2)
	clock_prescale_set(clock_div_2); // Устанавливаем делитель 1/2
	#elif (DIVIDER_OSC == 4)
	clock_prescale_set(clock_div_4); // Устанавливаем делитель 1/4
	#elif (DIVIDER_OSC == 8)
	clock_prescale_set(clock_div_8); // Устанавливаем делитель 1/8
	#elif (DIVIDER_OSC == 16)
	clock_prescale_set(clock_div_16); // Устанавливаем делитель 1/16
	#elif (DIVIDER_OSC == 32)
	clock_prescale_set(clock_div_32); // Устанавливаем делитель 1/32
	#elif (DIVIDER_OSC == 64)
	clock_prescale_set(clock_div_64); // Устанавливаем делитель 1/64
	#elif (DIVIDER_OSC == 128)
	clock_prescale_set(clock_div_128); // Устанавливаем делитель 1/128
	#elif (DIVIDER_OSC == 256)
	clock_prescale_set(clock_div_256); // Устанавливаем делитель 1/256
	#else
	#error Unknown divider for MAIN_OSC
	#endif


	sei();

	// Настройка менеджера задач
	MT_Init();

	// Запуск задач
	MT_TaskInit(Task_Info, TASK_ACTIVE);
	MT_TaskInit(Task_Fonts, TASK_ACTIVE);
	MT_TaskInit(Task_Blink, TASK_ACTIVE);
	

    while (TRUE)
    {
	    MT_DISPATCH(); // Один проход менеджера задач
    }
}
///////////////////////////////////////////////////////////////////////////////
