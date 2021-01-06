//////////////////////////////////////////////////////////////////////////
// ������ ��������� ���������������.
// ������������ ��� ������ ��������� � ����������� �������� Protothreads.
//////////////////////////////////////////////////////////////////////////

#include "Mt.h"


//////////////////////////////////////////////////////////////////////////
// �������� ������������ ������������� ��������
//////////////////////////////////////////////////////////////////////////

#if defined(MT_USE_SYSTIMER)

#if !defined(F_CPU)
#error "MT: F_CPU must be defined"
#endif

#if !defined(MT_SYSTIMER_PERIOD_MS)
#error "MT: MT_SYSTIMER_PERIOD_MS must be defined"
#endif

#if ((MT_SYSTIMER_PERIOD_MS < 1) || (MT_SYSTIMER_PERIOD_MS > 20))
#error "MT: MT_SYSTIMER_PERIOD_MS must be in the range 1...20"
#endif

#if (MT_SYSTIMER_PERIOD_MS >= 20) && (F_CPU > (50 * (1024 * (1 + 255))))
#error "MT: CPU freq must be less then 13107200 Hz for MT_SYSTIMER_PERIOD_MS 20"
#endif

#endif




//////////////////////////////////////////////////////////////////////////
// ���������� ������
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// ������
//////////////////////////////////////////////////////////////////////////

// ��������� �� ������
MT_TASK_TYPE __MT_Task[__MT_TASK_COUNT];

// ��������� �����
struct pt __MT_Context[__MT_TASK_COUNT];




#if !defined(MT_USE_GPIOR)

// ������� ����� �������� ����� (������� � ����������)
MT_TYPE __MT_TaskActiveFlags;

// ID ������� ������
uint8_t __MT_TaskCur;

// ������� ����� ������� ������ (������������� ��� � �������, ������ ID
// ������� ������).
MT_TYPE __MT_TaskCurMask;

#endif




#if defined(MT_USE_TASK_RUN_STOP)

// ������� ����� ��������� �����
MT_TYPE	__MT_TaskPassiveFlags;

// ������� ����� �������� ��� �������� ����� � �������� ���������
MT_TYPE	__MT_TaskRequestActiveFlags;

#endif




#if defined(MT_USE_HIGH_PRIORITY_TASK)

// ID ������������ ������
uint8_t __MT_TaskPriority;

// ������� ����� ������������ ������
MT_TYPE __MT_TaskPriorityMask;

#endif




// ���������� ������������������ �����
// (������������ ������ ��� ������������� �����)
uint8_t __MT_TaskCount;





#if defined(MT_USE_SYSTIMER)




//////////////////////////////////////////////////////////////////////////
// �������
//////////////////////////////////////////////////////////////////////////

#if defined(MT_USE_TASK_SLEEP)

// �������� ������ ���������� ������� ��� "������" �����
uint16_t __MT_TaskSysTimer[__MT_TASK_COUNT];

#endif




//////////////////////////////////////////////////////////////////////////
// ��������
//////////////////////////////////////////////////////////////////////////

#if (__MT_TIMEOUT_COUNT > 0)

// �������� ��� ���������
MT_TIMEOUT_TYPE __MT_Timeout[__MT_TIMEOUT_COUNT];

#endif





#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)

// ������� ������ ���������� �������
uint32_t __MT_SysTimer = 0;

#endif




#endif




//////////////////////////////////////////////////////////////////////////
// ��������
//////////////////////////////////////////////////////////////////////////

#if (__MT_MUTEX_COUNT > 0)

// ��������� ��������� (>=1 - ��������, 0 - ��������)
uint8_t __MT_Mutex[__MT_MUTEX_COUNT];

// ID �����, ����������� ��������������� �������
uint8_t __MT_MutexOwner[__MT_MUTEX_COUNT];
										
// ������� ����� �����, ������� ���� ������������ ����������������
// ��������
MT_TYPE __MT_MutexWaitFlags[__MT_MUTEX_COUNT];
								
#endif




//////////////////////////////////////////////////////////////////////////
// ��������
//////////////////////////////////////////////////////////////////////////

#if (__MT_DRV_COUNT > 0)

// ������� ����� �����, ������ ���������� ������ ��������������� ���������
MT_TYPE __MT_DrvWaitFlags[__MT_DRV_COUNT];

#endif
					



#if defined(MT_USE_SYSTIMER)

//////////////////////////////////////////////////////////////////////////
// ���������� ���������� ���������� �������
//////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) ||\
	defined(__AVR_ATmega8515__) ||\
	defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
	defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
	defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
	defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

#if (MT_SYSTIMER == 0)
ISR(TIMER0_COMP_vect)
#elif (MT_SYSTIMER == 1)
ISR(TIMER1_COMPA_vect)
#elif (MT_SYSTIMER == 2)
ISR(TIMER2_COMP_vect)
#elif (MT_SYSTIMER == 3)
ISR(TIMER3_COMPA_vect)
#endif

#else

#if (MT_SYSTIMER == 0)
ISR(TIMER0_COMPA_vect)
#elif (MT_SYSTIMER == 1)
ISR(TIMER1_COMPA_vect)
#elif (MT_SYSTIMER == 2)
ISR(TIMER2_COMPA_vect)
#elif (MT_SYSTIMER == 3)
ISR(TIMER3_COMPA_vect)
#elif (MT_SYSTIMER == 4)
ISR(TIMER4_COMPA_vect)
#elif (MT_SYSTIMER == 5)
ISR(TIMER5_COMPA_vect)
#endif 
#endif
{
	#if defined(MT_USE_TASK_SLEEP)
	
	uint8_t task;
	MT_TYPE mask;

	for (task = 0, mask = 1; task < __MT_TASK_COUNT; task++, mask <<= 1)
	{
		if (__MT_TaskSysTimer[task] == 0) continue;

		__MT_TaskSysTimer[task]--;

		if (__MT_TaskSysTimer[task] == 0)
		{
			__MT_TaskActiveFlags |= mask; // ��������� ������ ������, ���������
											// ������
		}
	}
	
	#endif // #if defined(MT_USE_TASK_SLEEP)

	#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)
	__MT_SysTimer++;
	#endif

	#if defined(MT_SYSTIMER_CALLBACK)
	MT_SYSTIMER_CALLBACK();
	#endif
}
//------------------------------------------------------------------------
#endif





#if defined(MT_USE_SYSTIMER)

//////////////////////////////////////////////////////////////////////////
// ������ ���������� �������
//////////////////////////////////////////////////////////////////////////

#if ((MT_SYSTIMER == 2) &&\
	!defined(__AVR_ATmega128__) && !defined(__AVR_ATmega128A__) &&\
	!defined(__AVR_ATmega64__) && !defined(__AVR_ATmega64A__)) ||\
	((MT_SYSTIMER == 0) &&\
	(defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) ||\
	defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__)))

#if (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 1"
#define	MT_SYSTIMER_DIVIDER	1UL
#define DIV_VALUE	TIM2_DIV_1

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (8UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 8"
#define	MT_SYSTIMER_DIVIDER	8UL
#define DIV_VALUE	TIM2_DIV_8

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (32UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 32"
#define	MT_SYSTIMER_DIVIDER	32UL
#define DIV_VALUE	TIM2_DIV_32

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (64UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 64"
#define	MT_SYSTIMER_DIVIDER	64UL
#define DIV_VALUE	TIM2_DIV_64

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (128UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 128"
#define	MT_SYSTIMER_DIVIDER	128UL
#define DIV_VALUE	TIM2_DIV_128

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (256UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 256"
#define	MT_SYSTIMER_DIVIDER	256UL
#define DIV_VALUE	TIM2_DIV_256

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1024UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 1024"
#define	MT_SYSTIMER_DIVIDER	1024UL
#define DIV_VALUE	TIM2_DIV_1024

#else
#error "MT: The system timer cannot be configured! (Prescaler for system timer cannot be more than 1024)"
#endif


#elif ((MT_SYSTIMER == 0) &&\
	!defined(__AVR_ATmega128__) && !defined(__AVR_ATmega128A__) &&\
	!defined(__AVR_ATmega64__) && !defined(__AVR_ATmega64A__)) ||\
	((MT_SYSTIMER == 2) &&\
	(defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) ||\
	defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__)))

#if (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 1"
#define	MT_SYSTIMER_DIVIDER	1UL
#define DIV_VALUE	TIM_DIV_1

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (8UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 8"
#define	MT_SYSTIMER_DIVIDER	8UL
#define DIV_VALUE	TIM_DIV_8

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (64UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 64"
#define	MT_SYSTIMER_DIVIDER	64UL
#define DIV_VALUE	TIM_DIV_64

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (256UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 256"
#define	MT_SYSTIMER_DIVIDER	256UL
#define DIV_VALUE	TIM_DIV_256

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1024UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 1024"
#define	MT_SYSTIMER_DIVIDER	1024UL
#define DIV_VALUE	TIM_DIV_1024

#else
#error "MT: The system timer cannot be configured! (Prescaler for system timer cannot be more than 1024)"
#endif


#elif ((MT_SYSTIMER == 1) ||\
	(MT_SYSTIMER == 3) || (MT_SYSTIMER == 4) || (MT_SYSTIMER == 5))

#if (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 1"
#define	MT_SYSTIMER_DIVIDER	1UL
#define DIV_VALUE	TIM_DIV_1

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (8UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 8"
#define	MT_SYSTIMER_DIVIDER	8UL
#define DIV_VALUE	TIM_DIV_8

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (64UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 64"
#define	MT_SYSTIMER_DIVIDER	64UL
#define DIV_VALUE	TIM_DIV_64

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (256UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 256"
#define	MT_SYSTIMER_DIVIDER	256UL
#define DIV_VALUE	TIM_DIV_256

#elif (1000UL /MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1024UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 1024"
#define	MT_SYSTIMER_DIVIDER	1024UL
#define DIV_VALUE	TIM_DIV_1024

#else
#error "MT: The system timer cannot be configured! (Prescaler for system timer cannot be more than 1024)"
#endif

#endif // #if (MT_SYSTIMER == 0)




// ��������� ��� �������� ��� �������� OCR.
// �� ��������, � ������� �������� ����� ������� ����� ������ ������
// ������������ ���������� ������� � ����� �������

// �������� OCR ��� ������� �����
#define	OCR_VALUE1 (((F_CPU) * 100UL) /\
	(MT_SYSTIMER_DIVIDER * ((1000UL * 100) / MT_SYSTIMER_PERIOD_MS)) - 1)

// �������� OCR ��� ������� �����, ����������� �� 1
#define	OCR_VALUE2	(OCR_VALUE1 + 1UL)

// �������, ���������� � ������� OCR_VALUE1, ���������� �� 100
#define	FREQ_VALUE1\
	(((F_CPU) * 100UL) / (MT_SYSTIMER_DIVIDER * (1UL + (OCR_VALUE1))))

// �������, ���������� � ������� OCR_VALUE2, ���������� �� 100
#define	FREQ_VALUE2\
	(((F_CPU) * 100UL) / (MT_SYSTIMER_DIVIDER * (1UL + (OCR_VALUE2))))

// ��������� �������, ���������� �� 100
#define	FREQ_VALUE	((1000UL * 100) / MT_SYSTIMER_PERIOD_MS)

// ����������, ����� �� �������� (OCR_VALUE1 ��� OCR_VALUE2) �����
// ������ ��� ���������� ������� ������������ �������
#if (FREQ_VALUE1 - FREQ_VALUE) < (FREQ_VALUE - FREQ_VALUE2)

#define	OCR_VALUE	OCR_VALUE1

#if (FREQ_VALUE1 - FREQ_VALUE) < (FREQ_VALUE / 1000UL)
#warning "MT: Inaccuracy of the system timer period is less than 0.1%"
#elif (FREQ_VALUE1 - FREQ_VALUE) <= (FREQ_VALUE / 100UL)
#warning "MT: Inaccuracy of the system timer period is 0.1%...1%"
#else
#warning "MT: Inaccuracy of the system timer period is more 1%!"
#endif

#else

#define	OCR_VALUE	OCR_VALUE2

#if (FREQ_VALUE - FREQ_VALUE2) < (FREQ_VALUE / 1000UL)
#warning "MT: Inaccuracy of the system timer period is less than 0.1%"
#elif (FREQ_VALUE - FREQ_VALUE2) <= (FREQ_VALUE / 100UL)
#warning "MT: Inaccuracy of the system timer period is 0.1%...1%"
#else
#warning "MT: Inaccuracy of the system timer period is more 1%!"
#endif

#endif

#endif




#if defined(MT_USE_SYSTIMER)

//------------------------------------------------------------------------
// ��������� ��� ��������� ���������� ������� AVR
//------------------------------------------------------------------------

#if (MT_SYSTIMER == 0)

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__)
#error "MT: Using timer 0 for ATmega8(A) is not allowed!"
#else

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ���������� ������� ��� ������������� ���������� �����
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// ��������� ������� �� ����� CTC � ���������� ����������.
	// �������: Ftimer = Fclk / (N * (1 + OCRnx)),
	// ��� N - ������������ (1, 8, 32(*), 64, 128(*), 256, 1024)

	// ��� ���������� �������� OCR2A ������������ �������:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	#if defined(__AVR_ATmega8515__) ||\
		defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
		defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
		defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

	OCR0 = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRn)
	TCCR0 = (0<<FOC0) | (0<<WGM00) | (0<<COM01) | (0<<COM00) |
		(1<<WGM01) | DIV_VALUE;

	// ��������� ���������� ������� �� ���������� � ��������� OCRn
	TIMSK |= (1<<OCIE0) | (0<<TOIE0);

	#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__)

	TCCR0B = (0<<FOC0A) | (0<<FOC0B) | (0<<WGM02) | DIV_VALUE;
	OCR0A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) |
		(1<<WGM01) | (0<<WGM00);

	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK |= (0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);

	#else

	TCCR0B = (0<<FOC0A) | (0<<FOC0B) | (0<<WGM02) | DIV_VALUE;
	OCR0A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) |
		(1<<WGM01) | (0<<WGM00);

	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK0 |= (0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);

	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif

#elif (MT_SYSTIMER == 1)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ���������� ������� ��� ������������� ���������� �����
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// ��������� ������� �� ����� CTC � ���������� ����������.
	// �������: Ftimer = Fclk / (N * (1 + OCRnx)),
	// ��� N - ������������ (1, 8, 32(*), 64, 128(*), 256, 1024)

	// ��� ���������� �������� OCRnA ������������ �������:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) ||\
		defined(__AVR_ATmega8515__) ||\
		defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
		defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
		defined(__AVR_ATmega64__)  || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) |
		DIV_VALUE;

	OCR1A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) |
		(0<<WGM11) | (0<<WGM10);

	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK = (TIMSK & ~((1<<TICIE1) | (1<<OCIE1A) | (1<<OCIE1B) |
		(1<<TOIE1))) | (1<<OCIE1A);

	#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__)
	
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) |
		DIV_VALUE;

	OCR1A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) |
		(0<<WGM11) | (0<<WGM10);

	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK = (TIMSK & ~((1<<ICIE1) | (1<<OCIE1B) | (1<<OCIE1A) |
		(1<<TOIE1))) | (0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
	
	#else
	
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) |
		DIV_VALUE;

	OCR1A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) |
		(0<<WGM11) | (0<<WGM10);
	
	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK1 = (0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
	
	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 2)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ���������� ������� ��� ������������� ���������� �����
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// ��������� ������� 2 �� ����� CTC � ���������� ����������.
	// �������: Ftimer = Fclk / (N * (1 + OCR2A)),
	// ��� N - ������������ (1, 8, 32, 64, 128, 256, 1024)

	// ��� ���������� �������� OCR2A ������������� �������:
	// OCR2A = Fclk / (N * Ftimer) - 1
	
	ENTER_CRITICAL();
	
	#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) ||\
		defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
		defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
		defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

	OCR2 = OCR_VALUE;
	
	// WGM20:WGM21=0:1 (����� CTC), COM21:COM20=0:0 (OC2 disconnected)
	TCCR2 = (0<<FOC2) | (0<<WGM20) | (0<<COM21) | (0<<COM20) |
		(1<<WGM21) | DIV_VALUE;

	TIMSK |= 1<<OCIE2;

	#else
	
	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0<<WGM22) | DIV_VALUE;
	OCR2A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCR2A)
	TCCR2A = (0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) |
		(1<<WGM21) | (0<<WGM20);

	// ��������� ���������� ������� 2 �� ���������� � ��������� OCR2A
	TIMSK2 |= (0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);
	
	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 3)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ���������� ������� ��� ������������� ���������� �����
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// ��������� ������� �� ����� CTC � ���������� ����������.
	// �������: Ftimer = Fclk / (N * (1 + OCRnx)),
	// ��� N - ������������ (1, 8, 32(*), 64, 128(*), 256, 1024)

	// ��� ���������� �������� OCRnA ������������� �������:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	TCCR3B = (0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (1<<WGM32) |
		DIV_VALUE;

	OCR3A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR3A = (0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) |
		(0<<WGM31) | (0<<WGM30);

	#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)
	
	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	ETIMSK = (ETIMSK & ~((1<<TICIE3) | (1<<OCIE3A) | (1<<OCIE3B) |
		(1<<TOIE3) | (1<<OCIE3C) | (1<<OCIE3C))) | (1<<OCIE3A);
	#else
	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK3 = (0<<ICIE3) | (0<<OCIE3B) | (1<<OCIE3A) | (0<<TOIE3);
	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 4)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ���������� ������� ��� ������������� ���������� �����
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// ��������� ������� �� ����� CTC � ���������� ����������.
	// �������: Ftimer = Fclk / (N * (1 + OCRnx)),
	// ��� N - ������������ (1, 8, 32(*), 64, 128(*), 256, 1024)

	// ��� ���������� �������� OCRnA ������������ �������:
	// OCRnx = Fclk / (N * Ftimer) - 1
	
	ENTER_CRITICAL();
	
	TCCR4B = (0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (1<<WGM42) |
		DIV_VALUE;

	OCR4A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR4A = (0<<COM4A1) | (0<<COM4A0) | (0<<COM4B1) | (0<<COM4B0) |
		(0<<WGM41) | (0<<WGM40);

	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK4 = (0<<ICIE4) | (0<<OCIE4B) | (1<<OCIE4A) | (0<<TOIE4);

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 5)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ���������� ������� ��� ������������� ���������� �����
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// ��������� ������� �� ����� CTC � ���������� ����������.
	// �������: Ftimer = Fclk / (N * (1 + OCRnx)),
	// ��� N - ������������ (1, 8, 32(*), 64, 128(*), 256, 1024)

	// ��� ���������� �������� OCRnA ������������� �������:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	TCCR5B = (0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (1<<WGM52) |
		DIV_VALUE;

	OCR5A = OCR_VALUE;

	// ����� CTC (��������� ���������� �� ���������� OCRnA)
	TCCR5A = (0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) |
		(0<<WGM51) | (0<<WGM50);

	// ��������� ���������� ������� �� ���������� � ��������� OCRnA
	TIMSK5 = (0<<ICIE5) | (0<<OCIE5B) | (1<<OCIE5A) | (0<<TOIE5);

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

#endif





#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ���������� ������� ��������� ����� (32 �������)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t MT_GetSysTimer(void)
{
	uint32_t ticks;

	ENTER_CRITICAL();
	ticks = __MT_SysTimer;
	EXIT_CRITICAL();

	return ticks;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ��������� �����
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_Init(void)
{
	uint8_t i;

	__MT_TaskCount = 0; // ���������� ������������������ �����

	#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)
	__MT_SysTimer = 0;
	#endif

	__MT_TaskActiveFlags = 0; // ��� �������� �����
	__MT_TaskCur = 0;
	__MT_TaskCurMask = 1 << 0;
	

	#if defined(MT_USE_HIGH_PRIORITY_TASK)
	__MT_TaskPriority = ID_UNKNOWN; // ��� ������������ ������
	__MT_TaskPriorityMask = 0; // ��� ������������ ������
	#endif
	
	for (i = 0; i < __MT_TASK_COUNT; i++)
	{
		__MT_Task[i] = NULL; // ����� ������
		#if defined(MT_USE_TASK_SLEEP)
		__MT_TaskSysTimer[i] = 0; // ������ �������� ("������") ��� ������
		#endif
	}
	
	#if (__MT_MUTEX_COUNT > 0)
	for (i = 0; i < __MT_MUTEX_COUNT; i++)
	{
		// ������� ����� �����, ������ ������������ ��������.
		// (0 - ��� �����, ������ ������������ ��������)
		__MT_MutexWaitFlags[i] = 0; 
		
		// ��������� �������� (0 - ��������)		
		__MT_Mutex[i] = 0;

		// ID ������, ��������� ���������.
		// (0xFF - ��� ������, ��������� ���������)
		__MT_MutexOwner[i] = ID_UNKNOWN;
	}
	#endif
	
	#if (__MT_TIMEOUT_COUNT > 0)
	for (i = 0; i < __MT_TIMEOUT_COUNT; i++)
	{
		__MT_Timeout[i] = 0; // ��������
	}
	#endif

	#if (__MT_DRV_COUNT > 0)
	for (i = 0; i < __MT_DRV_COUNT; i++)
	{
		__MT_DrvWaitFlags[i] = 0; // ������� ����� �����, ������
									// ������������ ���������
	}
	#endif
	
	#if defined(MT_USE_SYSTIMER)
	MT_SysTimerInit();
	#endif
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ������������� ������
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint8_t MT_TaskInit(MT_TASK_TYPE Task, uint8_t fActive)
{
	if (__MT_TaskCount >= __MT_TASK_COUNT) return ID_UNKNOWN;

	// ������������� ��������� ������
	PT_INIT(&__MT_Context[__MT_TaskCount]);

	// ��������� ��������� �� ������
	__MT_Task[__MT_TaskCount] = Task;

	if (fActive)
	{
		// ���� ������ �������, �� ���������� ��������������� ��� �
		// ���������� __MT_TaskActiveFlags
		__MT_TaskActiveFlags |= (MT_TYPE)1 << __MT_TaskCount;
	}

	#if defined(MT_USE_TASK_RUN_STOP)
	else
	{
		// ���� ������ ��������, �� ���������� ��������������� ��� �
		// ���������� __MT_TaskPassiveFlags
		__MT_TaskPassiveFlags |= (MT_TYPE)1 << __MT_TaskCount;
	}
	#endif

	__MT_TaskCount++;

	return (__MT_TaskCount - 1); // ������� ID ������
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




#if defined(MT_USE_TASK_SLEEP)

//////////////////////////////////////////////////////////////////////////
// ������� MT_SetSleep � MT_TaskSetSleep ������������� �����, � �������
// �������� ������ ����� ���������� � ���������� ���������. �����
// ����������� � ������ ���������� �������. ��� �������� ������� �
// ������������� ���������� ������������ ������ MS_TO_SYSTICK. � �������
// ������� MT_SetSleep ������ ����� ��������� � ��������� �������� ������
// ���� ����, � � ������� ������� MT_TaskSetSleep ����� ������ ������.
// ����� �������� ���������� ������������ (��������, � ������� �������
// PT_END ��� PT_YIELD) ������ �� ����� ���������� �������������, ���� ��
// ��������� ��������������� ������� ������� "������" (����������
// __MT_TaskSysTimer[idTask]), ������������� � ������� ���� �������, �����
// ���� ������ ����� ���������� � �������� ��������� ����������
// ���������������� ���� � ���������� __MT_TaskActiveFlags.
// ��������� ��������, � ����� ��������� ���� ���������� ����� ���������
// �������� ������������ � ����������� ���������� ���������� �������.
// ��� ������������� ���� ������� � ������� ���������� � ����� Mt_Cfg.h
// ����������������� ��������� ������:
// #define MT_USE_TASK_SLEEP
//
//  ������:
//
//  PT_THREAD(TaskN(struct pt *Context))
//  {
//	  PT_BEGIN(Context);
//
//	  //...
//
//	  // ��������� �������� ������� "������", ������� 1000 ��.
//	  MT_SetSleep(MS_TO_SYSTICK(1000));
//
//	  // �������� ���������� ������������. ����������� ������� ������ ������
//	  // ������ �� ���������� �������, ������� 1000 ��, ����� ���� ������
//	  // ��������� ���� ���������� � �����, �������������� �����
//	  // ������� PT_YIELD.
//	  PT_YIELD(Context);
//
//	  //...
//
//	  PT_END(Context);
//  }
//////////////////////////////////////////////////////////////////////////

void MT_SetSleep(uint16_t Takts)
{
	if (Takts)
	{
		ENTER_CRITICAL();

		// �������� ��� ���������� ������
		__MT_TaskActiveFlags &= ~__MT_TaskCurMask;

		// ������������� ����� "������"
		__MT_TaskSysTimer[__MT_TaskCur] = Takts;
	
		EXIT_CRITICAL();
	}
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// ������� ������ � "������". ����� ����������� � ������ ����������
// �������.
// ����� ��������� � "������" ������ ������
// � ���� ������� ������������� ��������� ������� ID ������� ������.
//////////////////////////////////////////////////////////////////////////
void MT_TaskSetSleep(uint8_t idTask, uint16_t Takts)
{
	if (Takts)
	{
		ENTER_CRITICAL();
		
		// �������� ��� ���������� ������
		__MT_TaskActiveFlags &= ~((MT_TYPE)1 << idTask);
		
		// ������������� ����� "������"
		__MT_TaskSysTimer[idTask] = Takts;

		EXIT_CRITICAL();
	}
}
//////////////////////////////////////////////////////////////////////////

#endif




#if (__MT_MUTEX_COUNT > 0)
//////////////////////////////////////////////////////////////////////////
// ������ ��������.
// ���������� 1, ���� ������� ������� ��������.
// ���������� 0, ���� ������� ������ ���������� � ������ ��������� ��
// ������������ �������� �������, ����������� ���.
// ����������: ������ ������ ������ �������� ������� ��� ����������������
// ������ MT_MutexFree() ����� 255 ��� (�� ������ ����������� ��������)!
//////////////////////////////////////////////////////////////////////////
uint8_t __MT_MutexWait(uint8_t Mutex)
{
	MT_TYPE mask;

	if (__MT_TaskCur == __MT_MutexOwner[Mutex])
	{
		// ���� ������ ��� ������� ���� ���������, �� �������������� ���
		// ��������.
		__MT_Mutex[Mutex]++;
	}
	else // ���� ������� ������ �� ������� ���������
	{	
		if (__MT_Mutex[Mutex] != 0) // ���� ������� �����
		{
			// ������ ������� ����� ������� ������
			mask = __MT_TaskCurMask;

			ENTER_CRITICAL();

			// ���������� ������� ����� ������, ��������� ������������
			// ������� ��������.
			__MT_MutexWaitFlags[Mutex] |= mask;

			// ������� ��� ���������� ������.
			__MT_TaskActiveFlags &= ~mask;

			EXIT_CRITICAL();

			return 0;
		}
		__MT_Mutex[Mutex] = 1; // ������� �����.
		
		// ��������� ID ������, ����������� �������.
		__MT_MutexOwner[Mutex] = __MT_TaskCur;
	}
	return 1;
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// ������������ ��������
//////////////////////////////////////////////////////////////////////////
void MT_MutexFree(uint8_t Mutex)
{
	ENTER_CRITICAL();

	__MT_Mutex[Mutex]--;

	if (__MT_Mutex[Mutex] == 0) // ���� ������� ��������
	{
		// ���������� ������, ������� ���� ������������ ������� ��������
		__MT_TaskActiveFlags |= __MT_MutexWaitFlags[Mutex];

		// ������� ������� ����� �����, ������ ������������ �������
		// ��������
		__MT_MutexWaitFlags[Mutex] = 0;
		
		// ������ ��������� ����� �� �������
		__MT_MutexOwner[Mutex] = ID_UNKNOWN;
	}

	EXIT_CRITICAL();
}
//////////////////////////////////////////////////////////////////////////

#endif // __MT_MUTEX_COUNT > 0




#if defined(MT_USE_TASK_RUN_STOP)

//////////////////////////////////////////////////////////////////////////
// ������ ������ (������� � �������� ���������) ��� �����������
// �����������
//////////////////////////////////////////////////////////////////////////
void MT_TaskSetActiveIrq(uint8_t idTask)
{
	MT_TYPE mask;
	
	mask = (MT_TYPE)1 << idTask; // ������� ����� ������
	
	if (__MT_TaskPassiveFlags & mask) // ���� ������ ��������
	{
		// ������� ������ � �������� ���������
		__MT_TaskActiveFlags |= mask;
		__MT_TaskPassiveFlags &= ~mask;
	}
	else
	{
		// ������������� ������ �� ��������� ������
		__MT_TaskRequestActiveFlags |= mask;
	}
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// ������� ������ � �������� ���������
//////////////////////////////////////////////////////////////////////////
void MT_TaskSetActive(uint8_t idTask)
{
	ENTER_CRITICAL();
	MT_TaskSetActiveIrq(idTask);
	EXIT_CRITICAL();
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// ������� ������� ������ � ��������� ���������
//////////////////////////////////////////////////////////////////////////
uint8_t MT_TaskSetPassive(void)
{
	uint8_t result;

	ENTER_CRITICAL();
	if (__MT_TaskRequestActiveFlags & __MT_TaskCurMask)
	{
		////__MT_TaskActiveFlags |= 1 << __MT_TaskCur; // ������, �.�. ��� ����������
		__MT_TaskRequestActiveFlags &= ~__MT_TaskCurMask;
		
		// ������ �������� ��������, �.�. ��� ������ �� ������� � ��������
		// ���������
		result = 0;
	}
	else
	{
		// ������� ������ � ��������� ���������
		__MT_TaskActiveFlags &= ~__MT_TaskCurMask;
		__MT_TaskPassiveFlags |= __MT_TaskCurMask;
		
		
		result = 1; // ������ ������� � ��������� ���������
	}
	EXIT_CRITICAL();
	return result;
}
//////////////////////////////////////////////////////////////////////////
#endif



