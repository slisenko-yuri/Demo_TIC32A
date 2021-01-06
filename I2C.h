#if !defined(_I2C_H_)
#define _I2C_H_

#include "I2C_Cfg.h"

// Максимальный размер передаваемых/принимаемых данных по шине I2Cю
#define I2C_SIZE_BUF	24

// Коды ошибок
#define I2C_NO_ERROR		0
#define I2C_ERR_SIZE_RX		1
#define I2C_ERR_DATA_NACK	2
#define I2C_ERR_SLA_W_NACK	3
#define I2C_ERR_SLA_R_NACK	4
#define I2C_ERR_BUS_ERROR	5
#define I2C_ERR_BIG_DATA	6
#define I2C_ERR_ARB_LOST	7

// Инициализация драйвера I2C в режиме МАСТЕР
extern void I2C_InitMaster(void);

// Возвращает код ошибки драйвера I2C
extern uint8_t I2C_GetLastError(void);

extern uint8_t I2C_WaitReady(uint8_t fWaitTransfer);

// Функция копирует передаваемые данные в буфер драйвера I2C), после
// чего запускает передачу ведомому устройству.
extern uint8_t I2C_Send(uint8_t Address, uint8_t *Buf, uint8_t Size);

// Функция передает данные ведомому устройству.
extern uint8_t I2C_SendPtr(uint8_t Address, uint8_t *Buf, uint8_t Size);

// Копирует данные из буфера драйвера I2C
extern uint8_t I2C_GetData(uint8_t *Data, uint8_t Size);

// Функция принимет данные в буфер драйвера I2C.
// Принятые данные можно получить с помощью функции I2C_GetData().
extern uint8_t I2C_Receive(uint8_t Address, uint8_t Size);

// Функция принимает данные по адресу, указанному параметром BufRx.
extern uint8_t I2C_ReceivePtr(uint8_t Address, uint8_t *BufRx, uint8_t Size);

// Функция копирует передаваемые данные в буфер драйвера I2C, запускает
// передачу ведомому устройству, а после передачи принимает данные из
// ведомого устройства в буфер драйвера I2C. Принятые данные можно получить с
// помощью функции I2C_GetData().
extern uint8_t I2C_SendReceive(uint8_t Address, uint8_t *BufTx, uint8_t SizeTx,
	uint8_t SizeRx);

// Функция запускает передачу данных, адресуемых параметром BufTx ведомому
// устройству, а после передачи принимает данные из ведомого устройства по
// адресу, указанному параметром BufRx.
extern uint8_t I2C_SendPtrReceivePtr(uint8_t Address, uint8_t *BufTx,
	uint8_t SizeTx, uint8_t *BufRx, uint8_t SizeRx);

// Функция копирует передаваемые данные, адресуемые параметром BufTx в буфер
// драйвера I2C, запускает передачу ведомому устройству, а после передачи
// принимает данные из ведомого устройства по адресу, указанному параметром
// BufRx.
extern uint8_t I2C_SendReceivePtr(uint8_t Address, uint8_t *BufTx,
	uint8_t SizeTx, uint8_t *BufRx, uint8_t SizeRx);

// Функция запускает передачу данных, адресуемых параметром BufTx ведомому
// устройству, а после передачи принимает данные из ведомого устройства в буфер
// драйвера I2C. Принятые данные можно получить с помощью функции
// I2C_GetData().
extern uint8_t I2C_SendPtrReceive(uint8_t Address, uint8_t *BufTx,
	uint8_t SizeTx, uint8_t SizeRx);

// Функция передает две порции данных как одну.
extern uint8_t I2C_SendSendPtr(uint8_t Address, uint8_t *BufTx, uint8_t SizeTx, uint8_t *BufTx2, uint8_t SizeTx2);

#endif
