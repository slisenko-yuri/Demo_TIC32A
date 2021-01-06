#if !defined(_I2C_H_)
#define _I2C_H_

#include "I2C_Cfg.h"

// ������������ ������ ������������/����������� ������ �� ���� I2C�
#define I2C_SIZE_BUF	24

// ���� ������
#define I2C_NO_ERROR		0
#define I2C_ERR_SIZE_RX		1
#define I2C_ERR_DATA_NACK	2
#define I2C_ERR_SLA_W_NACK	3
#define I2C_ERR_SLA_R_NACK	4
#define I2C_ERR_BUS_ERROR	5
#define I2C_ERR_BIG_DATA	6
#define I2C_ERR_ARB_LOST	7

// ������������� �������� I2C � ������ ������
extern void I2C_InitMaster(void);

// ���������� ��� ������ �������� I2C
extern uint8_t I2C_GetLastError(void);

extern uint8_t I2C_WaitReady(uint8_t fWaitTransfer);

// ������� �������� ������������ ������ � ����� �������� I2C), �����
// ���� ��������� �������� �������� ����������.
extern uint8_t I2C_Send(uint8_t Address, uint8_t *Buf, uint8_t Size);

// ������� �������� ������ �������� ����������.
extern uint8_t I2C_SendPtr(uint8_t Address, uint8_t *Buf, uint8_t Size);

// �������� ������ �� ������ �������� I2C
extern uint8_t I2C_GetData(uint8_t *Data, uint8_t Size);

// ������� �������� ������ � ����� �������� I2C.
// �������� ������ ����� �������� � ������� ������� I2C_GetData().
extern uint8_t I2C_Receive(uint8_t Address, uint8_t Size);

// ������� ��������� ������ �� ������, ���������� ���������� BufRx.
extern uint8_t I2C_ReceivePtr(uint8_t Address, uint8_t *BufRx, uint8_t Size);

// ������� �������� ������������ ������ � ����� �������� I2C, ���������
// �������� �������� ����������, � ����� �������� ��������� ������ ��
// �������� ���������� � ����� �������� I2C. �������� ������ ����� �������� �
// ������� ������� I2C_GetData().
extern uint8_t I2C_SendReceive(uint8_t Address, uint8_t *BufTx, uint8_t SizeTx,
	uint8_t SizeRx);

// ������� ��������� �������� ������, ���������� ���������� BufTx ��������
// ����������, � ����� �������� ��������� ������ �� �������� ���������� ��
// ������, ���������� ���������� BufRx.
extern uint8_t I2C_SendPtrReceivePtr(uint8_t Address, uint8_t *BufTx,
	uint8_t SizeTx, uint8_t *BufRx, uint8_t SizeRx);

// ������� �������� ������������ ������, ���������� ���������� BufTx � �����
// �������� I2C, ��������� �������� �������� ����������, � ����� ��������
// ��������� ������ �� �������� ���������� �� ������, ���������� ����������
// BufRx.
extern uint8_t I2C_SendReceivePtr(uint8_t Address, uint8_t *BufTx,
	uint8_t SizeTx, uint8_t *BufRx, uint8_t SizeRx);

// ������� ��������� �������� ������, ���������� ���������� BufTx ��������
// ����������, � ����� �������� ��������� ������ �� �������� ���������� � �����
// �������� I2C. �������� ������ ����� �������� � ������� �������
// I2C_GetData().
extern uint8_t I2C_SendPtrReceive(uint8_t Address, uint8_t *BufTx,
	uint8_t SizeTx, uint8_t SizeRx);

// ������� �������� ��� ������ ������ ��� ����.
extern uint8_t I2C_SendSendPtr(uint8_t Address, uint8_t *BufTx, uint8_t SizeTx, uint8_t *BufTx2, uint8_t SizeTx2);

#endif
