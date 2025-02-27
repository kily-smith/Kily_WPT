#include "ina226.h"
#include "i2c.h"

/*
**************************************************
* ˵������ȡBUS��ѹ����ת��Ϊ��������
**************************************************
*/
float INA226_GetBusV(void) 
{
	uint16_t regData;
	float fVoltage;
    regData = INA226_GetBusVReg();
	fVoltage = regData * 0.00125f*(1-0.001487023);	///*��ѹ��LSB = 1.25mV*/������Ϊ���У׼;
	return fVoltage;
}
/*
**************************************************
* ˵������ȡ��������ת��Ϊ��������
**************************************************
*/
float INA226_GetCurrent() 
{
	uint16_t regData;
	float fCurrent;
	regData = INA226_GetCurrentReg();
	if(regData >= 0x8000)	regData = 0;
	fCurrent = regData * 0.00064f;/*������LSB = 0.64mA�����û�����*/
	return fCurrent;
}
/*
**************************************************
* ˵������ȡ���ʣ���ת��Ϊ��������
**************************************************
*/
float INA226_GetPower() 
{
	uint16_t regData;
	float fPower;
	regData = INA226_GetPowerReg();
	fPower = regData * 0.016f;/*���ʵ�LSB = ������LSB*25,���û�����*/
	return fPower;
}

uint8_t INA226_SetConfig(uint16_t ConfigWord) 
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_CONFIG;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(&INA226_COM_PORT, INA226_ADDRESS, SentTable, 3, INA226_I2C_TIMEOUT);
}

uint16_t INA226_GetConfig() 
{
    uint8_t SentTable[1] = {INA226_CONFIG};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_GetShuntV() 
{
    uint8_t SentTable[1] = {INA226_SHUNTV};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_GetBusVReg() 
{
    uint8_t SentTable[1] = {INA226_BUSV};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint8_t INA226_SetCalibrationReg(uint16_t ConfigWord) 
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_CALIB;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(&INA226_COM_PORT, INA226_ADDRESS, SentTable, 3, INA226_I2C_TIMEOUT);
}

uint16_t INA226_GetCalibrationReg() 
{
    uint8_t SentTable[1] = {INA226_CALIB};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_GetPowerReg() 
{
    uint8_t SentTable[1] = {INA226_POWER};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_GetCurrentReg() 
{
    uint8_t SentTable[1] = {INA226_CURRENT};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_GetManufID() 
{
    uint8_t SentTable[1] = {INA226_MANUF_ID};
    uint8_t ReceivedTable[2];
	
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_GetDieID() 
{
    uint8_t SentTable[1] = {INA226_DIE_ID};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint8_t INA226_SetMaskEnable(uint16_t ConfigWord) 
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_MASK;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(&INA226_COM_PORT, INA226_ADDRESS, SentTable, 3, INA226_I2C_TIMEOUT);
}

uint16_t INA226_GetMaskEnable() 
{
    uint8_t SentTable[1] = {INA226_MASK};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint8_t INA226_SetAlertLimit(uint16_t ConfigWord) 
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_ALERTL;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(&INA226_COM_PORT, INA226_ADDRESS, SentTable, 3, INA226_I2C_TIMEOUT);
}

uint16_t INA226_GetAlertLimit() 
{
    uint8_t SentTable[1] = {INA226_ALERTL};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&INA226_COM_PORT,INA226_ADDRESS, SentTable, 1, INA226_I2C_TIMEOUT);
    if (HAL_I2C_Master_Receive(&INA226_COM_PORT,INA226_ADDRESS, ReceivedTable, 2, INA226_I2C_TIMEOUT) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}


