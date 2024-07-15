/*
 * SHTC3.c
 *
 *  Created on: Apr 30, 2022
 *      Author: Intel
 */
#include "SHTC3.h"

extern I2C_HandleTypeDef hi2c1;

void i2c_trans_temp(float *temp, float *Hum, uint8_t *ok){

	HAL_StatusTypeDef status;
	float stmp, stmp1=0xFFFF;
	uint8_t crc1,crc2,i=0;
	uint8_t wake[2]={0x35,0x17};							//команда "включения" датчика SHCT3
	uint8_t Sleep[2]={0xB0,0x98};							//команда перехода в "сон" датчика SHTC3
	uint8_t read_t[2]={0x78,0x66};							//команда чтения данных: первые 3 байта показания темпертатуры, слдеующие 3 байта показания влажности
	uint8_t data[6]={0x00,0x00,0x00,0x00,0x00,0x00};		//массив для приема данных с датчика и для дальнейшей обработки


	if(HAL_I2C_Master_Transmit(&hi2c1,adr,wake,2,100)!=HAL_OK) {
		Error_Handler();
	}

	HAL_Delay(12);
	HAL_I2C_Master_Transmit(&hi2c1,adr,read_t,2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,adr1,0,0, 100);
	HAL_Delay(12);
	status=HAL_I2C_Master_Receive(&hi2c1,adr1,data,6,100);

	HAL_I2C_Master_Transmit(&hi2c1,adr,Sleep,2, 100);
	//crc1=crc;

//----Проверка контрольной суммы принятой информации crc8:: polin=0x31:  crc (init) =0xFF---------
	crc1=crc;
	while(i<2){
		crc1 ^= data[i];
		for(int i=0; i<8; i++){
			if(crc1 & 0x80)
				crc1=(crc1<<1u)^polin;
			else
				crc1=crc1<<1u;
		}
		++i;
	}
	++i;
	crc2=crc;
	while(i<5){
		crc2 ^= data[i];
		for(int i=0; i<8; i++){
			if(crc2 & 0x80)
				crc2=(crc2<<1u)^polin;
			else
				crc2=crc2<<1u;
		}
		++i;
	}
	if((crc2 == data[5]) && (crc1 == data[2]))
		*ok=1;
	//-----------------------------------------------------------------------------------------------

	if(status== HAL_OK){
		stmp=(float)((data[0]<<8) | data[1]);
		stmp1=(float)((data[3]<<8) | data[4]);
		*temp=((-45+((175*stmp)/65536)));
		*Hum=(100*stmp1)/65536;
		//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	}
	else if(status==HAL_ERROR){
		Error_Handler();
	}
}
void i2c_trans_hum(float *temp, float *Hum, uint8_t *ok){

	HAL_StatusTypeDef status;
	uint16_t stmp, stmp1;
	uint8_t crc1,crc2,i=0;
	uint8_t wake[2]={0x35,0x17};
	uint8_t Sleep[2]={0xB0,0x98};
	uint8_t read_h[2]={0x58,0xE0};										//команда чтения данных: первые 3 байта показания влажности, слдеующие 3 байта показания темпрартуры
	uint8_t data[6]={0x00,0x00,0x00,0x00,0x00,0x00};

	if(HAL_I2C_Master_Transmit(&hi2c1,adr,wake,2,100)!=HAL_OK) {
		Error_Handler();
	}

	HAL_Delay(12);
	HAL_I2C_Master_Transmit(&hi2c1,adr,read_h,2, 100);
	HAL_I2C_Master_Transmit(&hi2c1,adr1,0,0, 100);
	HAL_Delay(12);
	status=HAL_I2C_Master_Receive(&hi2c1,adr1,data,6,100);
	HAL_I2C_Master_Transmit(&hi2c1,adr,Sleep,2, 100);

	//----Проверка контрольной суммы принятой информации crc8:: polin=0x31: init =0xFF---------
	crc1=crc;
	while(i<2){
		crc1 ^= data[i];
		for(int i=0; i<8; i++){
			if(crc1 & 0x80)
				crc1=(crc1<<1u)^polin;
			else
				crc1=crc1<<1u;
		}
		++i;
	}
	++i;
	crc2=crc;
	while(i<5){
		crc2 ^= data[i];
		for(int i=0; i<8; i++){
			if(crc2 & 0x80)
				crc2=(crc2<<1u)^polin;
			else
				crc2=crc2<<1u;
		}
		++i;
	}
	if((crc2 == data[5]) && (crc1 == data[2]))
		*ok=1;
	//-----------------------------------------------------------------------------------------------
	if(status== HAL_OK){
		stmp=(data[3]<<8) | data[4] ;
		*temp=((-45+((175*stmp)/65536)));
		stmp1=(data[0]<<8) | data[1] ;
		*Hum=(100*stmp1)/65536;

		//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	}
	else if(status==HAL_ERROR){
		Error_Handler();
	}
}
