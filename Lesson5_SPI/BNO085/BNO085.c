/*
  This is a library written for the Adafruit BNO085

 */

#include "BNO085.h"
#include <math.h>


//Bi?n toàn c?c 
uint8_t shtpHeader[4]; //M?i packet có 4 header 
uint8_t shtpData[MAX_PACKET_SIZE]; //Data m?i packet 
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //Có 6 com channels
uint8_t commandSequenceNumber = 0;				//Các commands có các seqnums, seqnum tang leen sau khi truy?n 1 frame 
uint32_t metaData[MAX_METADATA_SIZE];			//Metadata 

//Giá tr? raw khi l?y d? li?u t? sensor 
uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;	// Giá tr? raw c?a gia t?c 3 tr?c x,y,z và d? chính xác c?a phép do.
uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;	// Giá tr? raw c?a gia t?c tuy?n tính 3 tr?c x,y,z sau khi lo?i b? g.
uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;	// Giá tr? raw v?n t?c góc quanh tr?c x,y,z và d? chính xác phép do.
uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;	// Giá tr? raw c?a t? tru?ng quanh tr?c x,y,z và d? chính xác phép do.
uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;	// Giá tr? raw t?a d? quaternion c?a ph?p quay quanh tr?c x,y,z và d? chính xác phép do.
uint16_t stepCount;
uint32_t timeStamp;
uint8_t stabilityClassifier;
uint8_t activityClassifier;
uint8_t *_activityConfidences; //con tr? tr? d?n t?p giá tr? c?a 9 possible activity 
uint8_t calibrationStatus;	 //Byte R0 c?a ME Calibration Response

//Metadata trên datasheet.
int16_t rotationVector_Q1 = 14;
int16_t accelerometer_Q1 = 8;
int16_t linear_accelerometer_Q1 = 8;
int16_t gyro_Q1 = 9;
int16_t magnetometer_Q1 = 4;


void BNO085_GPIO_SPI_Initialization(void)
{
	SPI_InitTypeDef SPI_InitStruct = {0};
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* M? clock cho các ngo?i vi */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/**SPI2 GPIO Configuration
	PB13   ------> SPI2_SCK
	PB14   ------> SPI2_MISO
	PB15   ------> SPI2_MOSI
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 10;
	SPI_Init(BNO085_SPI_CHANNEL, &SPI_InitStruct);
	
	/**BNO085 GPIO Control Configuration
	 * PB12 ------> BNO085_CS (output)
	 * PA10  ------> BNO085_P1/P2 (output)
	 * PA9  ------> BNO085_RST (output)
	 * PA8  ------> BNO085_INT (input)
	 */
	/**/
	GPIO_ResetBits(BNO085_RST_PORT, BNO085_RST_PIN);
	GPIO_ResetBits(BNO085_SPI_CS_PORT, BNO085_SPI_CS_PIN);
	GPIO_ResetBits(BNO085_P1_P2_PORT, BNO085_P1_P2_PIN);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_SPI_CS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_SPI_CS_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_RST_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_RST_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_P1_P2_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_P1_P2_PORT, &GPIO_InitStruct);
	
	/**/
	GPIO_InitStruct.GPIO_Pin = BNO085_INT_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BNO085_INT_PORT, &GPIO_InitStruct);

	SPI_Cmd(BNO085_SPI_CHANNEL, ENABLE);

	CHIP_DESELECT(BNO085);
	WAKE_HIGH();
	RESET_HIGH();
}

int BNO085_Initialization(void)
{
	BNO085_GPIO_SPI_Initialization();
	
	//printf("Checking BNO085...");
	
	CHIP_DESELECT(BNO085);
	
	//Configure BNO085 de bat dau giao tiep
	WAKE_HIGH();	//Set HIGH chan P1, P2 de chon giao thuc SPI
	RESET_LOW();	//Set LOW cho chan RST tren BNO085
	Delay_ms(200);	// Cho
	RESET_HIGH();	//SET HIGH cho chan RST tren BNO085
	
	BNO085_waitForSPI(); //Cho cho den khi chan INT duoc keo xuong LOW.
	
	//Khi khoi dong, BNO085 se bat dau gui data
	BNO085_waitForSPI();  //Moi lan truyen xong data, chan INT se duoc sensor keo xuong LOW, khi INT LOW thi mcu bat dau doc data
	BNO085_receivePacket();
	
	BNO085_waitForSPI();
	BNO085_receivePacket();
	
	//Kiem tra giao tiep vs BNO085
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Yeu cau BNO085 gui productID
	shtpData[1] = 0;						 //Trong
	
	//Gui packet 2byte bang channel 2
	BNO085_sendPacket(CHANNEL_CONTROL, 2);
	
	//Cho BNO085 phan hoi
	BNO085_waitForSPI();
	if (BNO085_receivePacket() == 1)
	{
		//printf("header: %d %d %d %d\n", shtpHeader[0], shtpHeader[1], shtpHeader[2], shtpHeader[3]);
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			//printf("BNO085 who_am_i = 0x%02x...ok\n\n", shtpData[0]);
			return (0);
		}// Sensor OK
	}
	
	//printf("BNO085 Not OK: 0x%02x Should be 0x%02x\n", shtpData[0], SHTP_REPORT_PRODUCT_ID_RESPONSE);
	return (1); //Sensor not OK
}

unsigned char SPI2_SendByte(unsigned char data)
{
	while(SPI_I2S_GetFlagStatus(BNO085_SPI_CHANNEL, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(BNO085_SPI_CHANNEL, data);
	
	while(SPI_I2S_GetFlagStatus(BNO085_SPI_CHANNEL, SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(BNO085_SPI_CHANNEL);
}


//////////////////////////////////////////////////////////////////////////
//init
//////////////////////////////////////////////////////////////////////////

//Kiem tra xem cos du lieu moi khong, neu co thi xu ly
//Returns false neu data not availabe
int BNO085_dataAvailable(void)
{
	//Kiem tra chan INT, INT = LOW => data availbe
	if (GPIO_ReadInputDataBit(BNO085_INT_PORT, BNO085_INT_PIN) == 1)
		return (0);

	if (BNO085_receivePacket() == 1)
	{
		//Kiem tra packet nhan duoc
		if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
		{
			BNO085_parseInputReport(); //Update gia tri cam bien, vi du accelx, accely,...
			return (1);
		}
		else if (shtpHeader[2] == CHANNEL_CONTROL)
		{
			BNO085_parseCommandReport(); //Update phan hoi tu cac lenh da gui
			return (1);
		}
	}
	return (0);
}

//Xu ly cac phan hoi tu cam bien khi gui lenh den

//Phan hoi theo cau truc sau:
//shtpHeader[0:3]: 4 byte header;
//	shtpHeader[0:1]: Length [LSB:MSB]
//	shtpHeader[2]:	Channel = 0: command
//									Channel = 1: executable
//									Channel = 2: sensor hub control channel
//									Channel = 3: input sensor report
//									Channel = 4: wake input sensor report
//									Channel = 5: gyro rotation vector
//	shtpHeader[3]: SeqNums
//shtpData[0]: Report ID
//shtpData[1]: Sequence number
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
void BNO085_parseCommandReport(void)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
	{
		uint8_t command = shtpData[2]; //command byte

		if (command == COMMAND_ME_CALIBRATE)
		{
			calibrationStatus = shtpData[5]; //R0 - Status (0 = success, non-zero = fail)
		}
	}
	else
	{
		
	}  
}

//Xu ly du lieu cam bien duoc gui ve tu BNO085
//Phan hoi theo cau truc sau:
//shtpHeader[0:3]: 4 byte header;
//	shtpHeader[0:1]: Length [LSB:MSB]
//	shtpHeader[2]:	Channel = 0: command
//									Channel = 1: executable
//									Channel = 2: sensor hub control channel
//									Channel = 3: input sensor report
//									Channel = 4: wake input sensor report
//									Channel = 5: gyro rotation vector
//	shtpHeader[3]: SeqNums
//shtpData[0:4]: 5 byte timestamp
//shtpData[5 + 0]: report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro,...
//shtpData[6:7]: j/accel y/gyro,...
//shtpData[8:9]: k/accel z/gyro,...
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
void BNO085_parseInputReport(void)
{
	//Tinh toan so luong byte cua packet nhan 
	int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(1 << 15); //Clear bit 15. Bit nay bieu thi rang framee nhan nay la frame tiep theo cua frame nhan trc do

	dataLength -= 4; //Tinh toan so luong byte data (sau khi loai bo 4 byte header)

	timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | (shtpData[3] << (8 * 2)) | (shtpData[2] << (8 * 1)) | (shtpData[1] << (8 * 0));

	uint8_t status = shtpData[7] & 0x03; //Get status bits
	uint16_t data1 = (uint16_t)shtpData[10] << 8 | shtpData[9];
	uint16_t data2 = (uint16_t)shtpData[12] << 8 | shtpData[11];
	uint16_t data3 = (uint16_t)shtpData[14] << 8 | shtpData[13];
	uint16_t data4 = 0;
	uint16_t data5 = 0;

	if (dataLength > 14)
	{
		data4 = (uint16_t)shtpData[16] << 8 | shtpData[15];
	}
	if (dataLength > 16)
	{
		data5 = (uint16_t)shtpData[18] << 8 | shtpData[17];
	}

	//Luu data nhan dc vao cac bien toan cuc tuong ung
	switch(shtpData[5])
	{
		case SENSOR_REPORTID_ACCELEROMETER:
		{
			accelAccuracy = status;
			rawAccelX = data1;
			rawAccelY = data2;
			rawAccelZ = data3;
			break;
		}
		case SENSOR_REPORTID_LINEAR_ACCELERATION:
		{
			accelLinAccuracy = status;
			rawLinAccelX = data1;
			rawLinAccelY = data2;
			rawLinAccelZ = data3;
			break;
		}
		case SENSOR_REPORTID_GYROSCOPE:
		{
			gyroAccuracy = status;
			rawGyroX = data1;
			rawGyroY = data2;
			rawGyroZ = data3;
			break;
		}
		case SENSOR_REPORTID_MAGNETIC_FIELD:
		{
			magAccuracy = status;
			rawMagX = data1;
			rawMagY = data2;
			rawMagZ = data3;
			break;
		}
		case SENSOR_REPORTID_ROTATION_VECTOR:
		case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
		{
			quatAccuracy = status;
			rawQuatI = data1;
			rawQuatJ = data2;
			rawQuatK = data3;
			rawQuatReal = data4;
			rawQuatRadianAccuracy = data5; 
			break;
		}
		case SENSOR_REPORTID_STEP_COUNTER:
		{
			stepCount = data3; //Bytes 8/9
			break;
		}
		case SENSOR_REPORTID_STABILITY_CLASSIFIER:
		{
			stabilityClassifier = shtpData[5 + 4]; //Byte 4 
			break;
		}
		case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
		{
			activityClassifier = shtpData[5 + 5]; 

			//Load activity classification confidences into the array
			for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9
				_activityConfidences[x] = shtpData[11 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
			break;
		}
		case SHTP_REPORT_COMMAND_RESPONSE:
		{
			//printf("!");
			//The BNO085 responds with this report to command requests. It's up to use to remember which command we issued.
			uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

			if (command == COMMAND_ME_CALIBRATE)
			{
				//printf("ME Cal report found!");
				calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
			}
			break;
		}
		default:
		{
			//This sensor report ID is unhandled.
			//See reference manual to add additional feature reports as needed
		}
	}

}

//Tra ve rotation vector quaternion I
float BNO085_getQuatI()
{
	return BNO085_qToFloat(rawQuatI, rotationVector_Q1);
}

//Tra ve rotation vector quaternion J
float BNO085_getQuatJ()
{
	return BNO085_qToFloat(rawQuatJ, rotationVector_Q1);
}

//Tra ve rotation vector quaternion K
float BNO085_getQuatK()
{
	return BNO085_qToFloat(rawQuatK, rotationVector_Q1);
}

//Tra ve rotation vector quaternion Real
float BNO085_getQuatReal()
{
	return BNO085_qToFloat(rawQuatReal, rotationVector_Q1);
}

//Tra ve rotation vector accuracy
float BNO085_getQuatRadianAccuracy()
{
	return BNO085_qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
}

//Tra ve acceleration component
uint8_t BNO085_getQuatAccuracy()
{
	return (quatAccuracy);
}

//Tra ve acceleration component
float BNO085_getAccelX()
{
	return BNO085_qToFloat(rawAccelX, accelerometer_Q1);
}

//Tra ve acceleration component
float BNO085_getAccelY()
{
	return BNO085_qToFloat(rawAccelY, accelerometer_Q1);
}

//Tra ve acceleration component
float BNO085_getAccelZ()
{
	return BNO085_qToFloat(rawAccelZ, accelerometer_Q1);
}

//Tra ve acceleration component
uint8_t BNO085_getAccelAccuracy()
{
	return (accelAccuracy);
}

// linear acceleration, i.e. minus gravity

//Tra ve acceleration component
float BNO085_getLinAccelX()
{
	return BNO085_qToFloat(rawLinAccelX, linear_accelerometer_Q1);
}

//Tra ve acceleration component
float BNO085_getLinAccelY()
{
	return BNO085_qToFloat(rawLinAccelY, linear_accelerometer_Q1);
}

//Tra ve acceleration component
float BNO085_getLinAccelZ()
{
	return BNO085_qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
}

//Tra ve acceleration component
uint8_t BNO085_getLinAccelAccuracy()
{
	return (accelLinAccuracy);
}

//Tra ve gyro component
float BNO085_getGyroX()
{
	return BNO085_qToFloat(rawGyroX, gyro_Q1);
}

//Tra ve gyro component
float BNO085_getGyroY()
{
	return BNO085_qToFloat(rawGyroY, gyro_Q1);
}

//Tra ve gyro component
float BNO085_getGyroZ()
{
	return BNO085_qToFloat(rawGyroZ, gyro_Q1);
}

//Tra ve gyro component
uint8_t BNO085_getGyroAccuracy()
{
	return (gyroAccuracy);
}

//Tra ve magnetometer component
float BNO085_getMagX()
{
	return BNO085_qToFloat(rawMagX, magnetometer_Q1);
}

//Tra ve magnetometer component
float BNO085_getMagY()
{
	return BNO085_qToFloat(rawMagY, magnetometer_Q1);
}

//Tra ve magnetometer component
float BNO085_getMagZ()
{
	return BNO085_qToFloat(rawMagZ, magnetometer_Q1);
}

//Tra ve mag component
uint8_t BNO085_getMagAccuracy()
{
	return (magAccuracy);
}

//Tra ve step count
uint16_t BNO085_getStepCount()
{
	return (stepCount);
}

//Tra ve stability classifier
uint8_t BNO085_getStabilityClassifier()
{
	return (stabilityClassifier);
}

//Tra ve activity classifier
uint8_t BNO085_getActivityClassifier()
{
	return (activityClassifier);
}

//Tra ve time stamp
uint32_t BNO085_getTimeStamp()
{
	return (timeStamp);
}

//Tra ve record ID bang cach doc metadata
//Q1 duoc su dung dee tinh toan cac gia tri cua sensor
int16_t BNO085_getQ1(uint16_t recordID)
{
	//Q1 is always the lower 16 bits of word 7
	return BNO085_readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
}

//Tra ve record ID bang cach doc metadata
//Q2 is used in sensor bias
int16_t BNO085_getQ2(uint16_t recordID)
{
	//Q2 is always the upper 16 bits of word 7
	return BNO085_readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
}

//Tra ve record ID bang cach doc metadata
//Q3 is used in sensor change sensitivity
int16_t BNO085_getQ3(uint16_t recordID)
{
	//Q3 is always the upper 16 bits of word 8
	return BNO085_readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
}

//Tra ve record ID bang cach doc metadata
float BNO085_getResolution(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = BNO085_getQ1(recordID);

	//Resolution is always word 2
	uint32_t value = BNO085_readFRSword(recordID, 2); //Get word 2

	return BNO085_qToFloat(value, Q);
}

//Tra ve range bang cach doc metadata
float BNO085_getRange(uint16_t recordID)
{
	//The resolution Q value are 'the same as those used in the sensor's input report'
	//This should be Q1.
	int16_t Q = BNO085_getQ1(recordID);

	//Range is always word 1
	uint32_t value = BNO085_readFRSword(recordID, 1); //Get word 1

	return BNO085_qToFloat(value, Q);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO085_readFRSword(uint16_t recordID, uint8_t wordNumber)
{
	if (BNO085_readFRSdata(recordID, wordNumber, 1) == 1) //Get word number, just one word in length from FRS
		return (metaData[0]);						  //Return this one word

	return (0); //Error
}

//Doc data tu Flash Record System
void BNO085_frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
	shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
	shtpData[1] = 0;							//Reserved
	shtpData[2] = (readOffset >> 0) & 0xFF;		//Read Offset LSB
	shtpData[3] = (readOffset >> 8) & 0xFF;		//Read Offset MSB
	shtpData[4] = (recordID >> 0) & 0xFF;		//FRS Type LSB
	shtpData[5] = (recordID >> 8) & 0xFF;		//FRS Type MSB
	shtpData[6] = (blockSize >> 0) & 0xFF;		//Block size LSB
	shtpData[7] = (blockSize >> 8) & 0xFF;		//Block size MSB

	//Transmit packet on channel 2, 8 bytes
	BNO085_sendPacket(CHANNEL_CONTROL, 8);
}

//Input la record id va start/stop bytes, doc data tu Flash Record System (FRS)
//Returns true neu metaData load thanh cong
int BNO085_readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
	uint8_t spot = 0;

	//Dau tien gui yeu cau doc FRS
	BNO085_frsReadRequest(recordID, startLocation, wordsToRead); //Tu startLocation, doc "wordsToRead" word

	//Qua trinh doc
	while (1)
	{
		//waiting
		while (1)
		{
			uint8_t counter = 0;
			while (BNO085_receivePacket() == 0)
			{
				if (counter++ > 100)
					return (0); //Thoat vong lap, false
				Delay_ms(1);
			}

			//Nhan day du packet
			//Report ID la 0xF3 va kiem tra data nhan vs record id duoc cung cap
			if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
				if (((uint16_t)shtpData[13] << 8 | shtpData[12]) == recordID)
					break; //Thoat vong lap, true
		}

		uint8_t dataLength = shtpData[1] >> 4;
		uint8_t frsStatus = shtpData[1] & 0x0F;

		uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
		uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

		if (dataLength > 0)
		{
			metaData[spot++] = data0;
		}
		if (dataLength > 1)
		{
			metaData[spot++] = data1;
		}

		if (spot >= MAX_METADATA_SIZE)
		{
			//printf("metaData array over run. Returning.");
			return (1); //out of range. fail.
		}

		if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
		{
			return (1); //Read done
		}
	}
}

//Send command to reset IC
//Doc advertisement packets tu sensor
void BNO085_softReset(void)
{
	shtpData[0] = 1; //Reset

	//Bat dau giao tiep vs sensor
	BNO085_sendPacket(CHANNEL_EXECUTABLE, 1); //Truyen packet bang channel 1, 1 byte

	//Doc tat ca data duoc gui den, luu y sensor reset 1 lan nen se co 2 packet advertisement dc nhan
	Delay_ms(50);
	while (BNO085_receivePacket() == 1);
	Delay_ms(50);
	while (BNO085_receivePacket() == 1);
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO085_resetReason()
{
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0;							  //Reserved

	//Transmit packet on channel 2, 2 bytes
	BNO085_sendPacket(CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (BNO085_receivePacket() == 1)
	{
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
		{
			return (shtpData[1]);
		}
	}

	return (0);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO085_qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
	return fixedPointValue * powf(2, qPoint * -1);
}

//Gui packet de enable rotation vector
void BNO085_enableRotationVector(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports, 0);
}

//Gui packet de enable game rotation vector
void BNO085_enableGameRotationVector(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports, 0);
}

//Gui packet de bat dau lay gia tri gia toc goc
void BNO085_enableAccelerometer(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports, 0);
}

//Gui packet de bat dau lay gia tri gia toc goc tuyen tinh
void BNO085_enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports, 0);
}

//Gui packet de bat dau lay gia tri gyro
void BNO085_enableGyro(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports, 0);
}

//Gui packet de bat dau lay gia tri tu truong
void BNO085_enableMagnetometer(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports, 0);
}

//Gui packet de bat dau lay gia tri bo dem buoc
void BNO085_enableStepCounter(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports, 0);
}

//Gui packet de bat dau lay gia tri Stability Classifier
void BNO085_enableStabilityClassifier(uint16_t timeBetweenReports)
{
	BNO085_setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports, 0);
}

//Gui command de bat dau calib gia toc ke
void BNO085_calibrateAccelerometer()
{
	BNO085_sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Gui command de bat dau calib gyro
void BNO085_calibrateGyro()
{
	BNO085_sendCalibrateCommand(CALIBRATE_GYRO);
}

//Gui command de bat dau calib magnetometer
void BNO085_calibrateMagnetometer()
{
	BNO085_sendCalibrateCommand(CALIBRATE_MAG);
}

//Gui command de bat dau calib planar accelerometer
void BNO085_calibratePlanarAccelerometer()
{
	BNO085_sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//Gui command de bat dau calib
void BNO085_calibrateAll()
{
	BNO085_sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO085_endCalibration()
{
	BNO085_sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}

//ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus
int BNO085_calibrationComplete()
{
	if (calibrationStatus == 0)
		return (1);
	return (0);
}

//Given a sensor's report ID, this tells the BNO085 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO085_setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig)
{
	shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
	shtpData[1] = reportID;						 //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0;							 //Feature flags
	shtpData[3] = 0;							 //Change sensitivity (LSB)
	shtpData[4] = 0;							 //Change sensitivity (MSB)
	shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0;							 //Batch Interval (LSB)
	shtpData[10] = 0;							 //Batch Interval
	shtpData[11] = 0;							 //Batch Interval
	shtpData[12] = 0;							 //Batch Interval (MSB)
	shtpData[13] = (specificConfig >> 0) & 0xFF;	   	 //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF;	   	 //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF;	 //Sensor-specific config
	shtpData[16] = (specificConfig >> 24) & 0xFF;	 //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	BNO085_sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO085_sendCommand(uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
	shtpData[2] = command;					   //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
	shtpData[4] = 0; //P1
	shtpData[5] = 0; //P2
	shtpData[6] = 0;
	shtpData[7] = 0;
	shtpData[8] = 0;
	shtpData[9] = 0;
	shtpData[10] = 0;
	shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	BNO085_sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO085 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO085_sendCalibrateCommand(uint8_t thingToCalibrate)
{
	/*shtpData[3] = 0; //P0 - Accel Cal Enable
	shtpData[4] = 0; //P1 - Gyro Cal Enable
	shtpData[5] = 0; //P2 - Mag Cal Enable
	shtpData[6] = 0; //P3 - Subcommand 0x00
	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	if (thingToCalibrate == CALIBRATE_ACCEL)
		shtpData[3] = 1;
	else if (thingToCalibrate == CALIBRATE_GYRO)
		shtpData[4] = 1;
	else if (thingToCalibrate == CALIBRATE_MAG)
		shtpData[5] = 1;
	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
		shtpData[7] = 1;
	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
	{
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	}
	else if (thingToCalibrate == CALIBRATE_STOP)
		; //Do nothing, bytes are set to zero

	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
	calibrationStatus = 1;

	//Using this shtpData packet, send a command
	BNO085_sendCommand(COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from BNO085
//See page 51 of reference manual
void BNO085_requestCalibrationStatus()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

	//Using this shtpData packet, send a command
	BNO085_sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO085 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO085_saveCalibration()
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - Reserved
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	//Using this shtpData packet, send a command
	BNO085_sendCommand(COMMAND_DCD); //Save DCD command
}

//Blocking wait for BNO085 to assert (pull low) the INT pin
//indicating it's ready for comm. Can take more than 104ms
//after a hardware reset
int BNO085_waitForSPI(void)
{
	for (uint32_t counter = 0; counter < 0xffffffff; counter++) //Don't got more than 255
	{
		if (GPIO_ReadInputDataBit(BNO085_INT_PORT, BNO085_INT_PIN) == 0)
		{
			//printf("\nData available\n");
			return (1);
		}
		//printf("SPI Wait %d\n", counter);
	}
	//printf("\nData not available\n");
	return (0);
}


//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
int BNO085_receivePacket(void)
{
	uint8_t incoming;

	if (GPIO_ReadInputDataBit(BNO085_INT_PORT, BNO085_INT_PIN) == 1)
		return (0); //Data is not available

	//Old way: if (BNO085_waitForSPI() == 0) return (0); //Something went wrong

	//Get first four bytes to find out how much data we need to read

	CHIP_SELECT(BNO085);

	//Get the first four bytes, aka the packet header
	uint8_t packetLSB = SPI2_SendByte(0);
	uint8_t packetMSB = SPI2_SendByte(0);
	uint8_t channelNumber = SPI2_SendByte(0);
	uint8_t sequenceNumber = SPI2_SendByte(0); //Not sure if we need to store this or not

	//Store the header info
	shtpHeader[0] = packetLSB;
	shtpHeader[1] = packetMSB;
	shtpHeader[2] = channelNumber;
	shtpHeader[3] = sequenceNumber;

	//Calculate the number of data bytes in this packet
	int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
	dataLength &= 0x7fff; //Clear the MSbit.
	//This bit indicates if this package is a continuation of the last. Ignore it for now.
	//TODO catch this as an error and exit
	if (dataLength == 0)
	{
		//Packet is empty
		return (0); //All done
	}
	dataLength -= 4; //Remove the header bytes from the data count

	//printf("length: %d\n", dataLength);

	//Read incoming data into the shtpData array
	for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
	{
		incoming = SPI2_SendByte(0xFF);
		//printf("%d ", incoming);
		if (dataSpot < MAX_PACKET_SIZE)	//BNO085 can respond with upto 270 bytes, avoid overflow
			shtpData[dataSpot] = incoming; //Store data into the shtpData array
	}
	//printf("\n");

	CHIP_DESELECT(BNO085); //Release BNO085
	return (1); //We're done!
}


//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
int BNO085_sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
	uint8_t packetLength = dataLength + 4; //Add four bytes for the header

	//Wait for BNO085 to indicate it is available for communication
	if (BNO085_waitForSPI() == 0)
		return (0); //Data is not available

	//BNO085 has max CLK of 3MHz, MSB first,
	//The BNO085 uses CPOL = 1 and CPHA = 1. This is mode3
	CHIP_SELECT(BNO085);

	//Send the 4 byte packet header
	SPI2_SendByte(packetLength & 0xFF);			//Packet length LSB
	SPI2_SendByte(packetLength >> 8);				//Packet length MSB
	SPI2_SendByte(channelNumber);					//Channel number
	SPI2_SendByte(sequenceNumber[channelNumber]++); 	//Send the sequence number, increments with each packet sent, different counter for each channel

	//Send the user's data packet
	for (uint8_t i = 0; i < dataLength; i++)
	{
		SPI2_SendByte(shtpData[i]);
	}

	CHIP_DESELECT(BNO085);

	return (1);
}
///////////////////////////////////////////////////////////////////////////



