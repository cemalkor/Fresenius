/******************************************************************************
 * Title                 :   Fresenius Kabi Agilira Range
 * Author                :   Cemal KÖR
 * Notes                 :   Informations in this document only refer to devices belonging to the Agilia Family
 *******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "fresenius.h"
#include "fresenius_crc_tables.h"

/******************************************************************************
 * Module Typedefs
 ******************************************************************************/
RequestForNOPTypedefStruct RequestForNOP;
FreseniusKabiAgiliaRangeTypedefStruct InfusPomp;
AgiliaRangeSendDataTypedefStruct AgiliaRangeSendData;
RequestForAdvancedInfusionDataTypedefStruct RequestForAdvInf;
RequestForOpeningClosingCommunicationSessionTypedefStruct RequestForOpeningClosingComm;

extern ErrFlagStructTypedef ErrFlag;
extern SentToDataForInfusPompTypedefStruct SentToDataForInfusPomp;
/******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void Device_Task(void *pvParameters);
uint8_t ParseMessage(uint8_t Data);
void SendToDevice(uint8_t *ucRxData, uint32_t len);

static uint8_t ParseData(uint8_t *Data);
// static void RequestForNOPFunc(void);									// Not Used
// static void RequestForAdvancedInfusionData(uint8_t DataID);			// Not Used
// static void SetRecommendedCommunicationParameters(uint8_t CommMode);	// Not Used
static uint8_t Comm_CalcCrc8(const uint8_t *i_pbyBuffer, int i_iSize);
static uint16_t crc_C_Calc16(uint8_t *i_pbyBuf, uint16_t i_dwSize);
static uint16_t crc_C_Update16(uint8_t *i_pbyBuf, uint16_t i_dwSize, uint16_t i_wCrc);

/******************************************************************************
 * Module Variable Definitions
 ******************************************************************************/
static uint8_t parseflag = 0;
static uint8_t parse_flag = 0;

/******************************************************************************
 * Module Tasks
 ******************************************************************************/

/******************************************************************************
 * Function Definitions
 ******************************************************************************/
uint8_t ParseMessage(uint8_t Data)
{ /* Mesaj paketinin doğru alınması fonksiyonu */

	parse_flag = 0;
	ErrFlag.errorflag = 0;
	static uint16_t crc16 = 0;
	static uint8_t CorretMsgFlag = 0;
	InfusPomp.MessageBuffer[InfusPomp.MsgCounter] = Data;

	if (InfusPomp.MessageBuffer[0] == START_OF_FRAME)
	{
		if (InfusPomp.MessageBuffer[4] > 0x7E)
		{
			memset(InfusPomp.MessageBuffer, 0, sizeof(InfusPomp.MessageBuffer));
			InfusPomp.MsgCounter = 0;
			ErrFlag.errorflag = 1;
			return parse_flag;
		}

		if (InfusPomp.MsgCounter == 5)
		{ /*CRC8 index*/
			if (Comm_CalcCrc8(&InfusPomp.MessageBuffer[1], 4) == InfusPomp.MessageBuffer[5 /*CRC8 Index*/] && CorretMsgFlag == 0)
			{ /* Correct CRC8 Message */
				CorretMsgFlag = 1;
			}
			else
			{ /* Wrong Message */
				CLEAR_FLAGS_BUFFERS_AND_RETURN;
				ErrFlag.errorflag = 1;
			}
		}
		if (CorretMsgFlag == 1)
		{
			/* (+7) sebebi (-1) Sonradan arttirdigimiz icin, (+8) CRC16, CRC8 ve oncesi size'a eklenmedigi icin */
			if (InfusPomp.MsgCounter == InfusPomp.MessageBuffer[4] + 7)
			{ /* Check CRC16 */

				crc16 = crc_C_Calc16(&InfusPomp.MessageBuffer[6], InfusPomp.MessageBuffer[4]);

				if (crc16 == ((InfusPomp.MessageBuffer[InfusPomp.MessageBuffer[4] + 6] << 8) + InfusPomp.MessageBuffer[InfusPomp.MessageBuffer[4] + 7])) // CRC16 TEST OK!!
				{																																		 /* Corret Message, Copy to structure then clear all buffers */

					parse_flag = ParseData(InfusPomp.MessageBuffer);
					CLEAR_FLAGS_BUFFERS_AND_RETURN;
					return parse_flag;
				}
				else
				{ /* Wrong Message */
					CLEAR_FLAGS_BUFFERS_AND_RETURN;
					ErrFlag.errorflag = 1;
				}
			}
		}
		InfusPomp.MsgCounter++;
	}
	else
	{
		InfusPomp.MsgCounter = 0;
	}

	return parse_flag;
}

void SendToDevice(uint8_t *ucRxData, uint32_t len)
{ /* Sorgu yapılması gerektiğinde burada yapılır */

	xStreamBufferSend(xTxStreamBuffer,
					  (void *)ucRxData,
					  len,
					  portMAX_DELAY);
}

static uint8_t ParseData(uint8_t *Data)
{ /* Mesaj paketinde verilerin parametrelere özgü parse edilme fonksiyonu */

	sprintf(SentToDataForInfusPomp.Flowrate_UNIT, "ml/s");
	sprintf(SentToDataForInfusPomp.SendVolume_UNIT, "ml");
	sprintf(SentToDataForInfusPomp.RemainingVolume_UNIT, "ml");

	parseflag = 0;

	switch (Data[6])
	{
	case OPENING_CLOSING_A_COMMUNICATION_SESSION_DtoH:

		memcpy(&InfusPomp.ResponseForOpeningClosingComm, &Data[7], 3);

		break;

	case REPLY_FOR_NOP_DtoH:

		InfusPomp.ResponseForNOP.dwID = Data[7] + (Data[8] << 8) + (Data[9] << 16) + (Data[10] << 24);

		break;

	case NACK_DtoH:

		memcpy(&InfusPomp.NACKandErr, &Data[7], Data[4] - 1);
		InfusPomp.NACKandErr.ErrorCode = ToLittleEndian(InfusPomp.NACKandErr.ErrorCode, UINT16Size);
		InfusPomp.NACKandErr.AddData = ToLittleEndian(InfusPomp.NACKandErr.AddData, UINT32Size);

		break;

	/* Device Status: Verilerin alındığı mod */
	case DEVICE_STATUS_INFORMATION_DtoH:

		memcpy((uint8_t *)(&InfusPomp.FirstBlockOfDataVolumatAgilia), &Data[7], sizeof(FirstBlockOfDataVolumatAgiliaTypedefStruct));
		memcpy((uint8_t *)(&InfusPomp.InfusionSubId), &Data[sizeof(FirstBlockOfDataVolumatAgiliaTypedefStruct)], sizeof(InfusionSubIdTypedefStruct));

		if (InfusPomp.FirstBlockOfDataVolumatAgilia.Device == 0x44 && InfusPomp.InfusionSubId.SubIdInfusion == 0x02)
		{ /* Volumat Agilia cihaz ID'si ve SubID sayıları doğru ise ilgili yerdeki parametre verileri alınır. */

			SentToDataForInfusPomp.Flowrate = ((InfusPomp.InfusionSubId.FlowRate[0] << 24) + (InfusPomp.InfusionSubId.FlowRate[1] << 16) + (InfusPomp.InfusionSubId.FlowRate[2] << 8) + InfusPomp.InfusionSubId.FlowRate[3]) / 1000.0F;
			memcpy(SentToDataForInfusPomp.MedicineName, InfusPomp.InfusionSubId.DrugName, sizeof(InfusPomp.InfusionSubId.DrugName));
			SentToDataForInfusPomp.SendVolume = ToLittleEndian(InfusPomp.InfusionSubId.Volume, UINT32Size) / 1000.0f;
			SentToDataForInfusPomp.RemainingVolume = ((InfusPomp.InfusionSubId.LeftVolume[0] << 24) + (InfusPomp.InfusionSubId.LeftVolume[1] << 16) + (InfusPomp.InfusionSubId.LeftVolume[2] << 8) + InfusPomp.InfusionSubId.LeftVolume[3]) / 1000.0F;
			SentToDataForInfusPomp.InfusMode = (ToLittleEndian(InfusPomp.FirstBlockOfDataVolumatAgilia.InfusMode[0], UINT8Size)); // - 0x80;
			SentToDataForInfusPomp.Alarm = (InfusPomp.FirstBlockOfDataVolumatAgilia.Alarm[0] << 24) + (InfusPomp.FirstBlockOfDataVolumatAgilia.Alarm[1] << 16) + (InfusPomp.FirstBlockOfDataVolumatAgilia.Alarm[2] << 8) + (InfusPomp.FirstBlockOfDataVolumatAgilia.Alarm[3]);

		}
		parseflag = 1;

		break;

	case ADVANCED_INFUSION_DATA_DtoH:

		memcpy(&InfusPomp.ResponseForAdvancedInfusionData, &Data[7], sizeof(ResponseForAdvancedInfusionDataTypedefStruct));
		InfusPomp.ResponseForAdvancedInfusionData.SampleTime = ToLittleEndian(InfusPomp.ResponseForAdvancedInfusionData.SampleTime, UINT32Size);

		switch (InfusPomp.ResponseForAdvancedInfusionData.DataID)
		{
		case ID_OF_THE_FLOWRATE:

			memcpy(&InfusPomp.Flowrate, &Data[7], sizeof(FlowrateTypedefStruct));
			InfusPomp.Flowrate.Data = ToLittleEndian(InfusPomp.Flowrate.Data, UINT32Size);
			break;

		case ID_OF_THE_INFUSED_VOLUME:

			memcpy(&InfusPomp.InfusedVolume, &Data[7], sizeof(InfusedVolumeTypedefStruct));
			InfusPomp.InfusedVolume.Data = ToLittleEndian(InfusPomp.InfusedVolume.Data, UINT32Size);
			InfusPomp.InfusedVolume.Mass = ToLittleEndian(InfusPomp.InfusedVolume.Mass, UINT32Size);
			break;

		case ID_OF_THE_MANUAL_BOLUS:

			memcpy(&InfusPomp.ManualBolus, &Data[7], sizeof(ManualBolusTypedefStruct));
			InfusPomp.ManualBolus.VolBolus = ToLittleEndian(InfusPomp.ManualBolus.VolBolus, UINT32Size);
			InfusPomp.ManualBolus.VolTot = ToLittleEndian(InfusPomp.ManualBolus.VolTot, UINT32Size);
			InfusPomp.ManualBolus.Mass = ToLittleEndian(InfusPomp.ManualBolus.Mass, UINT32Size);
			break;

		case ID_OF_THE_SIMPLE_BOLUS:

			memcpy(&InfusPomp.SimpleBolus, &Data[7], sizeof(SimpleBolusTypedefStruct));
			InfusPomp.SimpleBolus.Flowrate = ToLittleEndian(InfusPomp.SimpleBolus.Flowrate, UINT32Size);
			InfusPomp.SimpleBolus.Volume = ToLittleEndian(InfusPomp.SimpleBolus.Volume, UINT32Size);
			InfusPomp.SimpleBolus.Mass = ToLittleEndian(InfusPomp.SimpleBolus.Mass, UINT32Size);
			break;

		case ID_OF_THE_LOADING_DOSE:
			memcpy(&InfusPomp.LoadingBolus, &Data[7], sizeof(LoadingBolusTypedefStruct));
			InfusPomp.LoadingBolus.FlowRate = ToLittleEndian(InfusPomp.LoadingBolus.FlowRate, UINT32Size);
			InfusPomp.LoadingBolus.Volume = ToLittleEndian(InfusPomp.LoadingBolus.Volume, UINT32Size);
			InfusPomp.LoadingBolus.Duration = ToLittleEndian(InfusPomp.LoadingBolus.Duration, UINT32Size);
			InfusPomp.LoadingBolus.MassToInfus = ToLittleEndian(InfusPomp.LoadingBolus.MassToInfus, UINT32Size);
			break;

		case ID_OF_THE_PROGRAMMED_BOLUS:
			memcpy(&InfusPomp.ProgrammedBolus, &Data[7], sizeof(ProgrammedBolusTypedefStruct));
			InfusPomp.ProgrammedBolus.FlowRate = ToLittleEndian(InfusPomp.ProgrammedBolus.FlowRate, UINT32Size);
			InfusPomp.ProgrammedBolus.Volume = ToLittleEndian(InfusPomp.ProgrammedBolus.Volume, UINT32Size);
			InfusPomp.ProgrammedBolus.Duration = ToLittleEndian(InfusPomp.ProgrammedBolus.Duration, UINT32Size);
			InfusPomp.ProgrammedBolus.MassToInfus = ToLittleEndian(InfusPomp.ProgrammedBolus.MassToInfus, UINT32Size);
			break;

		default:
			break;
		}

		parseflag = 1;

		break;

	case DEVICE_INFORMATION_DtoH:

		break;

	case IP_ADDRESS_DtoH:

		memcpy(&InfusPomp.IPAddress.IPAddr, &Data[7], 15);

		break;

	default:

		ErrFlag.errorflag = 1;
		break;
	}

	return parseflag;
}

/* static void RequestForNOPFunc()		// Not Used
{ // NOP için Request Fonksiyonu 

	static uint8_t SendBuffer[50] = {0};
	static uint8_t CrcBuffer[50] = {0};

	RequestForNOP.dwID = ToLittleEndian(1, UINT32Size);

	AgiliaRangeSendData.StartFrame = START_OF_FRAME;
	AgiliaRangeSendData.Type = STANDARD_FRAMES;
	AgiliaRangeSendData.FrameNumber = 2;
	AgiliaRangeSendData.FrameNumberACK = 2;
	AgiliaRangeSendData.Size = sizeof(RequestForNOPTypedefStruct) + 1;
	AgiliaRangeSendData.Crc8 = Comm_CalcCrc8(((uint8_t *)&AgiliaRangeSendData.Type), 4);
	AgiliaRangeSendData.FrameId = REQUEST_FOR_NOP_HtoD;

	// CRC16 ID ve DATA nin hesaplanmasiyla oluyor 
	CrcBuffer[0] = AgiliaRangeSendData.FrameId;
	memcpy(&CrcBuffer[1], (uint8_t *)(&RequestForNOP), sizeof(RequestForNOPTypedefStruct));
	AgiliaRangeSendData.Crc16 = ToLittleEndian(crc_C_Calc16(CrcBuffer, sizeof(RequestForNOPTypedefStruct) + 1), UINT16Size);

	memcpy(SendBuffer, (uint8_t *)(&AgiliaRangeSendData), 7);
	memcpy(&SendBuffer[7], (uint8_t *)(&RequestForNOP), sizeof(RequestForNOPTypedefStruct));
	memcpy(&SendBuffer[7 + sizeof(RequestForNOPTypedefStruct)], (uint8_t *)(&AgiliaRangeSendData.Crc16), 2);

	SendToDevice(SendBuffer, sizeof(SendBuffer));
}
*/

/* static void RequestForAdvancedInfusionData(uint8_t DataID)	// Not Used
{ // Advanced Infuzyon Verileri için Request Fonksiyonu 

	static uint8_t SendBuffer[50] = {0};
	static uint8_t CrcBuffer[50] = {0};

	RequestForAdvInf.DataID = DataID;
	RequestForAdvInf.MaxDuration = 50;
	RequestForAdvInf.MinDuration = 50;

	AgiliaRangeSendData.StartFrame = START_OF_FRAME;
	AgiliaRangeSendData.Type = STANDARD_FRAMES;
	AgiliaRangeSendData.FrameNumber = 1;
	AgiliaRangeSendData.FrameNumberACK = 1;
	AgiliaRangeSendData.Size = sizeof(RequestForAdvancedInfusionDataTypedefStruct) + 1;
	AgiliaRangeSendData.Crc8 = Comm_CalcCrc8(((uint8_t *)&AgiliaRangeSendData.Type), 4);
	AgiliaRangeSendData.FrameId = REQUEST_FOR_ADVANCED_INFUSION_DATA_HtoD;

	// CRC16 ID ve DATA nin hesaplanmasiyla oluyor
	CrcBuffer[0] = AgiliaRangeSendData.FrameId;
	memcpy(&CrcBuffer[1], (uint8_t *)(&RequestForAdvInf), sizeof(RequestForAdvancedInfusionDataTypedefStruct));
	AgiliaRangeSendData.Crc16 = ToLittleEndian(crc_C_Calc16(CrcBuffer, sizeof(RequestForAdvancedInfusionDataTypedefStruct) + 1), UINT16Size);

	memcpy(SendBuffer, (uint8_t *)(&AgiliaRangeSendData), 7);
	memcpy(&SendBuffer[7], (uint8_t *)(&RequestForAdvInf), 3);
	memcpy(&SendBuffer[10], (uint8_t *)(&AgiliaRangeSendData.Crc16), 2);

	SendToDevice(SendBuffer, sizeof(SendBuffer));
}
*/

/* static void SetRecommendedCommunicationParameters(uint8_t CommMode)	// Not Used
{ // Önerilen İletişim Parametrelerinin Ayarlanma Fonksiyonu 

	static uint8_t SendBuffer[50] = {0};
	static uint8_t CrcBuffer[50] = {0};

	RequestForOpeningClosingComm.Mode = CommMode;
	RequestForOpeningClosingComm.HostWindow = 8;
	RequestForOpeningClosingComm.DeviceWindow = 8;
	RequestForOpeningClosingComm.DisconnectBehaviour = 0;
	RequestForOpeningClosingComm.PacketRepeatTimeout_ms = ToLittleEndian(RECOMMEND_PACKETREPEATTIMEOUT_MS, UINT16Size);
	RequestForOpeningClosingComm.PacketMinRate_ms = ToLittleEndian(RECOMMEND_PACKETMINRATE_MS, UINT16Size);
	RequestForOpeningClosingComm.DisconnectTimeout_ms = ToLittleEndian(RECOMMEND_DISCONNECTTIMEOUT_MS, UINT16Size);

	AgiliaRangeSendData.StartFrame = START_OF_FRAME;
	AgiliaRangeSendData.Type = STANDARD_FRAMES;
	AgiliaRangeSendData.FrameNumber = 0;
	AgiliaRangeSendData.FrameNumberACK = 0;
	AgiliaRangeSendData.Size = sizeof(RequestForOpeningClosingCommunicationSessionTypedefStruct) + 1;
	AgiliaRangeSendData.Crc8 = Comm_CalcCrc8(((uint8_t *)&AgiliaRangeSendData.Type), 4);
	AgiliaRangeSendData.FrameId = REQUEST_FOR_OPENING_CLOSING_A_COMMUNICATION_SESSION_HtoD;

	// CRC16 ID ve DATA nin hesaplanmasiyla oluyor
	CrcBuffer[0] = AgiliaRangeSendData.FrameId;
	memcpy(&CrcBuffer[1], (uint8_t *)(&RequestForOpeningClosingComm), sizeof(RequestForOpeningClosingCommunicationSessionTypedefStruct));
	AgiliaRangeSendData.Crc16 = ToLittleEndian(crc_C_Calc16(CrcBuffer, sizeof(RequestForOpeningClosingCommunicationSessionTypedefStruct) + 1), UINT16Size);

	memcpy(SendBuffer, (uint8_t *)(&AgiliaRangeSendData), 7);
	memcpy(&SendBuffer[7], (uint8_t *)(&RequestForOpeningClosingComm), sizeof(RequestForOpeningClosingCommunicationSessionTypedefStruct));
	memcpy(&SendBuffer[sizeof(RequestForOpeningClosingCommunicationSessionTypedefStruct) + 7], (uint8_t *)(&AgiliaRangeSendData.Crc16), 2);

	SendToDevice(SendBuffer, sizeof(SendBuffer));
}
*/

static uint8_t Comm_CalcCrc8(const uint8_t *i_pbyBuffer, int i_iSize)
{
	uint8_t byCRC = 0;

	for (uint32_t i = 0; i < i_iSize; i++)
		byCRC = k_abyCrcArray[(i_pbyBuffer[i] ^ byCRC)];

	return byCRC;
}

static uint16_t crc_C_Calc16(uint8_t *i_pbyBuf, uint16_t i_dwSize)
{/* CRC8 ve CRC16 Hesaplama Fonksiyonları */
	return crc_C_Update16(i_pbyBuf, i_dwSize, 0);
}

static uint16_t crc_C_Update16(uint8_t *i_pbyBuf, uint16_t i_dwSize, uint16_t i_wCrc)
{
	uint16_t byVal;
	while (i_dwSize != 0)
	{
		byVal = *i_pbyBuf++;
		i_wCrc = CRC_UPDATEBYTE16(byVal, i_wCrc);
		--i_dwSize;
	}

	return i_wCrc;
}

/*************** END OF FILE **************************************************/

/*************** NOTES ********************************************************
 * 1-
 ******************************************************************************/
