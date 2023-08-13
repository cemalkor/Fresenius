/******************************************************************************
 * Title                 :   Fresenius Kabi Agilira Range
 * Author                :   Cemal KÖR
 * Notes                 :   Informations in this document only refer to devices belonging to the Agilia Family
 *******************************************************************************/
#ifndef DEVICE_H_
#define DEVICE_H_
/******************************************************************************
 * Includes
 ******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************
 * Defines
 ******************************************************************************/
/* Variable Definitions */
#define UINT8Size				1
#define UINT16Size				2
#define UINT32Size				3

#define BYTE				uint8_t 		
#define ABYTE				uint8_t 		
#define STRING				uint8_t 		
#define SBYTE				int8_t				
#define WORD				uint16_t			
#define AWORD				uint16_t			
#define SWORD 				int16_t			
#define DWORD  				uint32_t			
#define SDWORD 				int32_t					
#define QWORD 				uint64_t			
#define UNIT 				UnitEncodingTypedefStruct	
#define DATETIME			uint32_t				

/*****************************************************************************/
#define REQUEST_LOG_ENABLE		0

#define MESSAGE_BUFFER_SIZE		300

#define	SPONTANEOUS_FRAMES		0x60
#define STANDARD_FRAMES			0x40

#define START_OF_FRAME			0x1B

#define HOST_WINDOW 			8
#define DEVICE_WINDOW 			8
/*
 * If there is no communication between the host and the device during the time
 * defined by PacketMinRate_ms, the host and the device send an empty frame to
 * each other to maintain the connection.
*/
#define RECOMMEND_PACKETMINRATE_MS 			20000
/*
 * Each frame that is emitted must be acknowledged. If the device which has
 * emitted a frame has not received an acknowledgement before the end of the
 * duration defined by PacketRepeatTimeout_ms, it emits the frame an other time.
*/
#define RECOMMEND_PACKETREPEATTIMEOUT_MS	5000
/*
 * If no data are received by the host from the device during the time
 * defined by DisconnectTimeout_ms, the session is automatically closed.
*/
#define RECOMMEND_DISCONNECTTIMEOUT_MS		0xFFFF

/* Commands And Message Description */
/* Funtional Mode */
#define IDLE_MODE_DEVICE_IN_IDLE_MODE 								0x01
#define THE_DEVICE_IS_SWITCHED_ON									0x02

/* Description */ /*Host (Computer,...) */
#define REQUEST_FOR_OPENING_CLOSING_A_COMMUNICATION_SESSION_HtoD	0x70
#define OPENING_CLOSING_A_COMMUNICATION_SESSION_DtoH				0xF0
#define REQUEST_FOR_NOP_HtoD										0x71
#define REPLY_FOR_NOP_DtoH											0xF1
#define NACK_DtoH													0xFF

/* Device Status */
#define REQUEST_FOR_DEVICE_STATUS_INFORMATION_HtoD					0x00
#define DEVICE_STATUS_INFORMATION_DtoH								0x80
#define REQUEST_FOR_ADVANCED_INFUSION_DATA_HtoD						0x5F
#define ADVANCED_INFUSION_DATA_DtoH									0xDF
#define REQUEST_FOR_DEVICE_INFORMATION_HtoD							0x40
#define DEVICE_INFORMATION_DtoH										0xC0
#define REQUEST_FOR_IP_ADDRESS_HtoD									0x64
#define IP_ADDRESS_DtoH												0xE4

/*Advanced infusion data List ID*/
#define ID_OF_THE_FLOWRATE											0x07
#define ID_OF_THE_INFUSED_VOLUME									0x0C
#define ID_OF_THE_MANUAL_BOLUS										0x0E
#define ID_OF_THE_SIMPLE_BOLUS										0x10
#define ID_OF_THE_LOADING_DOSE										0x4D
#define ID_OF_THE_PROGRAMMED_BOLUS									0x4E

/******************************************************************************
 * Macros
 ******************************************************************************/
#define		CLEAR_FLAGS_BUFFERS_AND_RETURN	CorretMsgFlag = 0; \
											memset(InfusPomp.MessageBuffer, 0, MESSAGE_BUFFER_SIZE); \
											InfusPomp.MsgCounter = 0; \
											return parse_flag;

/******************************************************************************
 * Typedefs
 ******************************************************************************/
typedef enum
{
	ERRCLOSE 		= 01,
	ERROPEN  		= 2,
	ERRID	 		= 3,
	ERRIDMODE		= 4,
	ERRIDSTATE		= 5,
	ERRFUNCID		= 6,
	ERRFUNCMODE		= 7,
	ERRFUNCSTATE	= 8,
	ERRDATAID		= 9,
	ERRVALIDMODE	= 10,
	ERRMODE			= 11,
	ERRSIZE			= 12,
	ERRSUBSIZE		= 13,
	ERRZONEID		= 14,
	ERRPARID		= 15,
	ERRPARINDEX		= 16,
	ERRLIMIT		= 17,
	ERRSTORING		= 18,
	ERRALARM		= 19,
	ERRSYSINGE		= 20,
	ERRSPONT		= 21,
	ERRDRUG 		= 0x1A,
	ERRPARSUBSIZE 	= 0x1B,
	ERRPARNOSUB 	= 0x1C,
	ERRPARFULL 		= 0x1D,
}ErrorCodesTypedefEnum;

typedef struct __attribute__((packed, aligned(2)))
{
	uint16_t Plasma : 1;
	uint16_t Dilution : 2;
	uint16_t Time : 2;
	uint16_t PatientsWeight : 1;
	uint16_t Unit : 3;
	uint16_t Prefix : 3;
	uint16_t Surcafe : 1;
	uint16_t reserved : 3;
} UnitEncodingTypedefStruct;

/* Data packet : START + TYPE + NO + ACK + SZ + CRC8 + ID + DATA(1,2.. 254) + CRC16 */
typedef struct
{
	uint8_t StartFrame; /*0x1B*/
	uint8_t Type;
	uint8_t FrameNumber;
	uint8_t FrameNumberACK;
	/* Size of the FrameID and DATA fields */
	uint8_t Size;
	/*CRC (8 bits) computed from the TYPE, NO, ACK and SZ fields*/
	uint8_t Crc8;
	/*Frame Id (if size is different from 0)*/
	uint8_t FrameId;
	/*CRC (16 bits) computed from the ID and DATA fields (if SZ is different from 0)*/
	uint16_t Crc16;

} AgiliaRangeSendDataTypedefStruct;

/* FRAME ID: 0x70 Request for opening/closing a communication session (HOST ? DEVICE) */
typedef struct
{
	/* Mode
	 * 00 : Closing the session
	 * 01 : Opening session in Full duplex mode
	 */
	BYTE Mode;
	BYTE HostWindow;
	BYTE DeviceWindow;
	BYTE DisconnectBehaviour; // Not used
	WORD PacketRepeatTimeout_ms;
	WORD PacketMinRate_ms;
	WORD DisconnectTimeout_ms;
} RequestForOpeningClosingCommunicationSessionTypedefStruct;

/* FRAME ID: 0xF0 Opening/closing a communication session (DEVICE ? HOST) */
typedef struct
{
	/* Mode
	 * 00 : Acknowledge for Closing the session
	 * 01 : Session opened in Full duplex mode
	 * 02 : Session opened in Half duplex mode
	 */
	BYTE Mode;
	BYTE HostWindow;
	BYTE DeviceWindow;
} ResponseForOpeningClosingCommunicationSessionTypedefStruct;

/* FRAME ID: 0x71 Request for NOP (HOST ? DEVICE) */
typedef struct
{
	DWORD dwID;
} RequestForNOPTypedefStruct;

/* FRAME ID: 0xF1 Reply for NOP (DEVICE ? HOST) */
typedef struct
{
	DWORD dwID;
} ResponseForNOPTypedefStruct;

/* FRAME ID: 0xFF NACK (DEVICE ? HOST) */
typedef struct
{
	BYTE FrameNo;
	WORD ErrorCode;
	DWORD AddData;
	BYTE *Frame;
} NACKandErrosTypedefStruct;
/*
 * The NACK frame is sent by the device for different reasons:
 * - The frame format is not correct
 * - The command can not be processed (bad parameters, command incompatible with the device state,)
 */

/* FRAME ID: 0x00  Request for Device status information (HOST ? DEVICE) */
typedef struct
{
	BYTE byMinDuration;
	BYTE byMaxDuration;
} RequestforDeviceStatusInformationTypedefStruct;

/* FRAME ID: 0x80  Device status information (DEVICE ? HOST) */
typedef struct // Link+ (all version id)
{
	/*
	 * Version ID:
	 * 0x02 : Link+ (old version)
	 * 0x12 : Link+ (new version with Alarm Management)
	 */
	BYTE Device;
	BYTE VersionOfTheBoot[2];
	BYTE VersionOfTheApp[2];
	BYTE SerialNo[16];
	BYTE Alarms[4];
	BYTE dummyData[11];
	BYTE InfMode;
	DWORD Flowrate;
	BYTE drugName[21];
	BYTE InfusedVolume[4];
	BYTE dummyData2[10];
	BYTE LeftVolume[4];
	/*
	 * 0xFF for boot
	 * 0x02 for application
	 */
	/*
	BYTE Mode;
	BYTE NbSlots;
	WORD PresentDevices;
	WORD Reserved1;
	WORD AlarmActive;
	BYTE Reserved2;
	BYTE Reserved3;
	BYTE Reserved4;
	*/
} DeviceStatusInformationTypedefStruct;

/* First Block of Data : Injectomat Agilia */
typedef struct
{
	BYTE Device;
	STRING Version[4];
	STRING SerialNo[16];
	DWORD Alarm;
	BYTE Error;
	BYTE SilalCnt;
	DWORD Internal;
	BYTE OperMode;
	BYTE Status;
	/*
	 * Injectomat, not present if
	 * OperMode=0x1 and Status=0x4
	 */
	BYTE Step;
	/*
	 * On Injectomat, not present if
	 * OperMode=0x1 and Status=0x4
	 */
	WORD Status2;
	/*
	 * On Injectomat, not present if
	 * OperMode=0x1 and Status=0x4
	 */
	BYTE Mode;
} FirstBlockOfDataInjectomatAgiliaTypedefStruct;

/* First Block of Data : Volumat Agilia */
typedef struct
{
	BYTE Device;
	STRING Version[4];
	STRING SerialNo[16];
	BYTE Alarm[4];
	BYTE Error;
	BYTE SilalCnt;
	DATETIME Internal;
	BYTE OperMode;
	BYTE Status;
	BYTE StepStatus;
	BYTE SumStatus;
	BYTE InfusMode[5];
} FirstBlockOfDataVolumatAgiliaTypedefStruct;

/*4.4.2 Volumat and Injectomat Device status SubId*/
/*4.4.2.1 Patient SubId*/
typedef struct // SubId 01: Patient
{
	BYTE PatientZoneSize;
	BYTE SubIdPatient;
	DATETIME CreatePatient;
	STRING PatientLastName[30];
	STRING PatientFirstName[30];
	DWORD PatientWeight;
	WORD PatientAge;
	BYTE PatientGender;
	STRING PatientCode[30];
	WORD PatientHeight;
	WORD Bsa;
} PatientSubIdTypedefStruct;

/*4.4.2.2 Infusion SubId*/
typedef struct // SubId 02: Infusion
{
	BYTE SubIdInfusionZoneSize;
	BYTE SubIdInfusion;
	BYTE FlowRate[4];
	STRING DrugName[20];
	UNIT FlowRateUnit;
	DWORD Volume;
	WORD Pressure;
	DWORD Dilution;
	UNIT DilutionUnit;
	// WORD 		VolDilution;
	BYTE LeftVolume[4];
	BYTE SyringeSize;
	STRING SyringeNameorTubingSetName[20];
	DWORD UserVolume;
	DWORD UserMass;
	UNIT UserMassUnit;
	DWORD UserVolumePri;
	DWORD UserVolumeSec;
} InfusionSubIdTypedefStruct;

// 4.4.2.3 TCI SubId
typedef struct // SubId 04: TCI
{
	BYTE TciZoneSize;
	BYTE SubIdTci;
	DWORD Plasma;
	DWORD Effect;
	DWORD Target;
	UNIT Unit;
	BYTE TciMode;
} TCISubIdTypedefStruct;

// 4.4.2.4 Device SubId
typedef struct // SubId 05: Device
{
	BYTE DeviceZoneSize;
	BYTE SubIdDevice;
	BYTE SubDevice;
} DeviceSubIdTypedefStruct;
/*
 * Note on Alarmactive:
 * If an alarm on the device is set and the device is beeping, then
 * the rack will set the corresponding bit to 1.
 * If an alarm on the device is set and the device is not beeping (because of
 * SILAL key pressed), then the rack will set the corresponding bit to 0.
 */
/* 4.6.1
 * Advanced data infusion reading
 * The following frame can be sent by the host to the device to get advanced
 * infusion data such as bolus volumes (simple, manual, programmed, loading dose)
 * and infused volume.
 * The values in these frames shall not be used when the pump is
 * in idle mode: OFF / OperMode=0x1 and Status=0x4 (see 4.4.1.1 and 4.4.1.2)
 * FRAME ID: 0x5F  Request for advanced infusion data (HOST -> DEVICE)
 * FRAME ID: 0xDF  Advanced infusion data (DEVICE -> HOST)
 */

/* FRAME ID : 0x40  Request for device information (HOST ? DEVICE) */
typedef struct
{
	BYTE DataId; // DataId = 1
} RequestForDeviceInformationTypedefStruct;

/* FRAME ID : 0xC0  Device information (DEVICE ? HOST) */
typedef struct
{
	BYTE Size;
	BYTE DataId;
	ABYTE Data[30]; //(Size  1) ???
} ResponseForDeviceInformationTypedefStruct;

/* FRAME ID: 0x5F  Request for advanced infusion data (HOST ? DEVICE) */
typedef struct
{
	BYTE DataID;
	BYTE MinDuration;
	BYTE MaxDuration;
} RequestForAdvancedInfusionDataTypedefStruct;

/* Note : If this frame is sent without parameter, no reply is sent by the device and all spontaneous frames ID 0x5F are no more sent */
/* FRAME ID: 0xDF  Advanced infusion data (DEVICE ? HOST) */
typedef struct
{
	DWORD SampleTime;
	BYTE Size;
	BYTE DataID;
	BYTE Data[15];
} ResponseForAdvancedInfusionDataTypedefStruct;

typedef struct
{
	/* ID of the flowrate: always 0x07 */
	BYTE Flowrate_;
	SDWORD Data;
} FlowrateTypedefStruct;

typedef struct
{
	/*ID of the infused volume: always 0x0C*/
	BYTE InfusedVolume;
	DWORD Data;
	DWORD Mass;
	UNIT MassUnit;
} InfusedVolumeTypedefStruct;

typedef struct
{
	/*ID of the manual bolus: always 0x0E*/
	BYTE ManualBolus;
	DWORD VolBolus;
	DWORD VolTot;
	DWORD Mass;
	UNIT MassBolusUnit;
} ManualBolusTypedefStruct;

typedef struct
{
	/* ID of the simple bolus: always 0x10 */
	BYTE SimpleBolus;
	DWORD Flowrate;
	DWORD Volume;
	DWORD Mass;
	UNIT MassBolusUnit;
} SimpleBolusTypedefStruct;

typedef struct
{
	/* ID of the loading dose: always 0x4D */
	BYTE LoadingDose;
	DWORD FlowRate;
	DWORD Volume;
	DWORD Duration;
	DWORD MassToInfus;
	UNIT MassToInfusUnit;
} LoadingBolusTypedefStruct;

typedef struct
{
	/* ID of the programmed bolus: always 0x4E */
	BYTE ProgrammedBolus;
	DWORD FlowRate;
	DWORD Volume;
	DWORD Duration;
	DWORD MassToInfus;
	UNIT MassToInfusUnit;
} ProgrammedBolusTypedefStruct;

/*
 * The following frame is used to retrieve the device IP address.
 * The following frame is only available for Link+, Version ID 0x12,
 * Software Version >= BW_rel_D8
 */
typedef struct
{
	char IPAddr[15];
} IPAddressRcvTypedefStruct;

typedef struct
{
	char IPAddr[15];
} IPAddressSndTypedefStruct;

/* FreseniusKabiAgiliaRangeTypedefStruct */
typedef struct
{
	ResponseForOpeningClosingCommunicationSessionTypedefStruct ResponseForOpeningClosingComm;
	ResponseForNOPTypedefStruct ResponseForNOP;
	NACKandErrosTypedefStruct NACKandErr;
	DeviceStatusInformationTypedefStruct DeviceStatusInfo;
	FirstBlockOfDataInjectomatAgiliaTypedefStruct FirstBlockOfDataInjectomatAgilia;

	FirstBlockOfDataVolumatAgiliaTypedefStruct FirstBlockOfDataVolumatAgilia;
	PatientSubIdTypedefStruct PatientSubId;
	InfusionSubIdTypedefStruct InfusionSubId;

	TCISubIdTypedefStruct TCISubId;
	DeviceSubIdTypedefStruct DeviceSubId;
	RequestForDeviceInformationTypedefStruct RequestForDeviceInfo;
	ResponseForDeviceInformationTypedefStruct ResponseForDeviceInfo;
	RequestForAdvancedInfusionDataTypedefStruct RequestForAdvancedInfusionData;
	ResponseForAdvancedInfusionDataTypedefStruct ResponseForAdvancedInfusionData;
	FlowrateTypedefStruct Flowrate;
	InfusedVolumeTypedefStruct InfusedVolume;
	ManualBolusTypedefStruct ManualBolus;
	SimpleBolusTypedefStruct SimpleBolus;
	LoadingBolusTypedefStruct LoadingBolus;
	ProgrammedBolusTypedefStruct ProgrammedBolus;
	IPAddressRcvTypedefStruct IPAddress;
	ErrorCodesTypedefEnum ErrorCodes;
	uint8_t MessageBuffer[MESSAGE_BUFFER_SIZE];
	uint32_t MsgCounter;
} FreseniusKabiAgiliaRangeTypedefStruct;

/******************************************************************************
 * Externs
 ******************************************************************************/
extern FreseniusKabiAgiliaRangeTypedefStruct InfusPomp;

/******************************************************************************
 * Function Prototypes
 ******************************************************************************/
uint8_t ParseMessage(uint8_t Data);

/******************************************************************************/
#ifdef __cplusplus
} // extern "C"
#endif

#endif /*File_H_*/

/*** END OF FILE **************************************************************/

/*************** NOTES ********************************************************
 * -> 1- Serial
 *	The serial communication uses the standard following
 *  configuration (cannot be modified):
 *	Baud rate : 115200
 *	Data bits : 8
 *	Parity bit : no
 *	Stop bit: yes (1 stop bit)
 *	RTS (Request To Send) : enabled
 *	DTR (Data Terminal Ready) : enabled
 * -> 2- TCP
 *	The TCP communication uses the standard following configuration:
 *	IP of the Link : configured in WebInterface ( default 192.168.0.1 )
 *	Protocol : TCP
 *	Port : configured in WebInterface ( default 52000 )
 * -> 3- The developed application/driver communicating with Agilia devices shall
 * not request more than 10 frames per second, spread over the connected pumps.
 * -> 4- Start of communication
 * All communication between the host and the device must begin with the
 * opening of a communication session between them (frame ID: 0x70).
 * Any Message sent before the session is opened generates an error Message.
 * Only the device can send spontaneous Messages out of a communication session.
 * These Messages are not sent during the communication session. The emitted frames
 * out of a communication session are not numbered (number of frame = 0)
 * -> 5- End of communication
 * Similarly to the opening of a communication session, when the host wishes to
 * end communication, it must close the communication session (frame ID: 0x70).
 * The session is automatically closed in the following cases:
 * & Physical disconnection of the device
 * & No reply of the device during DisconnectTimeout_ms
 ******************************************************************************/
