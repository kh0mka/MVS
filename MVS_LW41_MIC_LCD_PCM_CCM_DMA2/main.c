#include <stdint.h>
#include <string.h>
#include "stm32f4xx_conf.h"

#define ABS(x)		((x) < 0) ? -(x) : (x)
#define MIN(x,y)		((x) < (y)) ? (x) : (y)
#define MAX(x,y)		((x) > (y)) ? (x) : (y)

#define NVIC_GROUP	NVIC_PriorityGroup_0
/*----- Codec control interface -----*/
// I2C1_SDA	PB9
// I2C1_SCL	PB6
/* Codec audio interface -----*/
// I2S3_MCK	PC7
// I2S3_SCK	PC10
// I2S3_SD	PC12
// I2S3_WS	PA4
/* Codec reset pin	PD4 -----*/
/* MEMS microphone interface -----*/
// I2S2_PDM_OUT	PC3
// I2S3_CLK_IN	PB10
/* The 7 bits Codec address (sent through I2C interface) */
#define CODEC_ADDRESS		0x94
/* Mask for the bit EN of the I2S CFGR register */
#define I2S_ENABLE_MASK	0x0400
#define TIMEOUT_MAX		10000
#define CODEC_I2C_SCL_RCC		RCC_AHB1Periph_GPIOB
#define CODEC_I2C_SCL_PORT		GPIOB
#define CODEC_I2C_SCL_PINSOURCE	GPIO_PinSource6
#define CODEC_I2C_SCL_AF		GPIO_AF_I2C1
#define CODEC_I2C_SDA_RCC		RCC_AHB1Periph_GPIOB
#define CODEC_I2C_SDA_PORT		GPIOB
#define CODEC_I2C_SDA_PINSOURCE	GPIO_PinSource9
#define CODEC_I2C_SDA_AF		GPIO_AF_I2C1
#define CODEC_I2S_MCK_RCC		RCC_AHB1Periph_GPIOC
#define CODEC_I2S_MCK_PORT		GPIOC
#define CODEC_I2S_MCK_PIN		GPIO_Pin_7
#define CODEC_I2S_MCK_PINSOURCE	GPIO_PinSource7
#define CODEC_I2S_MCK_AF		GPIO_AF_SPI3
#define CODEC_I2S_SCK_RCC		RCC_AHB1Periph_GPIOC
#define CODEC_I2S_SCK_PORT		GPIOC
#define CODEC_I2S_SCK_PIN		GPIO_Pin_10
#define CODEC_I2S_SCK_PINSOURCE	GPIO_PinSource10
#define CODEC_I2S_SCK_AF		GPIO_AF_SPI3
#define CODEC_I2S_SD_RCC		RCC_AHB1Periph_GPIOC
#define CODEC_I2S_SD_PORT		GPIOC
#define CODEC_I2S_SD_PIN		GPIO_Pin_12
#define CODEC_I2S_SD_PINSOURCE	GPIO_PinSource12
#define CODEC_I2S_SD_AF		GPIO_AF_SPI3
#define CODEC_I2S_WS_RCC		RCC_AHB1Periph_GPIOA
#define CODEC_I2S_WS_PORT		GPIOA
#define CODEC_I2S_WS_PIN		GPIO_Pin_4
#define CODEC_I2S_WS_PINSOURCE	GPIO_PinSource4
#define CODEC_I2S_WS_AF		GPIO_AF_SPI3
#define CODEC_RESET_RCC		RCC_AHB1Periph_GPIOD
#define CODEC_RESET_PORT		GPIOD
#define CODEC_RESET_PIN		GPIO_Pin_4
#define CODEC_I2S_RCC		RCC_APB1Periph_SPI3
#define CODEC_I2S_PORT		SPI3
#define CODEC_I2C_RCC		RCC_APB1Periph_I2C1
#define CODEC_I2C_PORT		I2C1
#define CODEC_DMA_RCC		RCC_AHB1Periph_DMA1
#define CODEC_DMA_STREAM		DMA1_Stream7
#define CODEC_DMA_FLAG		DMA_FLAG_TCIF7
#define CODEC_DMA_CHANNEL		DMA_Channel_0
#define CODEC_IRQ_CHANNEL		DMA1_Stream7_IRQn
#define MEMS_MIC_CLKIN_RCC		RCC_AHB1Periph_GPIOB
#define MEMS_MIC_CLKIN_PORT	GPIOB
#define MEMS_MIC_CLKIN_PIN		GPIO_Pin_10
#define MEMS_MIC_CLKIN_PINSOURCE	GPIO_PinSource10
#define MEMS_MIC_CLKIN_AF		GPIO_AF_SPI2
#define MEMS_MIC_PDMOUT_RCC	RCC_AHB1Periph_GPIOC
#define MEMS_MIC_PDMOUT_PORT	GPIOC
#define MEMS_MIC_PDMOUT_PIN	GPIO_Pin_3
#define MEMS_MIC_PDMOUT_PINSOURCE			GPIO_PinSource3
#define MEMS_MIC_PDMOUT_AF	GPIO_AF_SPI2
#define MEMS_MIC_I2S_RCC		RCC_APB1Periph_SPI2
#define MEMS_MIC_I2S_PORT		SPI2
#define MEMS_MIC_DMA_RCC		RCC_AHB1Periph_DMA1
#define MEMS_MIC_DMA_STREAM	DMA1_Stream3
#define MEMS_MIC_DMA_FLAG_HT	DMA_FLAG_HTIF3
#define MEMS_MIC_DMA_FLAG_TC	DMA_FLAG_TCIF3
#define MEMS_MIC_DMA_CHANNEL	DMA_Channel_0
#define MEMS_MIC_IRQ_CHANNEL	DMA1_Stream3_IRQn
volatile int32_t ITM_RxBuffer;
RCC_ClocksTypeDef RCC_Clocks;
PDMFilter_InitStruct Filter;
uint16_t AudioBuffer [ INTERNAL_BUFF_SIZE ];
uint16_t RecBuf [ PCM_OUT_SIZE ];
extern const unsigned char gImage_BaseImage[];
unsigned short BaseImage_coor[] = {0, 0, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT};
#define LCD_BASE_Data		((u32)(0x60000000|0x00100000))
#define LCD_BASE_Addr		((u32)(0x60000000|0x00000000))
#define FRAME_SIZE	(LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT)
#define CCMDATA_SIZE       (CCMDATARAM_END - CCMDATARAM_BASE - 1)
#define DATA_SHIFT			5
uint16_t * lcdFrame_1 = (uint16_t *)CCMDATARAM_BASE; // 64 KB
uint16_t lcdFrame_2 [ FRAME_SIZE - CCMDATA_SIZE / 2 ]; // 88066 bytes

#define GRAPH_WIDTH		256
uint16_t pcmGraphCoor[] = {49, 77, GRAPH_WIDTH, 128};

/* Temporary data sample */
static uint16_t InternalBuffer [ 2 * INTERNAL_BUFF_SIZE ];

#define DMA_BUFFER_SIZE		(LCD_PIXEL_WIDTH * 1)
uint16_t DmaFrame [ DMA_BUFFER_SIZE ];
static uint32_t g_FrameTotalBytesCopied = 0;
void Delay ( uint32_t nTime );
void CameraDisable(void);
void MEMS_MicCtrlLinesConfig(void);
void MEMS_MicI2SConfig(void);
void MEMS_MicDMAConfig(void);
void MEMS_MicDMA_NVIConfig(void);
void MEMS_MicInit(void);
uint8_t CODEC_I2C_SingleRandomWrite ( I2C_TypeDef* I2Cx,
	uint8_t Device, uint16_t Addr, uint8_t Data );
uint8_t CODEC_I2C_SingleRandomRead ( I2C_TypeDef* I2Cx,
	uint8_t Device, uint16_t Addr );
uint32_t CODEC_VolumeCtrl ( uint8_t Volume );
void CODEC_CtrlLinesConfig(void);
void CODEC_I2CConfig(void);
void CODEC_I2SConfig(void);
void CODEC_Config(void);
void CODEC_DMAConfig(void);
void CODEC_DMA_NVIConfig(void);
void CODEC_Init(void);
void FRAME_DmaToLcd ( void );
void FRAME_DrawLine ( uint16_t Xpos,
	uint16_t Ypos, uint16_t Length,
	uint8_t Direction );
void FRAME_RestoreArea (uint16_t graphRectCoors[4], uint16_t * pBaseImage );
void MIC_FRAME_DrawGraph (
	int16_t * dataArray, uint16_t dataArraySize,
	uint16_t graphRectCoors[4]);
void DMA_LCD_Config ( void );
#define PCM_BUFFER_LEN	(256)
static int16_t g_micPCM[PCM_BUFFER_LEN + 1];
#define PCM_QUEUE_LEN	(64)
static int16_t g_FIFO[PCM_QUEUE_LEN];
static uint16_t g_FifoBegin = 0;
static uint16_t g_FifoEnd = 0;
/* @brief Read next data from FIFO.
 * @param  pData: pointer to data be read
 * @retval Read status: SUCCESS if all OK or ERROR when nothing read */
ErrorStatus FIFO_GetNextData ( int16_t * pData ) {
	ErrorStatus res = ERROR;
	if (g_FifoBegin != g_FifoEnd) {
		*pData = g_FIFO[g_FifoBegin++];
		g_FifoBegin %= PCM_QUEUE_LEN;
		res = SUCCESS;	}
	return res;}
int main ( void ) {
	/* SysTick end of count event each 1 ms */
	RCC_GetClocksFreq ( &RCC_Clocks );
	SysTick_Config (RCC_Clocks.HCLK_Frequency / 1000);
	BaseImage_coor [2] = *(uint16_t*)(gImage_BaseImage + 2);
	BaseImage_coor [3] = *(uint16_t *)(gImage_BaseImage + 4);
	/* Отключение камеры, на всякий случай */
	CameraDisable();
	/* Инициализация видео канала	*/
	STM32f4_Discovery_LCD_Init();
         LCD_SetColors ( LCD_COLOR_MAGENTA, LCD_COLOR_BLACK );
	uint32_t framePixelIndex;
      for (framePixelIndex = 0; framePixelIndex < FRAME_SIZE; framePixelIndex++) {
		if (framePixelIndex <= CCMDATA_SIZE / 2) {
lcdFrame_1[framePixelIndex] = *((uint16_t *)(gImage_BaseImage + 8) + framePixelIndex);
		} else {
lcdFrame_2 [ framePixelIndex - CCMDATA_SIZE / 2 - 1 ] =
	*((uint16_t *)(gImage_BaseImage + 8) + framePixelIndex);	}}
	DMA_LCD_Config();
	/* Инициализация аудио канала */
	/* Enable CRC module */
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_CRC, ENABLE );
	/* Filter LP & HP Init */
	Filter.LP_HZ = 0;
	Filter.HP_HZ = 0;
	Filter.Fs = I2S_AudioFreq_16k;
	Filter.Out_MicChannels = 1;
	Filter.In_MicChannels = 1;
	PDM_Filter_Init((PDMFilter_InitStruct *) &Filter);
	/* Инициализация аудио канала записи */
	MEMS_MicInit();
	/* Инициализация аудио канала воспроизведения */
	CODEC_Init();
	uint32_t tick = GetTickCount();
	while (1) {
		static int16_t iMicPCMBufferLen = 0;
		int16_t data;
		/* Обновление линейного буфера */
		while (FIFO_GetNextData ( &data ) == SUCCESS) {
			g_micPCM[iMicPCMBufferLen++] = data;
			if (iMicPCMBufferLen > PCM_BUFFER_LEN) {
		memmove ( g_micPCM, g_micPCM + 1, PCM_BUFFER_LEN * sizeof(int16_t));
				iMicPCMBufferLen--;	}}
		/* Обновление сигналов на дисплее каждые 50 мс */
		if (GetTickCount() - tick > 50) {
			tick = GetTickCount();
		FRAME_RestoreArea ( pcmGraphCoor, (uint16_t *) (gImage_BaseImage + 8) );
			MIC_FRAME_DrawGraph ( g_micPCM, PCM_BUFFER_LEN, pcmGraphCoor);
			FRAME_DmaToLcd();
		}}
	return 0;}

/* ----- Camera disable function ----- */
void CameraDisable ( void ) {
	GPIO_InitTypeDef		GPIO_InitStruct;
RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );
	/* PD6 <--> Camera PWR_EN == 1 (DISABLE) */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init ( GPIOD, &GPIO_InitStruct );
	/*PD.6 = 1 (PWR_EN == 1, i.e. disable camera) */
	GPIO_SetBits ( GPIOD, GPIO_Pin_6 );
	/* PD12 <--> Camera RESET == 0 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init ( GPIOD, &GPIO_InitStruct );
	/* PD.12 = 0 (/RST camera) */
	GPIO_ResetBits ( GPIOD, GPIO_Pin_12 );	}
/**
  * @brief  Writes a byte at a specific Codec register
  * @param  Device: device address
  * @param  Addr: register address
  * @param  Data: data to be written to the specific register
  * @retval 0x00 if write operation is OK
  *         0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t CODEC_I2C_SingleRandomWrite ( I2C_TypeDef* I2Cx, uint8_t Device, uint16_t Addr, uint8_t Data ) {
	uint32_t timeout = TIMEOUT_MAX;
	/* Generate the Start Condition */
	I2C_GenerateSTART ( I2Cx, ENABLE );
	/* Test on I2Cx EV5 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_MODE_SELECT )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF; 	}
	/* Send Codec selected device slave Address for write */
	I2C_Send7bitAddress ( I2Cx, Device, I2C_Direction_Transmitter );
	/* Test on I2Cx EV6 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
while (!I2C_CheckEvent (I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}
	/* Send I2Cx location address LSB */
	I2C_SendData ( I2Cx, (uint8_t) (Addr) );
	/* Test on I2Cx EV8 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
while (!I2C_CheckEvent (I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;	}
/* Send Data */
I2C_SendData ( I2Cx, Data );
/* Test on I2Cx EV8 and clear it */
timeout = TIMEOUT_MAX; /* Initialize timeout value */
while (!I2C_CheckEvent (I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
  /* If the timeout delay is exceeded, exit with error code */
	if ((timeout--) == 0)
		return 0xFF;
}
/* Send I2Cx STOP Condition */
I2C_GenerateSTOP ( I2Cx, ENABLE );
/* If operation is OK, return 0 */
return 0;}
/**
  * @brief  Reads a byte from a specific Codec register
  * @param  Device: device address
  * @param  Addr: register address
  * @retval data read from the specific register or 0xFF if timeout
  * condition occured
  */
uint8_t CODEC_I2C_SingleRandomRead ( I2C_TypeDef* I2Cx, uint8_t Device, uint16_t Addr ) {
	uint32_t timeout = TIMEOUT_MAX;
	uint8_t Data = 0;
	/* Generate the Start Condition */
	I2C_GenerateSTART ( I2Cx, ENABLE );
	/* Test on I2Cx EV5 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_MODE_SELECT )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}
	/* Send Codec selected device slave Address for write */
	I2C_Send7bitAddress ( I2Cx, Device, I2C_Direction_Transmitter );
	/* Test on I2Cx EV6 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}
	/* Send I2Cx location address LSB */
	I2C_SendData ( I2Cx, (uint8_t) (Addr) );
	/* Test on I2Cx EV8 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}
	/* Clear AF flag if arised */
	I2Cx->SR1 |= (uint16_t) 0x0400;
	/* Generate the Stop Condition */
	I2C_GenerateSTOP ( I2Cx, ENABLE );
	/* Generate the Start Condition */
	I2C_GenerateSTART ( I2Cx, ENABLE );
	/* Test on I2Cx EV6 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_MODE_SELECT )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}
	/* Send Codec selected device slave Address for write */
	I2C_Send7bitAddress ( I2Cx, Device, I2C_Direction_Receiver );
	/* Test on I2Cx EV6 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED )) {
		/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}
	/* Prepare an NACK for the next data received */
	I2C_AcknowledgeConfig ( I2Cx, DISABLE );
	/* Test on I2Cx EV7 and clear it */
	timeout = TIMEOUT_MAX; /* Initialize timeout value */
	while (!I2C_CheckEvent ( I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED )) {
	/* If the timeout delay is exceeded, exit with error code */
		if ((timeout--) == 0)
			return 0xFF;
	}
	/* Prepare Stop after receiving data */
	I2C_GenerateSTOP ( I2Cx, ENABLE );
	/* Receive the Data */
	Data = I2C_ReceiveData ( I2Cx );
	/* return the read data */
	return Data;}
/* @brief  Sets higher or lower the codec volume level.
  * @param  Volume: a byte value from 0 to 255 (refer to codec registers
  *         description for more details).
  * @retval 0 if correct communication, else wrong communication  */
uint32_t CODEC_VolumeCtrl ( uint8_t Volume ) {
	uint32_t counter = 0;
	if ( Volume > 0xE6 ) {
		/* Set the Master volume */
		counter += CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x20, Volume - 0xE7 );
		counter += CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x21, Volume - 0xE7 );
	} else { /* Set the Master volume */
		counter += CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x20, Volume + 0x19 );
		counter += CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x21, Volume + 0x19 ); }
	return counter;	}
/* @brief  This function handles AUDIO_REC_SPI global interrupt request.
  * @param  None
  * @retval None */
void MEMS_MIC_DMA_STREAM_IRQHandler ( void ) {
uint16_t * pInternalBuffer_s = InternalBuffer, // Начальный адрес фильтруемой части массива
	* pInternalBuffer;
    if (DMA_GetFlagStatus ( MEMS_MIC_DMA_STREAM, MEMS_MIC_DMA_FLAG_TC ) != RESET) {
		pInternalBuffer_s += INTERNAL_BUFF_SIZE;	}
       DMA_ClearFlag ( MEMS_MIC_DMA_STREAM, MEMS_MIC_DMA_FLAG_HT | MEMS_MIC_DMA_FLAG_TC );
	pInternalBuffer = pInternalBuffer_s;
	uint8_t i = INTERNAL_BUFF_SIZE;
	do {
		uint16_t app = *pInternalBuffer;
		*pInternalBuffer++ = HTONS ( app );
	} while (--i);
	/* 22.6 mcs (44100 Hz) */
	PDM_Filter_64_LSB ( (uint8_t *) pInternalBuffer_s, RecBuf, 10,	&Filter );
}
/* @brief  This function handles main Media layer interrupt.
  * @param  None
  * @retval 0 if correct communication, else wrong communication */
void CODEC_DMA_STREAM_IRQHandler ( void ) {
	uint8_t i;
	uint16_t audioData;
	/* Transfer complete interrupt */
        if (DMA_GetFlagStatus ( CODEC_DMA_STREAM, CODEC_DMA_FLAG ) != RESET) {
		/* Disable the I2S DMA Stream*/
		DMA_Cmd ( CODEC_DMA_STREAM, DISABLE );
		/* Clear the Interrupt flag */
		DMA_ClearFlag ( CODEC_DMA_STREAM, CODEC_DMA_FLAG );
/* Manage the remaining file size and new address offset: This function
 *   should be coded by user (its prototype is already declared in stm32f4_discovery_audio_codec.h) */
		for (i = 0; i < (INTERNAL_BUFF_SIZE / 2); i++) {
			audioData = RecBuf[i >> 1];
			AudioBuffer[i] = audioData;		}
		/* Write to FIFO */
		g_FIFO[g_FifoEnd++] = audioData;
		g_FifoEnd %= PCM_QUEUE_LEN;
		/* Enable the I2S DMA Stream */
		DMA_Cmd ( CODEC_DMA_STREAM, ENABLE );	}}
void MEMS_MicCtrlLinesConfig(void) {
	GPIO_InitTypeDef	GPIO_InitStruct;
	/* PC3, PB10 clock enable */
     RCC_AHB1PeriphClockCmd ( MEMS_MIC_CLKIN_RCC | MEMS_MIC_PDMOUT_RCC, ENABLE );
	/* I2S3_CLK_IN (PB10) for MEMS microphone */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = MEMS_MIC_CLKIN_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init ( MEMS_MIC_CLKIN_PORT, &GPIO_InitStruct );
	GPIO_PinAFConfig ( MEMS_MIC_CLKIN_PORT,
			MEMS_MIC_CLKIN_PINSOURCE, MEMS_MIC_CLKIN_AF );
	/* I2S2_PDM_OUT (PC3) for MEMS microphone */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = MEMS_MIC_PDMOUT_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init ( MEMS_MIC_PDMOUT_PORT, &GPIO_InitStruct );
	GPIO_PinAFConfig ( MEMS_MIC_PDMOUT_PORT,
		MEMS_MIC_PDMOUT_PINSOURCE, MEMS_MIC_PDMOUT_AF );}
void MEMS_MicI2SConfig(void) {
	I2S_InitTypeDef	I2S_InitStruct;
	// I2S2 (SPI2) initialization for MEMS microphone
	RCC_APB1PeriphClockCmd (MEMS_MIC_I2S_RCC, ENABLE );
	I2S_InitStruct.I2S_AudioFreq = Filter.Fs;
	I2S_InitStruct.I2S_CPOL = I2S_CPOL_Low;
	I2S_InitStruct.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStruct.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	I2S_InitStruct.I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStruct.I2S_Standard = I2S_Standard_LSB;
	I2S_Init (MEMS_MIC_I2S_PORT, &I2S_InitStruct );
	I2S_Cmd (MEMS_MIC_I2S_PORT, ENABLE );
}
void MEMS_MicDMAConfig(void) {
	static DMA_InitTypeDef	DMA_InitStructureMic;
	/* Enable DMA1 clock */
	RCC_AHB1PeriphClockCmd ( MEMS_MIC_DMA_RCC, ENABLE );
	/* DMA1 Stream3 Configuration */
	DMA_DeInit ( MEMS_MIC_DMA_STREAM );
	DMA_InitStructureMic.DMA_Channel = MEMS_MIC_DMA_CHANNEL;
         DMA_InitStructureMic.DMA_PeripheralBaseAddr = (uint32_t)&MEMS_MIC_I2S_PORT->DR;
	DMA_InitStructureMic.DMA_Memory0BaseAddr = (uint32_t) InternalBuffer;
	DMA_InitStructureMic.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructureMic.DMA_BufferSize = 2 * INTERNAL_BUFF_SIZE;
	DMA_InitStructureMic.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructureMic.DMA_MemoryInc = DMA_MemoryInc_Enable;
          DMA_InitStructureMic.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
               DMA_InitStructureMic.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructureMic.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructureMic.DMA_Priority = DMA_Priority_Medium;
	DMA_Init ( MEMS_MIC_DMA_STREAM, &DMA_InitStructureMic );
	/* Enable DMA transfer */
	DMA_Cmd ( MEMS_MIC_DMA_STREAM, ENABLE );
	SPI_I2S_DMACmd ( MEMS_MIC_I2S_PORT, SPI_I2S_DMAReq_Rx, ENABLE );}
void MEMS_MicDMA_NVIConfig(void) {
	NVIC_InitTypeDef	NVIC_InitStruct;
      DMA_ITConfig ( MEMS_MIC_DMA_STREAM, DMA_IT_HT | DMA_IT_TC, ENABLE );
	/* Configure the SPI interrupt priority */
NVIC_InitStruct.NVIC_IRQChannel = MEMS_MIC_IRQ_CHANNEL;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init ( &NVIC_InitStruct ); }

void MEMS_MicInit(void) {
	MEMS_MicCtrlLinesConfig();
	MEMS_MicI2SConfig();
	MEMS_MicDMAConfig();
	MEMS_MicDMA_NVIConfig(); }
void CODEC_CtrlLinesConfig(void) {
	GPIO_InitTypeDef		GPIO_InitStruct;
	/* PA4, PB6, PB9, PC7, PC10, PC12, PD4 clock enable */
	RCC_AHB1PeriphClockCmd ( CODEC_I2C_SCL_RCC | CODEC_I2C_SDA_RCC |
	          CODEC_I2S_MCK_RCC | CODEC_I2S_SCK_RCC | CODEC_I2S_SD_RCC |
	          CODEC_I2S_WS_RCC | CODEC_RESET_RCC, ENABLE );
	/* I2C1_SCL (PB6) */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = CODEC_I2C_SCL_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init ( CODEC_I2C_SCL_PORT, &GPIO_InitStruct );
GPIO_PinAFConfig(CODEC_I2C_SCL_PORT, CODEC_I2C_SCL_PINSOURCE, CODEC_I2C_SCL_AF);
	/* I2C1_SDA (PB9) */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Pin = CODEC_I2C_SDA_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init ( CODEC_I2C_SDA_PORT, &GPIO_InitStruct );
GPIO_PinAFConfig(CODEC_I2C_SDA_PORT, CODEC_I2C_SDA_PINSOURCE, CODEC_I2C_SDA_AF);
/* Codec I2S3_MCK (PC7) */
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitStruct.GPIO_Pin = CODEC_I2S_MCK_PIN;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
GPIO_Init ( CODEC_I2S_MCK_PORT, &GPIO_InitStruct );
GPIO_PinAFConfig(CODEC_I2S_MCK_PORT, CODEC_I2S_MCK_PINSOURCE, CODEC_I2S_MCK_AF);
/* Codec I2S3_SCK (PC10) */
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitStruct.GPIO_Pin = CODEC_I2S_SCK_PIN;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
GPIO_Init ( CODEC_I2S_SCK_PORT, &GPIO_InitStruct );
GPIO_PinAFConfig(CODEC_I2S_SCK_PORT, CODEC_I2S_SCK_PINSOURCE,  CODEC_I2S_SCK_AF);
/* Codec I2S3_SD (PC12) */
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitStruct.GPIO_Pin = CODEC_I2S_SD_PIN;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
GPIO_Init ( CODEC_I2S_SD_PORT, &GPIO_InitStruct );
GPIO_PinAFConfig ( CODEC_I2S_SD_PORT, CODEC_I2S_SD_PINSOURCE,  CODEC_I2S_SD_AF );
/* I2S3_WS (PA4) */
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitStruct.GPIO_Pin = CODEC_I2S_WS_PIN;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
GPIO_Init ( CODEC_I2S_WS_PORT, &GPIO_InitStruct );
GPIO_PinAFConfig ( CODEC_I2S_WS_PORT, CODEC_I2S_WS_PINSOURCE,  CODEC_I2S_WS_AF );
/* Codec reset pin (PD4) */
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
GPIO_InitStruct.GPIO_Pin = CODEC_RESET_PIN;
GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init ( CODEC_RESET_PORT, &GPIO_InitStruct );
}

void CODEC_I2CConfig(void) {
	I2C_InitTypeDef		I2C_InitStruct;
	/* I2C1 (codec control interface) initialization */
RCC_APB1PeriphClockCmd(CODEC_I2C_RCC, ENABLE);
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 30000;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1 = 1;
	I2C_Init ( CODEC_I2C_PORT, &I2C_InitStruct );
	I2C_Cmd ( CODEC_I2C_PORT, ENABLE );}
void CODEC_I2SConfig(void) {
	I2S_InitTypeDef		I2S_InitStruct;
	/* I2S3 (SPI3) initialization (Codec) */
RCC_APB1PeriphClockCmd(CODEC_I2S_RCC, ENABLE);
	I2S_InitStruct.I2S_AudioFreq = Filter.Fs / 2;
	I2S_InitStruct.I2S_CPOL = I2S_CPOL_Low;
	I2S_InitStruct.I2S_DataFormat = I2S_DataFormat_16b;
I2S_InitStruct.I2S_MCLKOutput=I2S_MCLKOutput_Enable;
	I2S_InitStruct.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitStruct.I2S_Standard = I2S_Standard_Phillips;
	I2S_Init ( CODEC_I2S_PORT, &I2S_InitStruct );
	I2S_Cmd ( CODEC_I2S_PORT, ENABLE );}
void CODEC_Config(void) {
	/* Keep Codec powered OFF */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x02, 0x01);
	/* SPK always OFF & HP always ON */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x04, 0xAF);
	/* Clock configuration: Auto detection */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x05, 0x81);
	/* Set the Slave Mode and the audio Standard */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x06, 0x04);
	/* Set the Master volume */
     CODEC_VolumeCtrl ( VOLUME_CONVERT ( 90 ) );
	/* Power on the Codec */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT, CODEC_ADDRESS, 0x02, 0x9E);
/* Additional configuration for the CODEC. These configurations are done to reduce
 the time needed for the Codec to power off. If these configurations are removed,
 then a long delay should be added between powering off the Codec and switching
 off the I2S peripheral MCLK clock (which is the operating clock for Codec).
 If this delay is not inserted, then the codec will not shut down properly and
 it results in high noise after shut down. */
/* Disable the analog soft ramp */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT,
			CODEC_ADDRESS, 0x0A, 0x00);
/* Disable the digital soft ramp */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT,
			CODEC_ADDRESS, 0x0E, 0x04);
/* Disable the limiter attack level */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT,
			CODEC_ADDRESS, 0x27, 0x00);
/* Adjust Bass and Treble levels */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT,
			CODEC_ADDRESS, 0x1F, 0x0F);
/* Adjust PCM volume level */
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT,
			CODEC_ADDRESS, 0x1A, 0x0A);
CODEC_I2C_SingleRandomWrite (CODEC_I2C_PORT,
			CODEC_ADDRESS, 0x1B, 0x0A);}
void CODEC_DMAConfig(void) {
	DMA_InitTypeDef		DMA_InitStruct;
	/* Enable DMA1 clock */
	RCC_AHB1PeriphClockCmd ( CODEC_DMA_RCC, ENABLE );
	/* DMA1 Stream7 Configuration */
	DMA_DeInit ( CODEC_DMA_STREAM );
	DMA_InitStruct.DMA_Channel = CODEC_DMA_CHANNEL;
   DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&CODEC_I2S_PORT->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) AudioBuffer;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = 2 * PCM_OUT_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
      DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
      DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
             DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init ( CODEC_DMA_STREAM, &DMA_InitStruct );
	/* Enable DMA transfer */
	DMA_Cmd ( CODEC_DMA_STREAM, ENABLE );
	SPI_I2S_DMACmd ( CODEC_I2S_PORT, SPI_I2S_DMAReq_Tx, ENABLE );}
void CODEC_DMA_NVIConfig(void) {
	NVIC_InitTypeDef	NVIC_InitStruct;
	DMA_ITConfig ( CODEC_DMA_STREAM, DMA_IT_TC, ENABLE );
	NVIC_PriorityGroupConfig ( NVIC_GROUP );
	NVIC_InitStruct.NVIC_IRQChannel = CODEC_IRQ_CHANNEL;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init ( &NVIC_InitStruct );}
void CODEC_Init(void) {
	CODEC_CtrlLinesConfig();
	/* Reset codec */
	GPIO_ResetBits ( CODEC_RESET_PORT, CODEC_RESET_PIN );
	Delay ( 100 );
	GPIO_SetBits ( CODEC_RESET_PORT, CODEC_RESET_PIN );
	CODEC_I2SConfig();
	CODEC_I2CConfig();
	CODEC_Config();
	CODEC_DMAConfig();
	CODEC_DMA_NVIConfig();}
void FRAME_DmaToLcd ( void ) {
	g_FrameTotalBytesCopied = 2;

    LCD_WriteReg ( SSD2119_X_RAM_ADDR_REG, 0x00 );
    LCD_WriteReg ( SSD2119_Y_RAM_ADDR_REG, 0x00 );
   LCD_WriteReg (SSD2119_RAM_DATA_REG, lcdFrame_1[0]);

	DMA_Cmd ( DMA2_Stream3, DISABLE );
   DMA_ClearFlag ( DMA2_Stream3, DMA_FLAG_TCIF3 );
	DMA_SetCurrDataCounter ( DMA2_Stream3, 0 );
	DMA_Cmd ( DMA2_Stream3, ENABLE );
}
/* @brief  Displays a line.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @param Length: line length.
  * @param Direction: line direction.
  *   This parameter can be one of the following values: Vertical or Horizontal.
  * @retval None  */
void FRAME_DrawLine ( uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction ) {
	uint16_t textColor, backColor;
	uint32_t framePixelIndex;
	uint32_t i = 0;
	LCD_GetColors ( &textColor, &backColor );
	framePixelIndex = Xpos + Ypos * LCD_PIXEL_WIDTH;
	if (Direction == LCD_DIR_HORIZONTAL) {
		for (i = 0; i <= Length; i++) {
			if (framePixelIndex <= CCMDATA_SIZE / 2) {
				lcdFrame_1 [ framePixelIndex ] = textColor;
			} else {
	lcdFrame_2 [ framePixelIndex - CCMDATA_SIZE / 2 - 1 ] = textColor;}
		    framePixelIndex = (framePixelIndex + 1) % FRAME_SIZE;}
	} else {
		for (i = 0; i <= Length; i++) {
		   if (framePixelIndex <= CCMDATA_SIZE / 2) {
		lcdFrame_1 [ framePixelIndex ] = textColor;
			} else {
lcdFrame_2 [ framePixelIndex - CCMDATA_SIZE / 2 - 1 ] = textColor;
			}
        framePixelIndex = (framePixelIndex + LCD_PIXEL_WIDTH) % FRAME_SIZE;
		}
	}
}
void MIC_FRAME_DrawGraph ( int16_t * dataArray,
	uint16_t dataArraySize, uint16_t graphRectCoors[4]) {
	uint16_t i, ki = dataArraySize / GRAPH_WIDTH;
uint16_t ymin = graphRectCoors[1], ymax = graphRectCoors[1] + graphRectCoors[3],
	ymid = graphRectCoors[1] + graphRectCoors[3]/2;
	uint16_t i_0 = 0, i_1 = ki;
	uint16_t frame_y_0 = ymid - (dataArray[i_0] >> DATA_SHIFT);
	uint16_t frame_y_1 = ymid - (dataArray[i_1] >> DATA_SHIFT);
	for (i = 0; i < GRAPH_WIDTH-1; i++) {
		frame_y_0 = (frame_y_0 < ymin)? ymin : frame_y_0;
		frame_y_1 = (frame_y_1 < ymin)? ymin : frame_y_1;
		frame_y_0 = (frame_y_0 > ymax)? ymax : frame_y_0;
		frame_y_1 = (frame_y_1 > ymax)? ymax : frame_y_1;
		FRAME_DrawLine ( graphRectCoors[0]+i, MIN(frame_y_0,frame_y_1), ABS(frame_y_1-frame_y_0), LCD_DIR_VERTICAL);
		i_0 = i_1, i_1 += ki;
		frame_y_0 = frame_y_1;
		frame_y_1 = ymid - (dataArray[i_1] >> DATA_SHIFT);	}}
void FRAME_RestoreArea ( uint16_t graphRectCoors[4], uint16_t * pBaseImage ) {
	uint32_t framePixelIndex;
	uint32_t x = graphRectCoors[0], y = graphRectCoors[1];
	uint16_t i, j = graphRectCoors[3] + 1;

	do {
		framePixelIndex = x + y++ * LCD_PIXEL_WIDTH;
		i = graphRectCoors[2];
		do {
			if (framePixelIndex <= CCMDATA_SIZE / 2) {
		lcdFrame_1 [framePixelIndex] = pBaseImage[framePixelIndex];
			} else {
lcdFrame_2 [framePixelIndex - CCMDATA_SIZE / 2 - 1] = pBaseImage[framePixelIndex];
			}
			framePixelIndex = (framePixelIndex + 1) % FRAME_SIZE;
		} while (--i);
	} while (--j);}
void DMA_LCD_Config ( void ) {
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
       /* Configures the DMA2 to transfer Data from DmaCaptureFrame[] to the LCD */
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_DMA2, ENABLE );
	/* DMA2 Stream3 Configuration */
	DMA_DeInit ( DMA2_Stream3 );
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DmaFrame;
              DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)LCD_BASE_Data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;
	DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
DMA_Init ( DMA2_Stream3, &DMA_InitStructure );
/* Enable the DMA2_Stream3 Interrupt */
NVIC_PriorityGroupConfig ( NVIC_GROUP );
NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init ( &NVIC_InitStructure );
DMA_ITConfig ( DMA2_Stream3, DMA_IT_TC, ENABLE );}
void DMA2_Stream3_IRQHandler(void) {
	int32_t sizeToCopy;
DMA_ClearITPendingBit ( DMA2_Stream3, DMA_FLAG_TCIF3 );
if (g_FrameTotalBytesCopied < (sizeof(DmaFrame[0]) * FRAME_SIZE)) {
		if (g_FrameTotalBytesCopied < CCMDATA_SIZE) {
			sizeToCopy = MIN (DMA_BUFFER_SIZE,
(CCMDATA_SIZE - g_FrameTotalBytesCopied)/sizeof(DmaFrame[0]));
			memmove ( DmaFrame,
lcdFrame_1 + 1 + g_FrameTotalBytesCopied / sizeof(lcdFrame_1[0]),
				sizeToCopy * sizeof(DmaFrame[0]));
		} else {
			sizeToCopy = MIN (DMA_BUFFER_SIZE,
FRAME_SIZE - g_FrameTotalBytesCopied / sizeof(DmaFrame[0]));
			memmove ( DmaFrame,
lcdFrame_2 + (g_FrameTotalBytesCopied - CCMDATA_SIZE)/sizeof(lcdFrame_2[0]),
				sizeToCopy * sizeof(DmaFrame[0]));		}
		g_FrameTotalBytesCopied += sizeToCopy * sizeof(DmaFrame[0]);
		DMA_Cmd ( DMA2_Stream3, DISABLE );
		DMA_ClearFlag ( DMA2_Stream3, DMA_FLAG_TCIF3 );
		DMA_SetCurrDataCounter ( DMA2_Stream3, sizeToCopy );
		DMA_Cmd ( DMA2_Stream3, ENABLE );	}}
