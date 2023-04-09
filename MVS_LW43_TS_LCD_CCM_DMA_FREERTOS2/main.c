#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f4xx_conf.h"
#define DATA_SHIFT	4

#define ABS(x)		((x) < 0) ? -(x) : (x)
#define MIN(x,y)	((x) < (y)) ? (x):(y)
#define MAX(x,y)	((x) > (y)) ? (x):(y)
#define	MIN_X	110
#define	MAX_X	4000
#define	MIN_Y	150
#define	MAX_Y	3850
#define NVIC_GROUP NVIC_PriorityGroup_0
volatile int32_t ITM_RxBuffer;
extern const unsigned char gImage_BaseImage[];
unsigned short BaseImage_coor[] = {0, 0, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT};
#define LCD_BASE_Data  ((u32)(0x60000000|0x00100000))
#define LCD_BASE_Addr  ((u32)(0x60000000|0x00000000))
#define FRAME_SIZE (LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT)
#define CCMDATA_SIZE (CCMDATARAM_END - CCMDATARAM_BASE - 1)
uint16_t * lcdFrame_1 = (uint16_t *)CCMDATARAM_BASE;  // 64 KB
uint16_t lcdFrame_2 [ FRAME_SIZE - CCMDATA_SIZE / 2 ]; // 86 KB
#define GRAPH_WIDTH		256
const int16_t	tsLcdRect[] ={39, 77, GRAPH_WIDTH, 128};
static int16_t	g_TS[GRAPH_WIDTH + 1];
#define DMA_BUFFER_SIZE (LCD_PIXEL_WIDTH * 1)
uint16_t DmaFrame [ DMA_BUFFER_SIZE ];
static uint32_t g_FrameTotalBytesCopied = 0;
void DMA_LCD_Config ( void );
void FRAME_DmaToLcd ( void );
void FRAME_DrawLine ( uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction );
void FRAME_RestoreArea ( uint16_t graphRectCoors[4], uint16_t * pBaseImage );
void TS_FRAME_DrawGraph ( int16_t * dataArray,
	uint16_t dataArraySize, uint16_t graphRectCoors[4]);
void taskLCD ( void * pvParameters );
void taskTSLineBufferUpdate ( void * pvParameters );
void taskTS ( void * pvParameters );
xQueueHandle tsQueue;
int main( void ) {
tsQueue = xQueueCreate ( 8, sizeof(int16_t) );
xTaskCreate ( taskLCD, "TS GRAPH", 2 * GRAPH_WIDTH, NULL, 1, NULL );
xTaskCreate(taskTS,"TS data",128, NULL, 1, NULL);
xTaskCreate ( taskTSLineBufferUpdate, "TS LINE BUFFER", 128, NULL, 2, NULL );
vTaskStartScheduler();
	while (1);	return 0;}
// 115
void taskTS ( void * pvParameters ) {
	int16_t res_y = 0;
	int16_t tsRect[] = {
		MAX_X - (MAX_X - MIN_X) * (tsLcdRect[0] + tsLcdRect[2]) / LCD_PIXEL_WIDTH,
		MIN_Y + (MAX_Y - MIN_Y) * tsLcdRect[1] / LCD_PIXEL_HEIGHT,
			MAX_X - (MAX_X - MIN_X) * tsLcdRect[0] / LCD_PIXEL_WIDTH,
			MIN_Y + (MAX_Y - MIN_Y) * (tsLcdRect[1] + tsLcdRect[3]) / LCD_PIXEL_HEIGHT,	};
	TS_STATE *pstate = NULL;
	/* TouchScreen interface initialization */
	IOE_Config();
	portTickType tick = xTaskGetTickCount();
	while (1) {vTaskDelay(5);
	pstate = IOE_TS_GetState();
	if (pstate->TouchDetected ) {
	if ( (pstate->X > tsRect[0]) && (pstate->X < tsRect[2]) &&
	 (pstate->Y > tsRect[1]) && (pstate->Y < tsRect[3])) {
	res_y = (tsRect[1] + tsRect[3]) / 2 - pstate->Y;}}
	xQueueSend ( tsQueue, &res_y, 10 );}}
void taskTSLineBufferUpdate ( void * pvParameters ) {
	int16_t iTsBufLen = 0;
	while (1) {
		xQueueReceive( tsQueue, g_TS + iTsBufLen, portMAX_DELAY );
		iTsBufLen++;
		if (iTsBufLen > GRAPH_WIDTH) {
memmove ( g_TS, g_TS + 1, GRAPH_WIDTH * sizeof(int16_t));
			iTsBufLen--;	}}}
void taskLCD ( void * pvParameters ) {
  BaseImage_coor[2] = *(uint16_t *)(gImage_BaseImage + 2);
  BaseImage_coor[3] = *(uint16_t *)(gImage_BaseImage + 4);
	/* Инициализация видео канала */
	STM32f4_Discovery_LCD_Init();
LCD_SetColors (LCD_COLOR_MAGENTA, LCD_COLOR_BLACK);
	uint32_t framePixelIndex;
for (framePixelIndex = 0; framePixelIndex < FRAME_SIZE; framePixelIndex++) {
	if (framePixelIndex <= CCMDATA_SIZE / 2) {
lcdFrame_1 [ framePixelIndex ] = *((uint16_t*) (gImage_BaseImage + 8) + framePixelIndex);
	} else {
lcdFrame_2[framePixelIndex-CCMDATA_SIZE/2-1]= *((uint16_t *)(gImage_BaseImage + 8) + framePixelIndex);}}
	DMA_LCD_Config();
// 120
	while (1) {
   /* Обновление сигналов на дисплее каждые 50 мс */
	vTaskDelay ( 50 );
FRAME_RestoreArea ( (uint16_t *)tsLcdRect, (uint16_t *) (gImage_BaseImage + 8) );
TS_FRAME_DrawGraph ( g_TS, GRAPH_WIDTH, (uint16_t *)tsLcdRect);
	FRAME_DmaToLcd();	}}
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
void TS_FRAME_DrawGraph ( int16_t * dataArray,
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
		frame_y_1 = ymid - (dataArray[i_1] >> DATA_SHIFT);}}
// 125
void FRAME_RestoreArea ( uint16_t graphRectCoors[4], uint16_t * pBaseImage ) {
	uint32_t framePixelIndex;
	uint32_t x = graphRectCoors[0], y = graphRectCoors[1];
	uint16_t i, j = graphRectCoors[3] + 1;

	do {	framePixelIndex = x + y++ * LCD_PIXEL_WIDTH;
		i = graphRectCoors[2];
		do {	if (framePixelIndex <= CCMDATA_SIZE / 2) {
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
// 129
