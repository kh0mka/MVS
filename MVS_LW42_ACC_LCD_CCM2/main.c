#include <stdint.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#define ABS(x)		((x) < 0) ? -(x) : (x)
#define MIN(x,y)		((x) < (y)) ? (x) : (y)
#define MAX(x,y)		((x) > (y)) ? (x) : (y)
#define NVIC_GROUP	NVIC_PriorityGroup_0
#define ACCEL_TIM_RCC_PeriphClockCmd	RCC_APB2PeriphClockCmd
#define ACCEL_TIM_RCC_APBPeriphTIM	RCC_APB2Periph_TIM1
#define ACCEL_TIM	TIM1
#define ACCEL_TIM_PERIOD	9
#define ACCEL_TIM_CHANNEL	TIM1_UP_TIM10_IRQn
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} LIS302DL_OutXYZTypeDef;
volatile int32_t ITM_RxBuffer;
RCC_ClocksTypeDef RCC_Clocks;
extern const unsigned char gImage_BaseImage[];
#define FRAME_SIZE	(LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT)
#define CCMDATA_SIZE      (CCMDATARAM_END - CCMDATARAM_BASE - 1)
uint16_t * lcdFrame_1 = (uint16_t *)CCMDATARAM_BASE; // 64 KB
uint16_t lcdFrame_2 [ FRAME_SIZE - CCMDATA_SIZE / 2 ]; // 86 KB
#define DATA_SHIFT_X		2
#define DATA_SHIFT_Y		2
#define DATA_SHIFT_Z		3
uint16_t xGraphCoor[] = {42, 70, 256, 32};
uint16_t yGraphCoor[] = {42, 120, 256, 32};
uint16_t zGraphCoor[] = {42, 170, 256, 32};
void CameraDisable(void);
void FRAME_DrawLine ( uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction );
void ACCEL_FRAME_RestoreArea ( uint16_t xGraphRectCoors[4], uint16_t yGraphRectCoors[4], uint16_t zGraphRectCoors[4], uint16_t * pBaseImage );
void ACCEL_FRAME_DrawGraph ( LIS302DL_OutXYZTypeDef * dataArray, uint16_t dataArraySize, uint16_t xGraphRectCoors[4], uint16_t yGraphRectCoors[4], uint16_t zGraphRectCoors[4]);
void FRAME_ToLcd ( void );
void ACCEL_TIM_Init ( void );
#define ACCEL_BUFFER_LEN	256
#define ACCEL_QUEUE_LEN	(32)
static LIS302DL_OutXYZTypeDef g_FIFOxyz [ACCEL_QUEUE_LEN];
static uint16_t g_FifoBegin = 0;
static uint16_t g_FifoEnd = 0;
/* @brief Read next axis data from FIFO.
 * @param pResultXYZ : pointer to data be read
 * @retval Read status: SUCCESS if all OK or ERROR when nothing read */
ErrorStatus FIFO_GetNextData ( LIS302DL_OutXYZTypeDef * pResultXYZ) {
	ErrorStatus res = ERROR;
	if (g_FifoBegin != g_FifoEnd) {
	*pResultXYZ = g_FIFOxyz[g_FifoBegin++];
		g_FifoBegin %= ACCEL_QUEUE_LEN;
		res = SUCCESS;	}
	return res;}
int main ( void ) {
	/* SysTick end of count event each 1 ms */
	RCC_GetClocksFreq ( &RCC_Clocks );
	SysTick_Config (RCC_Clocks.HCLK_Frequency / 1000);
	/* Отключение камеры, на всякий случай */
	CameraDisable();
	ACCEL_TIM_Init();
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
      uint32_t tick = GetTickCount();
      while (1) {
      	static LIS302DL_OutXYZTypeDef accelXYZ[ACCEL_BUFFER_LEN + 1];
      	static int16_t iAccelBufferLength = 0;
      	LIS302DL_OutXYZTypeDef	curAxes;
      	/* Обновление линейного буфера */
      	while (FIFO_GetNextData ( &curAxes ) == SUCCESS) {
      		accelXYZ [ iAccelBufferLength++ ] = curAxes;
      		if (iAccelBufferLength > ACCEL_BUFFER_LEN) {
             memmove ( accelXYZ, accelXYZ + 1, ACCEL_BUFFER_LEN * sizeof(LIS302DL_OutXYZTypeDef));
      			iAccelBufferLength--; }}
      	/* Обновление сигналов на дисплее каждые 50 мс */
      	if (GetTickCount() - tick > 50) {
      		tick = GetTickCount();
      		ACCEL_FRAME_RestoreArea ( xGraphCoor, yGraphCoor, zGraphCoor, (uint16_t *) (gImage_BaseImage + 8) );
      		ACCEL_FRAME_DrawGraph ( accelXYZ, ACCEL_BUFFER_LEN, xGraphCoor, yGraphCoor, zGraphCoor);
      		FRAME_ToLcd(); 	}}
      return 0;}
// 80
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
void FRAME_ToLcd ( void ) {
	int i;
    LCD_WriteReg ( SSD2119_X_RAM_ADDR_REG, 0x00 );
    LCD_WriteReg ( SSD2119_Y_RAM_ADDR_REG, 0x00 );
   LCD_WriteReg (SSD2119_RAM_DATA_REG, lcdFrame_1[0]);
	for (i = 1; i <= CCMDATA_SIZE / 2; i++) {
		LCD_WriteRAM ( lcdFrame_1 [ i ] );
	}
    for (i = 0; i < FRAME_SIZE - CCMDATA_SIZE / 2; i++) {
		LCD_WriteRAM ( lcdFrame_2 [ i ] );
	}
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
void ACCEL_FRAME_DrawGraph ( LIS302DL_OutXYZTypeDef * dataArray,
uint16_t dataArraySize, uint16_t xGraphRectCoors[4], uint16_t yGraphRectCoors[4], uint16_t zGraphRectCoors[4]) {
	uint16_t _min, _max, _mid;
	uint16_t i, frame_y_0, frame_y_1;
	_min = xGraphRectCoors[1],
	_max = xGraphRectCoors[1] + xGraphRectCoors[3],
	_mid = xGraphRectCoors[1] + xGraphRectCoors[3]/2;
	for (i = 0; i < dataArraySize-1; i++) {
	frame_y_0 = _mid - (dataArray[i].x >> DATA_SHIFT_X);
	frame_y_1 = _mid - (dataArray[i+1].x >> DATA_SHIFT_X);
	frame_y_0 = (frame_y_0 < _min)? _min : frame_y_0;
	frame_y_1 = (frame_y_1 < _min)? _min : frame_y_1;
	frame_y_0 = (frame_y_0 > _max)? _max : frame_y_0;
	frame_y_1 = (frame_y_1 > _max)? _max : frame_y_1;
	FRAME_DrawLine ( xGraphRectCoors[0]+i, MIN(frame_y_0,frame_y_1), ABS(frame_y_1-frame_y_0), LCD_DIR_VERTICAL); }
// 85
	_min = yGraphRectCoors[1],
	_max = yGraphRectCoors[1] + yGraphRectCoors[3],
	_mid = yGraphRectCoors[1] + yGraphRectCoors[3]/2;
	for (i = 0; i < dataArraySize-1; i++) {
frame_y_0 = _mid - (dataArray[i].y >> DATA_SHIFT_Y);
frame_y_1 = _mid - (dataArray[i+1].y >> DATA_SHIFT_Y);
	frame_y_0 = (frame_y_0 < _min)? _min : frame_y_0;
	frame_y_1 = (frame_y_1 < _min)? _min : frame_y_1;
	frame_y_0 = (frame_y_0 > _max)? _max : frame_y_0;
	frame_y_1 = (frame_y_1 > _max)? _max : frame_y_1;
	FRAME_DrawLine ( yGraphRectCoors[0]+i, MIN(frame_y_0,frame_y_1), ABS(frame_y_1-frame_y_0), LCD_DIR_VERTICAL); 	}
	_min = zGraphRectCoors[1],
	_max = zGraphRectCoors[1] + zGraphRectCoors[3],
	_mid = zGraphRectCoors[1] + zGraphRectCoors[3]/2;
	for (i = 0; i < dataArraySize-1; i++) {
frame_y_0 = _mid - (dataArray[i].z >> DATA_SHIFT_Z);
frame_y_1 = _mid - (dataArray[i+1].z >> DATA_SHIFT_Z);
	frame_y_0 = (frame_y_0 < _min)? _min : frame_y_0;
	frame_y_1 = (frame_y_1 < _min)? _min : frame_y_1;
	frame_y_0 = (frame_y_0 > _max)? _max : frame_y_0;
	frame_y_1 = (frame_y_1 > _max)? _max : frame_y_1;
	FRAME_DrawLine ( zGraphRectCoors[0]+i, MIN(frame_y_0,frame_y_1), ABS(frame_y_1-frame_y_0), LCD_DIR_VERTICAL);	}}
void ACCEL_FRAME_RestoreArea ( uint16_t xGraphRectCoors[4], uint16_t yGraphRectCoors[4], uint16_t zGraphRectCoors[4], uint16_t * pBaseImage ) {
	uint32_t framePixelIndex;
         uint16_t * graphRectCoors[] = { xGraphRectCoors, yGraphRectCoors, zGraphRectCoors,};
	uint16_t i, j, k = 3;
	while (k--) {
		uint32_t x = graphRectCoors[k][0], y = graphRectCoors[k][1]-16;
		j = graphRectCoors[k][3] + 32;
		do { framePixelIndex = x + y++ * LCD_PIXEL_WIDTH;
		        i = graphRectCoors[k][2];
		        do {
			if (framePixelIndex <= CCMDATA_SIZE / 2) {
		lcdFrame_1 [ framePixelIndex ] = pBaseImage[ framePixelIndex ];
				} else {
   lcdFrame_2 [ framePixelIndex - CCMDATA_SIZE / 2 - 1 ] = pBaseImage[ framePixelIndex ];}
			framePixelIndex = (framePixelIndex + 1) % FRAME_SIZE;
			} while (--i);
		} while (--j);	}}
void ACCEL_TIM_Init ( void ) {
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef		NVIC_InitStruct;
           ACCEL_TIM_RCC_PeriphClockCmd ( ACCEL_TIM_RCC_APBPeriphTIM, ENABLE );
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = ACCEL_TIM_PERIOD;
/* Период 2.5 мс (TIM_TimeBaseInitStruct.TIM_Prescaler = 42000, ACCEL_TIM_PERIOD=9)
*   при считывании данных связана  с предельной частотой их формирования,
*   равной 1/400 = 2.5 мс */
	TIM_TimeBaseInitStruct.TIM_Prescaler = 42000;
	TIM_TimeBaseInit ( ACCEL_TIM, &TIM_TimeBaseInitStruct );
	TIM_Cmd ( ACCEL_TIM, ENABLE );
	TIM_ITConfig ( ACCEL_TIM, TIM_IT_Update, ENABLE );
	NVIC_PriorityGroupConfig ( NVIC_GROUP );
	NVIC_InitStruct.NVIC_IRQChannel = ACCEL_TIM_CHANNEL;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init ( &NVIC_InitStruct ); }
void ACCEL_TIM_IRQHandler(void) {
	uint8_t temp;
	LIS302DL_InitTypeDef  LIS302DL_InitStruct;
	LIS302DL_FilterConfigTypeDef LIS302DL_FilterStruct;
	LIS302DL_OutXYZTypeDef xyz;
if (TIM_GetFlagStatus ( ACCEL_TIM, TIM_FLAG_Update ) == SET) {
	TIM_ClearITPendingBit ( ACCEL_TIM, TIM_IT_Update );
/* MEMS configuration ------------------------------------------------------*/
/* Set configuration of LIS302DL*/
LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_400;
LIS302DL_InitStruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
		LIS302DL_Init(&LIS302DL_InitStruct);
// 90
		/* Set configuration of Internal High Pass Filter of LIS302DL*/
	LIS302DL_FilterStruct.HighPassFilter_Data_Selection	= LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER;
	LIS302DL_FilterStruct.HighPassFilter_CutOff_Frequency = LIS302DL_HIGHPASSFILTER_LEVEL_1;
	LIS302DL_FilterStruct.HighPassFilter_Interrupt = LIS302DL_HIGHPASSFILTERINTERRUPT_1_2;
			LIS302DL_FilterConfig(&LIS302DL_FilterStruct);
	/* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate = 3/100 = 30ms */
			/* Read WHO_AM_I register */
			LIS302DL_Read(&temp, LIS302DL_WHO_AM_I_ADDR, 1);
			/* Check device identification register, this register should contains
			 the device identifier that for LIS302DL is set to 0x3B */
	LIS302DL_Read((uint8_t *)&xyz, LIS302DL_OUT_X_ADDR, sizeof(LIS302DL_OutXYZTypeDef));
			xyz.x = (int16_t)(int8_t)xyz.x;
			xyz.y = (int16_t)(int8_t)xyz.y;
			xyz.z = (int16_t)(int8_t)xyz.z;
			g_FIFOxyz[g_FifoEnd++] = xyz;
			g_FifoEnd %= ACCEL_QUEUE_LEN;
			/* Восстановление альтернативной функции DC (Data/Command)
			 *  линии PE3 для LCD */
			GPIO_InitTypeDef GPIO_InitStructure;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init ( GPIOE, &GPIO_InitStructure );	}}
// 92
