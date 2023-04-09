#include <stdint.h>
#include <string.h>
#include "stm32f4xx_conf.h"
#define ABS(x)		((x) < 0) ? -(x) : (x)
#define MIN(x,y)		((x) < (y)) ? (x) : (y)
#define MAX(x,y)		((x) > (y)) ? (x) : (y)
#define NVIC_GROUP	NVIC_PriorityGroup_0
#define	MIN_X	110
#define	MAX_X	4000
#define	MIN_Y	150
#define	MAX_Y	3850
#define IMG_WIDTH (LCD_PIXEL_WIDTH / 2)
#define IMG_HEIGHT (LCD_PIXEL_HEIGHT / 2)
#define DATA_SHIFT	1
typedef enum {
	false, true,
} bool;
typedef enum {
	BUTTON_LUM, BUTTON_RED, BUTTON_GREEN, BUTTON_BLUE,
} button_t;
//150
volatile int32_t ITM_RxBuffer;
__IO uint32_t TimingDelay;
RCC_ClocksTypeDef RCC_Clocks;
#define BASE_IMAGE_X		0
#define BASE_IMAGE_Y		0
extern const unsigned char gImage_BaseImage[];
int16_t BaseImageCoor[] = {
BASE_IMAGE_X, BASE_IMAGE_Y,
LCD_PIXEL_WIDTH + BASE_IMAGE_X, LCD_PIXEL_HEIGHT + BASE_IMAGE_Y, };
#define LCD_BASE_Data ((u32)(0x60000000|0x00100000))
#define LCD_BASE_Addr ((u32)(0x60000000|0x00000000))
#define FRAME_SIZE	(LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT)
#define CCMDATA_SIZE       (CCMDATARAM_END - CCMDATARAM_BASE - 1)
uint16_t *lcdFrame_1 = (uint16_t*) CCMDATARAM_BASE; // 64 KB
uint16_t lcdFrame_2[FRAME_SIZE - CCMDATA_SIZE / 2]; // 86 KB
#define DMA_BUFFER_SIZE		(LCD_PIXEL_WIDTH * 1)
uint16_t DmaFrame[DMA_BUFFER_SIZE];
static uint32_t g_FrameTotalBytesCopied = 0;
extern const unsigned char gImage_buttonLumPressed[];
extern const unsigned char gImage_buttonLumUnpressed[];
int16_t buttonLumCoor[] = { 242, 196, 0, 0 };
extern const unsigned char gImage_buttonRedPressed[];
extern const unsigned char gImage_buttonRedUnpressed[];
int16_t buttonRedCoor[] = { 0, 196, 0, 0 };
extern const unsigned char gImage_buttonGreenPressed[];
extern const unsigned char gImage_buttonGreenUnpressed[];
int16_t buttonGreenCoor[] = { 81, 196, 0, 0 };
extern const unsigned char gImage_buttonBluePressed[];
extern const unsigned char gImage_buttonBlueUnpressed[];
int16_t buttonBlueCoor[] = { 162, 196, 0, 0 };
#define CAM_DMA_FRAME_SIZE (LCD_PIXEL_WIDTH * 2)
uint32_t frame[CAM_DMA_FRAME_SIZE];
uint16_t frame_index = 0;
uint16_t Frame[IMG_WIDTH * IMG_HEIGHT];
button_t currentButton = BUTTON_LUM;
int16_t x_0, y_0;
#define GRAPH_WIDTH		128
const int16_t camGraphCoor[] = { 176, 100, GRAPH_WIDTH, 64 };
uint8_t DCMI_OV9655Config(void);
void DCMI_Config(void);
void LIS302DL_Reset(void);
void Delay(uint32_t nTime);
uint32_t GetTickCount();
void FRAME_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length,
		uint8_t Direction);
void FRAME_DrawGraph(uint8_t *dataArray, uint16_t dataArraySize,
		int16_t graphRectCoors[4]);
void FRAME_RestoreArea(uint16_t graphRectCoors[4], uint16_t *pBaseImage);
void FRAME_ImageToLcdBuffer(const unsigned char *pImage, int16_t *pCoordinates,
		bool bAND);
void FRAME_DmaToLcd(void);
void DMA_LCD_Config(void);
//155
#define CAM_QUEUE_LEN 32
static uint8_t g_FIFO[CAM_QUEUE_LEN];
static uint16_t g_FifoBegin = 0;
static uint16_t g_FifoEnd = 0;
#define CAM_BUFFER_LEN GRAPH_WIDTH
/* @brief Read next data from FIFO.
 * @param  pData: pointer to data be read
 * @retval Read status: SUCCESS if all OK or ERROR when nothing read */
ErrorStatus FIFO_GetNextData(uint8_t *pData) {
	ErrorStatus res = ERROR;
	if (g_FifoBegin != g_FifoEnd) {
		*pData = g_FIFO[g_FifoBegin++];
		g_FifoBegin %= CAM_QUEUE_LEN;
		res = SUCCESS;
	}
	return res;
}
int main(void) {
	uint16_t i, j, k, *pointer;
	NVIC_InitTypeDef NVIC_InitStructure;
	TS_STATE *pstate = NULL;
	/* SysTick end of count event each 1 ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
	LIS302DL_Reset();
	/* Initialize the LCD */
	STM32f4_Discovery_LCD_Init();
	LCD_Clear( LCD_COLOR_BLACK);
	LCD_SetBackColor( LCD_COLOR_BLACK);
	LCD_SetTextColor( LCD_COLOR_MAGENTA);
	/* TouchScreen interface initialization */
	IOE_Config();
	DCMI_Control_IO_Init();
	LCD_DisplayStringLine(LINE(0), (uint8_t*) "                    ");
	LCD_DisplayStringLine(LINE(1), (uint8_t*) "                    ");
	LCD_DisplayStringLine(LINE(2), (uint8_t*) " Идёт инициализация ");
	LCD_DisplayStringLine(LINE(3), (uint8_t*) "      камеры...     ");
	LCD_DisplayStringLine(LINE(4), (uint8_t*) "                    ");
	LCD_DisplayStringLine(LINE(5), (uint8_t*) "                    ");
	LCD_DisplayStringLine(LINE(6), (uint8_t*) "                    ");
	LCD_DisplayStringLine(LINE(7), (uint8_t*) "                    ");
	LCD_DisplayStringLine(LINE(8), (uint8_t*) "                    ");
	LCD_DisplayStringLine(LINE(9), (uint8_t*) "                    ");
	/* Настройка сторожевого таймера */
	IWDG_SetReload(0xFFF);
	IWDG_ReloadCounter();
	IWDG_Enable();
	/* OV9655 Camera Module configuration */
	if (DCMI_OV9655Config() == 0x00) {
		LCD_DisplayStringLine(LINE(2), (uint8_t*) "                    ");
		LCD_DisplayStringLine(LINE(3), (uint8_t*) "                    ");
		LCD_SetDisplayWindow(0, 0, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT);
		LCD_WriteRAM_Prepare();
		/* Start Image capture and Display on the LCD ***/
		/* Enable DMA transfer */
		DMA_Cmd( DMA2_Stream1, ENABLE);
		// Для кадра в ОЗУ
		DMA_ITConfig( DMA2_Stream1, DMA_IT_HT | DMA_IT_TC, ENABLE);

		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_0);
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
//160
		/* Enable DCMI interface */
		DCMI_Cmd(ENABLE);
		/* Start Image capture */
		DCMI_CaptureCmd(ENABLE);
	} else {
		LCD_SetTextColor( LCD_COLOR_RED);
		LCD_DisplayStringLine(LINE(2), (uint8_t*) "Инициализация камеры");
		LCD_DisplayStringLine(LINE(3), (uint8_t*) "     не прошла!!!   ");
		LCD_DisplayStringLine(LINE(5), (uint8_t*) "Проверьте соединение");
		LCD_DisplayStringLine(LINE(6), (uint8_t*) " и повторите попытку");
		/* Go to infinite loop */
		while (1)
			;
	}
//161
	buttonRedCoor[2] = buttonRedCoor[0]
			+ *(int16_t*) (gImage_buttonRedPressed + 2);
	buttonRedCoor[3] = buttonRedCoor[1]
			+ *(int16_t*) (gImage_buttonRedPressed + 4);
	buttonGreenCoor[2] = buttonGreenCoor[0]
			+ *(int16_t*) (gImage_buttonGreenPressed + 2);
	buttonGreenCoor[3] = buttonGreenCoor[1]
			+ *(int16_t*) (gImage_buttonGreenPressed + 4);
	buttonBlueCoor[2] = buttonBlueCoor[0]
			+ *(int16_t*) (gImage_buttonBluePressed + 2);
	buttonBlueCoor[3] = buttonBlueCoor[1]
			+ *(int16_t*) (gImage_buttonBluePressed + 4);
	buttonLumCoor[2] = buttonLumCoor[0]
			+ *(int16_t*) (gImage_buttonLumPressed + 2);
	buttonLumCoor[3] = buttonLumCoor[1]
			+ *(int16_t*) (gImage_buttonLumPressed + 4);
	int16_t tsLumButtonCoor[4], tsRedButtonCoor[4], tsGreenButtonCoor[4],
			tsBlueButtonCoor[4];
	int16_t *buttonsCoor[] = { buttonLumCoor, buttonRedCoor, buttonGreenCoor,
			buttonBlueCoor, };
//162
	int16_t *tsButtonsCoor[] = { tsLumButtonCoor, tsRedButtonCoor,
			tsGreenButtonCoor, tsBlueButtonCoor, };
	button_t button;
	for (button = BUTTON_LUM; button <= BUTTON_BLUE; button++) {
		tsButtonsCoor[button][0] = MAX_X
				- (MAX_X - MIN_X) * buttonsCoor[button][2] / LCD_PIXEL_WIDTH;
		tsButtonsCoor[button][1] = MIN_Y
				+ (MAX_Y - MIN_Y) * buttonsCoor[button][1] / LCD_PIXEL_HEIGHT;
		tsButtonsCoor[button][2] = MAX_X
				- (MAX_X - MIN_X) * buttonsCoor[button][0] / LCD_PIXEL_WIDTH;
		tsButtonsCoor[button][3] = MIN_Y
				+ (MAX_Y - MIN_Y) * buttonsCoor[button][3] / LCD_PIXEL_HEIGHT;
	}
//163
	const unsigned char *buttonsUnpressed[] = { gImage_buttonLumUnpressed,
			gImage_buttonRedUnpressed, gImage_buttonGreenUnpressed,
			gImage_buttonBlueUnpressed, };
	const unsigned char *buttonsPressed[] = { gImage_buttonLumPressed,
			gImage_buttonRedPressed, gImage_buttonGreenPressed,
			gImage_buttonBluePressed, };
//164
#define FRAME_X		0
#define FRAME_Y		72
	int16_t FrameCoor[] = {
	FRAME_X, FRAME_Y,
	IMG_WIDTH + FRAME_X, IMG_HEIGHT + FRAME_Y, };

	int16_t tsFrameCoor[] = {
	MAX_X - (MAX_X - MIN_X) * FrameCoor[2] / LCD_PIXEL_WIDTH,
	MIN_Y + (MAX_Y - MIN_Y) * FrameCoor[1] / LCD_PIXEL_HEIGHT,
	MAX_X - (MAX_X - MIN_X) * FrameCoor[0] / LCD_PIXEL_WIDTH,
	MIN_Y + (MAX_Y - MIN_Y) * FrameCoor[3] / LCD_PIXEL_HEIGHT, };
//165
	extern const unsigned char gImage_circle[];
	int16_t CircleCoor[4];
	int16_t cx, cy;
	cx = (FrameCoor[0] + FrameCoor[2]) / 2;
	cy = (FrameCoor[1] + FrameCoor[3]) / 2;
	DMA_LCD_Config();
	FRAME_ImageToLcdBuffer(gImage_BaseImage + 8, BaseImageCoor, false);
	FRAME_ImageToLcdBuffer(gImage_buttonRedUnpressed + 8, buttonRedCoor, false);
	FRAME_ImageToLcdBuffer(gImage_buttonGreenUnpressed + 8, buttonGreenCoor,
			false);
	FRAME_ImageToLcdBuffer(gImage_buttonBlueUnpressed + 8, buttonBlueCoor,
			false);
	FRAME_ImageToLcdBuffer(gImage_buttonLumUnpressed + 8, buttonLumCoor, false);
	FRAME_ImageToLcdBuffer(buttonsPressed[currentButton] + 8,
			buttonsCoor[currentButton], false);
	uint32_t tick = GetTickCount();
	while (1) {
		static uint16_t FailConnectionCounter = 0;
		static uint8_t camBuf[CAM_BUFFER_LEN + 1];
		static int16_t iCamBufferLength = 0;
		uint8_t curData;
		/* Обновление линейного буфера */
		while (FIFO_GetNextData(&curData) == SUCCESS) {
			camBuf[iCamBufferLength++] = curData;
			if (iCamBufferLength > CAM_BUFFER_LEN) {
				memmove(camBuf, camBuf + 1, CAM_BUFFER_LEN * sizeof(uint8_t));
				iCamBufferLength--;
			}
		}
		x_0 = MAX((cx - FrameCoor[0]) << 1, 0);
		y_0 = MAX((cy - FrameCoor[1]) << 1, 0);
		/* Обновление сигналов на дисплее каждые 50 мс */
		if (GetTickCount() - tick > 50) {
			tick = GetTickCount();
			IWDG_ReloadCounter();
			pstate = IOE_TS_GetState();
			if (pstate->TouchDetected) {
				FailConnectionCounter = 0;
				for (button = BUTTON_LUM; button <= BUTTON_BLUE; button++) {
					if ((pstate->X > tsButtonsCoor[button][0])
							&& (pstate->X < tsButtonsCoor[button][2])
							&& (pstate->Y > tsButtonsCoor[button][1])
							&& (pstate->Y < tsButtonsCoor[button][3])
							&& (currentButton != button)) {
						FRAME_ImageToLcdBuffer(
								buttonsUnpressed[currentButton] + 8,
								buttonsCoor[currentButton], false);
						currentButton = button;
						FRAME_ImageToLcdBuffer(
								buttonsPressed[currentButton] + 8,
								buttonsCoor[currentButton], false);
						break;
					}
				}
				if ((pstate->X > tsFrameCoor[0]) && (pstate->X < tsFrameCoor[2])
						&& (pstate->Y > tsFrameCoor[1])
						&& (pstate->Y < tsFrameCoor[3])) {
					cx = LCD_PIXEL_WIDTH * (MAX_X - pstate->X)
							/ (MAX_X - MIN_X);
					if (cx
							< FrameCoor[0]
									+ *(uint16_t*) (gImage_circle + 2) / 2) {
						cx = FrameCoor[0]
								+ *(uint16_t*) (gImage_circle + 2) / 2;
					} else if (cx
							> FrameCoor[2]
									- *(uint16_t*) (gImage_circle + 2) / 2) {
						cx = FrameCoor[2]
								- *(uint16_t*) (gImage_circle + 2) / 2;
					}
					cy = LCD_PIXEL_HEIGHT * (pstate->Y - MIN_Y)
							/ (MAX_Y - MIN_Y);
					if (cy
							< FrameCoor[1]
									+ *(uint16_t*) (gImage_circle + 4) / 2) {
						cy = FrameCoor[1]
								+ *(uint16_t*) (gImage_circle + 4) / 2;
					} else if (cy
							> FrameCoor[3]
									- *(uint16_t*) (gImage_circle + 4) / 2) {
						cy = FrameCoor[3]
								- *(uint16_t*) (gImage_circle + 4) / 2;
					}
				}
			} else {
				if (((pstate->CurState & 0x01) != 0x01)
						&& (++FailConnectionCounter > 100)) {
					/* "Реанимирование" шины I2C */
					GPIO_InitTypeDef GPIO_InitStruct;
					GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
					GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
					GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
					GPIO_InitStruct.GPIO_Pin = IOE_I2C_SCL_PIN;
					GPIO_Init( IOE_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);
					GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
					GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
					GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
					GPIO_InitStruct.GPIO_Pin = IOE_I2C_SDA_PIN;
					GPIO_Init( IOE_I2C_SDA_GPIO_PORT, &GPIO_InitStruct);
					while (GPIO_ReadInputDataBit( IOE_I2C_SDA_GPIO_PORT,
							IOE_I2C_SDA_PIN) == Bit_RESET) {
						Delay(1);
						GPIO_ToggleBits( IOE_I2C_SCL_GPIO_PORT,
								IOE_I2C_SCL_PIN);
					}
					/* TouchScreen interface initialization */
					IOE_Config();
				}
			}
//170
			// Отображение кадра
			FRAME_ImageToLcdBuffer((const unsigned char*) Frame, FrameCoor,
					false);
			CircleCoor[0] = cx - *(uint16_t*) (gImage_circle + 2) / 2;
			CircleCoor[1] = cy - *(uint16_t*) (gImage_circle + 4) / 2;
			CircleCoor[2] = cx + *(uint16_t*) (gImage_circle + 2) / 2;
			CircleCoor[3] = cy + *(uint16_t*) (gImage_circle + 4) / 2;
			FRAME_ImageToLcdBuffer(gImage_circle + 8, CircleCoor, true);
			FRAME_RestoreArea((uint16_t*) camGraphCoor,
					(uint16_t*) (gImage_BaseImage + 8));
			FRAME_DrawGraph(camBuf, CAM_BUFFER_LEN, (int16_t*) camGraphCoor);
			FRAME_DmaToLcd();
		}
	}
	return 0;
}
/**
 * @brief  Configures all needed resources (I2C, DCMI and DMA) to interface with
 *         the OV9655 camera module
 * @param  None
 * @retval 0x00 Camera module configured correctly
 *         0xFF Camera module configuration failed
 */
uint8_t DCMI_OV9655Config(void) {
	/* Reset and check the presence of the OV9655 camera module */
	if (DCMI_SingleRandomWrite( OV9655_DEVICE_WRITE_ADDRESS, 0x12, 0x80)) {
		return (0xFF);
	}
	/* OV9655 Camera size setup */
	DCMI_OV9655_QVGASizeSetup();
	/* Set the RGB565 mode */
	DCMI_SingleRandomWrite( OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM7, 0x63);
	DCMI_SingleRandomWrite( OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM15, 0x10);
	/* Invert the HRef signal*/
	DCMI_SingleRandomWrite( OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM10, 0x08);
	/* Configure the DCMI to interface with the OV9655 camera module */
	DCMI_Config();

	return (0x00);
}
/**
 * @brief  Configures the DCMI to interface with the OV9655 camera module.
 * @param  None
 * @retval None
 */
void DCMI_Config(void) {
	DCMI_InitTypeDef DCMI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	/* Enable DCMI GPIOs clocks */
	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOB
					| RCC_AHB1Periph_GPIOA, ENABLE);

	/* Enable DCMI clock */
	RCC_AHB2PeriphClockCmd( RCC_AHB2Periph_DCMI, ENABLE);
	/* Connect DCMI pins to AF13 ************/
	/* PCLK */
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);
	/* D0-D7 */
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource0, GPIO_AF_DCMI);
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource1, GPIO_AF_DCMI);
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource4, GPIO_AF_DCMI);
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource5, GPIO_AF_DCMI);
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource6, GPIO_AF_DCMI);
//176
	/* VSYNC */
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
	/* HSYNC */
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
	/* DCMI GPIO configuration *****************/
	/* D0 D1(PC6/7) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( GPIOC, &GPIO_InitStructure);
	/* D2..D4(PE0/1/4) D6/D7(PE5/6) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
			| GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init( GPIOE, &GPIO_InitStructure);
	/* D5(PB6), VSYNC(PB7) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init( GPIOB, &GPIO_InitStructure);
	/* PCLK(PA6) HSYNC(PA4)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA, &GPIO_InitStructure);
	/* DCMI configuration **************/
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_Init(&DCMI_InitStructure);
	/* Configures the DMA2 to transfer Data from DCMI to the LCD ****/
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE);
	/* DMA2 Stream1 Configuration */
	DMA_DeInit( DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & DCMI->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) frame;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = CAM_DMA_FRAME_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
}
//180
const unsigned int screen_WH = LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT;
uint32_t screen_index = 0;
uint16_t *pFrame = (uint16_t*) Frame;

void DMA2_Stream1_Handler(void) {
	unsigned long i;
	DMA_ClearFlag( DMA2_Stream1, DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1);

	for (i = 0; i < (CAM_DMA_FRAME_SIZE / 2); i++) {
		uint16_t *pPixel = (uint16_t*) (frame + frame_index++);
		frame_index %= CAM_DMA_FRAME_SIZE;
		uint16_t x = screen_index % LCD_PIXEL_WIDTH;
		uint16_t y = screen_index / LCD_PIXEL_WIDTH;
		if (!(x & 1) && !(y & 1)) {
			uint16_t cameraPixel = *pPixel;
			uint8_t red = (cameraPixel & LCD_COLOR_RED) >> 11;
			uint8_t green = (cameraPixel & LCD_COLOR_GREEN) >> 6;
			uint8_t blue = cameraPixel & LCD_COLOR_BLUE;
			uint8_t lum = (77ul * red + 151 * green + 28 * blue) >> 8;
			uint8_t res[] = { lum, red, green, blue, };
			*pFrame++ = ((res[currentButton] << 11) | (res[currentButton] << 6)
					| res[currentButton]);
			if ((x == x_0) && (y == y_0)) {
				g_FIFO[g_FifoEnd++] = res[currentButton];
				g_FifoEnd %= CAM_QUEUE_LEN;
			}
		}
		screen_index += 2;
		if (screen_index >= screen_WH) {
			screen_index -= screen_WH;
			pFrame = (uint16_t*) Frame;
		}
	}
}
void FRAME_DmaToLcd(void) {
	g_FrameTotalBytesCopied = 0;
	LCD_WriteReg( SSD2119_X_RAM_ADDR_REG, 0x00);
	LCD_WriteReg( SSD2119_Y_RAM_ADDR_REG, 0x00);
	LCD_WriteReg(SSD2119_RAM_DATA_REG, lcdFrame_1[0]);
	DMA_Cmd( DMA2_Stream3, DISABLE);
	DMA_ClearFlag( DMA2_Stream3, DMA_FLAG_TCIF3);
	DMA_SetCurrDataCounter( DMA2_Stream3, 0);
	DMA_Cmd( DMA2_Stream3, ENABLE);
}
/* @brief  Displays a line.
 * @param Xpos: specifies the X position.
 * @param Ypos: specifies the Y position.
 * @param Length: line length.
 * @param Direction: line direction.
 *   This parameter can be one of the following values: Vertical or Horizontal.
 * @retval None  */
void FRAME_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length,
		uint8_t Direction) {
	uint16_t textColor, backColor;
	uint32_t framePixelIndex;
	uint32_t i = 0;
	LCD_GetColors(&textColor, &backColor);
	framePixelIndex = Xpos + Ypos * LCD_PIXEL_WIDTH;
	if (Direction == LCD_DIR_HORIZONTAL) {
		for (i = 0; i <= Length; i++) {
			if (framePixelIndex <= CCMDATA_SIZE / 2) {
				lcdFrame_1[framePixelIndex] = textColor;
			} else {
				lcdFrame_2[framePixelIndex - CCMDATA_SIZE / 2 - 1] = textColor;
			}
			framePixelIndex = (framePixelIndex + 1) % FRAME_SIZE;
		}
//185
	} else {
		for (i = 0; i <= Length; i++) {
			if (framePixelIndex <= CCMDATA_SIZE / 2) {
				lcdFrame_1[framePixelIndex] = textColor;
			} else {
				lcdFrame_2[framePixelIndex - CCMDATA_SIZE / 2 - 1] = textColor;
			}
			framePixelIndex = (framePixelIndex + LCD_PIXEL_WIDTH) % FRAME_SIZE;
		}
	}
}
void FRAME_DrawGraph(uint8_t *dataArray, uint16_t dataArraySize,
		int16_t graphRectCoors[4]) {
	uint16_t i, ki = dataArraySize / GRAPH_WIDTH;
	int16_t ymin = graphRectCoors[1], ymax = graphRectCoors[1]
			+ graphRectCoors[3];
	uint16_t i_0 = 0, i_1 = ki;
	int16_t frame_y_0 = ymax - (dataArray[i_0] << DATA_SHIFT), frame_y_1 = ymax
			- (dataArray[i_1] << DATA_SHIFT);
	frame_y_0 = (frame_y_0 < ymin) ? ymin : frame_y_0;
	for (i = 0; i < GRAPH_WIDTH - 1; i++) {
		frame_y_1 = (frame_y_1 < ymin) ? ymin : frame_y_1;
		frame_y_1 = (frame_y_1 > ymax) ? ymax : frame_y_1;
		FRAME_DrawLine(graphRectCoors[0] + i, MIN(frame_y_0, frame_y_1),
				ABS(frame_y_1 - frame_y_0), LCD_DIR_VERTICAL);
		i_0 = i_1, i_1 += ki;
		frame_y_0 = frame_y_1;
		frame_y_1 = ymax - (dataArray[i_1] << DATA_SHIFT);
	}
}
void FRAME_RestoreArea(uint16_t graphRectCoors[4], uint16_t *pBaseImage) {
	uint32_t framePixelIndex;
	uint32_t x = graphRectCoors[0], y = graphRectCoors[1];
	uint16_t i, j = graphRectCoors[3] + 1;

	do {
		framePixelIndex = x + y++ * LCD_PIXEL_WIDTH;
		i = graphRectCoors[2];
		do {
			if (framePixelIndex <= CCMDATA_SIZE / 2) {
				lcdFrame_1[framePixelIndex] = pBaseImage[framePixelIndex];
			} else {
				lcdFrame_2[framePixelIndex - CCMDATA_SIZE / 2 - 1] =
						pBaseImage[framePixelIndex];
			}
			framePixelIndex = (framePixelIndex + 1) % FRAME_SIZE;
		} while (--i);
	} while (--j);
}
void DMA_LCD_Config(void) {
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configures the DMA2 to transfer Data from DmaCaptureFrame[] to the LCD */
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE);
	/* DMA2 Stream3 Configuration */
	DMA_DeInit( DMA2_Stream3);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) DmaFrame;
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
	DMA_Init( DMA2_Stream3, &DMA_InitStructure);
	/* Enable the DMA2_Stream3 Interrupt */
	NVIC_PriorityGroupConfig( NVIC_GROUP);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig( DMA2_Stream3, DMA_IT_TC, ENABLE);
}
//190
void DMA2_Stream3_IRQHandler(void) {
	int32_t sizeToCopy;
	DMA_ClearITPendingBit( DMA2_Stream3, DMA_FLAG_TCIF3);
	if (g_FrameTotalBytesCopied < (sizeof(DmaFrame[0]) * FRAME_SIZE)) {
		if (g_FrameTotalBytesCopied < CCMDATA_SIZE) {
			sizeToCopy =
					MIN(DMA_BUFFER_SIZE,
							(CCMDATA_SIZE - g_FrameTotalBytesCopied)/sizeof(DmaFrame[0]));
			memmove(DmaFrame,
					lcdFrame_1 + 1
							+ g_FrameTotalBytesCopied / sizeof(lcdFrame_1[0]),
					sizeToCopy * sizeof(DmaFrame[0]));
		} else {
			sizeToCopy = MIN(DMA_BUFFER_SIZE,
					FRAME_SIZE - g_FrameTotalBytesCopied / sizeof(DmaFrame[0]));
			memmove(DmaFrame,
					lcdFrame_2
							+ (g_FrameTotalBytesCopied - CCMDATA_SIZE)
									/ sizeof(lcdFrame_2[0]),
					sizeToCopy * sizeof(DmaFrame[0]));
		}
		g_FrameTotalBytesCopied += sizeToCopy * sizeof(DmaFrame[0]);
		DMA_Cmd( DMA2_Stream3, DISABLE);
		DMA_ClearFlag( DMA2_Stream3, DMA_FLAG_TCIF3);
		DMA_SetCurrDataCounter( DMA2_Stream3, sizeToCopy);
		DMA_Cmd( DMA2_Stream3, ENABLE);
	}
}
void FRAME_ImageToLcdBuffer(const unsigned char *pImage, int16_t *pCoordinates,
		bool bAND) {
	uint32_t framePixelIndex;
	int16_t x = pCoordinates[0], y = pCoordinates[1];
	int16_t i, j = pCoordinates[3] - y;
	uint16_t *pImg = (uint16_t*) pImage;
	do {
		framePixelIndex = x + y++ * LCD_PIXEL_WIDTH;
		i = pCoordinates[2] - x;
		do {
			if (framePixelIndex <= CCMDATA_SIZE / 2) {
				lcdFrame_1[framePixelIndex] =
						bAND ? (lcdFrame_1[framePixelIndex] & *pImg++) : *pImg++;
			} else {
				lcdFrame_2[framePixelIndex - CCMDATA_SIZE / 2 - 1] =
						bAND ? (lcdFrame_2[framePixelIndex - CCMDATA_SIZE / 2
								- 1] & *pImg++) :
								*pImg++;
			}
			framePixelIndex = (framePixelIndex + 1) % FRAME_SIZE;
		} while (--i);
	} while (--j);
}
/**
 * @brief
 * @param  None
 * @retval None
 */
void LIS302DL_Reset(void) {
	uint8_t ctrl = 0;

	LIS302DL_InitTypeDef LIS302DL_InitStruct;
	LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;
	/* Set configuration of LIS302DL*/
	LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
	LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
	LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE
			| LIS302DL_Z_ENABLE;
	LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
	LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
	LIS302DL_Init(&LIS302DL_InitStruct);
	/* Set configuration of Internal High Pass Filter of LIS302DL*/
	LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
	LIS302DL_InterruptStruct.SingleClick_Axes =
			LIS302DL_CLICKINTERRUPT_Z_ENABLE;
	LIS302DL_InterruptStruct.DoubleClick_Axes =
			LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
	LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);
	/* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate
	 = 3/100 = 30ms */
	Delay(30);
	/* Configure Click Window */
	ctrl = 0xC0;
	LIS302DL_Write(&ctrl, LIS302DL_CLICK_CTRL_REG3_ADDR, 1);
}
//195
