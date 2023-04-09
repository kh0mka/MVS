# Target file name (without extension)
TARGET=MVS_LW42_ACC_LCD_CCM2

# Define commands
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
STLINK=C:\Program Files\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\st-link_cli.exe
LDSCRIPT=standalone.ld
REMOVE=	rm -f -r
COPY= cp -f
MOVE= mv -f
REMOVEDIR = rmdir
MAKEDIR= mkdir

CMSIS_PATH = ../Libraries/CMSIS
STD_PeriphDriver_PATH = ../Libraries/STM32F4xx_StdPeriph_Driver
DISCOVERY_PATH = ../Libraries/STM32F4-Discovery
USB_OTG_PATH = ../Libraries/STM32_USB_OTG_Driver
USB_DEVICE_PATH = ../Libraries/STM32_USB_Device_Library


LINKER_FLAGS=-nostartfiles -Xlinker -o$(TARGET).elf -Xlinker -M -Xlinker -Map=$(TARGET).map

DEBUG=-g
OPTIM=-O1
M_CPU=cortex-m4

CFLAGS=$(DEBUG) -I. \
	-I $(CMSIS_PATH)/Include \
	-I $(CMSIS_PATH)/ST/STM32F4xx/Include \
	-I $(STD_PeriphDriver_PATH)/inc \
	-I $(DISCOVERY_PATH)/inc \
	-I $(USB_OTG_PATH)/inc \
	-I $(USB_DEVICE_PATH)/Core/inc \
	-I $(USB_DEVICE_PATH)/Class/hid/inc \
	-mthumb \
	-D STM32F4XX \
	-D HSE_VALUE=8000000 \
	-D USE_DEVICE_MODE \
	-D USE_USB_OTG_FS \
	-D USE_STDPERIPH_DRIVER \
	-D USE_DEFAULT_TIMEOUT_CALLBACK \
	-mcpu=$(M_CPU) $(OPTIM) \
	-T $(LDSCRIPT) -ggdb

SOURCE= main.c \
	    startup.c \
	    BaseImage.c \
	    $(CMSIS_PATH)/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
	    $(STD_PeriphDriver_PATH)/src/misc.c \
	    $(STD_PeriphDriver_PATH)/src/stm32f4xx_rcc.c \
	    $(STD_PeriphDriver_PATH)/src/stm32f4xx_gpio.c \
	    $(STD_PeriphDriver_PATH)/src/stm32f4xx_fsmc.c \
	    $(STD_PeriphDriver_PATH)/src/stm32f4xx_i2c.c \
	    $(STD_PeriphDriver_PATH)/src/stm32f4xx_spi.c \
	    $(STD_PeriphDriver_PATH)/src/stm32f4xx_tim.c \
	    $(DISCOVERY_PATH)/src/stm32f4_discovery_lcd.c \
	    $(DISCOVERY_PATH)/src/fonts.c  \
        $(DISCOVERY_PATH)/src/stm32f4_discovery_lis302dl.c \



OBJS = $(SOURCE:.c=.o)

all: begin Debug $(TARGET).bin size finished move end
$(TARGET).bin : $(TARGET).elf
	$(OBJCOPY) $(TARGET).elf -O ihex $(TARGET).hex
$(TARGET).elf : $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(LINKER_FLAGS)
$(OBJS) : %.o : %.c
	$(CC) -c $(CFLAGS) $< -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -o $@
size:
	 $(SIZE) --format=berkeley $(TARGET).elf

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_MOVE = ------  Moving output files into Debug folder  ------
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
begin:
	@echo
	@echo $(MSG_BEGIN)
finished:
	@echo $(MSG_ERRORS_NONE)
move: Debug
	@echo
	@echo $(MSG_MOVE)
	@echo
	$(MOVE) $(OBJS) Debug
	$(MOVE) $(TARGET).elf Debug
	$(MOVE) $(TARGET).hex Debug
	$(MOVE) $(TARGET).map Debug
end:
	@echo $(MSG_END)
	@echo
Debug:
	$(MAKEDIR) Debug
clean : begin clean_list end
clean_list :
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OBJS)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) Debug
download : begin program end
program:
	$(STLINK) -p Debug/$(TARGET).hex -run