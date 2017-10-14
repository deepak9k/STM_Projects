##########################################################################################################################
# Makefile Created by Deepak kumar
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	14-10-2017  ----first version
#  
# ------------------------------------------------


######################################
# target
######################################
TARGET = STM_Projects


######################################
# building variables
######################################

# optimization
OPT = -Os



#######################################
# paths
#######################################
# source path
SOURCES_DIR =  \
~/STM_Projects/Drivers/CMSIS \
~/STM_Projects/Drivers/Device/Stm32f401xE \
~/STM_Projects/User

# Build path
BUILD_DIR = build


######################################
# source
######################################
# C sources

C_SOURCES =  \
User/main.c  \
User/system.c \

# ASM sources
ASM_SOURCES =  \
Drivers/Device/Stm32f401xE/Template/startup_stm32f401xe.s


#######################################
# binaries
#######################################
BINPATH = 
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S


#######################################
# CFLAGS
#######################################
# cpu

CPU = -mcpu=cortex-m4


# mcu
MCU = $(CPU) -mlittle-endian -mthumb 

# C defines
C_DEFS =  \
-DSTM32F401xE

# C includes
C_INCLUDES =  \
-IUser \
-IDrivers/CMSIS/Include \
-IDrivers/Device/Stm32f401xE/Include 


# compile gcc flags
ASFLAGS = $(MCU) $(AS_INCLUDES) $(OPT) -Wall

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F401RETx_FLASH.ld

# libraries
LDFLAGS = $(MCU)  -T$(LDSCRIPT) -Wl,--gc-sections


# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		


#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***

#arm-none-eabi-gcc -mcpu=cortex-m4 -mlittle-endian -mthumb -DSTM32F401xE -T ~/STM32Cube/Projects/STM32F401RE-Nucleo/Templates/TrueSTUDIO/STM32F4xx-Nucleo/STM32F401CE_FLASH.ld -Wl,--gc-sections system.o main.o startup_stm32f401xe.o -o main.elf




#arm-none-eabi-gcc -Wall -mcpu=cortex-m4 -mlittle-endian -mthumb -I ~/STM32Cube/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I ~/STM32Cube/Drivers/CMSIS/Include -DSTM32F401xE -Os -c system.c -o system.o

#openocd -f /usr/share/openocd/scripts/board/st_nucleo_f4.cfg
