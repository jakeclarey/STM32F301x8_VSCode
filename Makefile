###################################################################################################
# Makefile rev 1.02 [4/25/24]
###################################################################################################

# Project folder name holder (default to build Blinky if not specified in tasks.json or on CLI)
FN = Blinky

# make target should be named "main"
TARGET = main

# WARNING: when optimizing for speed, floating point operations can be corrupted.
DEBUG = 1
OPT = -O0 # preferred optimization. In order for speed: O0, O1, O2, O3, Ofast.

# Build path
BUILD_DIR = !build

###################################################################################################
# sources
###################################################################################################
# C sources
C_SOURCES = $(wildcard $(FN)/Src/*.c)

# C++ sources
CPP_SOURCES = $(wildcard $(FN)/Src/*.cpp)

# ASM sources
ASM_SOURCES = $(wildcard $(FN)/Src/*.s)
ASM_SOURCES += $(wildcard !DeviceSpecific/*.s)

###################################################################################################
# binaries
###################################################################################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CPP = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CPP = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
###################################################################################################
# CFLAGS
###################################################################################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32F446xx

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-I$(FN)/Inc \
-I!DeviceSpecific/Drivers/CMSIS \
-I!DeviceSpecific/Drivers/Device

CPP_INCLUDES = \
-I$(FN)/Core/Inc \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CPPFLAGS = $(CPP_INCLUDES) -std=c++17 -Wno-register

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

###################################################################################################
# LDFLAGS
###################################################################################################
# link script
LDSCRIPT = !DeviceSpecific/STM32F301K8T6_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) --specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

###################################################################################################
# help
###################################################################################################
.PHONY: help

# default action: help
help:
	@echo -----------------------------------------------------------------------------------------
	@echo Available Targets:
	@echo 	flash: Flashes main.bin in build directory to NUCLEO-F446RE via stlink.
	@echo 		Usage: make flash
	@echo 	all FN=folder_name: Builds specified project.
	@echo 		Usage: make all FN=folder_name
	@echo 	clean: Cleans out the build directory.
	@echo 		Usage: make clean
	@echo 	help: Display this help text. 
	@echo 		Usage: make help
	@echo -----------------------------------------------------------------------------------------

###################################################################################################
# flash
###################################################################################################
.PHONY: flash

flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash --reset write $< 0x08000000

###################################################################################################
# all
###################################################################################################
.PHONY: all

all: clean | $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

###################################################################################################
# build the application
###################################################################################################
# list of C objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
# list of C++ objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CPP) -c $(CFLAGS) $(CPPFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@ -G

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@
	$(SZ) $@ --target=binary -G	

$(BUILD_DIR): 
	mkdir $@		

###################################################################################################
# clean
###################################################################################################
.PHONY: clean

ifeq ($(OS), Windows_NT)
clean:
	del /f /Q $(BUILD_DIR)\

else
clean:
	rm -f $(BUILD_DIR)/*

endif

###################################################################################################
# dependencies
###################################################################################################
-include $(wildcard $(BUILD_DIR)/*.d)

# EOF #
