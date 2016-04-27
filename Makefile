########################  user modify here

#the name of the app
BINARY = app_main
ASM_SRC += $(shell find  ./src -name '*.s')
C_SRC+= $(shell find  ./src -name '*.c')
C_SRC+= $(shell find  ./ppz_stm32 -name '*.c')
OBJS += $(ASM_SRC:%.s=%.o)
OBJS += $(C_SRC:%.c=%.o)
LDSCRIPT = ./libopencm3/lib/stm32/f1/stm32f103xc.ld
CFLAGS += -I ./mavlinkNew -I src -I ./ppz_stm32/
CFLAGS += -std=c99

CFLAGS += -DBOARD_CONFIG=\<lisa_l_1.0.h\>
# comment it for do init in app, no in mcu_init()
CFLAGS += -DPERIPHERALS_AUTO_INIT

CFLAGS += -DUART_RX_BUFFER_SIZE=1024
CFLAGS += -DUART_TX_BUFFER_SIZE=1024
CFLAGS += -DUSE_UART1=1
CFLAGS += -DUSE_UART2=1
CFLAGS += -DUSE_UART3=1
#CFLAGS += -DUSE_UART5=1
CFLAGS += -DUART1_BAUD=115200
CFLAGS += -DUART2_BAUD=115200
CFLAGS += -DUART3_BAUD=57600
#CFLAGS += -DUART5_BAUD=115200

#CFLAGS += -DUSE_UART1_TX=TRUE
#CFLAGS += -DUSE_UART1_RX=FALSE
#CFLAGS += -DUART1_HW_FLOW_CONTROL=FALSE
#CFLAGS += -DUART1_BITS=UBITS_8
#CFLAGS += -DUART1_STOP=USTOP_2
#CFLAGS += -DUART1_PARITY=UPARITY_EVEN

#CFLAGS += -DUSE_UART2_TX=TRUE
#CFLAGS += -DUSE_UART2_RX=FALSE
#CFLAGS += -DUART2_HW_FLOW_CONTROL=FALSE
#CFLAGS += -DUART2_BITS=UBITS_8
#CFLAGS += -DUART2_STOP=USTOP_2
#CFLAGS += -DUART2_PARITY=UPARITY_EVEN




#CFLAGS += -DGPIOE
#CFLAGS += -DGPIOF
#CFLAGS += -DGPIOG
#CFLAGS += -DGPIOH
#CFLAGS += -DGPIOI

#CFLAGS += -DUSE_I2C0=1
#CFLAGS += -DUSE_I2C1=1
#CFLAGS += -DUSE_I2C2=1
#CFLAGS += -DUSE_I2C3=1


#//USE_AD1 mean use adc1 , USE_ADC_1 mean use the number 1 adc pin( for user view)
#// after default USE_ADC_1 1, there are some describe for it: { the real channel, real gpio  }
#//each adc has 4 channel (can be more), like here , AD1 will work out number 1-4 adc pin
CFLAGS += -DUSE_AD1
CFLAGS += -DUSE_AD2
CFLAGS += -DUSE_ADC_1
CFLAGS += -DUSE_ADC_2
CFLAGS += -DUSE_ADC_3
CFLAGS += -DUSE_ADC_4
CFLAGS += -DUSE_ADC_5
CFLAGS += -DUSE_ADC_6

#//choose the emiter, uart or spi
#CFLAGS += -DUSE_SPI1=1
#CFLAGS += -DSPI_MASTER
#CFLAGS += -DUSE_SPI_SLAVE0=1


CFLAGS += -DUSE_HANDSET=0
CFLAGS += -DUSE_SIMPLE_USB_SERIAL=0
CFLAGS += -DUSE_IMU60x0=0
CFLAGS += -DUSE_TIMER4_COUNTER_HANDSET=0
CFLAGS += -DUSE_IMU_AHRS_BOARD=0

CFLAGS += -DZFRAME_SENDER
CFLAGS += -DZFRAME_SENDER_USE_USB_CDCAM_CONFIG_UART=0
#CFLAGS += -DZFRAME_RECIVER
CFLAGS += -DUSE_PIX_SBUS_PROTOCOL=1

CFLAGS += -w

##+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
##++++++++++++++++++++++++++++++++++++++++++++++ copy the makefile in libopencm3-example: examples/stm32/f1/Makefile.include
##+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

LIBNAME		= opencm3_stm32f1
DEFS		= -DSTM32F1

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

################################################################################
# OpenOCD specific variables

OOCD		?= openocd
OOCD_INTERFACE	?= flossjtag
OOCD_BOARD	?= olimex_stm32_h103

################################################################################
# Black Magic Probe specific variables
# Set the BMP_PORT to a serial port and then BMP is used for flashing
BMP_PORT	?=

################################################################################
# texane/stlink specific variables
#STLINK_PORT	?= :4242













##+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
###+++++++++++++++++++++++++++++++++++++++++++ copy the makefile in libopencm3-example: examples/Makefile.include
##+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
## Copyright (C) 2013 Frantisek Burian <BuFran@seznam.cz>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q		:= @
NULL		:= 2>/dev/null
endif

###############################################################################
# Executables

PREFIX		?= arm-none-eabi

CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb
STFLASH		= $(shell which st-flash)
STYLECHECK	:= /checkpatch.pl
STYLECHECKFLAGS	:= --no-tree -f --terse --mailback
STYLECHECKFILES	:= $(shell find . -name '*.[ch]')


###############################################################################
# Source files

LDSCRIPT	?= $(BINARY).ld

#OBJS		+= $(BINARY).o


ifeq ($(strip $(OPENCM3_DIR)),)
# user has not specified the library path, so we try to detect it

# where we search for the library
LIBPATHS := ./libopencm3 ../../../../libopencm3 ../../../../../libopencm3

OPENCM3_DIR := $(wildcard $(LIBPATHS:=/locm3.sublime-project))
OPENCM3_DIR := $(firstword $(dir $(OPENCM3_DIR)))

ifeq ($(strip $(OPENCM3_DIR)),)
$(warning Cannot find libopencm3 library in the standard search paths.)
$(error Please specify it through OPENCM3_DIR variable!)
endif
endif

ifeq ($(V),1)
$(info Using $(OPENCM3_DIR) path to library)
endif

INCLUDE_DIR	= $(OPENCM3_DIR)/include
LIB_DIR		= $(OPENCM3_DIR)/lib
SCRIPT_DIR	= $(OPENCM3_DIR)/scripts

###############################################################################
# C flags

CFLAGS		+= -Os -g
CFLAGS		+= -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS		+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS		+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C++ flags

CXXFLAGS	+= -Os -g
CXXFLAGS	+= -Wextra -Wshadow -Wredundant-decls  -Weffc++
CXXFLAGS	+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C & C++ preprocessor common flags

CPPFLAGS	+= -MD
CPPFLAGS	+= -Wall -Wundef
CPPFLAGS	+= -I$(INCLUDE_DIR) $(DEFS)

###############################################################################
# Linker flags

LDFLAGS		+= --static -nostartfiles
LDFLAGS		+= -L$(LIB_DIR)
LDFLAGS		+= -T$(LDSCRIPT)
LDFLAGS		+= -Wl,-Map=$(*).map
LDFLAGS		+= -Wl,--gc-sections
ifeq ($(V),99)
LDFLAGS		+= -Wl,--print-gc-sections
endif

###############################################################################
# Used libraries

LDLIBS		+= -l$(LIBNAME)
LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

###############################################################################
###############################################################################
###############################################################################

.SUFFIXES: .elf .bin .hex .srec .list .map .images
.SECONDEXPANSION:
.SECONDARY:

all: elf

elf: $(BINARY).elf
bin: $(BINARY).bin
hex: $(BINARY).hex
srec: $(BINARY).srec
list: $(BINARY).list

images: $(BINARY).images
flash: $(BINARY).flash

$(LDSCRIPT):
    ifeq (,$(wildcard $(LDSCRIPT)))
        $(error Unable to find specified linker script: $(LDSCRIPT))
    endif

%.images: %.bin %.hex %.srec %.list %.map
	@#printf "*** $* images generated ***\n"

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@#printf "  OBJCOPY $(*).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@#printf "  OBJCOPY $(*).srec\n"
	$(Q)$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@#printf "  OBJDUMP $(*).list\n"
	$(Q)$(OBJDUMP) -S $(*).elf > $(*).list

%.elf %.map: $(OBJS) $(LDSCRIPT) $(LIB_DIR)/lib$(LIBNAME).a
	@#printf "  LD      $(*).elf\n"
	$(Q)$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $(*).elf

%.o: %.c
	@#printf "  CC      $(*).c\n"
	$(Q)$(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(*).o -c $(*).c

%.o: %.cxx
	@#printf "  CXX     $(*).cxx\n"
	$(Q)$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(*).o -c $(*).cxx

%.o: %.cpp
	@#printf "  CXX     $(*).cpp\n"
	$(Q)$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(*).o -c $(*).cpp

clean:
	@#printf "  CLEAN\n"
	$(Q)rm -f *.o *.d *.elf *.bin *.hex *.srec *.list *.map
	$(Q)rm -f src/*.o src/*.d
	$(Q)rm -f ppz_stm32/*.o ppz_stm32/*.d ppz_stm32/mcu_periph/*.o ppz_stm32/mcu_periph/*.d

stylecheck: $(STYLECHECKFILES:=.stylecheck)
styleclean: $(STYLECHECKFILES:=.styleclean)

# the cat is due to multithreaded nature - we like to have consistent chunks of text on the output
%.stylecheck: %
	$(Q)$(SCRIPT_DIR)$(STYLECHECK) $(STYLECHECKFLAGS) $* > $*.stylecheck; \
		if [ -s $*.stylecheck ]; then \
			cat $*.stylecheck; \
		else \
			rm -f $*.stylecheck; \
		fi;

%.styleclean:
	$(Q)rm -f $*.stylecheck;


%.stlink-flash: %.bin
	@printf "  FLASH  $<\n"
	$(Q)$(STFLASH) write $(*).bin 0x8000000

ifeq ($(STLINK_PORT),)
ifeq ($(BMP_PORT),)
ifeq ($(OOCD_SERIAL),)
%.flash: %.hex
	@printf "  FLASH   $<\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "init" -c "reset init" \
		    -c "flash write_image erase $(*).hex" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
else
%.flash: %.hex
	@printf "  FLASH   $<\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		    -f board/$(OOCD_BOARD).cfg \
		    -c "ft2232_serial $(OOCD_SERIAL)" \
		    -c "init" -c "reset init" \
		    -c "flash write_image erase $(*).hex" \
		    -c "reset" \
		    -c "shutdown" $(NULL)
endif
else
%.flash: %.elf
	@printf "  GDB   $(*).elf (flash)\n"
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(BMP_PORT)' \
		   -x $(SCRIPT_DIR)/black_magic_probe_flash.scr \
		   $(*).elf
endif
else
%.flash: %.elf
	@printf "  GDB   $(*).elf (flash)\n"
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(STLINK_PORT)' \
		   -x $(SCRIPT_DIR)/stlink_flash.scr \
		   $(*).elf
endif

.PHONY: images clean stylecheck styleclean elf bin hex srec list

-include $(OBJS:.o=.d)

