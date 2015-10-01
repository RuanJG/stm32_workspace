##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

#the name of the app
BINARY = app_main
ASM_SRC += $(shell find  ./ -name '*.s')
C_SRC+= $(shell find  ./ -name '*.c')
OBJS += $(ASM_SRC:%.s=%.o)
OBJS += $(C_SRC:%.c=%.o)

CFLAGS += -I ./mavlinkNew

LDSCRIPT = ../stm32f103rbt6.ld

include ../../Makefile.include

