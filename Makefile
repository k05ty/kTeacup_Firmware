##############################################################################
#                                                                            #
# Teacup - Lean and efficient firmware for RepRap printers                   #
#                                                                            #
# by Triffid Hunter, Traumflug, jakepoz, many others.                        #
#                                                                            #
# This firmware is Copyright (c) ...                                         #
#   2009 - 2010 Michael Moon aka Triffid_Hunter                              #
#   2010 - 2013 Markus "Traumflug" Hitter <mah@jump-ing.de>                  #
#                                                                            #
# This program is free software; you can redistribute it and/or modify       #
# it under the terms of the GNU General Public License as published by       #
# the Free Software Foundation; either version 2 of the License, or          #
# (at your option) any later version.                                        #
#                                                                            #
# This program is distributed in the hope that it will be useful,            #
# but WITHOUT ANY WARRANTY; without even the implied warranty of             #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              #
# GNU General Public License for more details.                               #
#                                                                            #
# You should have received a copy of the GNU General Public License          #
# along with this program; if not, write to the Free Software                #
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA #
#                                                                            #
##############################################################################

##############################################################################
#                                                                            #
# Makefile for AVR (ATmega-based) targets. Use it with                       #
#                                                                            #
#   make -f Makefile-AVR                                                     #
#                                                                            #
# or copy/link it to Makefile for convenience.                               #
#                                                                            #
##############################################################################

##############################################################################
#                                                                            #
# Change these to suit your hardware                                         #
#                                                                            #
##############################################################################

# MCU ?= atmega168
MCU ?= atmega328p
# MCU ?= atmega644
# MCU ?= atmega644
# MCU ?= atmega1284p
# MCU ?= atmega1280
# MCU ?= atmega2560
# MCU ?= at90usb1286
# MCU ?= atmega32u4

# CPU clock rate
# F_CPU ?= 8000000L
F_CPU ?= 16000000L
# F_CPU ?= 20000000L

##############################################################################
#                                                                            #
# Where to find your compiler and linker. Later, this is completed like      #
#   CC = $(TOOLCHAIN)gcc                                                     #
#                                                                            #
##############################################################################

TOOLCHAIN = avr-
# TOOLCHAIN = /usr/bin/avr-
# TOOLCHAIN = <path-to-arduino-folder>/hardware/tools/avr/bin/avr-

##############################################################################
#                                                                            #
# Programmer settings for "make program"                                     #
#                                                                            #
##############################################################################

# avrdude, typical for AVR-based architectures.
#
# Flags:
#   -c <programmer-type>      Typically stk500 or stk500v2.
#   -b <baudrate>             Upload baud rate. Depends on the bootloader in
#                             use. Not used for USB programmers.
#   -p <mcu type>             See MCU above.
#   -P <port>                 Serial port the electronics is connected to.
#   -C <config file>          Optional, default is /etc/avrdude.conf.

UPLOADER ?= avrdude
# UPLOADER = <path-to-arduino-folder>/hardware/tools/avrdude

ifndef UPLOADER_FLAGS
UPLOADER_FLAGS  = -c arduino
# UPLOADER_FLAGS += -b 19200
UPLOADER_FLAGS += -b 57600
# UPLOADER_FLAGS += -b 115200
UPLOADER_FLAGS += -p $(MCU)
# UPLOADER_FLAGS += -P COM1
# UPLOADER_FLAGS += -P /dev/ttyACM0
UPLOADER_FLAGS += -P /dev/ttyUSB0
# UPLOADER_FLAGS += -C <path-to-arduino-folder>/hardware/tools/avrdude.conf
endif


##############################################################################
#                                                                            #
# Below here, defaults should be ok.                                         #
#                                                                            #
##############################################################################

PROGRAM = teacup

# The thing we build by default, and also the thing we clean.
TARGET = $(PROGRAM).hex

# Arduino IDE takes the "compile everything available"-approach, so we have
# to keep this working and can take a shortcut:
SOURCES = $(wildcard *.c)

# Link time optimization is on by default
ifeq ($(USE_FLTO),)
  USE_FLTO=yes
endif
CFLAGS  = $(EXTRA_CFLAGS)
CFLAGS += -DF_CPU=$(F_CPU)
CFLAGS += -DMCU_STR=\"$(MCU)\"
CFLAGS += -mmcu=$(MCU)
CFLAGS += -g
CFLAGS += -Wall
CFLAGS += -Wstrict-prototypes
CFLAGS += -std=gnu99
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -Winline
CFLAGS += -fno-move-loop-invariants
CFLAGS += -fno-tree-scev-cprop
CFLAGS += -Os
CFLAGS += -ffunction-sections
CFLAGS += -finline-functions-called-once
CFLAGS += -mcall-prologues
ifeq ($(USE_FLTO),yes)
  CFLAGS += -flto
endif
CFLAGS += -Wa,-adhlns=$(@:.o=.al)
#CFLAGS += -dM -E # To list all predefined macros into the .o file.

LDFLAGS  = -Wl,--as-needed
LDFLAGS += -Wl,--gc-sections

ifneq ($(realpath ../simulavr/src/simulavr_info.h),)
  # Neccessary for simulavr support, doesn't hurt others.
  CFLAGS += -DSIMINFO
  LDFLAGS += -Wl,--section-start=.siminfo=0x900000
  LDFLAGS += -u siminfo_device
  LDFLAGS += -u siminfo_cpufrequency
  LDFLAGS += -u siminfo_serial_in
  LDFLAGS += -u siminfo_serial_out
endif

LIBS  = -lm

-include Makefile-common

# Architecture specific targets

.PHONY: all program size

all: $(PROGRAM).hex $(BUILDDIR)/$(PROGRAM).lst $(BUILDDIR)/$(PROGRAM).sym size

program: $(PROGRAM).hex config.h
	$(UPLOADER) $(UPLOADER_FLAGS) -U flash:w:$(PROGRAM).hex

## Interpret TARGET section sizes wrt different AVR chips.
## Usage: $(call show_size,section-name,section-regex,168-size,328p-size,644p-size,1280-size)
define show_size
	@$(SIZE) -A $^ | perl -MPOSIX -ne \
	                 '/\.($2)\s+([0-9]+)/g && \
		             do {$$data += $$2}; \
					 END { printf "%8s %6d bytes      %3d%%      %3d%%      %3d%%      %3d%%\n", \
					 	 $1, $$data, \
	         		 	 ceil($$data * 100 / ($3 * 1024)), \
	         		 	 ceil($$data * 100 / ($4 * 1024)), \
	         		 	 ceil($$data * 100 / ($5 * 1024)), \
	         		 	 ceil($$data * 100 / ($6 * 1024));\
	                 }'
endef

size: $(BUILDDIR)/$(PROGRAM).elf
	@echo "ATmega sizes               '168   '328(P)   '644(P)     '1280"
	$(call show_size,FLASH,text|data|bootloader,14,30,62,126)
	$(call show_size,RAM,data|bss|noinit,1,2,4,8)
	$(call show_size,EEPROM,eeprom,1,2,2,4)

.PHONY: simulavr-check simulavr performancetest
simulavr-check:
	@if [ ! -x ../simulavr/src/simulavr ]; then \
	  echo "Can't find SimulAVR executable in ../simulavr/src/simulavr."; \
	  echo "Please install and build it next to the Teacup_Firmware folder."; \
	  echo "Sources can be found at https://github.com/Traumflug/simulavr"; \
	  false; \
	fi

simulavr:
	@$(MAKE) --no-print-directory -f Makefile-AVR simulavr-check
	@echo "Compiling for SimulAVR and running the result in SimulAVR."
	$(MAKE) -f Makefile-AVR USER_CONFIG=testcases/config.h.Profiling \
	  MCU=atmega644 F_CPU=20000000UL all
	../simulavr/src/simulavr -f build/testcases/config.h.Profiling/teacup.elf

performancetest:
	@$(MAKE) --no-print-directory -f Makefile-AVR simulavr-check
	@echo "Compiling for SimulAVR and running performance tests."
	$(MAKE) -f Makefile-AVR USER_CONFIG=testcases/config.h.Profiling \
	  MCU=atmega644 F_CPU=20000000UL all
	cd testcases && USER_CONFIG=config.h.Profiling \
	  ./run-in-simulavr.sh short-moves.gcode smooth-curves.gcode triangle-odd.gcode
