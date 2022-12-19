#
# Cross compile on cygwin
#  Compile -static because shared libraries don't match
#

UNAME := $(shell uname -po)
ifeq ('$(UNAME)', 'unknown Cygwin')
CROSS_COMPILE := arm-none-linux-gnueabihf-
CFLAGS        := -static
export PATH   := /usr/local/gcc-arm-10.3-2021.07-mingw-w64-i686-arm-none-linux-gnueabihf/bin:$(PATH)
endif

#
# Cross compile on WSL
#

UNAME := $(shell uname -po)
ifeq ('$(UNAME)', 'x86_64 GNU/Linux')
CROSS_COMPILE := arm-linux-gnueabihf-
CFLAGS        := -static
endif

#
# Native compiler on target
#

G++    := $(CROSS_COMPILE)g++
CFLAGS := -g -W -Wall  -Os -std=c++11

#
# Build the FPGA loader
#
# If this is not a native compile, transfer the executable from
# the Host to the target.
#

fpga_loader : main.cpp fpga_loader.cpp fpga_loader.hpp Makefile
	$(G++) $(CFLAGS) main.cpp fpga_loader.cpp -o $@
ifneq ('$(UNAME)', 'armv7l GNU/Linux')
	scp -q fpga_loader root@ks10:/home/root
endif

#
# Clean up directory
#

.PHONY: clean
clean:
	rm -f *~ .*~
	rm -f fpga_loader

