################################################################################
 # Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 # $Date: 2016-03-11 12:50:27 -0600 (Fri, 11 Mar 2016) $
 # $Revision: 21840 $
 #
 ###############################################################################

ifeq "$(BOARD_DIR)" ""
$(error BOARD_DIR must be set)
endif

PROJ_CFLAGS+=-DRO_FREQ=96000000

# Source files for this test (add path to VPATH below)
SRCS += board.c
SRCS += stdio.c
SRCS += led.c
SRCS += pb.c
# SRCS += nhd12832.c
# SRCS += max14690.c
# SRCS += mx25.c
# SRCS += mx25_stub.c

# Where to find BSP source files
VPATH += $(BOARD_DIR)/Source
VPATH += $(BOARD_DIR)/../Source

# Where to find BSP header files
IPATH += $(BOARD_DIR)/Include
IPATH += $(BOARD_DIR)/../Include


