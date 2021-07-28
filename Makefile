# enable app support
APP=1
APP_STACKSIZE=300

# paths & objects & includes
VPATH += src/
PROJ_OBJ += ai_hover.o
PROJ_OBJ += uart_dma_pulp.o
INCLUDES += -Iinc

# location of firmware base
CRAZYFLIE_BASE=../crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile
