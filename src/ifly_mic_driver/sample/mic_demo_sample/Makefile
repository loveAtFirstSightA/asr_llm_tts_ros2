#common makefile header

DIR_INC = ../../include
DIR_BIN = ../../bin
DIR_LIB = ../../lib

TARGET	= mic_demo_sample
BIN_TARGET = $(DIR_BIN)/$(TARGET)

CROSS_COMPILE = 
CFLAGS = -g -Wall -I$(DIR_INC)

ifdef LINUX_X64
LDFLAGS := -L$(DIR_LIB)/x64 
endif

ifdef LINUX_X86
LDFLAGS := -L$(DIR_LIB)/x32 
endif

ifdef LINUX_ARM64
LDFLAGS := -L$(DIR_LIB)/arm64
endif

ifdef LINUX_ARM32
LDFLAGS := -L$(DIR_LIB)/arm32
endif

LDFLAGS += -lhid_lib

OBJECTS := $(patsubst %.c,%.o,$(wildcard *.c))

$(BIN_TARGET) : $(OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)

%.o : %.c
	$(CROSS_COMPILE)gcc -c $(CFLAGS) $< -o $@ $(LDFLAGS)
clean:
	@rm -f *.o $(BIN_TARGET)

.PHONY:clean

#common makefile foot
