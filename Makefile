CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Werror -Iinclude
LDFLAGS ?= -lpthread -lm

TARGET  := bin/qnx_rt_watchdog
SRCS    := src/sensor_fusion.c src/qnx_rt_watchdog.c

.PHONY: all clean test qnx

all: $(TARGET)

$(TARGET): $(SRCS) include/qnx_rt.h
	@mkdir -p bin
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

qnx:
	$(MAKE) CC=qcc CFLAGS="-Vgcc_ntoaarch64le -O2 -Wall -Wextra -Iinclude" LDFLAGS="-lm"

test: $(TARGET)
	python3 -m unittest discover -s tests -v

clean:
	rm -rf bin
