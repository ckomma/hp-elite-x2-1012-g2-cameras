CC ?= gcc
CFLAGS ?= -O3 -Wall -Wextra -D_GNU_SOURCE
TARGET = hp-ir-feeder

all: $(TARGET)

$(TARGET): hp-ir-feeder.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(TARGET)

.PHONY: all clean
