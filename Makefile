CC = gcc
CFLAGS = -Wall -Wextra -O2
LDFLAGS = -pthread -lm
TARGET = vehicule

# Source files - ADD sonar.c here!
SRCS = main.c pwm.c imu.c lsm9ds1.c sonar.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
	@echo "Clean complete"

run: $(TARGET)
	sudo ./$(TARGET)

.PHONY: all clean run
