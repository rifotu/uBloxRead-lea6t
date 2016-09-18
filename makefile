CC=gcc
CFLAGS= -g -Wall -std=gnu99
LFLAGS= -pthread -lrt -lm
INCLUDE = -I../inc
OBJ = serial.o
TARGET = serial

all: $(TARGET)

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) $(INCLUDE) -c -o $@ $<

$(TARGET): $(OBJ)
	$(CC) -o $(TARGET) $^ $(LFLAGS)

clean:
	rm ./${TARGET} ./*.o
