CC = g++
CFLAGS = -Wall -g
TARGET = output.out

all: $(TARGET) run clean
output.out:	main.o shape.o
	$(CC) $(CFLAGS) -o output.out main.o shape.o
shape.o:	shape.h
	$(CC) $(CFLAGS) -c shape.cpp
main.o:	shape.cpp shape.h
	$(CC) $(CFLAGS) -c main.cpp
run:
	./$(TARGET)
clean:
	rm $(TARGET)
	rm -f *.o
