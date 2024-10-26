CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g
LIBS = -lpng -lm

all: drone_simulation

drone_simulation: drone_simulation.o
	$(CC) $(CFLAGS) -o drone_simulation drone_simulation.o $(LIBS)

drone_simulation.o: drone_simulation.c
	$(CC) $(CFLAGS) -c drone_simulation.c -o drone_simulation.o

clean:
	rm -f drone_simulation drone_simulation.o
