CC := g++
CFLAGS := -Wall -std=c++14 -pthread

SRCS = $(wildcard *.cpp)
OBJS := $(patsubst %.cpp,%.o,$(SRCS))
	
debug: CFLAGS += -g
debug: main

release: CFLAGS += -O3
release: main

profile: CFLAGS += -Og -pg
profile: main

main: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) -c $<

clean:
	rm -rf *.o

.PHONY: all clean
