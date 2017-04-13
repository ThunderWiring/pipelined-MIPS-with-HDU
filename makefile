# All Rights Reserved - 2016
# Technion - Israel Institute of Technology, Faculty of Electrical Engineering.
 
# 046267 Computer Architecture - Spring 2016 - HW #1
# makefile for test environment

all: sim_main

# Automatically detect whether the core is C or C++
# Must have either sim_core.c or sim_core.cpp - NOT both
SRC_CORE = $(wildcard sim_core.c sim_core.cpp)
SRC_GIVEN = sim_main.c sim_mem.c
EXTRA_DEPS = sim_api.h

OBJ_GIVEN = $(patsubst %.c,%.o,$(SRC_GIVEN))
OBJ_CORE = sim_core.o
OBJ = $(OBJ_GIVEN) $(OBJ_CORE)

#$(info OBJ=$(OBJ))

CFLAGS = -std=c99

$(OBJ_GIVEN): %.o: %.c
	gcc -c $(CFLAGS) -o $@ $^

ifeq ($(SRC_CORE),sim_core.c)
sim_main: $(OBJ)
	gcc -o $@ $(OBJ)

sim_core.o: sim_core.c
	gcc -c $(CFLAGS) -o $@ $^

else
sim_main: $(OBJ)
	g++ -o $@ $(OBJ)

sim_core.o: sim_core.cpp
	g++ -c -o $@ $^
endif

.PHONY: clean
clean:
	rm -f sim_main $(OBJ_GIVEN) $(OBJ_CORE)
