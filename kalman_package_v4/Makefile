CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
LDLIBS = -lm
CORE = kalman_interface.c kalman_internal.c kalman_math.c
HEADERS = $(wildcard *.h)

all: kalman_filter
kalman_filter: kalman_filter.c $(CORE) $(HEADERS)
	$(CC) $(CFLAGS) kalman_filter.c $(CORE) -o $@ $(LDLIBS)
test_core: tests/test_core.c $(CORE) $(HEADERS)
	$(CC) $(CFLAGS) -I. tests/test_core.c $(CORE) -o $@ $(LDLIBS)
test: kalman_filter test_core
	./test_core
	python3 tests/test_cli.py
clean:
	rm -f kalman_filter test_core
.PHONY: all test clean
