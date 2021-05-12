CC = gcc
CFLAGS = -Wall -O3 -static -g -pthread # -Wno-unused-result # with static linking for better debug @ valgrin

all: simulator

simulator:
	$(CC) $(CFLAGS) *.c -o simulator

dev: clean customtest

# customtest: simulator
# 	mkdir test_dump
# 	for i in 3 4 5 6 7 8 9 10; do \
# 		./world < "tests/cases/inp$${i}.txt" > "test_dump/test$${i}.txt"; \
# 		diff "test_dump/test$${i}.txt" "tests/outputs/out$${i}.txt"; \
# 	done
# 	rm -rf test_dump

clean:
	rm -rf *.o simulator

rebuild: clean all

#.PHONY: all clean
.PHONY: all tests clean