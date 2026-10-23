.PHONY: build run demo clean
EZXML := third_party/ezxml
build:
	mkdir -p build
	cc -std=c17 -O2 -D_POSIX_C_SOURCE=200809L -o main src/main.c
	cc -std=c17 -O2 -D_POSIX_C_SOURCE=200809L -DEZXML_NOMMAP -c $(EZXML)/ezxml.c -o build/ezxml.o
	cc -std=c17 -O2 -D_POSIX_C_SOURCE=200809L -I$(EZXML) -o build/urdf_example src/urdf_example.c build/ezxml.o
run:
	./main
demo:
	@echo "TODO: make demo (recommended, ungraded): make run plus your joint_state_publisher and finite_state_machine" >&2
	@exit 1
clean:
	rm -rf main build
