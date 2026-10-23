CXX ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -Wpedantic
.PHONY: build run map clean
build:
	mkdir -p build
	$(CXX) $(CXXFLAGS) -o build/project1 src/main.cpp
run: build
	./build/project1
map:
	@test -f maps/student_map.json || { echo "Create your own maps/student_map.json first; see maps/README.md" >&2; exit 1; }
	python3 tools/map_to_rosbridge.py maps/student_map.json
clean:
	rm -rf build
