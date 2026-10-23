.PHONY: build run demo clean
PUGIXML := third_party/pugixml
build:
	mkdir -p build
	g++ -std=c++20 -O2 -o main src/main.cpp
	g++ -std=c++20 -O2 -I$(PUGIXML) -o build/urdf_example src/urdf_example.cpp $(PUGIXML)/pugixml.cpp
run:
	./main
demo:
	@echo "TODO: make demo (recommended, ungraded): make run plus your joint_state_publisher and finite_state_machine" >&2
	@exit 1
clean:
	rm -rf main build
