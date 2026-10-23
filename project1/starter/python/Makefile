.PHONY: build run map clean
build:
	python3 -m py_compile src/main.py
run:
	python3 src/main.py
map:
	@test -f maps/student_map.json || { echo "Create your own maps/student_map.json first; see maps/README.md" >&2; exit 1; }
	python3 tools/map_to_rosbridge.py maps/student_map.json
clean:
	rm -rf src/__pycache__
