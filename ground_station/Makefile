BUILD_DIR = $(wildcard build/)

build: build_dir
	cmake --build ${BUILD_DIR}

build_dir:
	@IF exist build ( echo build directory exists ) ELSE ( mkdir build && echo build directory created)

run:
	@cd build && .\ground_station.exe
