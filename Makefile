.PHONY: all build cmake clean format

BUILD_DIR := build
BUILD_TYPE ?= Debug

all: build

${BUILD_DIR}/Makefile:
	cmake \
		-B${BUILD_DIR} \
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE}

cmake: ${BUILD_DIR}/Makefile

build: cmake
	$(MAKE) -C ${BUILD_DIR} --no-print-directory -j4

clean:
	rm -rf $(BUILD_DIR)

flash: build 
	st-flash --reset write ${BUILD_DIR}/$(notdir $(CURDIR)).bin 0x08000000

