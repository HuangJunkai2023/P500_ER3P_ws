SHELL := /bin/bash

BUILD_DIR ?= build
CMAKE ?= cmake
CMAKE_FLAGS ?= -DXCORE_USE_XMATE_MODEL=ON
JOBS ?= 4

.PHONY: all configure phone_teleop_follow_cart clean rebuild help

all: build_all

configure:
	@$(CMAKE) -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)

build_all: configure
	@$(CMAKE) --build $(BUILD_DIR) -j$(JOBS)

phone_teleop_follow_cart: configure
	@$(CMAKE) --build $(BUILD_DIR) --target phone_teleop_follow_cart -j$(JOBS)

clean:
	@$(CMAKE) --build $(BUILD_DIR) --target clean

rebuild:
	@python3 -c "import shutil; shutil.rmtree('$(BUILD_DIR)', ignore_errors=True)"
	@$(MAKE) build_all

help:
	@echo "Usage (run in src/xCoreSDK_cpp-v0.6.0):"
	@echo "  make                         # configure + build ALL targets"
	@echo "  make build_all"
	@echo "  make phone_teleop_follow_cart"
	@echo "  make clean"
	@echo "  make rebuild"
	@echo "Variables:"
	@echo "  BUILD_DIR=build"
	@echo "  JOBS=4"
	@echo "  CMAKE_FLAGS='-DXCORE_USE_XMATE_MODEL=ON'"
