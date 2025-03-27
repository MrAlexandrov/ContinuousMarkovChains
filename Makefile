PROJECT_NAME = project_template

INPUT_FILE = input.txt
OUTPUT_FILE = output.txt

NPROCS ?= $(shell nproc)

BUILD_DIR = build

all: build test run

build:
	@echo "==> Configuring the project_template..."
	@cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -B$(BUILD_DIR) -H.
	@echo "==> Building the project_template..."
	@cmake --build $(BUILD_DIR) -j $(NPROCS)

test: build
	@echo "==> Running tests..."
	@cd $(BUILD_DIR) && ctest --verbose --parallel $(NPROCS)

run: build
	@echo "==> Running ${PROJECT_NAME}"
	@${BUILD_DIR}/${PROJECT_NAME} < ${INPUT_FILE} > ${OUTPUT_FILE}

clang-tidy: build
	@echo "==> Running clang-tidy with $(NPROCS) threads..."
	find include tests -type f \( -name '*.cpp' -o -name '*.hpp' \) -print -o -name 'main.cpp' -print \
	| xargs -P$(NPROCS) -n1 clang-tidy --p=build \
	  --extra-arg=-std=c++20

clean:
	@echo "==> Cleaning up..."
	@rm -rf $(BUILD_DIR)
	@rm -rf coverage project_template.profdata
	@rm *.gp *.dat *.png *.dot

rebuild: clean build

install:
	sudo apt-get update
	sudo apt-get install -y cmake clang libgtest-dev ninja-build

coverage: test
	@llvm-profdata merge -sparse $(BUILD_DIR)/tests/default.profraw -o project_template.profdata
	@llvm-cov show $(BUILD_DIR)/tests/test_project_template -instr-profile=project_template.profdata -format=html -show-branches=count -output-dir=coverage

.PHONY: all build test clean rebuild install
