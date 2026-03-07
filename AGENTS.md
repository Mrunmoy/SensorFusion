# Repository Guidelines

## Project Structure & Module Organization
- `drivers/`: portable sensor drivers (`mpu6050/`, `lsm6dso/`, `bmm350/`, etc.), HAL interfaces in `drivers/hal/`, calibration and factory test helpers.
- `middleware/`: fusion and processing layers (`ahrs/`, `hub/`, `codec/`, `altitude/`, `ecg/`, `motion/`).
- `test/`: GoogleTest/GMock unit tests plus HAL mocks in `test/mocks/`.
- `main/`: optional stub application entrypoint.
- `docs/`: design notes and review documents; `datasheets/`: vendor PDFs; `scripts/`: utility scripts (for example, size report generation).

## Build, Test, and Development Commands
- `cmake -B build -DCMAKE_BUILD_TYPE=Release`: configure a host build.
- `cmake --build build --parallel`: compile drivers, middleware, and tests.
- `./build/test/driver_tests`: run the full unit test binary.
- `ctest --test-dir build --output-on-failure`: run discovered tests with CTest output.
- `cmake -B build -DSENSORFUSION_BUILD_TESTS=OFF`: library-only build.
- `cmake -B build -DSENSORFUSION_BUILD_APP=ON && cmake --build build`: build the stub app.
- `scripts/generate_size_report.sh build _site`: generate static size-report pages.

## Coding Style & Naming Conventions
- Language level is C++17 (`CMAKE_CXX_STANDARD 17`).
- Match existing style: 4-space indentation, braces on same line, and concise comments.
- Types/classes use `PascalCase` (`MahonyAHRS`, `CalibrationStore`); methods use `camelCase` (`readRegister`); constants use `UPPER_SNAKE_CASE`.
- Keep platform dependencies behind HAL interfaces (`II2CBus`, `IGpioInterrupt`, etc.) to preserve portability.

## Testing Guidelines
- Framework: GoogleTest + GMock (fetched in `test/CMakeLists.txt`).
- Add tests in `test/test_<module>.cpp` and reuse mocks from `test/mocks/`.
- Cover success paths, conversion/math correctness, and bus/error handling.
- Run tests locally before PRs; CI expects `./build/test/driver_tests` to pass on `main` and PRs.

## Commit & Pull Request Guidelines
- Follow observed commit style: imperative, scoped summaries (for example, `Add portable LPS22DF barometric pressure driver with 11 tests`, `Fix critical review issues CR-001 through CR-004`).
- Keep commits focused (feature, fix, or review batch), and include tests with behavior changes.
- PRs should include: concise description, affected modules, test evidence (command + result), and links to related issues/review IDs.
