# pid-controller-sim — Makefile
#   make          : bangun ./pid-sim
#   make test     : bangun + jalankan tes
#   make examples : bangun + jalankan semua contoh ke examples/
#   make clean    : hapus biner + objek

CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic

SRC := src/motor.cpp src/pid.cpp src/metrics.cpp src/tuning.cpp
OBJ := $(SRC:.cpp=.o)

BIN      := pid-sim
TEST_BIN := test_pid

.PHONY: all test examples clean

all: $(BIN)

$(BIN): src/main.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ src/main.cpp $(OBJ)

$(TEST_BIN): tests/test_pid.cpp $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ tests/test_pid.cpp $(OBJ)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

examples: $(BIN)
	bash examples/run_examples.sh

clean:
	rm -f $(BIN) $(TEST_BIN) $(OBJ)
