CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic
TARGET   := test_link_clock

.PHONY: test clean

test: $(TARGET)
	./$(TARGET)

$(TARGET): test_link_clock.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
