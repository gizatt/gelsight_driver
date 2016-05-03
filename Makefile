BUILD_DIR = build

CFLAGS += `pkg-config --cflags opencv`
LIBS += `pkg-config --libs opencv`
LIB_OBJS = $(addprefix $(BUILD_DIR)/, CaptureFrm.o MarkerTrack.o VideoRecord.o WindowDisplay.o)

all: prepare $(LIB_OBJS) main

main: $(LIB_OBJS)
	g++ -O2 $(CFLAGS) $(INCLUDES) main.cpp $(LIB_OBJS) $(LIBS)  -o $(BUILD_DIR)/$@

$(BUILD_DIR)/%.o: %.cpp
	g++ -O2 -c -std=c++11 $(CFLAGS) $(INCLUDES) $< -o $@

prepare:
	@mkdir -p build

.PHONY: clean
clean:
	@rm -rf $(BUILD_DIR)
