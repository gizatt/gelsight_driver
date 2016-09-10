BUILD_DIR = build

CFLAGS_MAIN += `pkg-config --cflags opencv eigen3`
LIBS_MAIN += `pkg-config --libs opencv eigen3`
LIB_OBJS = $(addprefix $(BUILD_DIR)/, CaptureFrm.o MarkerTrack.o VideoRecord.o WindowDisplay.o)


CFLAGS_DRIVER += `pkg-config --cflags opencv lcm bot2-core eigen3`
LIBS_DRIVER += `pkg-config --libs opencv lcm bot2-core eigen3`

CFLAGS_CAPTUREFRM += `pkg-config --cflags opencv`

all: prepare $(LIB_OBJS) $(BUILD_DIR)/main $(BUILD_DIR)/gelsight_depth_driver $(BUILD_DIR)/groundtruth_gen

$(BUILD_DIR)/main: $(LIB_OBJS) main.cpp
	g++ -O2 $(CFLAGS_MAIN) $(INCLUDES_MAIN) main.cpp $(LIB_OBJS) $(LIBS_MAIN)  -o $@

$(BUILD_DIR)/gelsight_depth_driver: gelsight_depth_driver.cpp
	g++ -O2 $(CFLAGS_DRIVER) $(INCLUDES_DRIVER) gelsight_depth_driver.cpp $(LIBS_DRIVER) -o $@

$(BUILD_DIR)/groundtruth_gen: gelsight_groundtruth_gen.cpp
	g++ -O2 $(CFLAGS_DRIVER) $(INCLUDES_DRIVER) gelsight_groundtruth_gen.cpp $(LIBS_DRIVER) -o $@

$(BUILD_DIR)/%.o: %.cpp
	g++ -O2 -c $(CFLAGS_CAPTUREFRM) $(INCLUDES) $< -o $@

prepare:
	@mkdir -p build

.PHONY: clean
clean:
	@rm -rf $(BUILD_DIR)
