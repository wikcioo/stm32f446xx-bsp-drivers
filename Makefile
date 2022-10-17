CC = arm-none-eabi-gcc
MACH = cortex-m4
CORE_DRIVERS_DIR = drivers/stm32f446xx/drivers
BSP_DRIVERS_DIR = bsp
BUILD_DIR = build
SOURCES  = $(wildcard $(CORE_DRIVERS_DIR)/src/*.c)
SOURCES += $(wildcard $(BSP_DRIVERS_DIR)/**/*.c)
SOURCES += $(wildcard ./*.c)
OBJECTS  = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(notdir $(SOURCES)))))
CFLAGS = -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -g -Wall -Wformat -Wpedantic -Wshadow -O0 -I$(CORE_DRIVERS_DIR)/inc

.PHONY = all clean

all:
	@mkdir -p $(BUILD_DIR)
	@make --no-print-directory $(OBJECTS)

$(BUILD_DIR)/%.o: $(CORE_DRIVERS_DIR)/src/%.c
	$(CC) $(CFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: $(BSP_DRIVERS_DIR)/**/%.c
	$(CC) $(CFLAGS) $^ -o $@

$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf $(BUILD_DIR)
