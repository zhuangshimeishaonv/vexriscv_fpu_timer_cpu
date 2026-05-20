PROJ_NAME=hello_world
DEBUG=no
BENCH=no
BENCHMARK=no

MULDIV=yes
CPU_FPU ?= no

# Normal build sources (include main.c, exclude uavground.c and benchmark_main.c)
SRCS = $(filter-out src/uavground.c src/benchmark_main.c src/main.c src/math_utils.c,$(wildcard src/*.c)) \
       $(wildcard src/*.cpp)  \
       $(wildcard src/*.S)

# Benchmark build sources (include benchmark_main.c)
BENCHMARK_SRCS = $(filter-out src/uavground.c src/main.c,$(wildcard src/*.c)) \
                 $(wildcard src/*.cpp)  \
                 $(wildcard src/*.S)

OBJDIR = build

INC  =

LIBS = -lm -lgcc
LIBSINC = -L$(OBJDIR)
LDSCRIPT = ./src/linker.ld

SIFIVE_GCC_PACK = no
RISCV_NAME ?= riscv-none-elf
RISCV_PATH ?= /opt/riscv-toolchain-xpack/

MABI=ilp32
MARCH := rv32i
ifeq ($(MULDIV),yes)
	MARCH := $(MARCH)m
endif
ifeq ($(CPU_FPU),yes)
	MARCH := $(MARCH)f
	MABI := ilp32f
	CFLAGS += -DCPU_FPU=1
endif
ifeq ($(COMPRESSED),yes)
	MARCH := $(MARCH)ac
endif
MARCH := $(MARCH)_zicsr
CFLAGS += -march=$(MARCH)  -mabi=$(MABI) -DNDEBUG
LDFLAGS += -march=$(MARCH)  -mabi=$(MABI)

ifeq ($(DEBUG),yes)
	CFLAGS += -g3 -O0
endif

ifeq ($(DEBUG),no)
	CFLAGS += -g -O3 -funroll-loops -ftree-vectorize
endif

ifeq ($(BENCH),yes)
	CFLAGS += -fno-inline
endif

RISCV_OBJCOPY = $(RISCV_PATH)/bin/$(RISCV_NAME)-objcopy
RISCV_OBJDUMP = $(RISCV_PATH)/bin/$(RISCV_NAME)-objdump
RISCV_CC=$(RISCV_PATH)/bin/$(RISCV_NAME)-gcc

CFLAGS +=  -MD -fstrict-volatile-bitfields -fno-strict-aliasing

LDFLAGS += --specs=nano.specs -mcmodel=medany -nostartfiles -ffreestanding -Wl,-Bstatic,-T,$(LDSCRIPT),-Map,$(OBJDIR)/$(PROJ_NAME).map,--print-memory-usage -u _printf_float

OBJS := $(SRCS)
OBJS := $(OBJS:.c=.o)
OBJS := $(OBJS:.cpp=.o)
OBJS := $(OBJS:.S=.o)
OBJS := $(OBJS:..=miaou)
OBJS := $(addprefix $(OBJDIR)/,$(OBJS))

# Benchmark objects
BENCHMARK_OBJS := $(BENCHMARK_SRCS)
BENCHMARK_OBJS := $(BENCHMARK_OBJS:.c=.o)
BENCHMARK_OBJS := $(BENCHMARK_OBJS:.cpp=.o)
BENCHMARK_OBJS := $(BENCHMARK_OBJS:.S=.o)
BENCHMARK_OBJS := $(addprefix $(OBJDIR)/,$(BENCHMARK_OBJS))

all: $(OBJDIR)/$(PROJ_NAME).elf $(OBJDIR)/$(PROJ_NAME).hex $(OBJDIR)/$(PROJ_NAME).asm $(OBJDIR)/$(PROJ_NAME).v

$(OBJDIR)/%.elf: $(OBJS) | $(OBJDIR)
	$(RISCV_CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LIBSINC) $(LIBS)

$(OBJDIR)/benchmark.elf: $(BENCHMARK_OBJS) | $(OBJDIR)
	$(RISCV_CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LIBSINC) $(LIBS)

$(OBJDIR)/benchmark.hex: $(OBJDIR)/benchmark.elf
	$(RISCV_OBJCOPY) -O ihex $^ $@

$(OBJDIR)/benchmark.bin: $(OBJDIR)/benchmark.elf
	$(RISCV_OBJCOPY) -O binary $^ $@

$(OBJDIR)/benchmark.asm: $(OBJDIR)/benchmark.elf
	$(RISCV_OBJDUMP) -S -d $^ > $@

%.hex: %.elf
	$(RISCV_OBJCOPY) -O ihex $^ $@

%.bin: %.elf
	$(RISCV_OBJCOPY) -O binary $^ $@

%.v: %.elf
	$(RISCV_OBJCOPY) -O verilog $^ $@

%.asm: %.elf
	$(RISCV_OBJDUMP) -S -d $^ > $@

$(OBJDIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(RISCV_CC) -c $(CFLAGS)  $(INC) -o $@ $^
	$(RISCV_CC) -S $(CFLAGS)  $(INC) -o $@.disasm $^

$(OBJDIR)/%.o: %.cpp
	mkdir -p $(dir $@)
	$(RISCV_CC) -c $(CFLAGS)  $(INC) -o $@ $^

$(OBJDIR)/%.o: %.S
	mkdir -p $(dir $@)
	$(RISCV_CC) -c $(CFLAGS) -o $@ $^ -D__ASSEMBLY__=1

$(OBJDIR):
	mkdir -p $@

.PHONY: clean benchmark

# Normal build: make
# Benchmark build: make benchmark

benchmark: $(OBJDIR)/benchmark.elf $(OBJDIR)/benchmark.hex $(OBJDIR)/benchmark.asm
	@echo "Benchmark build complete!"
	@echo "ELF: $(OBJDIR)/benchmark.elf"
	@echo "HEX: $(OBJDIR)/benchmark.hex"
	@echo "ASM: $(OBJDIR)/benchmark.asm"

clean:
	rm -rf $(OBJDIR)/src
	rm -f $(OBJDIR)/$(PROJ_NAME).elf
	rm -f $(OBJDIR)/$(PROJ_NAME).hex
	rm -f $(OBJDIR)/$(PROJ_NAME).map
	rm -f $(OBJDIR)/$(PROJ_NAME).v
	rm -f $(OBJDIR)/$(PROJ_NAME).asm
	rm -f $(OBJDIR)/benchmark.elf
	rm -f $(OBJDIR)/benchmark.hex
	rm -f $(OBJDIR)/benchmark.asm
	find $(OBJDIR) -type f -name '*.o' -print0 | xargs -0 -r rm
	find $(OBJDIR) -type f -name '*.d' -print0 | xargs -0 -r rm

clean-all : clean 

.PHONY: check-fpu-asm
check-fpu-asm: clean all
	@echo "Checking for hard-float instructions in $(OBJDIR)/$(PROJ_NAME).asm"
	@grep -E "f(add|mul|lw|sw)\\.s|flw|fsw" $(OBJDIR)/$(PROJ_NAME).asm >/dev/null
	@echo "Hard-float instruction check passed."

.SECONDARY: $(OBJS)
