AS = as
GCC = gcc
ASFLAGS = -o
LDFLAGS = -no-pie

SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

ASSEMBLY_OBJ := $(OBJ_DIR)/out_primes.o
ASSEMBLY_OBJ_A := $(OBJ_DIR)/main_a.o

EXECUTABLE := $(BIN_DIR)/call_from_c
EXECUTABLE_A := $(BIN_DIR)/call_from_a

ASSEMBLY_FILE := $(SRC_DIR)/func.asm
ASSEMBLY_FILE_A := $(SRC_DIR)/main_a.asm
C_FILE := $(SRC_DIR)/main_c.c

all: $(EXECUTABLE) $(EXECUTABLE_A)

$(ASSEMBLY_OBJ): $(ASSEMBLY_FILE)
	@mkdir -p $(OBJ_DIR)
	$(AS) $(ASFLAGS) $@ $< 

$(ASSEMBLY_OBJ_A): $(ASSEMBLY_FILE_A)
	@mkdir -p $(OBJ_DIR)
	$(AS) $(ASFLAGS) $@ $< 

$(EXECUTABLE): $(ASSEMBLY_OBJ)
	@mkdir -p $(BIN_DIR)
	$(GCC) -o $@ $(C_FILE) $< $(LDFLAGS)

$(EXECUTABLE_A): $(ASSEMBLY_OBJ_A) $(ASSEMBLY_OBJ)
	@mkdir -p $(BIN_DIR)
	$(GCC) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(OBJ_DIR)/*.o
	rm -f $(BIN_DIR)/call_from_c
	rm -f $(BIN_DIR)/call_from_a

.PHONY: all clean
