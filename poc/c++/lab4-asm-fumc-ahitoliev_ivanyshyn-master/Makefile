FUNC_1_DIR := func_1
FUNC_2_DIR := func_2

all:
	@$(MAKE) -C $(FUNC_1_DIR)
	@$(MAKE) -C $(FUNC_2_DIR)

clean:
	@$(MAKE) -C $(FUNC_1_DIR) clean
	@$(MAKE) -C $(FUNC_2_DIR) clean

.PHONY: all clean