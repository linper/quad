
export ROOT_DIR = $(abspath ./)
export CTL_DIR = $(ROOT_DIR)/control
export VIEW_DIR = $(ROOT_DIR)/view
export SIM_DIR = $(ROOT_DIR)/sim
export RES_DIR = $(ROOT_DIR)/res

MAKE = make

.PHONY: build

all: build

build:
	$(MAKE) -C $(CTL_DIR)		
