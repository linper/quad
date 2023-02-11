BIN = control
DEPS = $(wildcard *.h)
SRC = $(wildcard *.c)
OBJ = $(patsubst %.c,%.o,$(SRC))

#CFLAGS = -O0
CFLAGS = -Og -g -Werror -Wall -Wextra -Wno-unused-function
LDFLAGS += -lreadline -lm -ljson-c

RM = rm -f

all: compile 

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BIN): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)
	
.PHONY: clean compile

compile: $(BIN)

clean:
	$(RM) $(BIN) $(OBJ)
