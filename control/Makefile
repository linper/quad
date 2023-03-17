BIN = control
DEPS = $(wildcard *.h)
SRC = $(wildcard *.c)
OBJ = $(patsubst %.c,%.o,$(SRC))

#CFLAGS = -O0
CFLAGS = -rdynamic -Og -g -Werror -Wall -Wextra -D_GNU_SOURCE -DHAVE_INLINE
#CFLAGS += -Wno-unused-function
LDFLAGS += -lreadline -lm -ljson-c -lgsl -lgslcblas

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
