BIN = control
DEPS = $(wildcard *.h)
SRC = $(wildcard *.c)
OBJ = $(patsubst %.c,%.o,$(SRC))

#CFLAGS = -O0
CFLAGS = -rdynamic -O0 -g -Werror -Wall -Wextra -D_GNU_SOURCE -DHAVE_INLINE
#CFLAGS += -Wno-unused-function
#CFLAGS += -DNOSIM
LDFLAGS += -lreadline -lm -ljson-c
LDFLAGS += -lgsl -lgslcblas
#LDFLAGS += -I/home/archer/Local/gsl/ -L/home/archer/Local/gsl/.libs/gsl -L/home/archer/Local/gsl/cblas/.libs/gslcblas

RM = rm -f

all: clean compile check

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BIN): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)
	
.PHONY: clean compile check

compile: $(BIN)

clean:
	$(RM) $(BIN) $(OBJ)

check:
	cppcheck --enable=all -q --suppress=unusedFunction --suppress=missingIncludeSystem $(SRC)
