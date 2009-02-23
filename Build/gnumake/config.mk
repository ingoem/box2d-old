
#application may set these.
CXX?=g++
CC?=gcc
AR?=ar
RANLIB?=ranlib
CXXFLAGS?=-g -O2
CFLAGS?=-g -O2
LDFLAGS?=-lm -lgcc

.PHONY: clean-hook toolchain-hook
.SUFFIXES: .cpp .o .c

PKG_SOURCES=$(addprefix ${PACKAGE}/,${SOURCES})
COBJECTS=${PKG_SOURCES:.c=.o}
OBJECTS=${COBJECTS:.cpp=.o}

#how object files are built

# Check the toolchain or die with a verbose error.
toolchain:
	@if ! type $(CXX) 2>&1 > /dev/null; then echo "!!! Your c++-compiler '$(CXX)' does not exist"; exit 1; fi
	@if ! type $(RANLIB) 2>&1 > /dev/null; then echo "!!! Your ranlib '$(RANLIB)' does not exist"; exit 2; fi
	@if ! type $(AR) 2>&1 > /dev/null; then echo "!!! Your archiver '$(AR)' does not exist"; exit 3; fi

.cpp.o: toolchain
	@echo "  Compiling: $@"
	$(CXX) -c ${CXXFLAGS} ${DEFINES} ${INCLUDES} -o $@ $<

.c.o: toolchain
	@echo "  Compiling: $@"
	$(CC) -c ${DEFINES} ${INCLUDES} -o $@ $<

shared: ${TARGET}.so
static: ${TARGET}.a
binary: ${TARGET}

${TARGET}.a: toolchain ${OBJECTS}
	@echo "  Creating static library: $@"
	@mkdir -p $(dir $@)
	$(AR) cr $@ ${OBJECTS}
	$(RANLIB) $@

${TARGET}.so: toolchain ${OBJECTS}
	@echo "  Linking shared library: $@"
	@mkdir -p $(dir $@)
	$(CXX) -shared -fPIC ${CXXFLAGS} -C -o $@ ${OBJECTS} ${LDFLAGS}

${TARGET}: toolchain ${OBJECTS}
	@echo "  Creating binary: $@"
	@mkdir -p $(dir $@)
	$(CXX) ${CXXFLAGS} -o $@ ${OBJECTS} ${LDFLAGS}

clean: clean-hook
	$(RM) ${OBJECTS}
	$(RM) ${TARGET}.so
	$(RM) ${TARGET}.a
	$(RM) ${TARGET}
	-rmdir $(dir ${TARGET})
