NAME:=cansatClient
CC:=gcc
CXX:=g++
RM:=rm
CFLAGS:=-O2 -Wall -Wpedantic
CXXFLAGS:=-O2 -Wall -Wpedantic

LIBS:=$(shell pkg-config --libs --cflags raylib)
INCL:=$(addprefix -I, \
	./include/ \
	./thirdparty/cxxopts/include/ \
)

SRCS:= \
	src/client.cpp
OBJS:=$(patsubst %.cpp,%.o,${SRCS})

all: ${NAME}

${NAME}: ${OBJS}
	${CXX} ${OBJS}

%.o: %.cpp
	${CXX} -c ${CXXFLAGS} ${INCL} ${LIBS} $^ -o $@

clean:
	${RM} ${OBJS} ${NAME}
