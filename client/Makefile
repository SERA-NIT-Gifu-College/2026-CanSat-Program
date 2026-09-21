NAME:=cansatClient
CC:=gcc
CXX:=g++
RM:=rm
CFLAGS:=-O2 -Wall -Wpedantic -std=c11
CXXFLAGS:=-O2 -Wall -Wpedantic -std=c++20 -g

PREFIX:=./build

LIBS:=
INCL:=$(addprefix -I, \
	./include/ \
	./thirdparty/cxxopts/include/ \
)

SRCS:= \
	src/app.cpp \
	src/tcp_client.cpp
OBJS:=$(patsubst %.cpp,%.o,${SRCS})

all: ${NAME}

${NAME}: ${OBJS}
	@mkdir -p ${PREFIX}
	${CXX} ${OBJS} -o ${PREFIX}/${NAME}

%.o: %.cpp
	${CXX} -c ${CXXFLAGS} ${INCL} ${LIBS} $^ -o $@

clean:
	${RM} ${OBJS}

clean-all:
	${RM} ${OBJS}
	${RM} -dr ${PREFIX}
