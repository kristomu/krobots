
CC = g++
OPT = -O9
LIBS = -lSDL -lSDL_gfx -lSDL_ttf
OPTGEN = -fprofile-generate
OPTUSE = -fprofile-use

krobots: main.cc
	${CC} ${CFLAGS} ${OPT} ${LIBS} main.cc -o krobots

krobots-opt-gen: main.cc
	${CC} ${CFLAGS} ${OPT} ${LIBS} ${OPTGEN} main.cc -o krobots

krobots-opt-use: main.cc
	${CC} ${CFLAGS} ${OPT} ${LIBS} ${OPTUSE} main.cc -o krobots
