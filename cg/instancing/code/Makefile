IDIR     = ./include
CC       = gcc
CXX      = g++
LINK     = g++
CFLAGS   =-I$(IDIR)
CXXFLAGS =-I$(IDIR) -I/usr/include/SDL2/ -std=c++11 -O2

ODIR     = obj
LDIR     = ../lib

LIBS     = -lm -lGLEW -lSDL2 -lGL

_SRCDIR  = ./src
_OBJ     = $(wildcard $(_SRCDIR)/*.cpp)
OBJ      = $(patsubst $(_SRCDIR)/%.cpp,$(ODIR)/%.o,$(_OBJ))

_OBJ_C   = $(wildcard $(_SRCDIR)/*.c)
OBJ     += $(patsubst $(_SRCDIR)/%.c,$(ODIR)/%_c.o,$(_OBJ_C))

_OIMGUI  = $(wildcard $(IDIR)/imgui/*.cpp)
OBJ     += $(patsubst $(IDIR)/imgui/%.cpp,$(ODIR)/%_imgui.o,$(_OIMGUI))


$(ODIR)/%_c.o: $(_SRCDIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

$(ODIR)/%_imgui.o: $(IDIR)/imgui/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS)

$(ODIR)/%.o: $(_SRCDIR)/%.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS)

glframework: $(OBJ)
	$(LINK) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 