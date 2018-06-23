#Source files
SFILES = $(wildcard source/*.cpp)

#Object files
OFILES = $(patsubst source/%.cpp,object/%.o,$(SFILES))

#Executable file name 
EXE = main 

#C++ compiler
CXX = x86_64-w64-mingw32-g++

#C++ COMPILER_FLAGS specifies the additional compilation options we're using
# -w suppresses all warnings
#  # -Wl,-subsystem,windows gets rid of the console window 
CXXFLAGS = -Wl,-subsystem,windows 

#Include Paths
IPATHS = -I/cygdrive/e/cygwin64/usr/x86_64-w64-mingw32/sys-root/mingw/include/SDL2 \
	 -I/cygdrive/e/cygwin64/usr/include/eigen3 \
	 -Iheader \
	 -I/cygdrive/e/CoinIpopt/build/include/coin

#Library Paths
LPATHS = -L/cygdrive/e/CoinIpopt/build/lib -L/usr/lib/gcc/x86_64-w64-mingw32/6.4.0 \
	 -L/usr/lib/gcc/x86_64-w64-mingw32/6.4.0/../../../../x86_64-w64-mingw32/lib/../lib \
	 -L/usr/x86_64-w64-mingw32/sys-root/mingw/lib/../lib \
	 -L/usr/lib/gcc/x86_64-w64-mingw32/6.4.0/../../../../x86_64-w64-mingw32/lib \
	 -L/usr/x86_64-w64-mingw32/sys-root/mingw/lib


#Linker Flags
LFLAGS = -lipopt -llapack -lblas -lm -lcoinhsl -lgfortran -lmingw32 -lmoldname -lmingwex -lmsvcrt -lquadmath -lm -lpthread -ladvapi32 -lshell32 -luser32 -lkernel32 -lSDL2main -lSDL2 -lSDL2_image


all: $(OFILES)
	$(CXX) -g $(OFILES) $(LPATHS) $(LFLAGS) -o $(EXE)

object/%.o: source/%.cpp
	$(CXX) -c $< $(CXXFLAGS) $(IPATHS) -o $@

debug: CXXFLAGS +=-Og -g
release: CXXFLAGS +=-O3 
debug release: all
