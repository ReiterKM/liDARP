SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic

########### FROM CPLEX MAKEFILE ###############
# Compiler selection
CCC = g++ -O0 
# Compiler options
CCOPT = -std=c++17 -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD -Wno-ignored-attributes -g # -g for debugging


# replace the following lines with location of your cplex directory
CPLEXDIR      = PATHTOCPLEX/cplex
CONCERTDIR    = PATHTOCPLEX/concert

# Link options and libraries
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) 
CCLNFLAGS = -lconcert -lilocplex -lcplex -m64 -lm -lpthread -ldl

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) 



# Set directory for static library and binaries
# Defaults to ./lib and ./bin
DARPH_LIB_DIR = ./lib
DARPH_BIN_DIR = ./bin

# Set names of executables
CPLEX_EXE_3 = $(DARPH_BIN_DIR)/darp_cplex_3
CPLEX_EXE_6 = $(DARPH_BIN_DIR)/darp_cplex_6


# Set name of libraries needed by applicaitons
LIBS= -ldarph -lm

# Various directories needed by the library and applications
INC_DIR= -I./inc/ 
LIB_DIR = -L$(DARPH_LIB_DIR)
DARPH_LIB = $(DARPH_LIB_DIR)/libdarph.a

SRCS= ./src/DARP.cpp ./src/DARPIO.cpp ./src/DARPDebug.cpp ./src/DARPGraph.cpp ./src/DARPSolver.cpp ./src/RollingHorizon.cpp ./src/DARPCplex.cpp 

OBJS=$(SRCS:.cpp=.o) 

CPLEX_SRC_3 = ./src/apps/darp_cplex_3.cpp
CPLEX_SRC_6 = ./src/apps/darp_cplex_6.cpp



all: $(DARPH_LIB) darp_cplex_3 darp_cplex_6


$(DARPH_LIB): $(OBJS) 
	mkdir -p $(DARPH_LIB_DIR)
	$(AR) $(ARFLAGS) $@ $(OBJS) 
	ranlib $@
	rm -rf $(OBJS)

.cpp.o: 
	$(CCC) $(CCFLAGS) -c $(INC_DIR) $< -o $@
# no need for using linker options when only compiling (-L -l options)





darp_cplex_3: $(OBJS) $(CPLEX_SRC_3)
	mkdir -p $(DARPH_BIN_DIR)
	$(CCC) $(CCFLAGS) $(CPLEX_SRC_3) $(INC_DIR) $(CCLNDIRS) $(LIB_DIR) $(LIBS) $(CCLNFLAGS) -o $(CPLEX_EXE_3) 

darp_cplex_6: $(OBJS) $(CPLEX_SRC_6)
	mkdir -p $(DARPH_BIN_DIR)
	$(CCC) $(CCFLAGS) $(CPLEX_SRC_6) $(INC_DIR) $(CCLNDIRS) $(LIB_DIR) $(LIBS) $(CCLNFLAGS) -o $(CPLEX_EXE_6) 

clean: 
	-rm -rf $(OBJS)
	-rm -rf $(DARPH_LIB)
	-rm -rf $(CPLEX_EXE_3)
	-rm -rf $(CPLEX_EXE_6)
	/bin/rm -rf *.o *~ 




