.SUFFIXES: .cpp .h

# Notations
SHELL 	= bash
CC     	= g++
LD	= ld
ECHO	= /bin/echo
PRINT	= printf


# Directory Paths
PROJECT_ROOT=$(shell pwd)
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin


LIBS = -lGL -lGLU -lglut -lode -lpthread -g
TARGET = QuadSimulator


SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.h)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

.PHONY: all setup clean distclean

all: setup $(BINDIR)/$(TARGET)

setup:
	@$(ECHO) "Starting...."
	@$(ECHO) "Creating required directories...."
	@mkdir -p obj
	@mkdir -p bin
	@$(ECHO) "Compiling...."

$(BINDIR)/$(TARGET): $(OBJS)
	@$(CC) -o $@  $(OBJS) $(LIBS)
	@$(ECHO) "Building executable..."

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINT) "$(notdir $<)\n"
	@$(CC) $(LIBS) -c $< -o $@

clean:
	@$(ECHO) -n "Cleaning up..."
	@rm -rf $(OBJDIR) *~ $(SRCDIR)/*~ 
	@rm -rf $(BINDIR)
	@$(ECHO) "Done"

distclean:
	@$(ECHO) -n "Cleaning up.."
	@rm -rf $(OBJDIR) *~  $(BINDIR) 
	@$(ECHO) "Done"