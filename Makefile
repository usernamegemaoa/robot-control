# Compiler
CC = gcc
OPTS = -c -Wall
# Project name
PROJECT = bin/Fanuc
# Directories
OBJDIR = obj
SRCDIR = src
# Libraries
LIBS = -lm
# Files and folders
SRCS    = $(shell find $(SRCDIR) -name '*.c')
SRCDIRS = $(shell find . -name '*.c' | dirname {} | sort | uniq | sed 's/\/$(SRCDIR)//g' )
OBJS    = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))

# Targets
$(PROJECT): buildrepo $(OBJS)
	$(CC) $(OBJS) $(LIBS) -o $@
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CC) $(OPTS) -c $< -o $@
clean:
	rm $(PROJECT) $(OBJDIR) -Rf
buildrepo:
	@$(call make-repo)

# Create obj directory structure
define make-repo
	mkdir -p $(OBJDIR)
	for dir in $(SRCDIRS); \
	do \
		mkdir -p $(OBJDIR)/$$dir; \
	done
endef

