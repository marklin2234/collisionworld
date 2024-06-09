ifeq ($(CILKSAN),1)
CXXFLAGS += -fsanitize=cilk
LDFLAGS += -fsanitize=cilk
endif

ifeq ($(CILKSCALE),1)
CXXFLAGS += -fcilktool=cilkscale
LDFLAGS += -fcilktool=cilkscale
endif

