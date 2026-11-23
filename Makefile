# Makefile for OrderKVoronoiD (GNU make).
#
#   make            same as "make release"
#   make release    optimized build:              ./OrderKVoronoiD
#   make debug      debug build:                  ./OrderKVoronoiD_d
#   make asan       AddressSanitizer + UndefinedBehaviorSanitizer build (not static):
#                                                 ./OrderKVoronoiD-asan
#   make variants   all three binaries (release, debug, asan)
#   make clean      delete the binaries
#
# CGAL_DIR must point to the CGAL 4.11.2 build directory (the one containing lib/),
# e.g. "export CGAL_DIR=$HOME/CGAL-4.11.2" in ~/.profile; or pass it on the command
# line: make CGAL_DIR=/path/to/CGAL-4.11.2
# Extra compiler flags can be added with EXTRA_FLAGS, e.g. make release EXTRA_FLAGS=-DNDEBUG
#
# All builds except asan are statically linked, so the binaries run on other x86-64
# Linux systems without CGAL, Boost, GMP or MPFR installed. The sanitizer runtime
# can't be linked statically, so the asan build links CGAL, Boost, GMP and MPFR
# statically but the C and C++ libraries (and the sanitizer runtime) dynamically.
# Sanitizer reports go to stderr. One UBSan report is known from OneSteinerTreeProblem,
# from inside CGAL 4.11.2, and harmless in practice:
#   CGAL/Compact_container.h:1192:51  "reference binding to null pointer": inserting
#     into a triangulation creates faces whose neighbours are still null handles, and
#     set_neighbors dereferences them.
# (ASan reserves a very large amount of virtual memory, so don't run it under ulimit -v.)

SRCS := src/OrderKVoronoiD.cpp src/PrettyPrint.cpp src/UtilityFunctions.cpp
HDRS := $(wildcard src/*.hpp src/*.h)

# Flags shared by all builds. -frounding-math is required by CGAL with g++.
COMMON_FLAGS := -frounding-math -std=c++11 -pthread
WARN_FLAGS   := -Wall -Wextra

RELEASE_FLAGS := -O2 $(WARN_FLAGS)
DEBUG_FLAGS   := -g
# -O1 keeps the stack traces readable; asserts stay on (no NDEBUG).
# -fno-sanitize=vptr: CGAL's data structures cast their internal records to derived
# handle classes that add no data members; UBSan's vptr check reports each such cast.
ASAN_FLAGS    := -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize=vptr $(WARN_FLAGS)

LDFLAGS      := -static -L$(CGAL_DIR)/lib
LIBS         := -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread -lboost_system -lpthread
DEBUG_LIBS   := -lCGAL_Core_d -lCGAL_d -lmpfr -lgmp -lboost_thread -lboost_system -lpthread
# For the asan build: the third-party libraries static (-Bstatic), then back to dynamic
# (-Bdynamic) for pthread and the libraries g++ adds itself (libc, libstdc++, libasan,
# libubsan).
ASAN_LDFLAGS  := -L$(CGAL_DIR)/lib
ASAN_LIBS     := -Wl,-Bstatic -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread -lboost_system \
                 -Wl,-Bdynamic -lpthread

.PHONY: all release debug asan variants clean check-cgal

all: release

release: OrderKVoronoiD
debug: OrderKVoronoiD_d
asan: OrderKVoronoiD-asan
# Every binary: release, debug and asan.
variants: release debug asan

# Stop with a clear message if CGAL_DIR is not set (instead of a confusing link error).
check-cgal:
	@if [ -z "$(CGAL_DIR)" ]; then \
	    echo "CGAL_DIR is not set: see the comment at the top of the Makefile." >&2; exit 1; \
	fi

# Each binary is rebuilt when any source file or header changes. ($@ is the target's
# name; the "|" makes check-cgal run first without counting as a dependency for
# rebuild decisions.)
OrderKVoronoiD: $(SRCS) $(HDRS) | check-cgal
	$(CXX) $(RELEASE_FLAGS) $(COMMON_FLAGS) $(EXTRA_FLAGS) $(SRCS) -o $@ $(LDFLAGS) $(LIBS)

OrderKVoronoiD_d: $(SRCS) $(HDRS) | check-cgal
	$(CXX) $(DEBUG_FLAGS) $(COMMON_FLAGS) $(EXTRA_FLAGS) $(SRCS) -o $@ $(LDFLAGS) $(DEBUG_LIBS)

OrderKVoronoiD-asan: $(SRCS) $(HDRS) | check-cgal
	$(CXX) $(ASAN_FLAGS) $(COMMON_FLAGS) $(EXTRA_FLAGS) $(SRCS) -o $@ $(ASAN_LDFLAGS) $(ASAN_LIBS)

clean:
	rm -f OrderKVoronoiD OrderKVoronoiD_d OrderKVoronoiD-asan
