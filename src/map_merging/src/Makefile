OBJECTS = grid_map.o hough.o io.o  manipulatemap.o common.o

C++ = g++

CFLAGS =  -I/usr/local/include  

# uncomment these lines in Linux only
LIBRARY = libmapmerge.so
LIBFLAGS = -shared -lcv -lcxcore -lhighgui -lm 

# uncomment these lines in Os X only
#LIBRARY = libmapmerge.dylib
#LIBFLAGS = -dynamiclib -lcv -lcxcore -lhighgui -lm


LDFLAGS = -L. -lcv -lcxcore -lhighgui -lm -lmapmerge

all: $(LIBRARY) test

$(LIBRARY): $(OBJECTS)
	$(C++) $(LIBFLAGS) $(OBJECTS) -o $(LIBRARY)

test: $(LIBRARY) test.o
	$(C++) $(LDFLAGS) test.o   -o test


FLAGS= -Wall -O2

%.o : %.cpp
	$(C++) $(CFLAGS) $(FLAGS) -c $<

clean:
	rm -f *.o *~ $(LIBRARY) test

doc:
	doxygen Doxyfile

cleandoc:
	rm -rf html