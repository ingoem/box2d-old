PATCH=	box2d_fixed.patch

all: box2d glut glui testbed

box2d:
	make -f Build/gnumake/Makefile.box2d

glui: glut
	make -f Build/gnumake/Makefile.glui

glut:
	make -f Build/gnumake/Makefile.glut

testbed: box2d glui glut
	make -f Build/gnumake/Makefile.TestBed

install:
	make -f Makefile.Source install

clean:
	rm -rf Gen

patch:
	svn diff > $(PATCH)
#	- diff --unified /dev/null Makefile >> $(PATCH)
#	- diff --unified /dev/null Documentation/latex/Makefile >> $(PATCH)
#	- diff --unified /dev/null Contrib/freeglut/Makefile >> $(PATCH)
#	- diff --unified /dev/null Contrib/glui/Makefile >> $(PATCH)
#	- diff --unified /dev/null Source/Makefile >> $(PATCH)
#	- diff --unified /dev/null Examples/TestBed/Makefile >> $(PATCH)
#	- diff --unified /dev/null Source/Common/Fixed.h >> $(PATCH)
#	- diff --unified /dev/null Source/Common/jtypes.h >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Polygon.cpp >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Polygon.h >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Triangle.cpp >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Triangle.h >> $(PATCH)
