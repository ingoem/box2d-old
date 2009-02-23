all: box2d glut glui testbed

box2d:
	make -f Build/gnumake/Makefile.box2d all

box2d-static:
	make -f Build/gnumake/Makefile.box2d static

box2d-shared:
	make -f Build/gnumake/Makefile.box2d shared

box2d-fixed:
	FLOAT32_IS_FIXED=1 \
		make -f Build/gnumake/Makefile.box2d all

box2d-fixed-static:
	FLOAT32_IS_FIXED=1 \
		make -f Build/gnumake/Makefile.box2d static

box2d-fixed-shared:
	FLOAT32_IS_FIXED=1 \
		make -f Build/gnumake/Makefile.box2d shared

box2d-nds:
	TARGET_IS_NDS=1 \
		make -f Build/gnumake/Makefile.box2d all

box2d-nds-shared:
	TARGET_IS_NDS=1 \
		make -f Build/gnumake/Makefile.box2d shared

box2d-nds-static:
	TARGET_IS_NDS=1 \
		make -f Build/gnumake/Makefile.box2d static

box2d-nds-fixed:
	TARGET_IS_NDS=1 \
	FLOAT32_IS_FIXED=1 \
		make -f Build/gnumake/Makefile.box2d all

box2d-nds-fixed-static:
	TARGET_IS_NDS=1 \
	FLOAT32_IS_FIXED=1 \
		make -f Build/gnumake/Makefile.box2d static

box2d-nds-fixed-shared:
	TARGET_IS_NDS=1 \
	FLOAT32_IS_FIXED=1 \
		make -f Build/gnumake/Makefile.box2d shared

glut:
	make -f Build/gnumake/Makefile.glut all

glui: glut
	make -f Build/gnumake/Makefile.glui all

testbed: box2d-static glui glut
	make -f Build/gnumake/Makefile.TestBed all

clean:
	make -f Build/gnumake/Makefile.box2d clean
	make -f Build/gnumake/Makefile.glui clean
	make -f Build/gnumake/Makefile.glut clean
	make -f Build/gnumake/Makefile.TestBed clean

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
