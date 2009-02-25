=== Introduction ===

Box2D Version 2.0.2

Welcome to Box2D!

Box2D is a 2D physics engine for games.

For help with Box2D, please visit http://www.box2d.org. There is a forum there where you may post your questions.

The project files with this distribution work with Microsoft Visual C++ 2005 and above. You can download MSVC 2008 for free and it includes the necessary Platform SDK.

To run the demos, set "Testbed" as your startup project and press F5. Some test bed commands are:
- 'r' to reset the current test
- SPACE to launch a bomb
- arrow keys to pan
- 'x' and 'z' to zoom in/out
- use the mouse to click and drag objects

Erin Catto
http://www.box2d.org

=== BUILDING ===

Box2D can be built in two different versions. One using a Fixed Float type, and one using your a 32bit float (this could differ depending on system).

When building the fixed float version, there are a couple of things you have to think about:

  - the macro TARGET_FLOAT32_IS_FIXED must be defined, both when building the library and when building your application! see 'faq #1'.

------ Windows ------
  see: http://www.box2d.org/wiki/index.php?title=Building

------ Mac ------
  see: http://www.box2d.org/wiki/index.php?title=Building

------ Linux ------
Under linux, there are two building alternatives; Gnu Make or CMake.
There are still some things which cannot be properly done using cmake (like installing) but it is on it's way.

Using CMake:

  For cmake, it is better to create a separate directory when building, since this will not clutter the building root too much.
  Do this by typing:

    # mkdir build

  Then enter that directory:

    # cd build

  And run cmake to generate the proper files:

    # cmake ..

  Notice the '..', this means that cmake will look for the file CMakeLists.txt in the directory under the one in which you are currently located.

  If everything went O.K., it is time to decide what type of build you wish to do.
  Edit the file CMakeCache.txt, read it and you will probably find a few variables of interest.

  When you are done, rerun 'cmake ..' to generate your build environment, this is recommended to be 'Gnu Makefiles' under linux.

  After that, run:

    # make box2d

  You will find the generated library under Box2D/libbox2d.a (if you compiled as static library).

Using Gnu Make:

  If you feel lucky, try typing:

    # make box2d

  This will assume the target is 'box2d-static', there are a range of targets available (which can be run by typing 'make <target>'):

    clean: remove all generated files and directories.

    box2d:                  same as box2d-static
    box2d-static:           compile box2d as a static library (e.g. libbox2d.a)
    box2d-shared:           compile box2d as a shared library (e.g. libbox2d.so)
    box2d-fixed:            same as box2d-fixed-static
    box2d-fixed-static:     compile box2d as a static library using TARGET_FLOAT32_IS_FIXED
    box2d-fixed-shared:     compile box2d as a shared library using TARGET_FLOAT32_IS_FIXED
    
    (WARNING: EXPERIMENTAL, you must define the environment variable DEVKITPRO in order to find the compilers)
    box2d-nds:              same as box2d-nds-static
    box2d-nds-static:       compile box2d as a static library for the nintendo dual-screen (requires DEVKITPRO).
    box2d-nds-shared:       compile box2d as a shared library for the nintendo dual-screen (requires DEVKITPRO).
    box2d-nds-fixed:        same as box2d-nds-fixed-static
    box2d-nds-fixed-static: compile box2d as a static library for the nintendo dual-screen (requires DEVKITPRO) using TARGET_FLOAT32_IS_FIXED.
    box2d-nds-fixed-shared: compile box2d as a shared library for the nintendo dual-screen (requires DEVKITPRO) using TARGET_FLOAT32_IS_FIXED.

    (related to the TestBed)
    glut:                   compile the static glut library used with the testbed.
    glui:                   compile the static glui library used with the testbed.
    testbed:                compile the testbed.

=== INSTALLING ===

------ Linux ------
  After you have compiled the library, you might wan't to consider copying it to /usr/local/lib (or some other place in your LD_PATH), if you retain the 'lib' prefix, you can easily link to it by using the '-l' switch in gcc.

  Example:

    gcc myprogram.cpp -lbox2d

  You must also copy the header-files from the Box2D directory and retain the hierarchcal structure.
  The easiest sollution is to copy the entire directory to /usr/local/include (e.g. /usr/local/include/Box2D) and remove the .cpp files afterwards
  We might add an installation script for this in the future.
  
  This will make sure that you can have the following include macro in your source files:

    #include <Box2D/Box2D.h>

  Which should be consistent with any box2d coding examples that you encounter.

------ Windows ------
  see: http://www.box2d.org/wiki/index.php?title=Building

------ Mac ------
  see: http://www.box2d.org/wiki/index.php?title=Building

=== FAQ ===
faq #1
Question: The library compiles fine, but I get a lot of 'undefined reference' warnings when linking it to my application.
Answer: the macro TARGET_FLOAT32_IS_FIXED must be defined or undefined depending on how the library was compiled.
        The undefined references are triggered because the symbol table in the library does not match the one in your application when compiling against a mismatched existance of the macro TARGET_FLOAT32_IS_FIXED.
        That is, if you compile your library with TARGET_FLOAT32_IS_FIXED, you must compile your application with it aswell, or vice versa.
