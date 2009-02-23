
include_directories(Source)
include_directories(Contrib/freeglut)
include_directories(Contrib/glui)
include_directories(${box2d_SOURCE_DIR})

add_subdirectory(Contrib/freeglut)
add_subdirectory(Contrib/glui)

set(testbed_SOURCES
  "Source/Main.cpp"
  "Source/Test.cpp"
  "Source/Render.cpp"
  "Source/TestEntries.cpp"
  "Source/Tests/BroadPhaseTest.cpp"
  "Source/Tests/Biped.cpp"
  "Source/Tests/BipedDef.cpp"
  "Source/Tests/TriangleMesh.cpp"
)

if (TARGET_FLOAT32_IS_FIXED)
  add_definitions(-DTARGET_FLOAT32_IS_FIXED)
endif (TARGET_FLOAT32_IS_FIXED)

add_executable(testbed ${testbed_SOURCES})

if (WIN32)
  target_link_libraries(testbed box2d glui glut glu32 opengl32)
else (WIN32)
  target_link_libraries(testbed box2d glui glut GLU GL)
endif (WIN32)

#set_target_properties (testbed PROPERTIES LINKER_LANGUAGE CXX)
