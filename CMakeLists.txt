cmake_minimum_required (VERSION 2.6)
project (box2d CXX)
enable_language (CXX C)

include_directories (Include)

set(TARGET_FLOAT32_IS_FIXED false CACHE BOOL "Set to 'true' to compile box2d against an internal fixed float type")
set(TARGET_IS_NDS           false CACHE BOOL "Set to 'true' if target is NDS")

#set(NDS_DEVKITARM           "/usr" CACHE STRING "Set the path to your ARM developer kit")
#set(NDS_DECKITPRO           "/usr" CACHE STRING "Set the path to your PRO developer kit")

###
# Box2d engine
##
add_subdirectory (Source)
##
# TestBed
##
add_subdirectory (Examples/TestBed)
##
# The HelloWorld example
##
add_subdirectory (Examples/HelloWorld)
