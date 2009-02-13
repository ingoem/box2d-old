#!/bin/bash

test -z "$1" && exit 255

echo "##"
echo "# This file was automatically generated using:"
echo "#    $0 $@"
echo "##"
echo "set($1_SOURCES"
find . -path "*/.svn" -prune -o -iname "*.cpp" -printf "  \"%p\"\n"
echo ")"
echo ""
echo "add_library ($1 \${$1_SOURCES})"
echo "set_target_properties ($1 PROPERTIES LINKER_LANGUAGE CXX)"
