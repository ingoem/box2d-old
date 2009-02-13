##
# This file was automatically generated using:
#    /usr/local/bin/box2d_gen_sublib.sh glut c
##
set(glut_SOURCES
  "freeglut_misc.c"
  "freeglut_callbacks.c"
  "freeglut_font.c"
  "freeglut_glutfont_definitions.c"
  "freeglut_gamemode.c"
  "freeglut_main.c"
  "freeglut_menu.c"
  "freeglut_stroke_mono_roman.c"
  "freeglut_geometry.c"
  "freeglut_teapot.c"
  "freeglut_state.c"
  "freeglut_ext.c"
  "freeglut_joystick.c"
  "freeglut_videoresize.c"
  "freeglut_stroke_roman.c"
  "freeglut_display.c"
  "freeglut_overlay.c"
  "freeglut_cursor.c"
  "freeglut_window.c"
  "freeglut_font_data.c"
  "freeglut_structure.c"
  "freeglut_init.c"
)

if (WIN32)
  add_definitions (-DFREEGLUT_STATIC)
endif (WIN32)

add_library (glut ${glut_SOURCES})
set_target_properties (glut PROPERTIES LINKER_LANGUAGE C)
