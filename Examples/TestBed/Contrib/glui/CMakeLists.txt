##
# This file was automatically generated using:
#    /usr/local/bin/box2d_gen_sublib.sh glui
##
include_directories (../freeglut)

set(glui_SOURCES
  "./glui_column.cpp"
  "./glui_textbox.cpp"
  "./glui_checkbox.cpp"
  "./glui_treepanel.cpp"
  "./glui_bitmap_img_data.cpp"
  "./glui_rotation.cpp"
  "./glui_control.cpp"
  "./glui_separator.cpp"
  "./glui.cpp"
  "./glui_mouse_iaction.cpp"
  "./glui_filebrowser.cpp"
  "./glui_listbox.cpp"
  "./glui_tree.cpp"
  "./glui_commandline.cpp"
  "./glui_list.cpp"
  "./glui_spinner.cpp"
  "./glui_panel.cpp"
  "./glui_bitmaps.cpp"
  "./glui_window.cpp"
  "./glui_translation.cpp"
  "./glui_add_controls.cpp"
  "./glui_edittext.cpp"
  "./glui_radio.cpp"
  "./glui_rollout.cpp"
  "./glui_statictext.cpp"
  "./glui_node.cpp"
  "./arcball.cpp"
  "./quaternion.cpp"
  "./algebra3.cpp"
  "./glui_string.cpp"
  "./glui_button.cpp"
  "./glui_scrollbar.cpp"
)

add_library (glui ${glui_SOURCES})
set_target_properties (glui PROPERTIES LINKER_LANGUAGE CXX)
