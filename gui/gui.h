#ifndef GUI_H
#define GUI_H

#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/entypo.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <nanogui/formhelper.h>
#include <memory>
#if defined(_WIN32)
#include <windows.h>
#endif
#include <iostream>

#ifdef NANOGUI_LINUX
    #include <SDL2/SDL.h>
    #include <SDL2/SDL_opengl.h>
#else
    #include <SDL/SDL_opengl.h>
#endif

#include <nanovg/stb_image.h>


#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 700

void gui_init();
void gui_set_done();
void gui_draw(SDL_Event e);
nanogui::FormHelper* gui_get_pointer();

#endif
