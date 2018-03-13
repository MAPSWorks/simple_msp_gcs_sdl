#include "gui.h"
#include "../rpi-udp-stream-client/common_util/common_util.h"

extern "C" {
    typedef GLubyte* (APIENTRY * glGetString_Func)(unsigned int);
    glGetString_Func glGetStringAPI = NULL;
}

using std::cout;
using std::cerr;
using std::endl;

using namespace nanogui;

static int winWidth;
static int winHeight;

static SDL_Window* window;
static Screen* screen;
static FormHelper* gui;

void gui_init()
{
    char rendername[256] = {0};
    SDL_RendererInfo info;

    SDL_Init(SDL_INIT_VIDEO);   // Initialize SDL2
    
#ifdef NANOVG_GL2_IMPLEMENTATION
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,0);
#elif defined(NANOVG_GL3_IMPLEMENTATION)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,3);
#elif defined(NANOVG_GLES2_IMPLEMENTATION)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,0);
#elif defined(NANOVG_GLES3_IMPLEMENTATION)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION,3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION,0);
#endif
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    winWidth = WINDOW_WIDTH;
    winHeight = WINDOW_HEIGHT;

    // Create an application window with the following settings:
    window = SDL_CreateWindow(
      "Simple MSP GCS",         //    const char* title
      SDL_WINDOWPOS_UNDEFINED,  //    int x: initial x position
      SDL_WINDOWPOS_UNDEFINED,  //    int y: initial y position
      winWidth,                      //    int w: width, in pixels
      winHeight,                      //    int h: height, in pixels
      SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN    | SDL_WINDOW_ALLOW_HIGHDPI      //    Uint32 flags: window options, see docs
    );

    // Check that the window was successfully made
    if(window==NULL){
      // In the event that the window could not be made...
      std::cout << "Could not create window: " << SDL_GetError() << '\n';
      SDL_Quit();
      exit(1);
    }

    auto context = SDL_GL_CreateContext(window);

    for (int it = 0; it < SDL_GetNumRenderDrivers(); it++) {
        SDL_GetRenderDriverInfo(it, &info);
        strcat(rendername, info.name);
        strcat(rendername, " ");
    }

    glGetStringAPI = (glGetString_Func)SDL_GL_GetProcAddress("glGetString");

    DEBUG_MSG("Available Renderers: %s\n", rendername);
    DEBUG_MSG("Vendor     : %s\n", glGetStringAPI(GL_VENDOR));
    DEBUG_MSG("Renderer   : %s\n", glGetStringAPI(GL_RENDERER));
    DEBUG_MSG("Version    : %s\n", glGetStringAPI(GL_VERSION));

    screen = new Screen( window, Vector2i(winWidth, winHeight), "NanoGUI test");

    gui = new FormHelper(screen);
}

void gui_set_done()
{
    screen->setVisible(true);
    screen->performLayout();
}

void gui_draw(SDL_Event e)
{
    screen->onEvent(e);
    
    glViewport(0, 0, winWidth, winHeight);
            
    glClearColor(0.9f, 0.9f, 0.9f, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    screen->drawAll();

    SDL_GL_SwapWindow(window);
}

FormHelper* gui_get_pointer()
{
    return gui;
}
