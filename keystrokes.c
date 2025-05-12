// key_listener.c
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

Display *display;
Window root;

void listen_for_keys(void (*callback)(char*)) {
    XEvent event;
    
    // Open display
    display = XOpenDisplay(NULL);
    if (display == NULL) {
        fprintf(stderr, "Unable to open X display\n");
        exit(1);
    }

    root = DefaultRootWindow(display);

    // Listen for key events
    XSelectInput(display, root, KeyPressMask);

    while (1) {
        XNextEvent(display, &event);
        if (event.type == KeyPress) {
            KeySym keysym = XLookupKeysym(&event.xkey, 0);
            char key_str[32];
            XKeysymToString(keysym, key_str);

            // Call the Python callback function with the key name
            callback(key_str);
        }
    }

    XCloseDisplay(display);
}


