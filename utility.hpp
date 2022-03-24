#ifndef BDH_MUJOCO_MOUSE_KEYBOARD_UTILITY_HPP_
#define BDH_MUJOCO_MOUSE_KEYBOARD_UTILITY_HPP_

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

class mujoco_mouse_keyboard
{
    public:
        mujoco_mouse_keyboard();
        
        void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
        void mouse_button(GLFWwindow* window, int button, int act, int mods);
        void mouse_move(GLFWwindow* window, double xpos, double ypos);
        void scroll(GLFWwindow* window, double xoffset, double yoffset);

    private:


};


#endif