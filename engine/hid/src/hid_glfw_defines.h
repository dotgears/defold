// Copyright 2020 The Defold Foundation
// Licensed under the Defold License version 1.0 (the "License"); you may not use
// this file except in compliance with the License.
// 
// You may obtain a copy of the License, together with FAQs at
// https://www.defold.com/license
// 
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#ifndef HID_GLFW_DEFINES_H
#define HID_GLFW_DEFINES_H

#include <graphics/glfw/glfw.h>

#define HID_MAX_GAMEPAD_COUNT GLFW_JOYSTICK_LAST + 1

#define HID_KEY_SPACE ' '
#define HID_KEY_EXCLAIM '!'
#define HID_KEY_QUOTEDBL '"'
#define HID_KEY_HASH '#'
#define HID_KEY_DOLLAR '$'
#define HID_KEY_AMPERSAND '&'
#define HID_KEY_QUOTE '\''
#define HID_KEY_LPAREN '('
#define HID_KEY_RPAREN ')'
#define HID_KEY_ASTERISK '*'
#define HID_KEY_PLUS '+'
#define HID_KEY_COMMA ','
#define HID_KEY_MINUS '-'
#define HID_KEY_PERIOD '.'
#define HID_KEY_SLASH '/'

#define HID_KEY_0 '0'
#define HID_KEY_1 '1'
#define HID_KEY_2 '2'
#define HID_KEY_3 '3'
#define HID_KEY_4 '4'
#define HID_KEY_5 '5'
#define HID_KEY_6 '6'
#define HID_KEY_7 '7'
#define HID_KEY_8 '8'
#define HID_KEY_9 '9'

#define HID_KEY_COLON ':'
#define HID_KEY_SEMICOLON ';'
#define HID_KEY_LESS '<'
#define HID_KEY_EQUALS '='
#define HID_KEY_GREATER '>'
#define HID_KEY_QUESTION '?'
#define HID_KEY_AT '@'

#define HID_KEY_A 'A'
#define HID_KEY_B 'B'
#define HID_KEY_C 'C'
#define HID_KEY_D 'D'
#define HID_KEY_E 'E'
#define HID_KEY_F 'F'
#define HID_KEY_G 'G'
#define HID_KEY_H 'H'
#define HID_KEY_I 'I'
#define HID_KEY_J 'J'
#define HID_KEY_K 'K'
#define HID_KEY_L 'L'
#define HID_KEY_M 'M'
#define HID_KEY_N 'N'
#define HID_KEY_O 'O'
#define HID_KEY_P 'P'
#define HID_KEY_Q 'Q'
#define HID_KEY_R 'R'
#define HID_KEY_S 'S'
#define HID_KEY_T 'T'
#define HID_KEY_U 'U'
#define HID_KEY_V 'V'
#define HID_KEY_W 'W'
#define HID_KEY_X 'X'
#define HID_KEY_Y 'Y'
#define HID_KEY_Z 'Z'

#define HID_KEY_LBRACKET '['
#define HID_KEY_BACKSLASH '\\'
#define HID_KEY_RBRACKET ']'
#define HID_KEY_CARET '^'
#define HID_KEY_UNDERSCORE '_'
#define HID_KEY_BACKQUOTE '`'

#define HID_KEY_LBRACE '{'
#define HID_KEY_PIPE '|'
#define HID_KEY_RBRACE '}'
#define HID_KEY_TILDE '~'

#define HID_KEY_ESC GLFW_KEY_ESC
#define HID_KEY_F1 GLFW_KEY_F1
#define HID_KEY_F2 GLFW_KEY_F2
#define HID_KEY_F3 GLFW_KEY_F3
#define HID_KEY_F4 GLFW_KEY_F4
#define HID_KEY_F5 GLFW_KEY_F5
#define HID_KEY_F6 GLFW_KEY_F6
#define HID_KEY_F7 GLFW_KEY_F7
#define HID_KEY_F8 GLFW_KEY_F8
#define HID_KEY_F9 GLFW_KEY_F9
#define HID_KEY_F10 GLFW_KEY_F10
#define HID_KEY_F11 GLFW_KEY_F11
#define HID_KEY_F12 GLFW_KEY_F12
#define HID_KEY_UP GLFW_KEY_UP
#define HID_KEY_DOWN GLFW_KEY_DOWN
#define HID_KEY_LEFT GLFW_KEY_LEFT
#define HID_KEY_RIGHT GLFW_KEY_RIGHT
#define HID_KEY_LSHIFT GLFW_KEY_LSHIFT
#define HID_KEY_RSHIFT GLFW_KEY_RSHIFT
#define HID_KEY_LCTRL GLFW_KEY_LCTRL
#define HID_KEY_RCTRL GLFW_KEY_RCTRL
#define HID_KEY_LALT GLFW_KEY_LALT
#define HID_KEY_RALT GLFW_KEY_RALT
#define HID_KEY_TAB GLFW_KEY_TAB
#define HID_KEY_ENTER GLFW_KEY_ENTER
#define HID_KEY_BACKSPACE GLFW_KEY_BACKSPACE
#define HID_KEY_INSERT GLFW_KEY_INSERT
#define HID_KEY_DEL GLFW_KEY_DEL
#define HID_KEY_PAGEUP GLFW_KEY_PAGEUP
#define HID_KEY_PAGEDOWN GLFW_KEY_PAGEDOWN
#define HID_KEY_HOME GLFW_KEY_HOME
#define HID_KEY_END GLFW_KEY_END

#define HID_KEY_KP_0 GLFW_KEY_KP_0
#define HID_KEY_KP_1 GLFW_KEY_KP_1
#define HID_KEY_KP_2 GLFW_KEY_KP_2
#define HID_KEY_KP_3 GLFW_KEY_KP_3
#define HID_KEY_KP_4 GLFW_KEY_KP_4
#define HID_KEY_KP_5 GLFW_KEY_KP_5
#define HID_KEY_KP_6 GLFW_KEY_KP_6
#define HID_KEY_KP_7 GLFW_KEY_KP_7
#define HID_KEY_KP_8 GLFW_KEY_KP_8
#define HID_KEY_KP_9 GLFW_KEY_KP_9
#define HID_KEY_KP_DIVIDE GLFW_KEY_KP_DIVIDE
#define HID_KEY_KP_MULTIPLY GLFW_KEY_KP_MULTIPLY
#define HID_KEY_KP_SUBTRACT GLFW_KEY_KP_SUBTRACT
#define HID_KEY_KP_ADD GLFW_KEY_KP_ADD
#define HID_KEY_KP_DECIMAL GLFW_KEY_KP_DECIMAL
#define HID_KEY_KP_EQUAL GLFW_KEY_KP_EQUAL
#define HID_KEY_KP_ENTER GLFW_KEY_KP_ENTER
#define HID_KEY_KP_NUM_LOCK GLFW_KEY_KP_NUM_LOCK
#define HID_KEY_CAPS_LOCK GLFW_KEY_CAPS_LOCK
#define HID_KEY_SCROLL_LOCK GLFW_KEY_SCROLL_LOCK
#define HID_KEY_PAUSE GLFW_KEY_PAUSE
#define HID_KEY_LSUPER GLFW_KEY_LSUPER
#define HID_KEY_RSUPER GLFW_KEY_RSUPER
#define HID_KEY_MENU GLFW_KEY_MENU
#define HID_KEY_BACK GLFW_KEY_BACK

#define HID_MOUSE_BUTTON_LEFT GLFW_MOUSE_BUTTON_LEFT
#define HID_MOUSE_BUTTON_MIDDLE GLFW_MOUSE_BUTTON_MIDDLE
#define HID_MOUSE_BUTTON_RIGHT GLFW_MOUSE_BUTTON_RIGHT
#define HID_MOUSE_BUTTON_1 GLFW_MOUSE_BUTTON_1
#define HID_MOUSE_BUTTON_2 GLFW_MOUSE_BUTTON_2
#define HID_MOUSE_BUTTON_3 GLFW_MOUSE_BUTTON_3
#define HID_MOUSE_BUTTON_4 GLFW_MOUSE_BUTTON_4
#define HID_MOUSE_BUTTON_5 GLFW_MOUSE_BUTTON_5
#define HID_MOUSE_BUTTON_6 GLFW_MOUSE_BUTTON_6
#define HID_MOUSE_BUTTON_7 GLFW_MOUSE_BUTTON_7
#define HID_MOUSE_BUTTON_8 GLFW_MOUSE_BUTTON_8

#endif
