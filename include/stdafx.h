/****************************************************************************************
* Application :	Camera Calibration Application using
*
*					-OpenCV3 (http://opencv.org/) and
*					-PS3EYEDriver C API Interface (by Thomas Perl) for Windows build
*					-Video4Linux2 for Linux build
*
* Author      :		Michael Stengel <virtuellerealitaet@gmail.com>
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    1. Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*
*    2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/

#pragma once

#ifdef _WIN32

    #if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
    #define _CRT_SECURE_NO_WARNINGS
    #endif

    #include <ctype.h>          // toupper, isprint
    #include <math.h>           // sqrtf, powf, cosf, sinf, floorf, ceilf
    #include <stdio.h>          // vsnprintf, sscanf, printf
    #include <stdlib.h>         // NULL, malloc, free, qsort, atoi

    #if defined(_MSC_VER) && _MSC_VER <= 1500 // MSVC 2008 or earlier
    #include <stddef.h>         // intptr_t
    #else
    #include <stdint.h>         // intptr_t
    #endif

    #ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strcpy, strdup, sprintf, vsnprintf, sscanf, fopen
    #define snprintf _snprintf
    #endif
    #ifdef __clang__
    #pragma clang diagnostic ignored "-Wold-style-cast"             // warning : use of old-style cast                              // yes, they are more terse.
    #pragma clang diagnostic ignored "-Wdeprecated-declarations"    // warning : 'xx' is deprecated: The POSIX name for this item.. // for strdup used in demo code (so user can copy & paste the code)
    #pragma clang diagnostic ignored "-Wint-to-void-pointer-cast"   // warning : cast to 'void *' from smaller integer type 'int'
    #pragma clang diagnostic ignored "-Wformat-security"            // warning : warning: format string is not a string literal
    #pragma clang diagnostic ignored "-Wexit-time-destructors"      // warning : declaration requires an exit-time destructor       // exit-time destruction order is undefined. if MemFree() leads to users code that has been disabled before exit it might cause problems. ImGui coding style welcomes static/globals.
    #if __has_warning("-Wreserved-id-macro")
    #pragma clang diagnostic ignored "-Wreserved-id-macro"          // warning : macro name is a reserved identifier                //
    #endif
    #elif defined(__GNUC__)
    #pragma GCC diagnostic ignored "-Wint-to-pointer-cast"          // warning: cast to pointer from integer of different size
    #pragma GCC diagnostic ignored "-Wformat-security"              // warning : format string is not a string literal (potentially insecure)
    #pragma GCC diagnostic ignored "-Wdouble-promotion"             // warning: implicit conversion from 'float' to 'double' when passing argument to function
    #pragma GCC diagnostic ignored "-Wconversion"                   // warning: conversion to 'xxxx' from 'xxxx' may alter its value
    #if (__GNUC__ >= 6)
    #pragma GCC diagnostic ignored "-Wmisleading-indentation"       // warning: this 'if' clause does not guard this statement      // GCC 6.0+ only. See #883 on github.
    #endif
    #endif

    // fix line endings for Windows users

    #define IM_NEWLINE "\r\n"
    #else
    #define IM_NEWLINE "\n"


    #define IM_ARRAYSIZE(_ARR)  ((int)(sizeof(_ARR)/sizeof(*_ARR)))
    #define IM_MAX(_A,_B)       (((_A) >= (_B)) ? (_A) : (_B))
#endif

#ifdef WIN32

    // Windows API
    //#include <Windows.h>
    //#include <windowsx.h>
    //#include <direct.h>
    //#include <mmsystem.h>

    //// windows dialog includes
    //#pragma region Includes and Manifest Dependencies
    //#include <windows.h>
    //#include <windowsx.h>
    //#include <strsafe.h>
    //#include <new>
    //#include <shlobj.h>
    //#include <shlwapi.h>
    //#include <atlstr.h>
    //#include <atlconv.h>

#endif


// system includes

//#include <iostream>     // std::cout
//#include <iomanip>      // std::setprecision
//#include <fstream>      // fopen
//#include <string>       // std::string
//#include <sstream>      // std::stringstream
//#include <vector>       // std::vector
//#include <map>          // std::map
//#include <unordered_map>
//#include <set>
//#include <typeinfo>
//#include <algorithm>
//#include <iterator>
//#include <stdio.h>
//#include <stdlib.h>
//#include <ctype.h>
#include <thread>

// OpenCV
#include <opencv2/opencv.hpp>

typedef uint8_t PIXELVALUE;

#ifdef WIN32

#define exit(_Code) { if (_Code) { assert(0); std::cout << "Quit..."; std::string s; std::cin.putback('X'); std::cin >> s; exit(_Code); } }

#define  LOGCON(...)  { printf(__VA_ARGS__); }
#define  LOGDEBUG(...)  {	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); SetConsoleTextAttribute(hConsole, 10); printf(__VA_ARGS__);SetConsoleTextAttribute(hConsole, 7);}
#define  LOGWARNING(...)  {	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); SetConsoleTextAttribute(hConsole, 14); printf("WARNING : "); printf(__VA_ARGS__);SetConsoleTextAttribute(hConsole, 7);}
#define  LOGERROR(...)  {	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); SetConsoleTextAttribute(hConsole, 12); printf("ERROR : "); printf(__VA_ARGS__);SetConsoleTextAttribute(hConsole, 7);}
#else
#define  LOGCON(...)		{	printf(__VA_ARGS__);}
#define  LOGDEBUG(...)		{	printf("DEBUG : "); printf(__VA_ARGS__);}
#define  LOGWARNING(...)	{	printf("WARNING : "); printf(__VA_ARGS__);}
#define  LOGERROR(...)		{	printf("ERROR : "); printf(__VA_ARGS__);}
#endif

static void Sleep(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
