@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /std:c++20 /O2 /EHsc /utf-8 /DUNICODE /D_UNICODE /DNOMINMAX /DWIN32_LEAN_AND_MEAN /DDIRECTINPUT_VERSION=0x0800 src\main.cpp /Fe:build\Release\RCTrainer.exe /link /SUBSYSTEM:WINDOWS dinput8.lib dxguid.lib winmm.lib comctl32.lib user32.lib gdi32.lib
exit /b %errorlevel%
