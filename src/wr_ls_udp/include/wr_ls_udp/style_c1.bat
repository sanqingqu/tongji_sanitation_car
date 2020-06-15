@echo off

set astyle="c:\Program Files\AStyle\bin\astyle.exe"

for /r . %%a in (*.cpp;*.c) do %astyle% -s4  -S  -K -xW -w -M40 -p -k3 -c -xL -j -A1 "%%a"

for /r . %%a in (*.hpp;*.h) do %astyle% -s4  -S  -K -xW -w -M40 -p -k3 -c -xL -j -A1 "%%a"

for /r . %%a in (*.orig) do del "%%a"

pause