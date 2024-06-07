rem @echo off
set "usage=usage: flashi [{Debug|Release}]"

setlocal

set "build_type=Debug"
if not [%1]==[] set "build_type=%1"

set "ws_root=C:\Users\gene\Documents\Gene\proj\STM32CubeIDE\workspace_1.6.1\G030-PROJECTS"
set "sn=066BFF575550897767162155"
set "image_file=%ws_root%\%build_type%\G030-PROJECTS.bin"

"%ws_root%\ci-cd-tools\flash.bat" %sn% "%image_file%"
