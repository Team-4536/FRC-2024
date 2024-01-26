:: installl python version 3.12.* the newest version
:: instal git


start https://www.python.org/ftp/python/3.12.1/python-3.12.1-amd64.exe

:waitlooppy
IF EXIST "python-3.12.1-amd64.exe" GOTO waitloopendpy
timeout /t 1
goto waitlooppy
:waitloopendpy

start python-3.12.1-amd64.exe





::make new repos folder in c:\
cd c:\
md repos
cd c:\repos
::download robotpy and upgrade it
py -3 -m pip install robotpy
py -3 -m pip install --upgrade robotpy
 
::puts repo into c:\ropos
git clone https://github.com/Team-4536/FRC-2024.git

cd c:\repos\FRC-2024\src

::downloads libraries found in src/pyproject.toml
py -3 -m robotpy sync 

::include pylance and python intellisense from vscode extensions

::download nitools
start https://www.ni.com/en/support/downloads/drivers/download/packaged.frc-game-tools.500107.html


cd C:\Users\minut\Downloads



::wait for file to be downloaded into downloads folder
:waitloopni
IF EXIST "ni-frc-2024-game-tools_24.0_online.exe" GOTO waitloopendni
timeout /t 1
goto waitloopni
:waitloopendni

::opens ni file
start ni-frc-2024-game-tools_24.0_online.exe