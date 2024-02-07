cd C:\repos\FRC-2024

rem delete .venv
RD /s /q .\.venv



:waitvenvdel
IF NOT EXIST ".venv" GOTO waitvenvdelend
timeout /t 1
goto waitvenvdel
:waitvenvdelend

mkdir .venv

timeout /t 5
rem have pipvenv make the eviroment
pipenv install --python 3.12


timeout /t 5

cd c:\repos\FRC-2024
call .\.venv\scripts\activate

py -m pip install robotpy

rem go to src to sync
cd .\src
rem GOTO src & RUN THIS AT THE END:
robotpy sync
cd..
.\.venv\scripts\activate