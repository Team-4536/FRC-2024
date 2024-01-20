echo on

start cmd

set pipvenvV=pipenv, version 2023.11.15

FOR /F "tokens=* USEBACKQ" %%F IN (`pipenv --version`) DO (
if "%%F" NEQ "%pipvenvV%" (
  echo Incorrect version of pipenv or is not installed! Expected %pipvenvV% but has %%F
  set err=1
))

cd c:\repos\FRC-2024



rem delete .venv
rem del /s /q .\.venv
RD /S /Q .venv

rem make new .venv file

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

rem go to src to sync
cd .\src
rem GOTO src & RUN THIS AT THE END:
pipenv run robotpy sync
.\.venv\scripts\activate