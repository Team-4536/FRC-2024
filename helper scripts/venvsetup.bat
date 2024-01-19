echo off

set pipvenvV=pipenv, version 2023.11.15

FOR /F "tokens=* USEBACKQ" %%F IN (`pipenv --version`) DO (
if "%%F" NEQ "%pipvenvV%" (
  echo Incorrect version of pipenv or is not installed! Expected %pipvenvV% but has %%F
  set err=1
))

.\.venv\scripts\deactivate.bat

cd c:\repos\FRC-2024

rem delete .venv
rmdir /s .venv

rem make new .venv file
mkdir .venv

rem have pipvenv make the eviroment
pipenv install




cd c:\repos\FRC-2024
.\.venv\scripts\activate

rem go to src to sync
cd .\src

rem RUN THIS AT THE END:
rem pipenv run robotpy sync