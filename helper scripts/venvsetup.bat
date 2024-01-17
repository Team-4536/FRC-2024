echo off

set pipvenvV=pipenv, version 2023.11.15

FOR /F "tokens=* USEBACKQ" %%F IN (`pipenv --version`) DO (
if "%%F" NEQ "%pipvenvV%" (
  echo Incorrect version of pipenv or is not installed! Expected %pipvenvV% but has %%F
  set err=1
))

cd c:\repos\FRC-2024

rem delete .venv
rmdir /s .venv

rem make new .venv file
mkdir .venv

rem have pipvenv make the eviroment
pipenv install

rem go to src to sync
cd .\src
py -m robotpy sync