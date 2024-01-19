echo on

set pipvenvV=pipenv, version 2023.11.15

FOR /F "tokens=* USEBACKQ" %%F IN (`pipenv --version`) DO (
if "%%F" NEQ "%pipvenvV%" (
  echo Incorrect version of pipenv or is not installed! Expected %pipvenvV% but has %%F
  set err=1
))

cd c:\repos\FRC-2024

rem deactivate

rem delete .venv
del /s /q .\.venv

rem make new .venv file
mkdir .venv

rem have pipvenv make the eviroment
pipenv install --python 3.12




cd c:\repos\FRC-2024
.\.venv\scripts\activate

rem go to src to sync
rem cd .\src
rem GOTO src & RUN THIS AT THE END:
rem pipenv run robotpy sync