@echo off
SETLOCAL ENABLEDELAYEDEXPANSION

rem err will be set to 1 if an inconsistency occures
set /a err=0

rem Python version found from py --version
set pyV=Python 3.12.1

rem git version found from git --version
set gitV=git version 2.43.0.windows.1

rem VSCode version found from code --version (only the first line is checked)
set vscodeV=1.85.1

rem robotpy version found from py -3 -m pip show robotpy
set robotpyV=Version: 2024.1.1.3
set robotpyInstall=py -3 -m pip install robotpy -v "robotpy==2024.1.1.3"

rem this line reads the output of "py --vserion" and puts it into %%F
FOR /F "tokens=* USEBACKQ" %%F IN (`py --version`) DO (
rem Checks if %%F != pyV
if "%%F" NEQ "%pyV%" (
  echo Incorrect version of python or is not installed! Expected %pyV% but has %pyVI%
  set err=1
))


FOR /F "tokens=* USEBACKQ" %%F IN (`git --version`) DO (
if "%%F" NEQ "%gitV%" (
  echo Incorrect version of Git or is not installed! Expected %gitV% but has %%F
  set err=1
))

SET /a count=1
FOR /F "tokens=* USEBACKQ" %%F IN (`py -3 -m pip show robotpy`) DO (
  SET line!count!=%%F
  SET /a count=!count!+1
)

if "%line1%" EQU "" (
    echo Robotpy is not installed.
    set /a err=1
) else (
    if "%line2%" NEQ "%robotpyV%" (
        echo Incorrect version of robotpy. Expected %robotpyV% but has %line2%
        echo delete curent version and install with %robotpyInstall%
        set /a err=1
    )
)





if %err% == 0 (
    echo no issues found
)
if %err% == 1 (
    echo Review issues found
)


pause