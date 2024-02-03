@echo off
echo This is a basic test to see if certain files and their versions are installed. Do not trust this to fully debug.
echo Checks python, git, vscode, pipenv, wpilib, and driver station
SETLOCAL ENABLEDELAYEDEXPANSION

rem err will be set to 1 if an inconsistency occures
set /a err=0

rem Python version found from py --version
set pyV=Python 3.12.1

rem git version found from git --version
set gitV=git version 2.43.0.windows.1

rem VSCode version found from code --version (only the first line is checked)
set vscodeV=1.85.1

rem checks to see that pipenv is installed based off of pipenv --version
set pipenvV=pipenv, version 2023.11.17

rem checking for the wpilib version by looking for path in desktop (the default download location)
set wpilibPath="2024 WPILib Tools"

rem checking if driver station is installed on desktop
set driverStation="FRC Driver Station.lnk"

rem this line reads the output of "py --vserion" and puts it into %%F
FOR /F "tokens=* USEBACKQ" %%F IN (`py --version`) DO (
rem Checks if %%F != pyV 
if "%%F" NEQ "%pyV%" (
  echo Incorrect version of python or is not installed! Expected %pyV% but has %%F
  set err=1
))


FOR /F "tokens=* USEBACKQ" %%F IN (`git --version`) DO (
if "%%F" NEQ "%gitV%" (
  echo Incorrect version of Git or is not installed! Expected %gitV% but has %%F
  set err=1
))

cd C:\Users\Public\Desktop
if not exist %wpilibPath% (
  echo wpilib2024 does not seem to be installed. Should be downloaded on the desktop.
  set /a err=1
)

cd C:\Users\Public\Desktop
if not exist %driverStation% (
  echo Driver Station does not seem to be installed. Should be downloaded on the desktop.
  set /a err=1
)


FOR /F "tokens=* USEBACKQ" %%F IN (`pipenv --version`) DO (
if "%%F" NEQ "%pipenvV%" (
  echo Incorrect version of pipenv or is not installed! Expected %pipenvV% but has %%F
  set err=1
))


if %err% == 0 (
    echo no issues found
)
if %err% == 1 (
    echo Review issues found
)


pause