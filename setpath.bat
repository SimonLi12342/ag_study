@echo off

REM ***************************************************************
REM * This script adds Mitsuba to the current path on Windows.
REM * It assumes that Mitsuba is either compiled within the 
REM * source tree or within a subdirectory named 'build'.
REM ***************************************************************

set NORI_DIR=%~dp0
set NORI_DIR=%NORI_DIR:~0,-1%
set PATH=%PATH%;%NORI_DIR%\build\Release