@echo off
rem SPDX-License-Identifier: MIT
rem Adapted from omarchyvr windows/capture-session.cmd (MIT OR Apache-2.0).
rem PiBridge headset data kit: double-click this file. It asks for Administrator once (USBPcap needs it) and starts
rem the guided run pibridge-headset-data.ps1. Read only towards the headset; nothing is uploaded.
setlocal
rem 64-bit PowerShell (the Pimax client library is 64-bit); Sysnative exists only when this runs as a 32-bit process
set PS=%SystemRoot%\Sysnative\WindowsPowerShell\v1.0\powershell.exe
if not exist "%PS%" set PS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe
"%PS%" -NoProfile -Command "Start-Process '%PS%' -Verb RunAs -ArgumentList '-NoProfile -ExecutionPolicy Bypass -File \"%~dp0pibridge-headset-data.ps1\"'"
if errorlevel 1 (
  echo.
  echo The kit did not start. Windows asks "Do you want to allow this app to make changes": please answer Yes.
  pause
)
endlocal
