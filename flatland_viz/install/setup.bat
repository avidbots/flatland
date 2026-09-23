:: generated from colcon_core/shell/template/prefix_chain.bat.em
@echo off

:: This script extends the environment with the environment of other prefix
:: paths which were sourced when this file was generated as well as all
:: packages contained in this prefix path.

:: source this prefix
call:_colcon_prefix_chain_bat_call_script "%%~dp0local_setup.bat"

goto:eof


:: function to source another script with conditional trace output
:: first argument: the path of the script
:_colcon_prefix_chain_bat_call_script
  if exist "%~1" (
    if "%COLCON_TRACE%" NEQ "" echo call "%~1"
    call "%~1%"
  ) else (
    echo not found: "%~1" 1>&2
  )
goto:eof
