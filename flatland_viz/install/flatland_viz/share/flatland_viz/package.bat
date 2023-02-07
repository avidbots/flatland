:: generated from colcon_core/shell/template/package.bat.em
@echo off

goto :eof


:: call the specified batch file and output the name when tracing is requested
:: first argument: the batch file
:call_file
  if exist "%~1" (
    if "%COLCON_TRACE%" NEQ "" echo call "%~1"
    call "%~1%"
  ) else (
    echo not found: "%~1" 1>&2
  )
goto:eof
