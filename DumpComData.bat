@echo off

::dontincludeheader Session should have include header set false!

:PortSet
reg query HKLM\HARDWARE\DEVICEMAP\SERIALCOMM
set /p "port=Enter COM Port(Or Enter to Reprint Ports): " 
if [%port%] == [] goto PortSet

putty -load dontincludeheader -serial %port% -sessionlog "%~dp0data/&Y_&M_&D_&T.bin" -sercfg 19200,8,1,n,N
echo Ended Writing To File!
pause