@echo off

:PortSet
chgport
set /p "port=Enter COM Port(Or Enter to Reprint Ports): " 
if [%port%] == [] goto PortSet

set file=%date%_%time%.bin
set file=%file:/=_%
set file=%file::=_%
set file=data/%file: =-%

echo Writing to File: %port% to %file%
mode %port% baud=19200 parity=n stop=1 data=8
type %port%: >> "%file%"
echo Ended Writing To File!
pause