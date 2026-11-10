@echo off
rem Double-click to start the robot packing cell simulator.
rem Modbus port 5022 (RobotIO), picture at http://localhost:8091
cd /d "%~dp0"
python robot_server.py %*
if errorlevel 1 pause
