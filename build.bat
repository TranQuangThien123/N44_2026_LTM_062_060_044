@echo off
echo ==========================================================
echo   BIEN DICH HE THONG UAV WINSOCK (MinGW GCC)
echo ==========================================================

if not exist bin mkdir bin

echo [*] Bien dich src/test_udp.c...
gcc -O2 -Iinclude src/test_udp.c -o bin/test_udp.exe -lws2_32
if %errorlevel% neq 0 (
    echo [-] Loi bien dich test_udp.c!
    exit /b %errorlevel%
)

echo [*] Bien dich src/telemetry_parser.c...
gcc -O2 -Iinclude src/telemetry_parser.c -o bin/telemetry_parser.exe -lws2_32
if %errorlevel% neq 0 (
    echo [-] Loi bien dich telemetry_parser.c!
    exit /b %errorlevel%
)

echo [*] Bien dich src/control_uav.c...
gcc -O2 -Iinclude src/control_uav.c -o bin/control_uav.exe -lws2_32
if %errorlevel% neq 0 (
    echo [-] Loi bien dich control_uav.c!
    exit /b %errorlevel%
)

echo [*] Bien dich src/mission_with_network_metrics.c...
gcc -O2 -Iinclude src/mission_with_network_metrics.c -o bin/mission_with_network_metrics.exe -lws2_32
if %errorlevel% neq 0 (
    echo [-] Loi bien dich mission_with_network_metrics.c!
    exit /b %errorlevel%
)

echo ==========================================================
echo [SUCCESS] Da bien dich thanh cong 4 file thuc thi trong bin/!
echo ==========================================================
dir bin\*.exe
