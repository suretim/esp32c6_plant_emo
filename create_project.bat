@echo off
echo 创建ESP32植物传感器项目...

:: 创建目录结构
mkdir plant_sensor_esp32
mkdir plant_sensor_esp32\main
mkdir plant_sensor_esp32\components
mkdir plant_sensor_esp32\tools

:: 创建文件
echo 创建CMakeLists.txt...
copy nul plant_sensor_esp32\CMakeLists.txt
echo 创建main/CMakeLists.txt...
copy nul plant_sensor_esp32\main\CMakeLists.txt
echo 创建main/main.c...
copy nul plant_sensor_esp32\main\main.c
echo 创建main/Kconfig.projbuild...
copy nul plant_sensor_esp32\main\Kconfig.projbuild
echo 创建sdkconfig.defaults...
copy nul plant_sensor_esp32\sdkconfig.defaults
echo 创建partitions.csv...
copy nul plant_sensor_esp32\partitions.csv
echo 创建README.md...
copy nul plant_sensor_esp32\README.md
echo 创建tools/test_connection.py...
copy nul plant_sensor_esp32\tools\test_connection.py

echo 项目结构创建完成！
echo.
echo 现在请将以下文件内容复制到对应的文件中：
echo.
echo 1. plant_sensor_esp32/CMakeLists.txt
echo 2. plant_sensor_esp32/main/CMakeLists.txt
echo 3. plant_sensor_esp32/main/main.c
echo 4. plant_sensor_esp32/main/Kconfig.projbuild
echo 5. plant_sensor_esp32/sdkconfig.defaults
echo 6. plant_sensor_esp32/partitions.csv
echo 7. plant_sensor_esp32/README.md
echo 8. plant_sensor_esp32/tools/test_connection.py
echo.
pause