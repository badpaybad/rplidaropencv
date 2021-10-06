# rplidaropencv
rplidar a1 and opencv

# vcpkg.io

clone to C:/ or / in linux 

                git clone https://github.com/Microsoft/vcpkg.git

                vcpkg install opencv

# rplidar sdk

clone to C:/ or / in linux 

                git clone https://github.com/Slamtec/rplidar_sdk.git

install driver for window

                C:\rplidar_sdk\tools\cp2102_driver\CP210x_Windows_Drivers\CP210xVCPInstaller_x64.exe

                you may find in device manager: Silicon Labs CP210x USB to UART Bridge (COM6)


# run or debug in vs code

find you COM port then replace in main.cpp  Window:  opt_com_path = "\\\\.\\COM6";  Linux  opt_com_path = "/dev/ttyUSB0"; 

                cmake -B build -S .
                cmake --build build

                check function: capture_and_display