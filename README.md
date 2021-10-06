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

we use git path cloned for CMakeLists.txt 

                MY_VCPKG_INSTALLED_DIR

                SDK_RPLIDAR_DIR


# run or debug in vs code

find you COM port then replace in main.cpp  Window:  opt_com_path = "\\\\.\\COM6";  Linux  opt_com_path = "/dev/ttyUSB0"; 

                cmake -B build -S .
                cmake --build build

                

# rplidar.png

how to convert from angle and distance to x,y 

                O.x=0 ; O.y=0 ; // the lidar 
                A.x= cos((90 - tmpAng) * pi / 180) * tmpDis;
                A.y= cos(tmpAng * pi / 180) * tmpDis;

                if we want draw image from O1 so O1.x=0, O1.y=0 
                thus now O.x= 1000; O.y=1000

                A.x1 = O.x + std::abs(x);
                A.y1 = O.y - std::abs(y);

                ....
                check function: capture_and_display


