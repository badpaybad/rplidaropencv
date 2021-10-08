
#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include <math.h>
#include <cmath>
#include <iostream>

#include <chrono>
#include <ctime>
#include <thread>
#include <mutex>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x) ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms)
{
    while (ms >= 1000)
    {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}
#endif

using namespace cv;
using namespace rp::standalone::rplidar;

void print_usage(int argc, const char *argv[])
{

    // // print out the device serial number, firmware and hardware version number..
    // printf("RPLIDAR S/N: ");
    // for (int pos = 0; pos < 16; ++pos)
    // {
    //     printf("%02X", devinfo.serialnum[pos]);
    // }

    // printf("\n"
    //        "Version: " RPLIDAR_SDK_VERSION "\n"
    //        "Firmware Ver: %d.%02d\n"
    //        "Hardware Rev: %d\n",
    //        devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF, (int)devinfo.hardware_version);
    printf("Simple LIDAR data grabber for RPLIDAR.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n"
           "Usage:\n"
           "%s <com port> [baudrate]\n"
           "The default baudrate is 115200(for A2) or 256000(for A3). Please refer to the datasheet for details.\n",
           argv[0]);
}

u_result capture_and_display(RPlidarDriver *drv)
{
    const int maxPoints = 8192;
    float pi = 3.1416f;
    float data_lidar[maxPoints][4];
    u_result ans;

    //rplidar_response_measurement_node_t nodes[maxPoints];
    rplidar_response_measurement_node_hq_t nodes[maxPoints];
    size_t count = _countof(nodes);

    // printf("waiting for data...\n");

    // fetech extactly one 0-360 degrees' scan

    //ans = drv->grabScanData(nodes, count);

    ans = drv->grabScanDataHq(nodes, count);
    if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT)
    {
        drv->ascendScanData(nodes, count);
        //plot_histogram(nodes, count);

        //  printf("Do you want to see all the data? (y/n) ");
        // int key = getchar();
        // int key = 'y';
        // if (key == 'Y' || key == 'y')
        // {

        // }

        //rplidar will return distance in mili met
        // we will show object scaned by lidar arount 1000 mili met
        int mapBase = 10000;
        int ratio = mapBase / 1000;

        int mapW = mapBase;
        int mapH = mapBase;

        // Create black empty images
        cv::Mat image = cv::Mat::zeros(mapW / ratio, mapH / ratio, CV_8UC3);

        int x, y = 0;
        int newX = mapW / 2;
        int newY = mapH / 2;

        //the lidar
        data_lidar[0][0] = 0;
        data_lidar[0][1] = 0;
        data_lidar[0][2] = newX;
        data_lidar[0][3] = newY;

        for (int posNext = 1; posNext < (int)count; ++posNext)
        {
            int pos = posNext - 1;

            //float tmpDis = nodes[pos].distance_q2 / 4.0f;
            float tmpDis = nodes[pos].dist_mm_q2 / 4.0f;
            //float tmpDis = nodes[pos].dist_mm_q2;
            //float tmpAng = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
            float tmpAng = nodes[pos].angle_z_q14 * 90.f / 16384.0f;
            if (tmpDis > 0.0f && tmpDis <= newX)
            {
                if (tmpAng >= 0 && tmpAng <= 90)
                {
                    x = cos((90 - tmpAng) * pi / 180) * tmpDis;
                    y = cos(tmpAng * pi / 180) * tmpDis;
                    x = newX + std::abs(x);
                    y = newY - std::abs(y);
                    data_lidar[pos][0] = tmpAng;
                    data_lidar[pos][1] = tmpDis;
                    data_lidar[pos][2] = x;
                    data_lidar[pos][3] = y;
                }
                else if (tmpAng > 90 && tmpAng <= 180)
                {
                    x = cos((tmpAng - 90) * pi / 180) * tmpDis;
                    y = cos((180 - tmpAng) * pi / 180) * tmpDis;
                    x = newX + std::abs(x);
                    y = newY + std::abs(y);
                    data_lidar[pos][0] = tmpAng;
                    data_lidar[pos][1] = tmpDis;
                    data_lidar[pos][2] = x;
                    data_lidar[pos][3] = y;
                }
                else if (tmpAng > 180 && tmpAng <= 270)
                {
                    x = cos((270 - tmpAng) * pi / 180) * tmpDis;
                    y = cos((tmpAng - 180) * pi / 180) * tmpDis;
                    x = newX - std::abs(x);
                    y = newY + std::abs(y);
                    data_lidar[pos][0] = tmpAng;
                    data_lidar[pos][1] = tmpDis;
                    data_lidar[pos][2] = x;
                    data_lidar[pos][3] = y;
                }
                else
                {
                    x = cos((tmpAng - 270) * pi / 180) * tmpDis;
                    y = cos((360 - tmpAng) * pi / 180) * tmpDis;
                    x = newX - std::abs(x);
                    y = newY - std::abs(y);
                    data_lidar[pos][0] = tmpAng;
                    data_lidar[pos][1] = tmpDis;
                    data_lidar[pos][2] = x;
                    data_lidar[pos][3] = y;
                }
            }
        }

        int data_lidar_len = _countof(data_lidar);

        newX = newX / ratio;
        newY = newY / ratio;

        //the lidar position
        cv::putText(image,
                    "lidar: x=" + std::to_string(newX) + ", y=" + std::to_string(newY), cv::Point(25, 25), cv::FONT_HERSHEY_PLAIN, 0.9, CV_RGB(255, 255, 0), 2);

        //cv::line(image, Point(0, 0), Point(2000, 2000), Scalar(0, 255, 0), 5, 8);
        // Draw a line

        float near_by[maxPoints][4];
        int counter_nearby = 0;
        int nearbyx = 0;
        int nearbyy = 0;
        int left = 0, top = 0, right = 0, bottom = 0;

        float disTb = 0;

        for (int pos = 0; pos < (int)data_lidar_len; ++pos)
        {
            int a = data_lidar[pos][0];
            int d = data_lidar[pos][1];
            int x = data_lidar[pos][2];
            int y = data_lidar[pos][3];

            if (x == 0 && y == 0)
                continue;

            if (x < 0 || y < 0)
                continue;

            x = x / ratio;
            y = y / ratio;

            if (pos == 0)
            {
                left = x;
                top = y;
                right = x;
                bottom = y;

                nearbyx = x;
                nearbyy = y;
                near_by[counter_nearby][0] = a;
                near_by[counter_nearby][1] = d;
                near_by[counter_nearby][2] = x;
                near_by[counter_nearby][3] = y;

                counter_nearby++;
                disTb = disTb + d;
            }
            else
            {
                if (x > nearbyx - 20 && x < nearbyx + 20 && y > nearbyy - 20 && y < nearbyy + 20)
                {
                }
                else
                {
                    nearbyx = x;
                    nearbyy = y;
                    near_by[counter_nearby][0] = a;
                    near_by[counter_nearby][1] = d;
                    near_by[counter_nearby][2] = x;
                    near_by[counter_nearby][3] = y;

                    counter_nearby++;

                    cv::line(image, Point(newX + 2, newY + 2), Point(x, y), Scalar(255, 0, 255), 1, 8);

                    cv::putText(image, std::to_string(d), cv::Point(x + 15, y + 15), cv::FONT_HERSHEY_PLAIN, 0.8, CV_RGB(255, 0, 255), 2);

                    // cv::putText(image,
                    //             "x=" + std::to_string(x) + ", y=" + std::to_string(x),
                    //             cv::Point(x + 50, y + 15),
                    //             cv::FONT_HERSHEY_PLAIN,
                    //             0.8,
                    //             CV_RGB(255, 0, 255),
                    //             2);

                    disTb = disTb + d;
                }
            }

            if (left > x)
                left = x;
            if (top > y)
                top = y;
            if (right < x)
                right = x;
            if (bottom < y)
                bottom = y;

            std::cout << "\r\nx: " << x << " y: " << y << " dis: " << d << " a: " << a;

            //object scaned by lidar position
            cv::line(image, Point(x, y), Point(x + 2, y + 2), Scalar(0, 0, 255), 2, 8);
        }

        // int near_by_len = _countof(near_by);

        // for (int pos = 0; pos < (int)near_by_len; ++pos)
        // {
        //     int a = data_lidar[pos][0];
        //     int d = data_lidar[pos][1];
        //     int x = data_lidar[pos][2];
        //     int y = data_lidar[pos][3];

        //     cv::line(image, Point(newX, newX), Point(newX + 2, newY + 2), Scalar(0, 255, 255), 2, 8);

        // }

        disTb = (disTb / counter_nearby) / ratio;

        cv::circle(image, Point(newX, newY), disTb, Scalar(255, 255, 255), 1);
        cv::line(image, Point(newX, newY), Point(newX + 2, newY + 2), Scalar(0, 255, 255), 2, 8);
        cv::circle(image, Point(newX, newY), newX, Scalar(255, 255, 255), 1);
        //cv::rectangle(image, Point(left, top), Point(right, bottom), Scalar(255, 0, 0), 2, 4);

        //cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);

        std::cout << "\r\n newY: " << newY << " newY: " << newY << " disTb: " << disTb << " total: " << data_lidar_len;

        imshow("data_lidar", image);
        waitKey(1);
    }
    else
    {
        printf("error code: %x\n", ans);
    }

    return ans;
}
u_result connect_rplidar(RPlidarDriver *drv, const char *opt_com_path, _u32 opt_com_baudrate)
{
    u_result op_result;
    rplidar_response_device_info_t devinfo;
    // try to connect
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate)))
    {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result))
    {
        if (op_result == RESULT_OPERATION_TIMEOUT)
        {
            // you can check the detailed failure reason
            fprintf(stderr, "Error, operation time out.\n");
        }
        else
        {
            fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
            // other unexpected result
            throw "Error : calling  drv->reset();";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return op_result;
}

u_result disconnect_rplidar(RPlidarDriver *drv)
{
    auto result = drv->reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    result = drv->stop();
    result = drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);

    return result;
}
//
// opencvrplidar.exe \\.\COM4  # on Windows
// ./opencvrplidar /dev/ttyUSB0 #linux
int main(int argc, const char *argv[])
{
    // create the driver instance
    RPlidarDriver *drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    const char *opt_com_path = NULL;
    _u32 opt_com_baudrate = 115200;

    if (argc < 2)
    {
        //   print_usage(argc, argv);

        opt_com_path = "\\\\.\\COM4"; // argv[1];
        //opt_com_path = "/dev/ttyUSB0"; // argv[1];
    }

    if (argc > 2)
        opt_com_baudrate = strtoul(argv[2], NULL, 10);

    std::cout << "\nopt_com_path: " << opt_com_path << "need to check window (\\\\.\\COM4) or linux (/dev/ttyUSB0) opt_com_baudrate:" << opt_com_baudrate;

    if (!drv)
    {
        std::cout << "\ninsufficent memory, exit\n";
        exit(-2);
    }

    rplidar_response_device_health_t healthinfo;

    auto op_result = connect_rplidar(drv, opt_com_path, opt_com_baudrate);
    do
    {
        try
        {

            // check the device health
            ////////////////////////////////////////
            op_result = drv->getHealth(healthinfo);
            if (IS_OK(op_result))
            { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
                std::cout << ("\nRPLidar health status : ");
                switch (healthinfo.status)
                {
                case RPLIDAR_STATUS_OK:
                    std::cout << ("\nOK.");
                    break;
                case RPLIDAR_STATUS_WARNING:
                    std::cout << ("\nWarning.");
                    break;
                case RPLIDAR_STATUS_ERROR:
                    std::cout << ("\nError.");
                    throw "\n errorcode: " + healthinfo.error_code;
                    break;
                }
            }
            else
            {
                throw "\n Error : calling  drv->reset();, cannot retrieve the lidar health code: " + op_result;
            }

            if (healthinfo.status == RPLIDAR_STATUS_ERROR)
            {
                throw "\n Error, rplidar internal error detected. Please reboot the device to retry.\n";
                // enable the following code if you want rplidar to be reboot by software
                // drv->reset();
            }

            std::cout << "started new round";
            //drv->startMotor();

            // take only one 360 deg scan and display the result as a histogram
            ////////////////////////////////////////////////////////////////////////////////
            if (IS_FAIL(drv->startScan(0, 1))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
            {
                throw "\nError, cannot start the scan operation.\n";
            }

            if (IS_FAIL(capture_and_display(drv)))
            {
                throw "\nError, cannot grab scan data.\n";
            }
        }
        catch (char *msg)
        {
            std::cout << msg;
        }
        catch (std::string msg)
        {
            std::cout << msg;
        }
        catch (...)
        {
            std::cout << "\nERROR rplidar restarting ...";

            disconnect_rplidar(drv);

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

            connect_rplidar(drv, opt_com_path, opt_com_baudrate);
        }
    } while (1);

    drv->stop();
    drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);

    float a = 90.0f;
    float pi = 3.1416f;

    std::cout << sinf(a * pi / 180) << "\n";

    return (0);
}
