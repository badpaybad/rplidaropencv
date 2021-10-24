
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <cmath>
#include <iostream>

#include <string>
#include <chrono>
#include <ctime>
#include <thread>
#include <mutex>
#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "uwebsockets/App.h"
#include <nlohmann/json.hpp>

//#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include "common.h"

class SlLidarConnector
{
public:
    int _scanDiameterMilimet = 10000;
    int _scanRadius = 5000;
    float _mapRatio;
    int _mapWidth;
    int _mapHeight;
    int _stop = 0;

    int _opt_channel_type = sl::ChannelType::CHANNEL_TYPE_SERIALPORT;

    const char *_opt_com_path = NULL;
    sl_u32 _opt_com_baudrate;

    sl::ILidarDriver *_drv;
    sl::IChannel *_channel;
    /**
     * @brief angle, distance
     *
     */
    std::function<void(float, float)> _onLidarCapture = NULL;
    /**
     * @brief x,y, Lidar.x, Lidar.y, mapRatio, scanDeameter, angle, distance
     *
     */
    std::function<void(RplidarPoint)> _onLidarPositionCalculated = NULL;

    std::function<void(RplidarPoint[],int)> _onFrameCaptured = NULL;

    /**
     * @brief
     *
     * @param portCOM_Connect  win: "\\\\.\\COM4" , linux: /dev/ttyUSB0
     * @param opt_com_baudrate 115200
     * @param scanDiameterMilimet 10000
     */
    void init(const char *portCOM_Connect, sl_u32 opt_com_baudrate = 115200, int scanRadiusMilimet = 5000, float mapRatio = 0)
    {
        _opt_com_baudrate = opt_com_baudrate;
        _opt_com_path = portCOM_Connect;
        _scanRadius = scanRadiusMilimet;
        _scanDiameterMilimet = _scanRadius * 2;
       
        if (mapRatio == 0)
            _mapRatio = (float)_scanDiameterMilimet / 1000.0f;
        else
            _mapRatio = mapRatio;

        _mapWidth = _scanDiameterMilimet;
        _mapHeight = _scanDiameterMilimet;

        // create the driver instance
        _drv = (*sl::createLidarDriver());
        ///  Create a communication channel instance
        if (_opt_channel_type == sl::ChannelType::CHANNEL_TYPE_SERIALPORT)
        {
            // Result<ISerialChannel*>
            _channel = (*sl::createSerialPortChannel(_opt_com_path, _opt_com_baudrate));
        }

        std::cout << "\nopt_com_path: " << _opt_com_path << " opt_com_baudrate:" << _opt_com_baudrate << " [need to check window (\\\\.\\COM4) or linux (/dev/ttyUSB0)]";

        if (!_drv)
        {
            std::cout << "\ninsufficent memory, exit\n";
            throw "\ninsufficent memory, exit\n";
        }
    }

    void registerHandle(std::function<void(RplidarPoint[],int)> onFrameCaptured,
                        std::function<void(RplidarPoint)> onLidarPositionCalculated = NULL,
                        std::function<void(float, float)> onLidarCapture = NULL)
    {
        _onFrameCaptured = onFrameCaptured;
        _onLidarCapture = onLidarCapture;
        _onLidarPositionCalculated = onLidarPositionCalculated;
    }

    sl_result capture_and_display(sl::ILidarDriver *drv)
    {
        float pi = _pi31416;
        sl_result ans;

        sl_lidar_response_measurement_node_hq_t nodes[_maxPoints];
        size_t count = _countof(nodes);

        ans = drv->grabScanDataHq(nodes, count);
        if (SL_IS_OK(ans))
        {
            drv->ascendScanData(nodes, count);

            int x, y = 0;
            int newX = _mapWidth / 2;
            int newY = _mapHeight / 2;

            RplidarPoint frame[_maxPoints];

            for (int posNext = 1; posNext < (int)count; ++posNext)
            {
                /*
                  printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ",
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                    */
                int pos = posNext - 1;

                // float tmpDis = nodes[pos].distance_q2 / 4.0f;
                float tmpDis = nodes[pos].dist_mm_q2 / 4.0f;
                // float tmpDis = nodes[pos].dist_mm_q2;
                // float tmpAng = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
                float tmpAng = nodes[pos].angle_z_q14 * 90.f / 16384.0f;
                if (tmpDis > 0.0f && tmpDis <= _scanRadius)
                {
                    if (_onLidarCapture && _onLidarCapture != NULL)
                        try
                        {
                            _onLidarCapture(tmpAng, tmpDis);
                        }
                        catch (...)
                        {
                        }

                    if (tmpAng >= 0 && tmpAng <= 90)
                    {
                        x = cos((90 - tmpAng) * pi / 180) * tmpDis;
                        y = cos(tmpAng * pi / 180) * tmpDis;
                        x = newX + std::abs(x);
                        y = newY - std::abs(y);
                    }
                    else if (tmpAng > 90 && tmpAng <= 180)
                    {
                        x = cos((tmpAng - 90) * pi / 180) * tmpDis;
                        y = cos((180 - tmpAng) * pi / 180) * tmpDis;
                        x = newX + std::abs(x);
                        y = newY + std::abs(y);
                    }
                    else if (tmpAng > 180 && tmpAng <= 270)
                    {
                        x = cos((270 - tmpAng) * pi / 180) * tmpDis;
                        y = cos((tmpAng - 180) * pi / 180) * tmpDis;
                        x = newX - std::abs(x);
                        y = newY + std::abs(y);
                    }
                    else
                    {
                        x = cos((tmpAng - 270) * pi / 180) * tmpDis;
                        y = cos((360 - tmpAng) * pi / 180) * tmpDis;
                        x = newX - std::abs(x);
                        y = newY - std::abs(y);
                    }

                    if (x == 0 && y == 0)
                        continue;

                    if (x < 0 || y < 0)
                        continue;

                    if (_onLidarPositionCalculated && _onLidarPositionCalculated != NULL)
                        try
                        {
                            _onLidarPositionCalculated(RplidarPoint(x, y, newX, newY, _mapRatio, _scanDiameterMilimet, tmpAng, tmpDis));
                        }
                        catch (...)
                        {
                        }

                    frame[pos] = RplidarPoint(x, y, newX, newY, _mapRatio, _scanDiameterMilimet, tmpAng, tmpDis);
                }
            }

            if (_onFrameCaptured && _onFrameCaptured != NULL)
                try
                {
                    _onFrameCaptured(frame,_maxPoints);
                }
                catch (...)
                {
                }
        }
        else
        {
            printf("\r\nerror code: %x\n", ans);
        }

        return ans;
    }
    sl_result connect_rplidar(sl::ILidarDriver *drv, const char *opt_com_path, sl_u32 opt_com_baudrate)
    {
        sl_result op_result;

        op_result = drv->connect(_channel);
        if (SL_IS_OK(op_result))
        {
            sl_lidar_response_device_info_t deviceInfo;
            op_result = drv->getDeviceInfo(deviceInfo);
            if (SL_IS_OK(op_result))
            {
                printf("\r\nModel: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                       deviceInfo.model,
                       deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
                       deviceInfo.hardware_version);
            }
            else
            {
                fprintf(stderr, "\r\nFailed to get device information from LIDAR %08x\r\n", op_result);               
            }
        }
        else
        {
            fprintf(stderr, "\r\nFailed to connect to LIDAR %08x\r\n", op_result);
            return 0;
        }

        return op_result;
    }

    sl_result disconnect_rplidar(sl::ILidarDriver *drv)
    {
        auto result = drv->reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        result = drv->stop();
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        if (_opt_channel_type == sl::ChannelType::CHANNEL_TYPE_SERIALPORT)
            result = drv->setMotorSpeed(0);

        if (drv)
        {
            delete drv;
            drv = NULL;
        }
        std::cout << "\r\nRplidar Disconnected";
        return result;
    }

    /**
     * @brief loop infinity, should call use thread
     *
     */
    void start()
    {
        sl_lidar_response_device_health_t healthinfo;

        auto op_result = connect_rplidar(_drv, _opt_com_path, _opt_com_baudrate);

        std::cout << "\n\rStarted";

        if (_opt_channel_type == sl::ChannelType::CHANNEL_TYPE_SERIALPORT)
            _drv->setMotorSpeed();

        while (_stop == 0)
        {
            try
            {
                // check the device health
                ////////////////////////////////////////
                op_result = _drv->getHealth(healthinfo);
                if (SL_IS_OK(op_result))
                {
                    // the macro IS_OK is the preperred way to judge whether the operation is succeed.

                    switch (healthinfo.status)
                    {
                    case SL_LIDAR_STATUS_OK:

                        break;
                    case SL_LIDAR_STATUS_WARNING:

                        break;
                    case SL_LIDAR_STATUS_ERROR:
                        std::cout << "\n errorcode: " << healthinfo.error_code;
                        throw "\n errorcode: " + healthinfo.error_code;
                        break;
                    }
                }
                else
                {
                    std::cout << "\n Error : calling  drv->reset(); cannot retrieve the lidar health code: " << op_result;
                    throw "\n Error : calling  drv->reset(); cannot retrieve the lidar health code: " + op_result;
                }

                if (healthinfo.status == SL_LIDAR_STATUS_ERROR)
                {
                    std::cout << "\n Error, rplidar internal error detected. Please reboot the device to retry.\n";
                    throw "\n Error, rplidar internal error detected. Please reboot the device to retry.\n";
                    // enable the following code if you want rplidar to be reboot by software
                    // drv->reset();
                }

                // take only one 360 deg scan and display the result as a histogram
                ////////////////////////////////////////////////////////////////////////////////
                if (SL_IS_FAIL(_drv->startScan(0, 1))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
                {
                    std::cout << "\nError, cannot start the scan operation.\n";
                    throw "\nError, cannot start the scan operation.\n";
                }

                if (SL_IS_FAIL(capture_and_display(_drv)))
                {
                    std::cout << "\nError, cannot grab scan data.\n";
                    throw "\nError, cannot grab scan data.\n";
                }

                std::this_thread::sleep_for(std::chrono::microseconds(1)); // this oki
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

                disconnect_rplidar(_drv);

                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                _drv = (*sl::createLidarDriver());

                connect_rplidar(_drv, _opt_com_path, _opt_com_baudrate);
            }
        }
    }

    void stop()
    {
        _stop = 1;
    }
};