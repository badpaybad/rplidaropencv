#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

#include <chrono>
#include <ctime>
#include <thread>
#include <mutex>
#include <functional>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include "common.h"

// #include <opencv2/core.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>

#ifndef sys_HlsLfcd2LidarConnector
#define sys_HlsLfcd2LidarConnector

class Lds01LidarConnector
{

public:
    uint16_t rpms; ///< @brief RPMS derived from the rpm bytes in an LFCD packet

    std::string port_;                ///< @brief The serial port the driver is attached to
    uint32_t baud_rate_;              ///< @brief The baud rate for the serial connection
    bool shutting_down_;              ///< @brief Flag for whether the driver is supposed to be shutting down or not
    boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the LFCD Laser Scanner
    ///< @brief current motor speed as reported by the LFCD.

    uint16_t motor_speed_;

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

    std::function<void(RplidarPoint[], int)> _onFrameCaptured = NULL;

    int _scanRadius;
    int _scanDiameterMilimet;
    float _mapRatio;
    int _mapWidth;
    int _mapHeight;

    /**
     * @brief Construct a new Hls Lfcd 2 Lidar Connector object
     * https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/
     * 
     * @param io 
     * @param scanRadiusMilimet 
     * @param mapRatio 
     * @param port 
     * @param baud_rate 
     */

    Lds01LidarConnector(boost::asio::io_service &io, int scanRadiusMilimet = 5000, float mapRatio = 0.0f, std::string port = "\\\\.\\COM4", uint32_t baud_rate = 230400)
        : serial_(io, port)
    {
        shutting_down_ = false;

        _scanRadius = scanRadiusMilimet;
        _scanDiameterMilimet = _scanRadius * 2;

        if (mapRatio == 0.0f)
            _mapRatio = (float)_scanDiameterMilimet / 1000.0f;
        else
            _mapRatio = mapRatio;

        _mapWidth = _scanDiameterMilimet;
        _mapHeight = _scanDiameterMilimet;

        // baud_rate = 230400;
        port_ = port;
        baud_rate_ = baud_rate;

        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

        // Below command is not required after firmware upgrade (2017.10)
        boost::asio::write(serial_, boost::asio::buffer("b", 1)); // start motor

        std::cout << "\r\nLidar init " << port_ << " " << std::to_string(baud_rate_);
    }

    ~Lds01LidarConnector()
    {        
        stop();
        boost::asio::write(serial_, boost::asio::buffer("e", 1)); // stop motor
    }

    void init(std::string portCOM_Connect, int opt_com_baudrate = 230400, int scanRadiusMilimet = 5000, float mapRatio = 0)
    {
        std::cout << "Declare in constructor, define this function for compatible to other LidarConnector";
    }

    void registerHandle(std::function<void(RplidarPoint[], int)> onFrameCaptured,
                        std::function<void(RplidarPoint)> onLidarPositionCalculated = NULL,
                        std::function<void(float, float)> onLidarCapture = NULL)
    {
        _onFrameCaptured = onFrameCaptured;
        _onLidarCapture = onLidarCapture;
        _onLidarPositionCalculated = onLidarPositionCalculated;
    }

    void start()
    {

        std::cout << "\r\nLidar started " << port_ << " " << std::to_string(baud_rate_) << std::to_string(shutting_down_);

        float pi = _pi31416;

        int x = 0, y = 0;
        int newX = _mapWidth / 2;
        int newY = _mapHeight / 2;

        // time_t now = time(0);
        // char *dt = ctime(&now);
        bool got_scan = false;
        uint8_t start_count = 0;
        uint8_t good_sets = 0;
        uint32_t motor_speed = 0;
        int index;
        const int maxPoint = 360;
        boost::array<uint8_t, 2520> raw_bytes;

        // while (!shutting_down_ && !got_scan)
        while (!shutting_down_)
        {
            // now = time(0);
            // dt = ctime(&now);
            // cv::Mat image = cv::Mat::zeros(1000, 1000, CV_8UC3);

            // Wait until first data sync of frame: 0xFA, 0xA0
            boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));

            if (start_count == 0)
            {
                if (raw_bytes[start_count] == 0xFA)
                {
                    start_count = 1;
                }
            }
            else if (start_count == 1)
            {
                if (raw_bytes[start_count] == 0xA0)
                {
                    start_count = 0;
                    // Now that entire start sequence has been found, read in the rest of the message
                    got_scan = true;

                    // boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 2518));
                    boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 2518));

                    // scan->angle_min = 0.0;
                    // scan->angle_max = 2.0*M_PI;
                    // scan->angle_increment = (2.0*M_PI/360.0);
                    // scan->range_min = 0.12;
                    // scan->range_max = 3.5;
                    // scan->ranges.resize(360);
                    // scan->intensities.resize(360);

                    RplidarPoint frame[maxPoint];

                    // read data in sets of 6
                    for (uint16_t i = 0; i < raw_bytes.size(); i = i + 42)
                    {
                        if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42)) //&& CRC check
                        {
                            good_sets++;
                            motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2]; // accumulate count for avg. time increment
                            rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10;

                            for (uint16_t j = i + 4; j < i + 40; j = j + 6)
                            {
                                index = 6 * (i / 42) + (j - 4 - i) / 6;

                                // Four bytes per reading
                                uint8_t byte0 = raw_bytes[j];
                                uint8_t byte1 = raw_bytes[j + 1];
                                uint8_t byte2 = raw_bytes[j + 2];
                                uint8_t byte3 = raw_bytes[j + 3];

                                // Remaining bits are the range in mm
                                uint16_t intensity = (byte1 << 8) + byte0;

                                // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                                // uint16_t intensity = (byte3 << 8) + byte2;
                                uint16_t range = (byte3 << 8) + byte2;

                                // scan->ranges[359-index] = range / 1000.0;
                                // scan->intensities[359-index] = intensity;

                                float tmpDis = range;
                                int tmpAng = index;

                                std::cout << "\r\na: " << std::to_string(tmpAng) << " i: " << std::to_string(intensity) << " r: " << std::to_string(range);

                                // frame[pos] = RplidarPoint(0, 0, 0, 0, 0, 0, range, intensity);
                                //  frame[pos] = RplidarPoint(x, y, newX, newY, _mapRatio, _scanDiameterMilimet, tmpAng, tmpDis);
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

                                frame[index] = RplidarPoint(x, y, newX, newY, _mapRatio, _scanDiameterMilimet, (float)tmpAng, tmpDis);

                                // cv::line(image, cv::Point(x, y), cv::Point(x + 2, y + 2), cv::Scalar(255, 0, 255), 3, 8);

                                // cv::line(image, cv::Point(newX, newY), cv::Point(newX + 2, newY + 2), cv::Scalar(0, 0, 255), 3, 8);

                                // cv::imshow("img", image);
                                // cv::waitKey(1);
                            }
                        }

                        // scan->time_increment = motor_speed/good_sets/1e8;

                        // std::cout << "\r\n"<< dt << "\r\n";
                    }
                    if (_onFrameCaptured && _onFrameCaptured != NULL)
                        try
                        {
                            _onFrameCaptured(frame, maxPoint);
                        }
                        catch (...)
                        {
                        }
                }
                else
                {
                    start_count = 0;
                }
            }
        }
    }
    void stop()
    {
        shutting_down_ = true;
    }
};

#endif