#include <stdio.h>
#include <stdlib.h>

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

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "libs/common.h"
#include "libs/Lds01LidarConnector.cpp"
#include "libs/MjpegHttpServer.cpp"
//#include "libs/RplidarConnector.cpp"

int main(int argc, const char *argv[])
{
    const char *opt_com_path = NULL;
    int opt_com_baudrate = 115200;
    // opt_com_baudrate = 230400;

    // std::queue<cv::Mat> *_lidarFrames = new std::queue<cv::Mat>();

    if (argc < 2)
    {
        //   print_usage(argc, argv);

        //opt_com_path = "\\\\.\\COM4"; // argv[1];//window
         opt_com_path = "\\\\.\\COM4"; // argv[1];
        //  opt_com_path = "/dev/ttyUSB0"; // argv[1];linux
    }

    if (argc > 2)
        opt_com_baudrate = strtoul(argv[2], NULL, 10);

    // auto *lidarConnect = new SlLidarConnector();
    // auto *lidarConnect = new RplidarConnector();

    boost::asio::io_service io;
    auto *lidarConnect = new Lds01LidarConnector(io, 5000, 0.0f,opt_com_path);

    //lidarConnect->init(opt_com_path, opt_com_baudrate, 10000);

    int mapW = lidarConnect->_mapWidth;
    int mapH = lidarConnect->_mapHeight;
    float ratio = lidarConnect->_mapRatio;
    int maxPoints = _maxPoints;

    int newX = mapW / 2;
    int newY = mapH / 2;

    newX = newX / ratio;
    newY = newY / ratio;

    mapW = mapW / ratio;
    mapH = mapH / ratio;

    MjpegHttpServer *_mjpegServer = new MjpegHttpServer(9001);

    lidarConnect->registerHandle(
        [_mjpegServer, mapW, mapH, newX, newY, ratio, maxPoints](RplidarPoint frame[], int maxPoint)
        {
            cv::Mat image = cv::Mat::zeros(mapW, mapH, CV_8UC3);

            // the lidar position
            cv::putText(image,
                        "map: w,h=" + std::to_string(mapW) + "," + std::to_string(mapH) +
                            ", lidar: x,y=" + std::to_string(newX) + "," + std::to_string(newY) + ", ratio: " + std::to_string(ratio),
                        cv::Point(25, 25), cv::FONT_HERSHEY_PLAIN, 0.9, CV_RGB(255, 255, 0), 2);

            // cv::line(image, Point(0, 0), Point(2000, 2000), Scalar(0, 255, 0), 5, 8);
            //  Draw a line

            RplidarPoint near_by;
            int counter_nearby = 0;
            int nearbyx = 0;
            int nearbyy = 0;
            int left = 0, top = 0, right = 0, bottom = 0;

            float disTb = 0;

            for (int pos = 0; pos < (int)maxPoint; ++pos)
            {
                int a = frame[pos]._angle;
                int d = frame[pos]._distance;
                int x = frame[pos]._x;
                int y = frame[pos]._y;

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

                    near_by = frame[pos];

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

                        near_by = frame[pos];

                        counter_nearby++;

                        cv::line(image, cv::Point(newX + 2, newY + 2), cv::Point(x, y), cv::Scalar(255, 0, 255), 1, 8);

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

                // std::cout << "\r\nx: " << x << " y: " << y << " dis: " << d << " a: " << a;

                // object scaned by lidar position
                cv::line(image, cv::Point(x, y), cv::Point(x + 2, y + 2), cv::Scalar(0, 0, 255), 2, 8);
            }

            disTb = (disTb / counter_nearby) / ratio;
            if (disTb > 0)
                cv::circle(image, cv::Point(newX, newY), (int)disTb, cv::Scalar(255, 255, 255), 1);

            cv::line(image, cv::Point(newX, newY), cv::Point(newX + 2, newY + 2), cv::Scalar(0, 255, 255), 2, 8);
            cv::circle(image, cv::Point(newX, newY), newX, cv::Scalar(255, 255, 255), 1);

            // std::cout << "\r\n newY: " << newY << " newY: " << newY << " disTb: " << disTb << " total: " << data_lidar_len;

            try
            {
                cv::Mat lastImageForStream = cv::Mat::zeros(image.cols, image.rows, CV_8UC3);
                if (__lockGlobal.try_lock())
                {
                    image.copyTo(lastImageForStream);

                    __lockGlobal.unlock();
                }
                _mjpegServer->Push(lastImageForStream);

                cv::imshow("imgShow", lastImageForStream);
                cv::waitKey(1);
            }
            catch (...)
            {
            }
        },
        NULL, NULL);
    // no block will join to end of main
    std::thread _threadLidar = std::thread([lidarConnect]()
                                           { lidarConnect->start(); });

    _mjpegServer->Start();

    // other logic here

    while (1)
    {

        std::this_thread::sleep_for(std::chrono::microseconds(1)); // this oki
    }

    _threadLidar.join();

    return (0);
}
