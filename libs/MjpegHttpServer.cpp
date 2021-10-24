//#include "http_server.h"
#include "uwebsockets/App.h"
#include <queue>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <ctime>
#include <thread>
#include <mutex>
#include <functional>
#include <algorithm>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "common.h"

#ifndef HttpServer_Class
#define HttpServer_Class

#ifdef __cplusplus
extern "C"
{
#endif
    class MjpegHttpServer
    {
    public:
        std::queue<std::vector<uchar>> *_qBuf = new std::queue<std::vector<uchar>>();

        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        std::thread _threadMjpegServer;

        /* Note that SSL is disabled unless you build with WITH_OPENSSL=1 */
        const int SSL = 1;

        char *_contenType = "Content-Type";
        char *_contenTypeTextHtml = "text/html; charset=utf-8";
        char *_contenTypeStreamFrame = "multipart/x-mixed-replace; boundary=frame";
        char *_imgVideoStream = "<img src='/video' style='width:100%' />";

        char *_contenTypeFrameBegin = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";

        int _port;
        void about(uWS::HttpResponse<false> *res, uWS::HttpRequest *req)
        {
            res->writeHeader(_contenType, _contenTypeTextHtml)
                ->end("Nguyen Phan Du");
        }
        void viewer(uWS::HttpResponse<false> *res, uWS::HttpRequest *req)
        {
            res->writeHeader(_contenType, _contenTypeTextHtml)
                ->end(_imgVideoStream);
        }
        void video(uWS::HttpResponse<false> *res, uWS::HttpRequest *req)
        {
            res->writeHeader(_contenType, _contenTypeStreamFrame);

            int isAborted = 0;

            res->onAborted([&]()
                           {
                               std::cout << "onAborted";
                               isAborted = 1;
                           });

            res->onWritable([](uintmax_t t)
                            {
                                std::cout << "onWritable";
                                return true;
                            });
            std::vector<uchar> buff_bgr;
            //
            int qsize = 0;
            int sleepInMiliseconds = 41;
            std::string imgc;
            while (isAborted == 0)
            {
                if (__lockGlobal.try_lock())
                {
                    qsize = _qBuf->size();
                    if (qsize > 0)
                    {
                        for (int i = 0; i < qsize; i++)
                        {
                            buff_bgr.swap(_qBuf->front()); 
                            //buff_bgr = _qBuf->front();
                            _qBuf->pop();
                        }
                    }
                    __lockGlobal.unlock();
                }

                imgc = std::string(_contenTypeFrameBegin) +
                       std::to_string(buff_bgr.size()) + "\r\n\r\n" +
                       std::string(buff_bgr.begin(), buff_bgr.end()) + "\r\n";

                if(! res->write(imgc)){
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(sleepInMiliseconds));
            }
        }

        std::vector<std::thread *> _threads;

        MjpegHttpServer(int port)
        {
            _port = port;
            int threadToRun=(int) (std::thread::hardware_concurrency()/2);
            if(threadToRun<=0) threadToRun=1;
            
            std::vector<std::thread *> threads(threadToRun);
            _threads = threads;

            std::cout << "\r\nhttp://0.0.0.0:" << _port << "/viewer";
            std::cout << "\r\nhttp://127.0.0.1:" << _port << "/viewer";
        }
        ~MjpegHttpServer()
        {
            std::for_each(_threads.begin(), _threads.end(), [](std::thread *t)
                          { t->join(); });
        }

        void Start()
        {
            std::transform(_threads.begin(), _threads.end(), _threads.begin(),
                           [this](std::thread * /*t*/)
                           {
                               return new std::thread([this]()
                                                      {
                                                          uWS::App *_app;
                                                          _app = new uWS::App();
                                                          _app->get("/", [this](auto *res, auto *req)
                                                                    { this->about(res, req); });

                                                          _app->get("/viewer", [this](auto *res, auto *req)
                                                                    { this->viewer(res, req); });

                                                          _app->get("/video", [this](auto *res, auto *req)
                                                                    { this->video(res, req); });

                                                          int port = _port;
                                                          _app->listen(port,
                                                                       [port, this](auto *listen_socket)
                                                                       {
                                                                           __lockGlobal.lock();
                                                                           if (listen_socket)
                                                                           {
                                                                               /* Note that us_listen_socket_t is castable to us_socket_t */
                                                                               std::cout << "\r\nThread " << std::this_thread::get_id() << " listening on port " << us_socket_local_port(SSL, (struct us_socket_t *)listen_socket) << std::endl;
                                                                           }
                                                                           else
                                                                           {
                                                                               std::cout << "\r\nThread " << std::this_thread::get_id() << " failed to listen on port 3000" << std::endl;
                                                                           }
                                                                           __lockGlobal.unlock();
                                                                       });

                                                          _app->run();
                                                      });
                           });
        }

        void Push(std::vector<uchar> buff_bgr)
        {
            try
            {
                if (__lockGlobal.try_lock())
                {
                    _qBuf->push(buff_bgr);
                    __lockGlobal.unlock();
                }
            }
            catch (...)
            {
            }
        }
        void Push(cv::Mat image)
        {
            //cv::imshow("adf",image);
            //cv::waitKey(1);
            try
            {
                std::vector<uchar> buff_bgr;
                cv::imencode(".jpg", image, buff_bgr, params);

                if (__lockGlobal.try_lock())
                {
                    _qBuf->push(buff_bgr);
                    __lockGlobal.unlock();
                }
            }
            catch (...)
            {
            }
        }

        std::vector<uchar> Mat2Vector(cv::Mat image, std::string imgExt = ".jpg")
        {
            std::vector<uchar> buff_bgr;
            cv::imencode(imgExt, image, buff_bgr, params);

            return buff_bgr;
        }
    };
#ifdef __cplusplus
}
#endif

#endif //HttpServer_Class