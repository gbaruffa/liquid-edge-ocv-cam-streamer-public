/**
 * @file le_ocv_streamer_rx.cpp
 * @author G. Baruffa (giuseppe.baruffa@unipg.it)
 * 
 * @brief OpenCV-based MJPEG UDP stream receiver
 *
 * @version 0.4
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2020-2021 gbaruffa - University of Perugia
 * 
 */
#include <iostream>
#include <fstream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <thread>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sockpp/tcp_acceptor.h>
#include <sockpp/udp_socket.h>
#include <sockpp/version.h>
#ifndef WIN32
#include <fcntl.h>
#endif
#include "config.h"
#include "le_ocv_streamer_common.hpp"

#define SPEED_DBG 0

// compressed frame information
struct cframe
{
  uint64_t timestamp;
  int framenum;
  int numparts;
  int numpartsin;
  std::vector<uchar> inbuf;
};

// this flag rules the whole process
bool process = true;

// Each UDP socket type knows its address type as `addr_t`
typename sockpp::udp_socket::addr_t srcAddr;
sockpp::tcp_socket tcpsock;
static int64 lastinticks;

// the queue where we receive the frames
BoundedBlockingQueue<cframe> decompress_queue({0, -1, 0, 0});
std::string framework;
std::string configfile;
std::string weightsfile;
std::string edge_key;
static double decompress_ms, inference_ms;
static SMA<20> decompress_ms_filter, inference_ms_filter;
bool freezescreen = false;
bool usetcp = false;

/**
 * @brief TCP receiving thread
 * 
 * @param acc Accepting connection
 */
void tcp_receiver(sockpp::tcp_acceptor *acc)
{
  cframe cf = {0, -1, 0, 0};
  while (usetcp && process)
  {
    // accept a new client connection
    sockpp::inet_address peer;
    tcpsock = acc->accept(&peer);
    if (!tcpsock)
    {
      if (acc->last_error() ==
#ifdef WIN32
          WSAEWOULDBLOCK
#else
          EWOULDBLOCK
#endif
      )
        std::this_thread::sleep_for(50ms); // timeout, sleep a little
      else
        std::cerr << "Error accepting incoming connection: " << tcpsock.last_error_str() << std::endl;
    }
    else
    {
      std::cout << "Received a connection request from " << peer << std::endl;

      // the socket must block
#ifdef WIN32
      u_long iMode = 0;
      ioctlsocket(tcpsock.handle(), FIONBIO, &iMode);
#else
      int flags = fcntl(tcpsock.handle(), F_GETFL);
      fcntl(tcpsock.handle(), F_SETFL, flags & ~O_NONBLOCK);
#endif

      // set timeouts on the socket
      if (!tcpsock.write_timeout(std::chrono::milliseconds(5000)))
        std::cerr << "Error setting timeout on TCP stream: " << tcpsock.last_error_str() << std::endl;
      if (!tcpsock.read_timeout(std::chrono::milliseconds(5000)))
        std::cerr << "Error setting timeout on TCP stream: " << tcpsock.last_error_str() << std::endl;

      while (tcpsock && process)
      {
        // first, read the frame information
        frameinfo fi;
        if (tcpsock.read_n(&fi, sizeof(fi)) != sizeof(fi))
        {
          std::cerr << "Error reading from the TCP stream: " << tcpsock.last_error_str() << std::endl;
          break;
        }

        // new frame
        cf.timestamp = fi.timestamp;
        cf.framenum = fi.framenum;
        cf.numparts = 1;
        cf.numpartsin = 0;
        cf.inbuf.resize(fi.framelen, 0);

        // then, read the frame itself
        if (tcpsock.read_n(cf.inbuf.data(), fi.framelen) != fi.framelen)
        {
          std::cerr << "Error reading from the TCP stream: " << tcpsock.last_error_str() << std::endl;
          break;
        }
        cf.numpartsin = 1;

        // push frame if complete
        if (cf.framenum >= 0 && cf.numpartsin == cf.numparts)
        {
#if SPEED_DBG
          std::cout << "rec0:" << fi.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
          decompress_queue.push(cf);
#if SPEED_DBG
          std::cout << "rec1:" << fi.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
          lastinticks = cv::getTickCount();
        }
      }
      std::cout << "Connection closed from " << tcpsock.peer_address() << std::endl;
    }
  }
}

/**
 * @brief UDP receiving thread
 * 
 * @param sock Connected socket
 */
void udp_receiver(sockpp::udp_socket *sock)
{
  ssize_t plen;
  char packet[2048];
  cframe cf = {0, -1, 0, 0};

  // Read some data, also getting the address of the sender
  while (!usetcp && process)
  {
    // check if timeout occurred
    if ((plen = sock->recv_from(packet, sizeof(packet), &srcAddr)) == -1)
      continue;

    // read info
    frameinfo fi;
    std::copy(packet, packet + sizeof(frameinfo), reinterpret_cast<char *>(&fi));

    // key control
    if (!std::equal(reinterpret_cast<char *>(&fi.key), reinterpret_cast<char *>(&fi.key) + sizeof(fi.key), edge_key.c_str()))
      continue;

    // assemble compressed frame
    if (fi.framenum != cf.framenum)
    {
      // push old frame if complete
      if (cf.framenum >= 0 && cf.numpartsin == cf.numparts)
      {
#if SPEED_DBG
        std::cout << "rec0:" << fi.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
        decompress_queue.push(cf);
#if SPEED_DBG
        std::cout << "rec1:" << fi.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
        lastinticks = cv::getTickCount();
      }

      // new frame
      cf.timestamp = fi.timestamp;
      cf.framenum = fi.framenum;
      cf.numparts = fi.numparts;
      cf.numpartsin = 0;
      cf.inbuf.resize(fi.numparts * (plen - sizeof(frameinfo)), 0);
    }

    // assemble frame
    if (cf.framenum >= 0 && fi.partnum < cf.numparts)
    {
      ssize_t psize = std::min<ssize_t>(plen - sizeof(frameinfo), plen);
      std::copy(packet + sizeof(frameinfo), packet + sizeof(frameinfo) + psize, cf.inbuf.data() + fi.partnum * (plen - sizeof(frameinfo)));
      cf.numpartsin++;
    }
  }
  //std::cerr << "Process out: udp_receiver\n";
}

// decompressing thread
int64 errorframes = 0, lasterrorframes = 0;
unsigned short lastframenum = -1;
bool firstcycle = true;
int64 doneframes = 0, lastframes = 0;
int accumbytes = 0;
BoundedBlockingQueue<std::pair<cv::Mat, uint64_t>> inference_queue(std::make_pair(cv::Mat(), 0), 10);

/**
 * @brief Decompressing thread
 */
void decompressor()
{
  while (process)
  {
    cframe cf = decompress_queue.pop(5);
    if (cf.inbuf.size())
    {
#if SPEED_DBG
      std::cout << "dec0:" << cf.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      // check if we lost one (or more) frame(s)
      if (!firstcycle)
        errorframes += (unsigned short)cf.framenum - (lastframenum + 1);
      lastframenum = (unsigned short)cf.framenum;

      // decode the image
      auto tickcount = cv::getTickCount();
      cv::Mat df = cv::imdecode(cf.inbuf, cv::IMREAD_COLOR);
#if SPEED_DBG
      std::cout << "dec1:" << cf.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      decompress_ms = decompress_ms_filter(1000.0 * (cv::getTickCount() - tickcount)) / cv::getTickFrequency();
      if (df.data)
      {
        inference_queue.push({df.clone(), cf.timestamp});
#if SPEED_DBG
        std::cout << "dec2:" << cf.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      }
      else
      {
        std::cerr << "Error decoding frame\n";
        errorframes++;
      }

      accumbytes += (int)cf.inbuf.size();
      firstcycle = false;
    }
  }
  //std::cerr << "Process out: decompressor\n";
}

bool performi;
cv::dnn::Net net;
bool showlocal;
bool peekweb;

// process the frame with the network
BoundedBlockingQueue<infresult> result_queue({0}, 10);

/**
 * @brief Process the decompressed image
 * 
 * @param frm Decompressed image
 * @param net Deep neural network
 * @param ts Image timestamp
 */
void infer_frame(cv::Mat &frm, cv::dnn::Net &net, uint64_t ts)
{
  // confidence threshold for object detection
  constexpr float confidenceThreshold = 0.15F;
  infresult ir = {.timestamp = ts};

  if (performi && framework == "caffe")
  {
    // convert to blob
    cv::Mat inputBlob = cv::dnn::blobFromImage(frm, 0.007843, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), false);

    // give the blob to the DNN
    net.setInput(inputBlob, "data");

    // do the inference
    cv::Mat detection = net.forward("detection_out");

    // extract the bounding boxes
    cv::Mat detectionMat = cv::Mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    // iterate all objects
    std::ostringstream serstream;
    serstream << ts;
    for (int i = 0; i < detectionMat.rows; i++)
    {
      // current object confidence
      float confidence = detectionMat.at<float>(i, 2);

      // above threshold?
      if (confidence > confidenceThreshold)
      {
        // detected class index
        infobj io = {.conf = confidence};
        int idx = io.idx = static_cast<int>(detectionMat.at<float>(i, 1));
        serstream << "\n" << idx << " " << confidence << " ";

        // coordinates of the bounding box
        int xLeftBottom = io.xlb = static_cast<int>(detectionMat.at<float>(i, 3) * frm.cols);
        serstream << xLeftBottom << " ";
        int yLeftBottom = io.ylb = static_cast<int>(detectionMat.at<float>(i, 4) * frm.rows);
        serstream << yLeftBottom << " ";
        int xRightTop = io.xrt = static_cast<int>(detectionMat.at<float>(i, 5) * frm.cols);
        serstream << xRightTop << " ";
        int yRightTop = io.yrt = static_cast<int>(detectionMat.at<float>(i, 6) * frm.rows);
        serstream << yRightTop;

        ir.objs.push_back(io);

        if (showlocal || peekweb)
        {
          // the bounding box
          cv::Rect object((int)xLeftBottom, (int)yLeftBottom, (int)(xRightTop - xLeftBottom), (int)(yRightTop - yLeftBottom));

          // overlay the box on the image in green
          cv::rectangle(frm, object, cv::Scalar(0, 255, 0), 1);

          // oject label
          std::ostringstream labeloss;
          labeloss << idx << ":" << std::setprecision(3) << confidence;

          // overlay text
          int baseLine = 0;
          cv::Size labelSize = cv::getTextSize(labeloss.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
          cv::putText(frm, labeloss.str(), cv::Point(xLeftBottom, yLeftBottom - baseLine), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0));
        }
      }
    }
  }
  else if (performi && framework == "darknet")
  {
    cv::Mat blobFromImg = cv::dnn::blobFromImage(frm, 1.0 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), false);
    //bool swapRB = false;
    //cv::dnn::blobFromImage(frm, blobFromImg, 1, cv::Size(416, 416), cv::Scalar(), swapRB, false);
    //std::cout << blobFromImg.size() << std::endl;

    //float scale = 1.0 / 255.0;
    //cv::Scalar mean = 0;
    //net.setInput(blobFromImg, "", scale, mean);
    net.setInput(blobFromImg, "");
    cv::Mat outMat;
    net.forward(outMat);

    // rows represent number of detected object (proposed region)
    // The columns looks like this, The first is region center x, center y, width
    // height, The class 1 - N is the column entries, which gives you a number,
    // where the biggist one corrsponding to most probable class.
    // [x ; y ; w; h; class 1 ; class 2 ; class 3 ;  ; ;....]

    // Loop over number of detected object.
    for (int j = 0; j < outMat.rows; ++j)
    {
      // for each row, the score is from element 5 up
      // to number of classes index (5 - N columns)
      cv::Mat scores = outMat.row(j).colRange(5, outMat.cols);

      // This function find indexes of min and max confidence and related index of element.
      // The actual index is match to the concrete class of the object.
      // First parameter is Mat which is row [5fth - END] scores,
      // Second parameter will gives you min value of the scores. NOT needed
      // confidence gives you a max value of the scores. This is needed,
      // Third parameter is index of minimal element in scores
      // the last is position of the maximum value.. This is the class!!
      cv::Point pMax;
      double confidence;
      cv::minMaxLoc(scores, 0, &confidence, 0, &pMax);

      if (confidence > confidenceThreshold)
      {
        // detected class index
        infobj io = {.conf = (float)confidence};
        io.idx = static_cast<int>(pMax.x);

        // these four lines are x ; y ; w; h;
        int centerX = (int)(outMat.at<float>(j, 0) * frm.cols);
        int centerY = (int)(outMat.at<float>(j, 1) * frm.rows);
        int width = (int)(outMat.at<float>(j, 2) * frm.cols);
        int height = (int)(outMat.at<float>(j, 3) * frm.rows);
        int left = io.xlb = centerX - width / 2;
        int top = io.ylb = centerY - height / 2;
        io.xrt = io.xlb + width;
        io.yrt = io.ylb + height;
        ir.objs.push_back(io);

        if (showlocal || peekweb)
        {
          // the bounding box
          cv::Rect object(left, top, width, height);

          // overlay the box on the image in blue
          cv::rectangle(frm, object, cv::Scalar(255, 0, 0), 1);

          // object label
          std::ostringstream labeloss;
          labeloss << pMax.x << ":" << std::setprecision(3) << confidence;

          // overlay text
          int baseLine = 0;
          cv::Size labelSize = cv::getTextSize(labeloss.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
          cv::putText(frm, labeloss.str(), cv::Point(left, top - baseLine), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
        }
      }
    }
  }

#if SPEED_DBG
  std::cout << "inf1:" << ts << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
  result_queue.push(ir);
}

// processor thread
BoundedBlockingQueue<std::pair<cv::Mat, uint64_t>> show_queue(std::make_pair(cv::Mat(), 0), 10);
BoundedBlockingQueue<std::pair<cv::Mat, uint64_t>> web_queue(std::make_pair(cv::Mat(), 0), 2);

/**
 * @brief Processing thread
 */
void inferrer()
{
  while (process)
  {
    std::pair<cv::Mat, uint64_t> df = inference_queue.pop(5);
    if (!df.first.empty())
    {
#if SPEED_DBG
      std::cout << "inf0:" << df.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      // process the image
      auto tickcount = cv::getTickCount();
      infer_frame(df.first, net, df.second);
#if SPEED_DBG
      std::cout << "inf2:" << df.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      inference_ms = inference_ms_filter(1000.0 * (cv::getTickCount() - tickcount)) / cv::getTickFrequency();

      // save for local viewing
      if (showlocal)
      {
        show_queue.push({df.first.clone(), df.second});
#if SPEED_DBG
        std::cout << "inf3:" << df.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      }

      // save for web peeking
      if (peekweb)
        web_queue.push({df.first.clone(), df.second});

      // count things and times
      doneframes++;
    }
  }
  //std::cerr << "Process out: processor\n";
}

/**
 * @brief Web serving thread
 */
void webserver(int port)
{
  //std::cerr << "Process in: webserver\n";
  auto httpacc = sockpp::tcp_acceptor(port);
  if (!httpacc)
  {
    std::cerr << "Error creating HTTP acceptor: " << httpacc.last_error_str() << std::endl;
    return;
  }

  // prepare socket for nonblocking mode
#ifdef WIN32
  u_long iMode = 1;
  ioctlsocket(httpacc.handle(), FIONBIO, &iMode);
#else
  int flags = fcntl(httpacc.handle(), F_GETFL);
  fcntl(httpacc.handle(), F_SETFL, flags | O_NONBLOCK); // Change the socket into non-blocking state
#endif
  //std::cout << "HTTP acceptor awaiting on " << httpacc.address() << std::endl;

  while (process)
  {
    // accept a new client connection
    sockpp::inet_address httppeer;
    auto httpsock = httpacc.accept(&httppeer);
    if (!httpsock)
    {
      if (httpacc.last_error() ==
#ifdef WIN32
          WSAEWOULDBLOCK
#else
          EWOULDBLOCK
#endif
      )
        std::this_thread::sleep_for(50ms); // timeout, sleep a little
      else
        std::cerr << "Error accepting HTTP incoming connection: " << httpsock.last_error_str() << std::endl;
    }
    else
    {
      //std::cout << "Received an HTTP connection request from " << httppeer << std::endl;

      // the socket must block
#ifdef WIN32
      u_long iMode = 0;
      ioctlsocket(httpsock.handle(), FIONBIO, &iMode);
#else
      int flags = fcntl(httpsock.handle(), F_GETFL);
      fcntl(httpsock.handle(), F_SETFL, flags & ~O_NONBLOCK);
#endif

      // set timeouts on the socket
      if (!httpsock.write_timeout(std::chrono::milliseconds(25000)))
        std::cerr << "Error setting timeout on HTTP stream: " << httpsock.last_error_str() << std::endl;
      if (!httpsock.read_timeout(std::chrono::milliseconds(25000)))
        std::cerr << "Error setting timeout on HTTP stream: " << httpsock.last_error_str() << std::endl;

      while (httpsock && process)
      {
        int maxlen = 2047;
        std::string request(maxlen + 1, 0);
        int len = httpsock.read(&request[0], maxlen);
        if (len == -1)
        {
          std::cerr << "Error reading from the HTTP stream: " << httpsock.last_error_str() << std::endl;
          break;
        }
        else if (len == 0)
          break;

        //std::cout << "$$$$$$$$$$$$$\n" << request << "\n$$$$$$$$$$$$$" << std::endl;

        auto getstartpos = request.find("GET ");
        if (getstartpos != std::string::npos)
        {
          auto getendpos = getstartpos + 3;
          auto pathquerystartpos = request.find_first_not_of(" ", getendpos);
          if (pathquerystartpos != std::string::npos)
          {
            auto pathqueryendpos = request.find_first_of(" ", pathquerystartpos);
            if (pathqueryendpos != std::string::npos)
            {
              auto pathquery = request.substr(pathquerystartpos, pathqueryendpos - pathquerystartpos);
              //std::cout << "PATH: \"" << path << "\"\n";

              auto pathstartpos = 0;
              auto pathendpos = pathquery.find("?");
              std::string path = pathquery.substr(pathstartpos, pathendpos - pathstartpos);
              std::cout << "PATH: \"" << path << "\"\n";

              std::string query = "";
              auto querystartpos = pathquery.find("?");
              if (querystartpos != std::string::npos)
                query = pathquery.substr(querystartpos + 1);
              std::cout << "QUERY: \"" << query << "\"\n";

              auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

              std::stringstream header;
              if (path == "/jpeg")
              {
                std::pair<cv::Mat, uint64_t> df = web_queue.pop(5);
                std::vector<uchar> cframe;
                std::vector<int> jparams = {cv::IMWRITE_JPEG_QUALITY, 100};
                cv::imencode(".jpg", df.first, cframe, jparams);

                // 200 OK
                header << "HTTP/1.1 200 OK\r\n"
                          //"Date: Fri, 22 Oct 2021 20:22:27 GMT\r\n"
                          "Date: "
                       << std::put_time(std::gmtime(&in_time_t), "%a, %d %h %Y %X GMT") << "\r\n"
                       //"Server: Apache/2.4.29 (Ubuntu)\r\n"
                       << "Server: le_ocv_streamer_rx/" VERSION " (OS)\r\n"
                       << "Last-Modified: "
                       << std::put_time(std::gmtime(&in_time_t), "%a, %d %h %Y %X GMT") << "\r\n"
                       // "Last-Modified: Tue, 18 Nov 2014 15:08:26 GMT\r\n"
                       //"ETag: \"47e-5082377924e5e\"\r\n"
                       //"Accept-Ranges: bytes\r\n"
                       << "Content-Length: "
                       << cframe.size() << "\r\n"
                       //"Keep-Alive: timeout=5, max=99\r\n"
                       //"Connection: Keep-Alive\r\n"
                       << "Connection: close\r\n"
                          "Content-Type: image/jpeg\r\n"
                          "\r\n";
                httpsock.write(header.str());
                httpsock.write(cframe.data(), cframe.size());
              }
              else if (path == "/")
              {
                // 200 OK
                // https://nakkaya.com/2011/03/23/streaming-opencv-video-over-the-network-using-mjpeg/
                auto delay = 200ms;
                if (!query.empty())
                {
                  try
                  {
                    auto delaystartpos = query.find("delay=");
                    if (delaystartpos != std::string::npos)
                    {
                      auto delayendpos = query.find_first_of("&", delaystartpos);
                      auto delaystr = query.substr(delaystartpos + 6, delayendpos - delaystartpos - 6);
                      std::cout << "DELAY: \"" << delaystr << "\"\n";
                      delay = 1ms * std::stoi(delaystr);
                    }
                  }
                  catch (...)
                  {
                    delay = 100ms;
                  }
                }
                delay = std::clamp(delay, 0ms, 2000ms);
                header << "HTTP/1.0 200 OK\r\n"
                          "Server: le_ocv_streamer_rx/" VERSION " (OS)\r\n"
                          "Content-Type: multipart/x-mixed-replace;boundary=informs\r\n"
                          "--informs\r\n";
                httpsock.write(header.str());
                std::vector<uchar> cframe;
                std::vector<int> jparams = {cv::IMWRITE_JPEG_QUALITY, 100};
                while (httpsock && process)
                {
                  std::pair<cv::Mat, uint64_t> df = web_queue.pop(5);
                  if (!df.first.empty())
                  {
                    cv::imencode(".jpg", df.first, cframe, jparams);
                    header.str("");
                    header << "\r\n"
                              "--informs\r\n"
                              "Content-Type: image/jpeg\r\n"
                           << "Content-Length: "
                           << cframe.size() << "\r\n"
                           << "\r\n";
                    httpsock.write(header.str());
                    httpsock.write(cframe.data(), cframe.size());
                    std::this_thread::sleep_for(delay);
                  }
                }
              }
              else
              {
                // 404 Not Found
                header << "HTTP/1.1 404 Not Found\r\n"
                          //"Date: Fri, 22 Oct 2021 20:30:54 GMT\r\n"
                          "Date: "
                       << std::put_time(std::gmtime(&in_time_t), "%a, %d %h %Y %X GMT") << "\r\n"
                       //"Server: Apache/2.4.29 (Ubuntu)\r\n"
                       << "Server: le_ocv_streamer_rx/" VERSION " (OS)\r\n"
                          "Content-Length: 0\r\n"
                          "Connection: close\r\n"
                          "\r\n";
                httpsock.write(header.str());
              }
              //std::cout << header.str();
            }
          }
        }

        // done
        httpsock.close();
      }
      //std::cout << "HTTP connection closed from " << httpsock.peer_address() << std::endl;
    }
  }

  //std::cerr << "Process out: webserver\n";
}

void feedback(sockpp::udp_socket *sock)
{
  unsigned char packet[2048];
  while (process)
  {
    infresult ir = result_queue.pop(5);
    if (ir.timestamp)
    {
#if SPEED_DBG
      std::cout << "fdb0:" << ir.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      // deserialize
      std::ostringstream serstream;
      serstream << ir.timestamp;
      for (auto &obj : ir.objs)
        serstream << "\n"
                  << obj.idx << " " << obj.conf << " " << obj.xlb << " " << obj.ylb << " " << obj.xrt << " " << obj.yrt;
      std::string ss = serstream.str();
      frameinfo fi;
      if (ss.size() + sizeof(fi) > sizeof(packet))
        continue;

      // prepare a feedback packet
      fi.timestamp = ir.timestamp;
      fi.framenum = 0;
      if (usetcp)
        fi.framelen = (unsigned int)ss.size();
      else
      {
        fi.numparts = 1;
        fi.partnum = 0;
      }
      std::copy(edge_key.c_str(), edge_key.c_str() + sizeof(fi.key), reinterpret_cast<char *>(&fi.key));
      fi.ft = FT_INF;
      std::copy(reinterpret_cast<char *>(&fi), reinterpret_cast<char *>(&fi) + sizeof(frameinfo), packet);
      std::copy(ss.c_str(), ss.c_str() + ss.size(), packet + sizeof(frameinfo));

      // send the packet
      if (!usetcp)
      {
        if (sock->send_to(packet, sizeof(frameinfo) + ss.size(), srcAddr) != sizeof(frameinfo) + ss.size())
          std::cerr << "Error writing to the UDP socket: " << sock->last_error_str() << std::endl;
#if SPEED_DBG
        std::cout << "fdb1:" << ir.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
      }
      else
      {
        if (tcpsock && tcpsock.write_n(packet, sizeof(frameinfo) + ss.size()) != sizeof(frameinfo) + ss.size())
        {
          std::cerr << "Error writing to the TCP stream: " << tcpsock.last_error_str() << std::endl;
          continue;
        }
      }
    }
  }
  //std::cerr << "Process out: feedback_thread\n";
}

/**
 * @brief Main function
 * 
 * @param argc Number of command line arguments
 * @param argv Pointer to pointers to arguments
 * @return int Exit code
 */
int main(int argc, char *argv[])
{
  // default values
  const std::string d_upaddress = "localhost";
  const std::string d_upport = default_udp_streaming_port;
  const std::string version = "LIQUID_EDGE UDP video streaming receiver v" + std::string(VERSION) + ", " + __DATE__ + " " + __TIME__ + ", sockpp " + sockpp::SOCKPP_VERSION + ", OpenCV " + CV_VERSION;
  const std::string d_framework = "caffe";
  const std::string d_configfile = "../liquid-edge-model-zoo/Caffe/MobileNetSSD_deploy.prototxt";
  const std::string d_weightsfile = "../liquid-edge-model-zoo/Caffe/MobileNetSSD_deploy.caffemodel";
  /*const std::string d_framework  = "darknet";
  const std::string d_configfile = "../liquid-edge-model-zoo/Darknet/yolov3-tiny.cfg";
  const std::string d_weightsfile = "../liquid-edge-model-zoo/Darknet/yolov3-tiny.weights";*/
  const std::string d_key = default_access_key;
  const std::string d_timeout = "0";
  const std::string d_peekport = "8087";

  const std::string keys =
      "{ help h usage ? | | Prints this help and exits }"
      "{ V version | | Prints version and exits }"
      "{ v plainversion | | Prints only the version number and exits }"
      "{ n novideo | | Do not show video }"
      "{ g gpu | | Use GPU where possible }"
      "{ i infer | | Perform inference using the given model and weights }"
      "{ f framework | " + d_framework + " | Inference framework (caffe, darknet) }"
      "{ c config | " + d_configfile + " | DNN configuration file }"
      "{ w weight | " + d_weightsfile + " | DNN weights file }"
      "{ k key | " + d_key + " | Engine access key }"
      "{ o timeout | " + d_timeout + " | Waiting timeout (0 = never ends) }"
      "{ p peekport | " + d_peekport + " | Web peeking port }"
      "{ tcp | | Use TCP/IP }"
      "{ udp | | Use UDP/IP (default) }"
      "{ @addr:port | " + d_upaddress + ":" + d_upport + " | Listening address and port }";

  // parse command line
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(version);

  // give help
  if (parser.has("h"))
  {
    parser.printMessage();
    return 0;
  }

  // print version
  if (parser.has("V"))
  {
    std::cout << version << std::endl;
    return 0;
  }

  // print plain version
  if (parser.has("v"))
  {
    std::cout << VERSION;
    return 0;
  }

  // perform inference or not
  performi = !parser.has("i");

  // use TCP or UDP
  usetcp = !parser.has("udp");
  usetcp = parser.has("tcp");

  // key
  edge_key = parser.get<std::string>("k");
  if (!edge_key.empty())
    std::cout << "Using key " << edge_key << "\n";

  // DNN framework, configuration, and weights
  if (parser.has("f"))
    framework = parser.get<std::string>("f");
  else
    framework = d_framework;
  if (framework != "caffe" && framework != "darknet")
  {
    std::cerr << "DNN framework '" << framework << "' not supported\n";
    return 1;
  }
  std::cout << "DNN framework: " << framework << "\n";
  if (parser.has("c"))
    configfile = parser.get<std::string>("c");
  else
    configfile = d_configfile;
  std::cout << "DNN configuration: " << configfile << "\n";
  if (!std::filesystem::exists(configfile))
  {
    std::cerr << "DNN configuration file '" << configfile << "' does not exist\n";
    return 1;
  }
  if (parser.has("w"))
    weightsfile = parser.get<std::string>("w");
  else
    weightsfile = d_weightsfile;
  std::cout << "DNN weights: " << weightsfile << "\n";
  if (!std::filesystem::exists(weightsfile))
  {
    std::cerr << "DNN weight file does not exist\n";
    return 1;
  }

  // create the DNN
  if (framework == "caffe")
  {
    net = cv::dnn::readNetFromCaffe(configfile, weightsfile);
    if (net.empty())
    {
      std::cerr << "Can't load CAFFE DNN by using the following files:\n";
      std::cerr << "prototxt:   " << configfile << std::endl;
      std::cerr << "caffemodel: " << weightsfile << std::endl;
      return 1;
    }
  }
  else if (framework == "darknet")
  {
    // https://funvision.blogspot.com/2020/04/simple-opencv-tutorial-for-yolo-darknet.html
    net = cv::dnn::readNetFromDarknet(configfile, weightsfile);
    if (net.empty())
    {
      std::cerr << "Can't load DARKNET DNN by using the following files:\n";
      std::cerr << "cfg:   " << configfile << std::endl;
      std::cerr << "weights: " << weightsfile << std::endl;
      return 1;
    }
  }
  if (parser.has("g"))
  {
    try
    {
      // try to load the GPU
      //net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
      //net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);
      net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
      std::cout << "GPU DNN enabled!\n";
    }
    catch (const std::exception &e)
    {
      std::cerr << "GPU DNN error: " << e.what() << '\n';
    }
  }
  std::cout << "DNN is ready to go!\n";

  // define listening address and port
  std::string hostport = parser.get<std::string>(0);
  std::string hostaddr = hostport.find(":") != std::string::npos ? hostport.substr(0, hostport.find(":")) : d_upaddress;
  in_port_t ipport = hostport.find(":") != std::string::npos ? (hostport.find(":") < hostport.length() - 1 ? std::stoi(hostport.substr(hostport.find(":") + 1)) : std::stoi(d_upport)) : std::stoi(d_upport);

  // init network
  sockpp::socket_initializer sockInit;
  sockpp::udp_socket udpsock;
  sockpp::tcp_acceptor tcpacc;

  // TCP or UDP?
  if (usetcp)
  {
    if (!(tcpacc = sockpp::tcp_acceptor(sockpp::inet_address(hostaddr, ipport))))
    {
      std::cerr << "Error creating TCP acceptor: " << tcpacc.last_error_str() << std::endl;
      return 1;
    }

    // prepare socket for nonblocking
#ifdef WIN32
    u_long iMode = 1;
    ioctlsocket(tcpacc.handle(), FIONBIO, &iMode);
#else
    int flags = fcntl(tcpacc.handle(), F_GETFL);
    fcntl(tcpacc.handle(), F_SETFL, flags | O_NONBLOCK); // Change the socket into non-blocking state
    //struct timeval stimeout = { .tv_sec = 0, .tv_usec = 5000 };
    //tcpacc.set_option(SOL_SOCKET, SO_RCVTIMEO, stimeout);
#endif
    std::cout << "Acceptor awaiting on " << tcpacc.address() << std::endl;
  }
  else
  {
    // try to open the UDP uplink socket
    if (!udpsock)
    {
      std::cerr << "Error creating the UDP socket: " << udpsock.last_error_str() << std::endl;
      return 1;
    }

    // prepare socket for timeout
#ifdef WIN32
    DWORD stimeout = 5;
#else
    struct timeval stimeout = {.tv_sec = 0, .tv_usec = 5000};
#endif
    udpsock.set_option(SOL_SOCKET, SO_RCVTIMEO, stimeout);

    // bind to port
    if (!udpsock.bind(sockpp::inet_address(hostaddr, ipport)))
    {
      std::cerr << "Error binding the UDP uplink socket: " << udpsock.last_error_str() << "\n";
      return 1;
    }
  }

  // create a window
  std::string window_name = "LE UDP Streamer RX v" + std::string(VERSION);
  showlocal = !parser.has("n");
  if (showlocal)
    cv::namedWindow(window_name);

  // peeking active
  peekweb = parser.has("w");

  // timeout
  double timeout = parser.get<double>("o");

  // starting time and frame
  int64 t0 = cv::getTickCount(), t1;
  lastinticks = t0;

  // spin up a thread to run the UDP socket
  std::thread udplthr(udp_receiver, &udpsock);

  // spin up a thread to run the TCP connection
  std::thread tcplthr(tcp_receiver, &tcpacc);

  // spin up a thread to run the decompressor
  std::thread decompthr(decompressor);

  // spin up a thread to run the processor
  std::thread processthr(inferrer);

  // spin up a thread to run the web server
  std::thread webthr(webserver, std::stoi(d_peekport));

  // feedback transmission thread
  std::thread feedback_thread(feedback, &udpsock);

  // main loop
  cframe cf = {0};
  bool firstcycle = true;
  int key;
  while ((key = cv::waitKey(1)))
  {
    // check if there is a frame in queue
    std::pair<cv::Mat, uint64_t> sf = show_queue.pop(100);
    if (!sf.first.empty() && showlocal && !freezescreen)
      cv::imshow(window_name, sf.first); // update image

    // print stats
    t1 = cv::getTickCount();
    double elapsedtime = double(t1 - t0) / cv::getTickFrequency();
    double lastframeage = (t1 - lastinticks) / cv::getTickFrequency();
    if (elapsedtime >= 1)
    {
      // measured rates
      double mframerate = double(doneframes - lastframes) / elapsedtime;
      double mbitrate = 8.0 * accumbytes / elapsedtime;

      // print report
      std::string status = lastframeage < 1 ? linksymbol : nolinksymbol;
      std::cout << "#" << lastframenum << ";";
      std::cout << "FR:" << std::setprecision(1) << std::fixed << mframerate << "fps;";
      std::cout << "BR:" << std::setprecision(0) << mbitrate / 1000 << "kbps;";
      std::cout << "LR:" << std::setprecision(1) << 100 * double(errorframes - lasterrorframes) / (doneframes - lastframes) << "%;";
      std::cout << "DT:" << decompress_ms << "ms;";
      std::cout << "IT:" << inference_ms << "ms;";
      std::cout << status << "       \r" << std::flush;

      // update variables
      t0 = t1;
      lastframes = doneframes;
      lasterrorframes = errorframes;
      accumbytes = 0;
    }

    //std::cout << lastinticks << " " << lastframeage << " " << timeout << "\n";

    // check for timeout
    //if (timeout > 0 && lastintime > 0 && cv::getTickCount() - lastintime > timeout * cv::getTickFrequency())
    if (timeout > 0 && lastframeage > timeout)
    {
      std::cerr << "Timeout occurred, exiting\n";
      process = false;
      break;
    }

    // wait is needed to manage the image loop
    if (key == 27)
    {
      std::cout << "ESC key is pressed by user. Stopping the video" << std::endl;
      process = false;
      break;
    }
    else if (key == 'f')
      freezescreen = !freezescreen;
  }

  // join all the started threads
  decompthr.join();
  processthr.join();
  udplthr.join();
  tcplthr.join();
  feedback_thread.join();
  webthr.join();
}
