/**
 * @file le_ocv_streamer_tx.cpp
 * @author G. Baruffa (giuseppe.baruffa@unipg.it)
 * 
 * @brief OpenCV-based MJPEG UDP stream transmitter
 * 
 * @version 0.4
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2020-2021 gbaruffa - University of Perugia
 * 
 */
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <fstream>
#include <filesystem>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#ifndef WIN32
#include <opencv2/quality/qualitypsnr.hpp>
#endif
#include "sockpp/tcp_connector.h"
#include <sockpp/udp_socket.h>
#include <sockpp/version.h>
#include <fcntl.h>
#ifndef WIN32
#include <sys/mman.h>
#include <unistd.h>
#endif
#include "config.h"
#include "le_ocv_streamer_common.hpp"

#define SPEED_DBG 0

using namespace std::chrono_literals;

// global parameters
bool process = true;
std::string edge_key;
static int64 lastintime;
static size_t detobjnum;
static uint64_t roundtripticks;
bool speedtest = false;
bool usetcp = true;
sockpp::tcp_connector tcpconn;
static double capture_ms, compress_ms, roundtriptime_ms;
static SMA<20> acquire_ms_filter, compress_ms_filter, roundtriptime_ms_filter;
std::mutex conn_mutex;

void common_listener(char *packet, ssize_t plen)
{
  // retrieve serialized string
  std::string serstring(packet, packet + plen);

  // deserialize
  std::istringstream serstream(serstring);

  // init inference structure
  infresult ir = {0};

  // deserialize in structure
  serstream >> ir.timestamp;
#if SPEED_DBG
  std::cout << "lis0:" << ir.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
  while (!serstream.eof())
  {
    infobj obj = {0};
    serstream >> obj.idx >> obj.conf >> obj.xlb >> obj.ylb >> obj.xrt >> obj.yrt;
    ir.objs.push_back(obj);
  }
#if SPEED_DBG
  std::cout << "lis1:" << ir.timestamp << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif

  detobjnum = ir.objs.size();
  roundtripticks = lastintime - ir.timestamp;
  roundtriptime_ms = roundtriptime_ms_filter(1000.0 * roundtripticks / cv::getTickFrequency());
}

/**
 * @brief Feedback receiving thread
 * 
 * @param sock Connected socket where we receive the data packets
 */
void feedback_listener(sockpp::udp_socket *sock)
{
  ssize_t plen;
  char packet[2048];

  while (process)
  {
    frameinfo fi;
    if (!usetcp)
    {
      // check if timeout occurred
      if ((plen = sock->recv(packet, sizeof(packet))) == -1)
        continue;

      // there is a full packet
      lastintime = cv::getTickCount();

      // read info
      std::copy(packet, packet + sizeof(frameinfo), reinterpret_cast<char *>(&fi));

      // access key control
      if (!std::equal(reinterpret_cast<char *>(&fi.key), reinterpret_cast<char *>(&fi.key) + sizeof(fi.key), edge_key.c_str()))
        continue;

      // extract data
      common_listener(packet + sizeof(frameinfo), plen - sizeof(frameinfo));
    }
    else
    {
      if (!tcpconn)
        continue;

      // read info
      if (tcpconn.read_n(&fi, sizeof(fi)) != sizeof(fi))
      {
        //std::cerr << "Error reading from the TCP stream: " << tcpconn.last_error_str() << std::endl;
        continue;
      }

      // there is a full packet
      lastintime = cv::getTickCount();

      // access key control
      if (!std::equal(reinterpret_cast<char *>(&fi.key), reinterpret_cast<char *>(&fi.key) + sizeof(fi.key), edge_key.c_str()))
        continue;

      // read frame
      if (sizeof(packet) < fi.framelen || tcpconn.read_n(packet, fi.framelen) != fi.framelen)
      {
        //std::cerr << "Error reading from the TCP stream: " << tcpconn.last_error_str() << std::endl;
        continue;
      }

      // extract data
      common_listener(packet, fi.framelen);
    }
  }
  //std::cerr << "Process out: feedback_listener\n";
}

/**
 * @brief Main function
 * 
 * @param argc Number of command line arguments
 * @param argv Pointer to pointers of arguments
 * @return int The exit code
 */
int main(int argc, char *argv[])
{
  // default values
  const std::string d_camera = "0";
  const std::string d_address = "localhost";
  const std::string d_port = default_udp_streaming_port;
  const std::string d_fsize = "640x480";
  const std::string d_frate = "30";
  const std::string d_vquality = "40";
  const std::string d_vbitrate = "2500000";
  const std::string d_plen = "512";
  const std::string d_time = "0";
  const std::string d_timeout = "0";
  const std::string d_key = default_access_key;
  const std::string d_movie = "";

  // the version
  const std::string version = "LIQUID_EDGE UDP video streaming transmitter v" + std::string(VERSION) + ", " + __DATE__ + " " + __TIME__ + ", sockpp " + sockpp::SOCKPP_VERSION + ", OpenCV " + CV_VERSION;

  // the command line switches
  const std::string keys =
      "{ help h usage ? | | Prints this help and exits }"
      "{ V version | | Prints version and exits }"
      "{ v plainversion | | Prints only the version number and exits }"
      "{ n novideo | | Do not show local video }"
      "{ p psnr | | Calculate PSNR }"
      "{ s speed | | Speed-test mode }"
      "{ t time | " + d_time + " | Operation time (seconds, 0 = do not stop) }"
      "{ c camera | " + d_camera + " | Camera index }"
      "{ m movie | " + d_movie + " | Movie path }"
      "{ r resolution | " + d_fsize + " | Desired frame resolution }"
      "{ f framerate | " + d_frate + " | Desired frame rate }"
      "{ q quality | " + d_vquality + " | Compressed video quality, 0 skips encoding (VBR) }"
      "{ b bitrate | | Compressed video bitrate (CBR) }"
      "{ u udpsize | " + d_plen + " | UDP packet size }"
      "{ tcp | | Use TCP/IP }"
      "{ udp | | Use UDP/IP (default) }"
      "{ k key | " + d_key + " | Engine access key }"
      "{ o timeout | " + d_timeout + " | Waiting timeout (0 = never ends) }"
      "{ @addr:port | " + d_address + ":" + d_port + " | Destination host address and port }";

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

  // just a speed-test
  speedtest = parser.has("s");

  // use TCP or UDP
  usetcp = !parser.has("udp");
  usetcp = parser.has("tcp");

  // open the video camera: if no success, exit the program
  cv::VideoCapture cap;
  if (parser.has("m"))
    cap.open(parser.get<std::string>("m"));
  else
  {
    cap.open(parser.get<int>("c")
#ifdef __linux
                 ,
             cv::CAP_V4L2
#elif __APPLE__
                 ,
             cv::CAP_AVFOUNDATION
#elif WIN32
                 ,
             cv::CAP_DSHOW
#endif
    );
  }
  if (!speedtest && cap.isOpened() == false)
  {
    std::cerr << "Cannot open the video camera\n";
    return 1;
  }

  // timeout
  double timeout = parser.get<double>("o");

  // resolution
  std::string resol = parser.get<std::string>("r");
  int width = (short)std::stoi(resol.substr(0, resol.find("x")));
  int height = (short)std::stoi(resol.substr(resol.find("x") + 1));
  double framerate = parser.get<double>("f");
  frameinfo fi = {0};
  if (!speedtest)
  {
    // needed to get the highest 4K resolutions for the Logitech BRIO
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // set the wanted resolution and frame rate
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, framerate);
  }
  if (parser.has("r") || parser.has("f"))
    std::cout << "Resolution (wanted): " << width << "x" << height << " @" << framerate << " fps" << std::endl;
  if (!speedtest)
  {
    width = (short)cap.get(cv::CAP_PROP_FRAME_WIDTH);   // get the real width
    height = (short)cap.get(cv::CAP_PROP_FRAME_HEIGHT); // get the real height
    framerate = cap.get(cv::CAP_PROP_FPS);              // get the real framerate
    std::cout << "Resolution (actual): " << width << "x" << height << " @" << framerate << " fps" << std::endl;
  }

  // define destination address and port
  std::string hostport = parser.get<std::string>(0);
  std::string hostaddr = hostport.find(":") != std::string::npos ? hostport.substr(0, hostport.find(":")) : d_address;
  in_port_t ipport = hostport.find(":") != std::string::npos ? (hostport.find(":") < hostport.length() - 1 ? std::stoi(hostport.substr(hostport.find(":") + 1)) : std::stoi(d_port)) : std::stoi(d_port);

  // init network
  sockpp::socket_initializer sockInit;
  sockpp::udp_socket udpsock;

  // TCP or UDP?
  if (!usetcp)
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
    udpsock.set_option(SOL_SOCKET, SO_RCVTIMEO, stimeout);
#else
    struct timeval stimeout;
    stimeout.tv_sec = 0;
    stimeout.tv_usec = 5000;
    udpsock.set_option(SOL_SOCKET, SO_RCVTIMEO, stimeout);
#endif

    // connect - is that really needed? yes
    if (!udpsock.connect(sockpp::inet_address(hostaddr, ipport)))
    {
      std::cerr << "Error connecting to " << hostaddr << ":" << ipport << "\n\t" << udpsock.last_error_str() << std::endl;
      return 1;
    }
    std::cout << "Created UDP uplink socket towards " << hostaddr << " @ " << udpsock.address() << std::endl;
  }

  // packet length
  const int plen = std::clamp(parser.get<int>("u"), 0, 2048);
  std::cout << "Packet size: " << plen << " bytes\n";

  // JPEG compression quality 1-100 or bitrate
  std::vector<int> jparams = {cv::IMWRITE_JPEG_QUALITY, parser.get<int>("q")};
#define vquality jparams[1]
  int vbitrate = 0;
  if (parser.has("b"))
  {
    vbitrate = parser.get<int>("b");
    std::cout << "Using JPEG bitrate: " << vbitrate << std::endl;
  }
  else
    std::cout << "Using JPEG quality: " << vquality << std::endl;

  // operating time
  double otime = parser.get<double>("t");
  if (otime < 0)
    otime = 0;
  if (otime)
    std::cout << "Operate for " << otime << " seconds\n";

  // key
  edge_key = parser.get<std::string>("k");
  if (!edge_key.empty())
    std::cout << "Using key " << edge_key << "\n";

  // create a window
  std::string window_name = "LE UDP Streamer TX v" + std::string(VERSION);
  bool showlocal = !parser.has("n");
  if (showlocal)
    cv::namedWindow(window_name);

  // PSNR
  bool dopsnr = parser.has("p");

  // frames capturing thread
  BoundedBlockingQueue<std::pair<cv::Mat, uint64_t>> capture_queue(std::make_pair(cv::Mat(), 0), 10);
  BoundedBlockingQueue<cv::Mat> show_queue(cv::Mat(), 10);
  std::thread capture_thread([&]()
                             {
                               cv::Mat frame;
                               int64 fnum = 0;
                               while (process)
                               {
                                 // read a new frame
                                 auto tickcountpre = cv::getTickCount();
                                 fnum = tickcountpre;
#if SPEED_DBG
                                 std::cout << "cap0:" << fnum << ":" << 1000.0 * tickcountpre / cv::getTickFrequency() << "\n";
#endif
                                 if (speedtest)
                                   // create a green-colored frame
                                   frame = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 255, 0));
                                 else if (parser.has("m"))
                                 {
                                   // read from movie and wait
                                   cap >> frame;
                                   if (frame.empty())
                                   {
                                     cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                                     cap >> frame;
                                   }
                                   std::this_thread::sleep_for(1000ms / framerate - 3ms);
                                 }
                                 else
                                   // read from the camera
                                   cap >> frame;
                                 auto tickcountpost = cv::getTickCount();
                                 capture_ms = acquire_ms_filter(1000.0 * (tickcountpost - tickcountpre) / cv::getTickFrequency());

#if SPEED_DBG
                                 std::cout << "cap1:" << fnum << ":" << 1000.0 * tickcountpost / cv::getTickFrequency() << "\n";
#endif

                                 // manage the queue
                                 if (!frame.empty())
                                 {
                                   // copy to queue
                                   capture_queue.push({frame.clone(), tickcountpost});
#if SPEED_DBG
                                   std::cout << "cap2:" << fnum << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif

                                   // put the frame in the show queue
                                   if (showlocal)
                                     show_queue.push(frame);
                                 }
                                 else
                                 {
                                   // breaking the while loop if the frames cannot be captured
                                   std::cerr << "Video camera is disconnected?\n";
                                   break;
                                 }
                                 ++fnum;
                               }
                               //std::cerr << "Process out: capture_thread\n";
                             });

  // frames processing thread
  double mpsnr = NAN;
  BoundedBlockingQueue<std::pair<std::vector<uchar>, uint64_t>> processing_queue(std::make_pair(std::vector<uchar>(), 0));
  std::thread process_thread([&]()
                             {
                               while (process)
                               {
                                 std::pair<cv::Mat, uint64_t> tframe = capture_queue.pop(5);
                                 if (!tframe.first.empty() && vquality > 0)
                                 {
#if SPEED_DBG
                                   std::cout << "pro0:" << tframe.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
                                   // encode the frame
                                   std::vector<uchar> cframe;
                                   auto tickcount = cv::getTickCount();
                                   cv::imencode(".jpg", tframe.first, cframe, jparams);
#if SPEED_DBG
                                   std::cout << "pro1:" << tframe.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
                                   compress_ms = compress_ms_filter(1000.0 * (cv::getTickCount() - tickcount) / cv::getTickFrequency());
                                   processing_queue.push({cframe, tframe.second});
#if SPEED_DBG
                                   std::cout << "pro2:" << tframe.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
                                   //std::cout << "in process!\n";
                                   if (dopsnr)
                                   {
                                     // decode image
                                     cv::Mat dframe = cv::imdecode(cframe, cv::IMREAD_COLOR);
                                     if (dframe.data != NULL)
                                     {
#ifndef WIN32
                                       cv::Scalar mpsnrscalar = cv::quality::QualityPSNR::compute(tframe.first, dframe, cv::noArray());
                                       mpsnr = (mpsnrscalar[0] + mpsnrscalar[1] + mpsnrscalar[2]) / 3.0; // this is not correct, but quick
#endif
                                     }
                                   }
                                 }
                               }
                               //std::cerr << "Process out: process_thread\n";
                             });

  // starting time and frame
  fi.framenum = 0;
  std::copy(edge_key.c_str(), edge_key.c_str() + sizeof(fi.key), reinterpret_cast<char *>(&fi.key));
  int64 doneframes = 0, lastframes = 0;
  int64 t0 = cv::getTickCount(), t1;
  int64 accumbytes = 0;

  ////////////////////////////////
  // frames transmission thread //
  ////////////////////////////////
  std::thread transmit_thread([&]()
                              {
                                unsigned char packet[2048];
                                while (process)
                                {
                                  std::pair<std::vector<uchar>, uint64_t> cframe = processing_queue.pop(5);
                                  if (!cframe.first.empty())
                                  {
#if SPEED_DBG
                                    std::cout << "tra0:" << cframe.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif
                                    // assoc the timestamp
                                    fi.timestamp = cframe.second;

                                    // reliable or unreliable delivery?
                                    if (usetcp)
                                    {
                                      // send the frame with TCP

                                      // try to open the TCP uplink socket
                                      if (!tcpconn)
                                      {
                                        if (!(tcpconn = sockpp::tcp_connector(sockpp::inet_address(hostaddr, ipport))))
                                        {
                                          //std::cerr << "Error connecting to server at " << sockpp::inet_address(hostaddr, ipport) << ":" << tcpconn.last_error_str() << std::endl;
                                          continue;
                                        }

                                        // set a timeout for the responses
                                        if (!tcpconn.read_timeout(std::chrono::milliseconds(5000)))
                                          std::cerr << "Error setting timeout on TCP stream: " << tcpconn.last_error_str() << std::endl;
                                        if (!tcpconn.write_timeout(std::chrono::milliseconds(5000)))
                                          std::cerr << "Error setting timeout on TCP stream: " << tcpconn.last_error_str() << std::endl;

                                        std::cout << "Created a TCP connection on " << tcpconn.address() << std::endl;
                                      }

                                      // only the frame length
                                      fi.framelen = (unsigned int)cframe.first.size();
                                      if (!tcpconn)
                                        continue;

                                      // send the frame information
                                      if (tcpconn.write_n(&fi, sizeof(fi)) != sizeof(fi))
                                      {
                                        std::cerr << "Error writing to the TCP stream: " << tcpconn.last_error_str() << std::endl;
                                        tcpconn.shutdown(SHUT_RDWR);
                                        continue;
                                      }

                                      // no error, accumulate transmitted bytes
                                      accumbytes += sizeof(fi);

                                      // send the frame
                                      if (tcpconn.write_n(cframe.first.data(), fi.framelen) != fi.framelen)
                                      {
                                        std::cerr << "Error writing to the TCP stream: " << tcpconn.last_error_str() << std::endl;
                                        tcpconn.shutdown(SHUT_RDWR);
                                        continue;
                                      }

                                      // no error, accumulate transmitted bytes
                                      accumbytes += fi.framelen;
                                    }
                                    else
                                    {
                                      // send the frame with UDP

                                      // first segment
                                      fi.partnum = 0;

                                      // find the number of segments per packet
                                      fi.numparts = (unsigned short)ceil((double)cframe.first.size() / (double)(plen - sizeof(frameinfo)));

                                      // split the frame and send packets
                                      unsigned int offset = 0;
                                      int remaining = static_cast<int>(cframe.first.size());
                                      for (fi.partnum = 0; fi.partnum < fi.numparts; fi.partnum++, offset += plen - sizeof(frameinfo), remaining -= plen - sizeof(frameinfo))
                                      {
                                        // copy a part of the frame in the buffer
                                        std::fill(packet, packet + plen, '\0');
                                        std::copy(reinterpret_cast<unsigned char *>(&fi), reinterpret_cast<unsigned char *>(&fi) + sizeof(frameinfo), packet);
                                        unsigned int psize = remaining < plen - sizeof(frameinfo) ? remaining : plen - sizeof(frameinfo);
                                        std::copy(cframe.first.data() + offset, cframe.first.data() + offset + psize, packet + sizeof(frameinfo));

                                        // down the socket
                                        if (udpsock.send(packet, plen) != ssize_t(plen))
                                        {
              // error occurred
#ifdef WIN32
                                          std::cerr << "Error writing to the UDP socket: " << udpsock.last_error_str() << std::endl;
#endif // WIN32
                                        }
                                        else
                                          // no error, accumulate transmitted bytes
                                          accumbytes += plen;
                                      }
                                    }

#if SPEED_DBG
                                    std::cout << "tra1:" << cframe.second << ":" << 1000.0 * cv::getTickCount() / cv::getTickFrequency() << "\n";
#endif

                                    // count things and times
                                    fi.framenum++;
                                    doneframes++;
                                  }
                                }
                                //std::cerr << "Process out: transmit_thread\n";
                              });

  // spin up a thread to run the feedback listening socket
  std::thread udpl_thread(feedback_listener, &udpsock);

  // wait until any key is pressed
  int64 lastaccumbytes = 0;
  int key;
  int64 tstart = lastintime = cv::getTickCount();
  while ((key = cv::waitKey(1)))
  {
    // show the image
    if (showlocal && show_queue.size())
      cv::imshow(window_name, show_queue.pop());

    // check the pressed key
    if (key == 27)
    {
      // ESC key
      std::cout << "ESC key is pressed by user. Terminating transmission\n";
      process = false;
      break;
    }
    else if (key == 43 || key == 45)
    {
      // +/- to decrease/increase quality, check quality limits
      vquality = std::clamp(vquality + 44 - key, 0, 100);
    }

    // periodically adjust the bitrate
    t1 = cv::getTickCount();
    double elapsedtime = double(t1 - t0) / cv::getTickFrequency();
    if (elapsedtime >= 1)
    {
      // measured rates
      double mframerate = double(doneframes - lastframes) / elapsedtime;
      double mbitrate = 8.0 * (accumbytes - lastaccumbytes) / elapsedtime;

      // print report
      std::string status = cv::getTickCount() - lastintime < cv::getTickFrequency() ? linksymbol : nolinksymbol;
      std::cout << "#" << doneframes << ";Q:" << vquality << ";";
      if (dopsnr)
        std::cout << "PSNR:" << std::setprecision(1) << std::fixed << mpsnr << "dB;";
      std::cout << "FR:" << std::setprecision(1) << std::fixed << mframerate << "fps;";
      std::cout << "BR:" << std::setprecision(0) << mbitrate / 1000 << "kbps;";
      std::cout << "NO:" << std::setprecision(0) << detobjnum << "objs;";
      std::cout << "AT:" << std::setprecision(1) << capture_ms << "ms;";
      std::cout << "CT:" << std::setprecision(1) << compress_ms << "ms;";
      std::cout << "RTT:" << std::setprecision(1) << roundtriptime_ms << "ms;";
      std::cout << status << "      \r" << std::flush;

      // update variables
      t0 = t1;
      lastframes = doneframes;
      lastaccumbytes = accumbytes;
    }

    // check for timeout
    if (timeout > 0 && lastintime > 0 && cv::getTickCount() - lastintime > timeout * cv::getTickFrequency())
    {
      std::cerr << "Timeout occurred, exiting\n";
      process = false;
      break;
    }

    // maybe we need to end
    if (otime > 0 && double(t1 - tstart) / cv::getTickFrequency() > otime)
    {
      process = false;
      break;
    }
  }

  // join all threads
  process_thread.join();
  capture_thread.join();
  transmit_thread.join();
  udpl_thread.join();
}
