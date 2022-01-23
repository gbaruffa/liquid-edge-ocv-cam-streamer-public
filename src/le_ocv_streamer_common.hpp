/**
 * @file le_ocv_streamer_common.hpp
 * @author G. Baruffa (giuseppe.baruffa@unipg.it)
 * @brief Common inclusion file
 * 
 * @version 0.4
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2020-2021 gbaruffa - University of Perugia
 * 
 */
#pragma once
#include <chrono>
#include <queue>
#include <vector>
#include <type_traits>

using namespace std::chrono_literals;

// https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Simple%20Moving%20Average/C++Implementation.html
template <size_t N, class input_t = double, class sum_t = double>
class SMA
{
public:
  input_t operator()(input_t input)
  {
    if (std::is_floating_point<input_t>::value)
    {
      // FOR FLOATING POINT
      previousInputs[index] = input;
      if (++index == N)
        index = 0;
      sum = 0;
      for (size_t i = 0; i < N; ++i)
        sum += previousInputs[i];
      return sum / N;
    }
    else
    {
      // FOR FIXED POINT
      sum -= previousInputs[index];
      sum += input;
      previousInputs[index] = input;
      if (++index == N)
        index = 0;
      return sum / N;
    }
  }

  // // Check that `sum_t` is an unsigned type
  // static_assert(!std::is_floating_point<input_t>::value && (sum_t(0) < sum_t(-1)),
  //   "Error: sum data type should be an unsigned integer, otherwise, the rounding operation in the return statement is invalid.");

private:
  size_t index = 0;
  input_t previousInputs[N] = {};
  sum_t sum = 0;
};

/**
 * @brief A queue with the capability to block among multiple accessing threads
 * 
 * Reference:
 * - https://github.com/gkalsi/barbecue/blob/master/bbq.h
 * 
 * @tparam T The contents type of the queue
 */
template <typename T>
class BoundedBlockingQueue
{
public:
  /**
   * @brief Construct a new Bounded Blocking Queue object
   * 
   * @param emptyelm An empty element, which is returned when the queue is empty
   * @param maxel The maximum number of contained elements (FIFO)
   * @param silence Enable reporting or not
   */
  BoundedBlockingQueue(T emptyelm, size_t maxel = UINT_MAX, bool silence = true) : m_maxel(maxel), m_emptyelm(emptyelm), m_silence(silence) {}

  /**
   * @brief Push an object in the queue
   * 
   * @param item The object to be inserted
   */
  void push(const T &item)
  {
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      if (m_queue.size() == m_maxel - 1)
      {
        if (!m_silence)
          std::cerr << "queue full\n";
        m_queue.pop();
        //return;
      }
      m_queue.push(item);
    }
    m_cv.notify_one();
  }

  /**
   * @brief Extract an object from the queue
   * 
   * @param numms Milliseconds to block on the extraction if the queue is empty
   * @return T The extracted element
   */
  T pop(int numms = INT_MAX)
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    while (m_queue.empty())
    {
      if (m_cv.wait_for(lock, numms * 1ms) == std::cv_status::timeout)
      {
        // return the empty element if timeout elapsed
        return m_emptyelm;
      }
    }
    auto result = m_queue.front();
    m_queue.pop();
    return result;
  }

  /**
   * @brief Return the size of the queue
   * 
   * @return size_t Number of contained elements
   */
  size_t size()
  {
    return m_queue.size();
  }

private:
  std::queue<T> m_queue;
  std::mutex m_mutex;
  std::condition_variable m_cv;
  size_t m_maxel;
  T m_emptyelm;
  bool m_silence;
};

/**
 * @brief Frame types that can be exchanged
 */
enum frametype
{
  FT_VIDEO,
  FT_CTRL,
  FT_INF
};

/**
 * @brief Frame information in the UDP packet
 */
struct frameinfo
{
  uint64_t timestamp; // a very long timestamp
  unsigned short framenum; // the frame number
  union
  {
    struct
    {
      unsigned short partnum; // the current part number
      unsigned short numparts; // the total number of parts
    };
    unsigned int framelen; // the length of this frame in bytes
  };
  char key[8]; // the access key to use
  frametype ft; // the conveyed frame tipe
};

/**
 * @brief Inference object
 */
struct infobj
{
  int idx; // index of the object from a known list
  float conf; // confidence of the inference
  int xlb, ylb, xrt, yrt; // coordinates of the vertices of the bounding box
};

/**
 * @brief Inference results
 */
struct infresult
{
  uint64_t timestamp; // the timestamp for which we are providing the results
  std::vector<infobj> objs; // the list of detected objects
};

// some default values
const std::string default_udp_streaming_port = "15000";
const std::string default_access_key = "00000000";

// https://emojipedia.org/
#ifdef WIN32
const std::string linksymbol = "LINK", nolinksymbol = "NOLINK";
#else
const std::string linksymbol = "📶", nolinksymbol = "📴";
#endif
