
#ifndef DECODER_HPP_
#define DECODER_HPP_

#include "misc.hpp"

#include <deque>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <array>
#include <functional>

enum class DecoderState {
  kSof,
  kLenLsb,
  kLenMsb,
  kSeq,
  kCrc8,
  kCrc16,
};

/**
 * @brief 裁判串口解码器，读取字节流，解析出完整的数据帧
 */
class RefereeDecoder {
 public:
  explicit RefereeDecoder(std::function<void(uint8_t *, OpCodeEnum)> callback);
  RefereeDecoder() = delete;
  ~RefereeDecoder();

  void worker();

  RefereeDecoder &operator<<(const std::string &s);

 private:
  DecoderState state_{DecoderState::kSof};
  std::thread worker_thread_;
  bool worker_thread_running_{true};
  std::condition_variable cv_in_buffer_not_empty_;
  std::mutex mtx_;
  std::function<void(uint8_t *, OpCodeEnum)> callback_;

  std::deque<uint8_t> in_buffer_{};
  std::array<uint8_t, 128> out_buffer_{};
};

#endif
