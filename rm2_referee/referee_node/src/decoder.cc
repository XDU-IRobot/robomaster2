
#include "decoder.hpp"

#include "crc.hpp"

#include <iostream>
#include <string>

using namespace std::string_literals;

constexpr auto SIZE = 517;
constexpr auto HEADER_SOF = 0xA5;
constexpr auto REF_PROTOCOL_FRAME_MAX_SIZE = 128;
constexpr auto REF_PROTOCOL_HEADER_SIZE = 5;
constexpr auto REF_PROTOCOL_CRC16_SIZE = 2;
constexpr auto REF_HEADER_CRC_CMDID_LEN = (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t));

RefereeDecoder::RefereeDecoder(std::function<void(uint8_t *, OpCodeEnum)> callback)
    : worker_thread_(&RefereeDecoder::worker, this), callback_(callback) {}

RefereeDecoder::~RefereeDecoder() {
  std::unique_lock<std::mutex> lock(this->mtx_);
  this->worker_thread_running_ = false;
  (*this) << ""s;  // dummy data to wake up the worker thread
  this->cv_in_buffer_not_empty_.notify_one();
  this->worker_thread_.join();
}

RefereeDecoder &RefereeDecoder::operator<<(const std::string &s) {
  std::unique_lock<std::mutex> lock(this->mtx_);
  for (const auto &c : s) {
    this->in_buffer_.push_back(c);
  }
  this->cv_in_buffer_not_empty_.notify_one();
  return *this;
}

void RefereeDecoder::worker() {
  size_t idx = 0;
  size_t data_len;
  while (this->worker_thread_running_) {
    std::unique_lock<std::mutex> lock(this->mtx_);
    this->cv_in_buffer_not_empty_.wait(lock, [this] { return !this->in_buffer_.empty(); });

    uint8_t byte = this->in_buffer_.front();
    this->in_buffer_.pop_front();

    switch (this->state_) {
      case DecoderState::kSof: {
        if (byte == HEADER_SOF) {
          this->state_ = DecoderState::kLenLsb;
          this->out_buffer_[idx++] = byte;
        } else {
          idx = 0;
        }
        break;
      }

      case DecoderState::kLenLsb: {
        data_len = byte;
        this->out_buffer_[idx++] = byte;
        this->state_ = DecoderState::kLenMsb;
        break;
      }

      case DecoderState::kLenMsb: {
        data_len |= (byte << 8);
        this->out_buffer_[idx++] = byte;

        if (data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
          this->state_ = DecoderState::kSeq;
        } else {
          this->state_ = DecoderState::kSof;
          idx = 0;
        }
        break;
      }

      case DecoderState::kSeq: {
        this->out_buffer_[idx++] = byte;
        this->state_ = DecoderState::kCrc8;
        break;
      }

      case DecoderState::kCrc8: {
        this->out_buffer_[idx++] = byte;

        if (idx == REF_PROTOCOL_HEADER_SIZE) {
          if (crc::VerifyCrc8Checksum(this->out_buffer_.data(), REF_PROTOCOL_HEADER_SIZE)) {
            this->state_ = DecoderState::kCrc16;
          } else {
            this->state_ = DecoderState::kSof;
            idx = 0;
          }
        }
        break;
      }

      case DecoderState::kCrc16: {
        if (idx < (REF_HEADER_CRC_CMDID_LEN + data_len)) {
          this->out_buffer_[idx++] = byte;
        }
        if (idx >= (REF_HEADER_CRC_CMDID_LEN + data_len)) {
          this->state_ = DecoderState::kSof;
          idx = 0;

          if (crc::VerifyCrc16Checksum(this->out_buffer_.data(), REF_HEADER_CRC_CMDID_LEN + data_len)) {
            // 整包接收完+校验通过，通知回调函数
            this->callback_(this->out_buffer_.data() + 7,
                            static_cast<OpCodeEnum>((this->out_buffer_[6] << 8) | this->out_buffer_[5]));
          }
        }
        break;
      }

      default: {
        this->state_ = DecoderState::kSof;
        idx = 0;
        break;
      }
    }
  }
}
