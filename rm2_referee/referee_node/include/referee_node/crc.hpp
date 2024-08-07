
#ifndef CRC_HPP_
#define CRC_HPP_

#include <cstdint>

namespace crc {

uint8_t GetCrc8Checksum(unsigned char *pch_message, unsigned int dw_length, unsigned char uc_crc8);

uint32_t VerifyCrc8Checksum(unsigned char *pch_message, unsigned int dw_length);

void AppendCrc8Checksum(unsigned char *pch_message, unsigned int dw_length);

uint16_t GetCrc16Checksum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc);

uint32_t VerifyCrc16Checksum(uint8_t *pch_message, uint32_t dw_length);

void AppendCrc16Checksum(uint8_t *pch_message, uint32_t dw_length);

}  // namespace crc

#endif  // CRC_HPP_
