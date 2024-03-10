#ifndef _CRC_CHECK_H_
#define _CRC_CHECK_H_

#include <cstdint>

namespace solais_serial {

/**
 * Get CRC16 checksum.
 * @param pchMessage  Data to check
 * @param dwLength    Data length
 * @return            CRC checksum
 */
uint8_t getCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * Append CRC8 to the end of data.
 * @param pchMessage  Data to append CRC
 * @param dwLength    Stream length = data + checksum
 */
bool verifyCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * Append CRC8 to the end of data.
 * @param pchMessage  Data to append CRC
 * @param dwLength    Stream length = data + checksum
 */
void appendCRC8CheckSum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * Get CRC16 checksum.
 * @param pchMessage  Data to check
 * @param dwLength    Data length
 * @return            CRC checksum
 */
uint16_t getCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * CRC16 verification function.
 * @param pchMessage  Data to verify
 * @param dwLength    Stream length = data + checksum
 * @return            CRC verify result
 */
bool verifyCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * Append CRC16 to the end of data.
 * @param pchMessage  Data to append CRC
 * @param dwLength    Stream length = data + checksum
 */
void appendCRC16CheckSum(uint8_t *pchMessage, uint32_t dwLength);

}

#endif
