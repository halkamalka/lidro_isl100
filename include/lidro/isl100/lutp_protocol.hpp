#ifndef __LUTP_ISL100_PROTOCOL_H__
#define __LUTP_ISL100_PROTOCOL_H__

#include <stdint.h>

typedef struct __attribute__((packed)) {
  uint16_t version;
  uint16_t prod_id;
  uint32_t stamp_h; // (us) Timestamp after sensor power on (higher 32bit)
  uint32_t stamp_l; // (us) Timestamp after sensor power on (lower 32bit)
  uint16_t len;     // LUTP payload length
  uint16_t reserved;
  uint16_t frag_total; // Number of LUTP fragments that make up a single LiDAR frame
  uint16_t frag_no;    // LUTP fragment count (zero-based)
  uint16_t strm_type;  // reserved_1;
  uint16_t opaque;
  uint32_t frame_count; // LiDAR Frame count (zero-based)
} LUTP;

#endif // __LUTP_ISL100_PROTOCOL_H__