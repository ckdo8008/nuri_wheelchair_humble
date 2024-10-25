#ifndef _NURI_HUMBLE_PROTOCOL_H
#define _NURI_HUMBLE_PROTOCOL_H

#define START_FRAME 0xFEFF

#include <cstdint>

uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}

// 속도 제어 요청
#pragma pack(push, 1)
typedef struct {
   uint16_t header;
   uint8_t  id;
   uint8_t  datasize;
   uint8_t  checksum;
   uint8_t  mode;
   uint8_t  direction;
   uint16_t speed;  // 빅엔디안
   uint8_t  arrival;

   void setValueSpeed(uint16_t val) {
      speed = swap_uint16(val);
   }

   uint16_t getValueSpeed() const {
      return swap_uint16(speed);
   }   
} SpeedCommand;
#pragma pack(pop)

// 피드백 요청
#pragma pack(push, 1)
typedef struct {
   uint16_t header;
   uint8_t  id;
   uint8_t  datasize;
   uint8_t  checksum;
   uint8_t  mode;
} FeedbackCallCommand;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
   uint16_t header;
   uint8_t  id;
   uint8_t  datasize;
   uint8_t  checksum;
   uint8_t  mode;
   uint8_t  data;
} FeedbackCallCommandParam;
#pragma pack(pop)

// 피드백 응답
#pragma pack(push, 1)
typedef struct {
   uint16_t header;
   uint8_t  id;
   uint8_t  datasize;
   uint8_t  checksum;
   uint8_t  mode;
   uint8_t  data[20];
} FeedbackResponse;
#pragma pack(pop)

// 속도 피드백 응답
#pragma pack(push, 1)
typedef struct {
   uint16_t header;
   uint8_t  id;
   uint8_t  datasize;
   uint8_t  checksum;
   uint8_t  mode;
   uint8_t  direction;
   uint16_t speed;  // 빅엔디안
   uint16_t pos;    // 빅엔디안
   uint8_t  current;

   uint16_t getValueSpeed() const {
      return swap_uint16(speed);
   }
   uint16_t getValuePos() const {
      return swap_uint16(pos);
   } 
} SpeedFeedbackResponse;
#pragma pack(pop)

// 스마트휠체어 전문
#pragma pack(push, 1)
typedef struct {
   uint16_t header;
   uint8_t  id;
   uint8_t  datasize;
   uint8_t  checksum;
   uint8_t  mode;
   uint16_t y;      // 리틀엔디안
   uint16_t x;      // 리틀엔디안
   uint16_t volt;   // 리틀엔디안
   uint8_t  btn;
   uint8_t speed;
} WheelchairResponse;
#pragma pack(pop)

#endif