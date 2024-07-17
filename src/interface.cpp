#include <iostream>
#include "lidro/isl100/frame.hpp"
#include "lidro/isl100/interface.hpp"
#include <sstream>
#include "log_message.h"
#include "lidro/isl100/lutp_protocol.hpp"
#include <unistd.h>
#include <stdexcept>

namespace lidro::isl100 {

Interface::Interface(const lidro::isl100::Config &config) {
  int err;
  stream_mode_ = config.mode;
  ctrl_io_ = std::make_unique<net::udp>();
  ctrl_io_->setAddr(config.host_addr);
  ctrl_io_->setPort(config.host_ctrl_port);
  ctrl_io_->setRemotePort(config.lidar_addr, config.lidar_ctrl_port);
  ctrl_io_->setOnRead([&](uint8_t *buf, int len) {
      LOG(INFO) << "response";
      H((char *)buf, len);
  });

  err = ctrl_io_->init();
  if (err) {
    throw std::runtime_error("exception ctrl-io");
  }
  ctrl_io_->start();

  data_io_ = std::make_unique<net::udp>();
  data_io_->setAddr(config.host_addr);
  data_io_->setPort(config.host_data_port);
  data_io_->setOnRead(
    std::bind(
      &Interface::onRead,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );
  
  err = data_io_->init();
  if (err) {
    throw std::runtime_error("exception data-io");
  }
  data_io_->start();

  restartStream();
}

Interface::~Interface() {
  stopStream();
}

void Interface::restartStream() {
  stopStream();
  startStream();
}

void Interface::startStream() {
  LUTP header{};
  header.version = 0xFFEE;
  header.prod_id = 0x41A2;
  stream_mode mode = stream_mode_;
  if (mode == stream_mode_1) {
    header.strm_type = 0x0180;
    header.opaque = 0x0900;
  }
  else if (mode == stream_mode_2) {
    header.strm_type = 0x0180;
    header.opaque = 0x0a00;
  }
  else if (mode == stream_mode_3) {
    header.strm_type = 0x0180;
    header.opaque = 0x0b00;
  }
  else if (mode == stream_mode_4) {
    header.strm_type = 0x0180;
    header.opaque = 0x0c00;
  }
  else {
    throw std::runtime_error("unhandled stream-mode");
  }

  size_t n = ctrl_io_->send((uint8_t *)&header, sizeof(header));
  LOG(INFO) << "[REQ] start";
}

void Interface::stopStream() {
  LUTP header{};
  header.version = 0xFFEE;
  header.prod_id = 0x41A2;
  header.opaque = 0x0400;
  header.strm_type = 0x0180;
  size_t n = ctrl_io_->send((uint8_t *)&header, sizeof(header));
  LOG(INFO) << "[REQ] stop";
}

void Interface::onRead(uint8_t *buf, int len) {
  LUTP *head = (LUTP *)buf;
  uint8_t *payload = buf + sizeof(LUTP);
  if(head->frag_no != (current_frag_no_)) {
    LOG(WARNING) << "not sync. current frag. no:" << head->frag_no << 
    ", expected frag. no: " << current_frag_no_+1;
  }

  if(head->frag_no < (head->frag_total)) {
    if(!frame_) {
      frame_.reset(new Frame(stream_mode_));
      current_frag_no_ = 0;
    }

    current_frag_no_ = head->frag_no;
    int n = frame_->fetch(head->frag_no, payload, (size_t)head->len);
    assert(n == head->len);
    current_frag_no_++;
    current_frag_no_ %= head->frag_total;

    if(head->frag_no == (head->frag_total-1)) {
      if(on_frame_ && frame_) {
        int err = on_frame_(std::move(frame_));
        if(err) {
          LOG(WARNING) << "error while processing payload";
        }
      }
    }
  }
}

} // end namespace lidro
