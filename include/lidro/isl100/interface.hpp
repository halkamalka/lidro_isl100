#ifndef __LIDRO_ISL100_INTERFACE_H__
#define __LIDRO_ISL100_INTERFACE_H__

#include "lidro/isl100/frame.hpp"
#include "lidro/isl100/udp.hpp"
#include "lidro/isl100/config.hpp"
#include <memory>
#include "lidro/isl100/udp.hpp"

namespace lidro::isl100 {

class Interface {
public:
  using OnFrame = std::function<int(std::unique_ptr<lidro::isl100::Frame>)>;
  Interface(const lidro::isl100::Config &config);
  ~Interface();

  void startStream();
  void stopStream();

  int sendCommand(const std::vector<uint8_t> &data);

  std::unique_ptr<net::udp> data_io_{nullptr};
  std::unique_ptr<net::udp> ctrl_io_{nullptr};

  void setOnFrame(OnFrame on_frame){on_frame_ = on_frame;}

private:
  void restartStream();
  void onRead(uint8_t *buf, int len);

private:
  std::unique_ptr<lidro::isl100::Frame> frame_{nullptr};
  OnFrame on_frame_{nullptr};
  stream_mode stream_mode_{stream_mode_1};

  uint32_t current_frag_no_{0};
};

} // end namespace lidro

#endif // __LIDRO_ISL100_INTERFACE_H__
