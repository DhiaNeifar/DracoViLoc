#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace dracoviloc_monitor
{

struct CameraGeometry
{
  uint32_t width{0};
  uint32_t height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  bool valid() const
  {
    return width > 0 && height > 0 && std::isfinite(fx) && std::isfinite(fy) &&
           std::isfinite(cx) && std::isfinite(cy) && fx > 0.0 && fy > 0.0;
  }
};

struct Pixel
{
  double x;
  double y;
};

struct BoxCorners
{
  Pixel top_left;
  Pixel bottom_right;
};

inline Pixel detection_to_raw(
  const Pixel & detection, const CameraGeometry & raw, const CameraGeometry & encoded)
{
  if (!raw.valid() || !encoded.valid()) {
    throw std::invalid_argument("valid raw and detection camera intrinsics are required");
  }
  return {
    (detection.x - encoded.cx) * raw.fx / encoded.fx + raw.cx,
    (detection.y - encoded.cy) * raw.fy / encoded.fy + raw.cy,
  };
}

inline BoxCorners detection_box_to_raw(
  double center_x, double center_y, double size_x, double size_y,
  const CameraGeometry & raw, const CameraGeometry & encoded)
{
  return {
    detection_to_raw({center_x - size_x / 2.0, center_y - size_y / 2.0}, raw, encoded),
    detection_to_raw({center_x + size_x / 2.0, center_y + size_y / 2.0}, raw, encoded),
  };
}

}  // namespace dracoviloc_monitor
