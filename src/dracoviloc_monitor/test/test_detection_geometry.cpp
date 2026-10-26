#include <cmath>

#include <gtest/gtest.h>

#include "dracoviloc_monitor/detection_geometry.hpp"

namespace
{
using dracoviloc_monitor::CameraGeometry;
using dracoviloc_monitor::Pixel;

const CameraGeometry kRaw{848, 480, 848.0, 480.0, 424.0, 240.0};

CameraGeometry letterbox()
{
  const double scale = 640.0 / 848.0;
  const double pad_y = (640.0 - 480.0 * scale) / 2.0;
  return {640, 640, kRaw.fx * scale, kRaw.fy * scale,
    kRaw.cx * scale, kRaw.cy * scale + pad_y};
}

CameraGeometry stretch()
{
  return {640, 640, kRaw.fx * 640.0 / 848.0, kRaw.fy * 640.0 / 480.0,
    kRaw.cx * 640.0 / 848.0, kRaw.cy * 640.0 / 480.0};
}

Pixel encode(const Pixel & raw, const CameraGeometry & encoded)
{
  return {
    (raw.x - kRaw.cx) * encoded.fx / kRaw.fx + encoded.cx,
    (raw.y - kRaw.cy) * encoded.fy / kRaw.fy + encoded.cy,
  };
}

void expect_box_round_trip(const CameraGeometry & encoded)
{
  const Pixel raw_top_left{137.0, 41.0};
  const Pixel raw_bottom_right{293.0, 167.0};
  const auto encoded_top_left = encode(raw_top_left, encoded);
  const auto encoded_bottom_right = encode(raw_bottom_right, encoded);
  const auto mapped = dracoviloc_monitor::detection_box_to_raw(
    (encoded_top_left.x + encoded_bottom_right.x) / 2.0,
    (encoded_top_left.y + encoded_bottom_right.y) / 2.0,
    encoded_bottom_right.x - encoded_top_left.x,
    encoded_bottom_right.y - encoded_top_left.y, kRaw, encoded);
  EXPECT_NEAR(mapped.top_left.x, raw_top_left.x, 1.0);
  EXPECT_NEAR(mapped.top_left.y, raw_top_left.y, 1.0);
  EXPECT_NEAR(mapped.bottom_right.x, raw_bottom_right.x, 1.0);
  EXPECT_NEAR(mapped.bottom_right.y, raw_bottom_right.y, 1.0);
}

}  // namespace

TEST(DetectionGeometry, LetterboxOverlay)
{
  expect_box_round_trip(letterbox());
}

TEST(DetectionGeometry, StretchOverlay)
{
  expect_box_round_trip(stretch());
}
