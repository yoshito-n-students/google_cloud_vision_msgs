#ifndef GOOGLE_CLOUD_VISION_MSGS_ENCODE
#define GOOGLE_CLOUD_VISION_MSGS_ENCODE

#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/insert_linebreaks.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp> // for cv::imencode

#include <cv_bridge/cv_bridge.h>
#include <google_cloud_vision_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace google_cloud_vision_msgs {

// binary data to base64 text
static inline void encode(const std::vector< boost::uint8_t > &from, Image &to) {
  namespace bai = boost::archive::iterators;
  typedef std::vector< boost::uint8_t >::const_iterator Iterator;
  typedef bai::insert_linebreaks< bai::base64_from_binary< bai::transform_width< Iterator, 6, 8 > >,
                                  72 >
      Base64Encoder;

  to = Image();
  std::copy(Base64Encoder(from.begin()), Base64Encoder(from.end()), std::back_inserter(to.content));
}

// compressed image to base64 text
static inline void encode(const sensor_msgs::CompressedImage &from, Image &to) {
  encode(from.data, to);
}

// cv::Mat to base64 text
static inline void encode(const cv::Mat &from, Image &to) {
  std::vector< boost::uint8_t > data;
  cv::imencode(".png", from, data);
  encode(data, to);
}

// raw image to base64 text
static inline void encode(const sensor_msgs::Image &from, Image &to) {
  encode(cv_bridge::toCvShare(from, boost::shared_ptr< void >())->image, to);
}

// binary stream to base64 text
static inline void encode(std::istream &from, Image &to) {
  namespace bai = boost::archive::iterators;
  typedef std::istreambuf_iterator< char > Iterator;
  typedef bai::insert_linebreaks< bai::base64_from_binary< bai::transform_width< Iterator, 6, 8 > >,
                                  72 >
      Base64Encoder;

  to = Image();
  std::istreambuf_iterator< char > begin(from), end;
  std::copy(Base64Encoder(begin), Base64Encoder(end), std::back_inserter(to.content));
}
}

#endif