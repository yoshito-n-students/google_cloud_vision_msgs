#ifndef GOOGLE_CLOUD_VISION_MSGS_ENCODE
#define GOOGLE_CLOUD_VISION_MSGS_ENCODE

#include <algorithm>
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

namespace bai = boost::archive::iterators;

typedef bai::insert_linebreaks< bai::base64_from_binary< bai::transform_width<
                                    std::vector< boost::uint8_t >::const_iterator, 6, 8 > >,
                                72 >
    Base64Encoder;

// compressed image to base64 text
static inline void encode(const sensor_msgs::CompressedImage &from, Image &to) {
  to = Image();
  std::copy(Base64Encoder(from.data.begin()), Base64Encoder(from.data.end()),
            std::back_inserter(to.content));
}

// cv::Mat to base64 text
static inline void encode(const cv::Mat &from, Image &to) {
  to = Image();
  std::vector< boost::uint8_t > data;
  cv::imencode(".png", from, data);
  std::copy(Base64Encoder(data.begin()), Base64Encoder(data.end()), std::back_inserter(to.content));
}

// raw image to base64 text
static inline void encode(const sensor_msgs::Image &from, Image &to) {
  encode(cv_bridge::toCvShare(from, boost::shared_ptr< void >())->image, to);
}
}

#endif