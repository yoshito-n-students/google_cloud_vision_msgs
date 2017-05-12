#ifndef GOOGLE_CLOUD_VISION_MSGS_DRAWING
#define GOOGLE_CLOUD_VISION_MSGS_DRAWING

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <google_cloud_vision_msgs/BoundingPoly.h>
#include <google_cloud_vision_msgs/EntityAnnotation.h>
#include <google_cloud_vision_msgs/FaceAnnotation.h>

namespace google_cloud_vision_msgs {

//
// utilities
//

static bool isDrawable(const std::string &text) { return text.find("\n") == std::string::npos; }

static bool isDrawable(const BoundingPoly &poly) {
  for (std::vector< Vertex >::const_iterator vertex = poly.vertices.begin();
       vertex != poly.vertices.end(); ++vertex) {
    if (!vertex->has_x || !vertex->has_y) {
      return false;
    }
  }
  return true;
}

//
// basic type drawings
//

void drawText(cv::Mat &image, const std::string &text, const cv::Point center,
              const double font_scale, const cv::Scalar &font_color, const cv::Scalar &box_color,
              const int tickness = 1) {
  if (!isDrawable(text)) {
    return;
  }
  // calculate size of objects
  const cv::Size text_size(
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, tickness, NULL));
  const cv::Size box_size(text_size.width + 2 * tickness, text_size.height + 2 * tickness);
  // draw the text box
  cv::rectangle(image, center - cv::Point(box_size.width, box_size.height) / 2,
                center + cv::Point(box_size.width, box_size.height) / 2, box_color, -1);
  // draw the text
  cv::putText(image, text, center - cv::Point(text_size.width, -text_size.height) / 2,
              cv::FONT_HERSHEY_SIMPLEX, font_scale, CV_RGB(255, 255, 255), tickness);
}

void drawBoundingPoly(cv::Mat &image, const BoundingPoly &poly, const cv::Scalar &color,
                      const int tickness = 1) {
  if (!isDrawable(poly)) {
    return;
  }
  std::vector< std::vector< cv::Point > > points(1);
  for (std::vector< Vertex >::const_iterator vertex = poly.vertices.begin();
       vertex != poly.vertices.end(); ++vertex) {
    points[0].push_back(cv::Point(vertex->x, vertex->y));
  }
  cv::polylines(image, points, true, color, tickness);
}

//
// composite type drawings
//

void drawFaceAnnotation(cv::Mat &image, const FaceAnnotation &annotation, const cv::Scalar &color,
                        const int tickness = 1) {
  drawBoundingPoly(image, annotation.bounding_poly, color, tickness);
}

void drawFaceAnnotations(cv::Mat &image, const std::vector< FaceAnnotation > &annotations,
                         const cv::Scalar &color, const int tickness = 1) {
  for (std::vector< FaceAnnotation >::const_iterator annotation = annotations.begin();
       annotation != annotations.end(); ++annotation) {
    drawFaceAnnotation(image, *annotation, color, tickness);
  }
}

void drawTextAnnotation(cv::Mat &image, const EntityAnnotation &annotation, const double font_scale,
                        const cv::Scalar &font_color, const cv::Scalar &poly_color,
                        const int tickness = 1) {
  if (!isDrawable(annotation.bounding_poly) || !isDrawable(annotation.description)) {
    return;
  }
  // draw the bounding polygon
  drawBoundingPoly(image, annotation.bounding_poly, poly_color, tickness);
  // draw the description
  cv::Point text_center;
  {
    std::vector< cv::Point2f > points;
    for (std::vector< Vertex >::const_iterator vertex = annotation.bounding_poly.vertices.begin();
         vertex != annotation.bounding_poly.vertices.end(); ++vertex) {
      points.push_back(cv::Point2f(vertex->x, vertex->y));
    }
    const cv::Moments m(cv::moments(points));
    text_center.x = m.m10 / m.m00;
    text_center.y = m.m01 / m.m00;
  }
  drawText(image, annotation.description, text_center, font_scale, font_color, poly_color,
           tickness);
}

void drawTextAnnotations(cv::Mat &image, const std::vector< EntityAnnotation > &annotations,
                         const double font_scale, const cv::Scalar &font_color,
                         const cv::Scalar &line_color, const int tickness = 1) {
  for (std::vector< EntityAnnotation >::const_iterator annotation = annotations.begin();
       annotation != annotations.end(); ++annotation) {
    drawTextAnnotation(image, *annotation, font_scale, font_color, line_color, tickness);
  }
}
}

#endif