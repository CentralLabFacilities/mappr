#pragma once

#include <QColor>
#include <geometry_msgs/Point.h>

geometry_msgs::Point
compute2DPolygonCentroid(const std::vector<geometry_msgs::Point> vertices) {
  int vertexCount = vertices.size();

  geometry_msgs::Point centroid;
  centroid.x = 0.0;
  centroid.y = 0.0;

  double signedArea = 0.0;
  double x0 = 0.0; // Current vertex X
  double y0 = 0.0; // Current vertex Y
  double x1 = 0.0; // Next vertex X
  double y1 = 0.0; // Next vertex Y
  double a = 0.0;  // Partial signed area

  // For all vertices except last
  int i = 0;
  for (i = 0; i < vertexCount - 1; ++i) {
    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[i + 1].x;
    y1 = vertices[i + 1].y;
    a = x0 * y1 - x1 * y0;
    signedArea += a;
    centroid.x += (x0 + x1) * a;
    centroid.y += (y0 + y1) * a;
  }

  // Do last vertex separately to avoid performing an expensive
  // modulus operation in each iteration.
  x0 = vertices[i].x;
  y0 = vertices[i].y;
  x1 = vertices[0].x;
  y1 = vertices[0].y;
  a = x0 * y1 - x1 * y0;
  signedArea += a;
  centroid.x += (x0 + x1) * a;
  centroid.y += (y0 + y1) * a;

  signedArea *= 0.5;
  centroid.x /= (6.0 * signedArea);
  centroid.y /= (6.0 * signedArea);

  return centroid;
}

bool isInsidePolygon(const std::vector<geometry_msgs::Point> points, double x,
                     double y) {
  int cross = 0;
  // ros grid_map_core polygon.cpp
  for (int i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    if (((points[i].y > y) != (points[j].y > y)) &&
        (x < (points[j].x - points[i].x) * (y - points[i].y) /
                     (points[j].y - points[i].y) +
                 points[i].x)) {
      cross++;
    }
  }
  return bool(cross % 2);
}

unsigned char *makeMapPalette() {
  auto palette = new unsigned char[256 * 4];
  unsigned char *palette_ptr = palette;

  // Boynton's list of 11 colors (use 9 Iks dee)
  QColor colors[9] = {
      QColor(0, 0, 255),     // Blue
      QColor(255, 0, 0),     // Red
      QColor(0, 255, 0),     // Green
      QColor(255, 255, 0),   // Yellow
      QColor(255, 0, 255),   // Magenta
      QColor(255, 128, 128), // Pink
      QColor(128, 128, 128), // Gray
      QColor(128, 0, 0),     // Brown
      QColor(255, 128, 0),   // Orange
  };

  for (int i = 0; i < 9; i++) {
    *palette_ptr++ = colors[i].red();   // red
    *palette_ptr++ = colors[i].green(); // green
    *palette_ptr++ = colors[i].blue();  // blue
    *palette_ptr++ = colors[i].alpha(); // alpha
  }

  // Rest tasteful blueish greenish grayish color
  for (int i = 9; i <= 255; i++) {
    *palette_ptr++ = 0x70; // red
    *palette_ptr++ = 0x89; // green
    *palette_ptr++ = 0x86; // blue
    *palette_ptr++ = 255;  // alpha
  }

  return palette;
}