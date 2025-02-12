// converter.h
#pragma once
#include <string>
#include <vector>
#include <array>
#include <optional>

enum class Units { MM, Inches };
enum class FillType { Solid, Zigzag };

struct ConverterParams {
   Units units;
   FillType fillType;
   double strokeWidth;
   double pixelSize;
   double docWidth;
   double docHeight;
   double angle;
   bool optimize;
};

struct PathSegment {
   double x1, y1, x2, y2;
   bool operator==(const PathSegment& other) const {
       return std::abs(x1 - other.x1) < 1e-10 && std::abs(y1 - other.y1) < 1e-10 &&
              std::abs(x2 - other.x2) < 1e-10 && std::abs(y2 - other.y2) < 1e-10;
   }
};

struct OptimizedPath {
   std::vector<PathSegment> segments;
   double strokeWidth;
};

class BitmapConverter {
public:
   void convert(const std::string& inputFile, const std::string& outputFile, const ConverterParams& params);

private:
   double mmToInches(double mm);
   double inchesToMm(double inches);
   double getPixelSizeFromDocSize(double docWidth, double docHeight, int pixelWidth, int pixelHeight);
   std::vector<std::array<double, 4>> createZigzagPath(double x, double y, double width, double height, 
                                                      double strokeWidth, double angle);
   std::optional<std::array<double, 4>> clipLineToRect(double x1, double y1, double x2, double y2, 
                                                      double rx, double ry, double rw, double rh);
   bool lineRectIntersect(double x1, double y1, double x2, double y2, 
                         double rx, double ry, double rw, double rh);
   std::vector<OptimizedPath> optimizePaths(const std::vector<std::vector<PathSegment>>& allPaths, 
                                          double strokeWidth);
   bool canConnect(const PathSegment& s1, const PathSegment& s2);
};

// mainwindow.h
#pragma once
#include <QMainWindow>
#include <QButtonGroup>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

