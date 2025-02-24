// converter.h
#pragma once
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <map>

enum class Units { MM, Inches };
enum class FillType { Solid, Zigzag };

/**
 * @brief Parameters for the bitmap to SVG conversion
 */
struct ConverterParams {
   Units units;              ///< MM or Inches
   FillType fillType;        ///< Solid or Zigzag fill type
   double strokeWidth;       ///< Width of the strokes in the selected units
   double pixelSize;         ///< Size of each pixel in the selected units (if set)
   double docWidth;          ///< Document width in the selected units (if pixelSize not set)
   double docHeight;         ///< Document height in the selected units (if pixelSize not set)
   double angle;             ///< Angle for zigzag fill in degrees
   bool optimize;            ///< Whether to optimize path connections
   bool optimizeForInkscape; ///< Whether to optimize output for Inkscape
};

/**
 * @brief Represents a line segment in the SVG output
 */
struct PathSegment {
   double x1, y1, x2, y2;
   bool operator==(const PathSegment& other) const {
       return std::abs(x1 - other.x1) < 1e-10 && std::abs(y1 - other.y1) < 1e-10 &&
              std::abs(x2 - other.x2) < 1e-10 && std::abs(y2 - other.y2) < 1e-10;
   }
};

/**
 * @brief Represents an optimized path consisting of multiple segments
 */
struct OptimizedPath {
   std::vector<PathSegment> segments;
   double strokeWidth;
};

/**
 * @brief Class to convert bitmap images to SVG
 */
class BitmapConverter {
public:
   /**
    * @brief Convert a bitmap image to SVG
    * @param inputFile Path to the input bitmap file
    * @param outputFile Path to the output SVG file
    * @param params Conversion parameters
    */
   void convert(const std::string& inputFile, const std::string& outputFile, const ConverterParams& params);

private:
   /**
    * @brief Convert millimeters to inches
    * @param mm Value in millimeters
    * @return Value in inches
    */
   double mmToInches(double mm);
   
   /**
    * @brief Convert inches to millimeters
    * @param inches Value in inches
    * @return Value in millimeters
    */
   double inchesToMm(double inches);
   
   /**
    * @brief Calculate pixel size to fit document dimensions
    * @param docWidth Document width
    * @param docHeight Document height
    * @param pixelWidth Image width in pixels
    * @param pixelHeight Image height in pixels
    * @return Pixel size to fit the document
    */
   double getPixelSizeFromDocSize(double docWidth, double docHeight, int pixelWidth, int pixelHeight);
   
   /**
    * @brief Create zigzag fill pattern for a pixel
    * @param x X coordinate of the pixel's top-left corner
    * @param y Y coordinate of the pixel's top-left corner
    * @param width Width of the pixel
    * @param height Height of the pixel
    * @param strokeWidth Width of the stroke
    * @param angle Angle of the zigzag lines in degrees
    * @return Vector of line segments forming the zigzag pattern
    */
   std::vector<std::array<double, 4>> createZigzagPath(double x, double y, double width, double height, 
                                                      double strokeWidth, double angle);
   
   /**
    * @brief Clip a line to a rectangle
    * @param x1 Line start X
    * @param y1 Line start Y
    * @param x2 Line end X
    * @param y2 Line end Y
    * @param rx Rectangle X
    * @param ry Rectangle Y
    * @param rw Rectangle width
    * @param rh Rectangle height
    * @return Clipped line segment or nullopt if no intersection
    */
   std::optional<std::array<double, 4>> clipLineToRect(double x1, double y1, double x2, double y2, 
                                                      double rx, double ry, double rw, double rh);
   
   /**
    * @brief Check if a line intersects a rectangle
    * @param x1 Line start X
    * @param y1 Line start Y
    * @param x2 Line end X
    * @param y2 Line end Y
    * @param rx Rectangle X
    * @param ry Rectangle Y
    * @param rw Rectangle width
    * @param rh Rectangle height
    * @return True if the line intersects the rectangle
    */
   bool lineRectIntersect(double x1, double y1, double x2, double y2, 
                         double rx, double ry, double rw, double rh);
   
   /**
    * @brief Optimize paths for continuous zigzag pattern with minimal pen lifts
    * @param allPaths Vector of paths to optimize
    * @param strokeWidth Width of the stroke
    * @return Vector of optimized paths
    */
   std::vector<OptimizedPath> optimizePaths(const std::vector<std::vector<PathSegment>>& allPaths, 
                                          double strokeWidth);
   
   /**
    * @brief Check if two line segments can be connected
    * @param s1 First segment
    * @param s2 Second segment
    * @return True if the segments can be connected
    */
   bool canConnect(const PathSegment& s1, const PathSegment& s2);
   
   /**
    * @brief Format a path for Inkscape using SVG path syntax
    * @param segments Vector of path segments
    * @param strokeWidth Width of the stroke
    * @return SVG path element as string
    */
   std::string formatPathForInkscape(const std::vector<PathSegment>& segments, double strokeWidth);
   
   /**
    * @brief Write optimized SVG content to output stream
    * @param svg Output stream
    * @param optimizedPaths Vector of optimized paths
    * @param forInkscape Whether to optimize for Inkscape
    */
   void writeOptimizedSvg(std::ofstream& svg, const std::vector<OptimizedPath>& optimizedPaths, bool forInkscape);
};