// converter.h
#pragma once
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <map>
#include <QColor>
#include <QImage> 
#include <thread>
#include <mutex>
#include <atomic>


// Forward declarations
class PathOptimizer;
class SvgWriter;
class ColorHandler;
class GeometryUtils;

enum class Units { MM, Inches };
enum class FillType { Solid, Zigzag };
enum class ColorMode { Monochrome, PreserveColors }; 

/**
 * @brief Parameters for the bitmap to SVG conversion
 */
struct ConverterParams {
   Units units;              ///< MM or Inches
   FillType fillType;        ///< Solid or Zigzag fill type
   ColorMode colorMode;      ///< Monochrome or preserve colors
   double strokeWidth;       ///< Width of the strokes in the selected units
   double pixelSize;         ///< Size of each pixel in the selected units (if set)
   double docWidth;          ///< Document width in the selected units (if pixelSize not set)
   double docHeight;         ///< Document height in the selected units (if pixelSize not set)
   double margin;            ///< Margin around the SVG content in the selected units
   double angle;             ///< Angle for zigzag fill in degrees
   bool optimize;            ///< Whether to optimize path connections
   bool optimizeForInkscape; ///< Whether to optimize output for Inkscape
   int numThreads;           ///< Number of threads to use (0 = auto)
   
   // Plotter-specific options
   bool optimizeColorOrder;     ///< Order colors by luminosity (dark to light)
   bool groupSimilarColors;     ///< Group similar colors to reduce pen changes
   double colorSimilarityThreshold; ///< Threshold for grouping similar colors (0-255)
   bool simplifyPaths;          ///< Whether to simplify paths to reduce file size
   double simplifyTolerance;    ///< Tolerance for path simplification (0-1)
   bool minimizeTravel;         ///< Whether to optimize path order to minimize travel
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
 * @brief Represents color information for multi-color processing
 */
struct ColorInfo {
   QRgb color;              ///< The RGB color value
   QString svgColor;        ///< SVG-compatible color string
   QString layerId;         ///< ID for the SVG layer
   
   bool operator==(const ColorInfo& other) const {
       return color == other.color;
   }
   
   bool operator<(const ColorInfo& other) const {
       return color < other.color;
   }
};

/**
 * @brief Thread-safe data structure to collect paths from multiple threads
 */
struct ThreadContext {
   std::vector<std::vector<PathSegment>> paths;
   std::map<ColorInfo, std::vector<std::vector<PathSegment>>> colorPaths;
   std::mutex mutex;
};

/**
 * @brief Class to convert bitmap images to SVG
 */
class BitmapConverter {
public:
   /**
    * @brief Constructor
    */
   BitmapConverter();
   
   /**
    * @brief Destructor
    */
   ~BitmapConverter();

   /**
    * @brief Convert a bitmap image to SVG
    * @param inputFile Path to the input bitmap file
    * @param outputFile Path to the output SVG file
    * @param params Conversion parameters
    */
   void convert(const std::string& inputFile, const std::string& outputFile, const ConverterParams& params);

private:
   /**
    * @brief Process a chunk of the image in a separate thread
    * @param img The image to process
    * @param startY Starting Y coordinate of the chunk
    * @param endY Ending Y coordinate of the chunk
    * @param width Width of the image
    * @param pixelSize Size of each pixel
    * @param strokeWidth Width of the stroke
    * @param inset Inset from the pixel edge
    * @param params Conversion parameters
    * @param context Thread context for collecting results
    */
   void processChunk(const QImage& img, int startY, int endY, int width, 
                    double pixelSize, double strokeWidth, double inset,
                    const ConverterParams& params, ThreadContext& context);

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

   // Helper objects for different aspects of conversion
   std::unique_ptr<PathOptimizer> m_pathOptimizer;
   std::unique_ptr<SvgWriter> m_svgWriter;
   std::unique_ptr<ColorHandler> m_colorHandler;
   std::unique_ptr<GeometryUtils> m_geometryUtils;
};