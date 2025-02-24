#include "converter.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <QImage>

// Unit conversions
double BitmapConverter::mmToInches(double mm) { return mm / 25.4; }
double BitmapConverter::inchesToMm(double inches) { return inches * 25.4; }

// Calculate pixel size to fit document dimensions
double BitmapConverter::getPixelSizeFromDocSize(double docWidth, double docHeight, int pixelWidth, int pixelHeight) {
   return std::min(docWidth / pixelWidth, docHeight / pixelHeight);
}

// Generate zigzag pattern for pixel with special case handling for orthogonal angles
std::vector<std::array<double, 4>> BitmapConverter::createZigzagPath(
    double x, double y, double width, double height, double strokeWidth, double angle) {
    
    std::vector<std::array<double, 4>> paths;
    
    // If angle is very close to 0, 90, 180, or 270 degrees, handle it as a special case
    bool isOrthogonal = (std::fmod(std::abs(angle), 90.0) < 0.001 || std::fmod(std::abs(angle), 90.0) > 89.999);
    
    if (isOrthogonal) {
        // Normalize to 0, 90, 180, or 270 degrees
        int angle90 = static_cast<int>(std::round(angle / 90.0)) % 4 * 90;
        
        // Determine if horizontal or vertical
        bool isHorizontal = (angle90 == 0 || angle90 == 180);
        int numLines = static_cast<int>(std::floor((isHorizontal ? height : width) / strokeWidth)) + 1;
        
        // Create evenly spaced horizontal or vertical lines
        for (int i = 0; i < numLines; i++) {
            if (isHorizontal) {
                // Horizontal lines
                double yPos = y + i * strokeWidth;
                if (yPos <= y + height) {
                    paths.push_back({x, yPos, x + width, yPos});
                }
            } else {
                // Vertical lines
                double xPos = x + i * strokeWidth;
                if (xPos <= x + width) {
                    paths.push_back({xPos, y, xPos, y + height});
                }
            }
        }
    } else {
        // Handle diagonal angles
        double angleRad = angle * M_PI / 180.0;
        double spacing = strokeWidth;
        double diagonal = std::sqrt(width*width + height*height);
        int numLines = static_cast<int>(diagonal / spacing * 1.5) + 2;
        
        // Direction vectors
        double cosA = std::cos(angleRad);
        double sinA = std::sin(angleRad);
        
        // Determine which corner to start from based on angle
        double cornerX, cornerY;
        
        if (angle > 0 && angle < 90) {
            cornerX = x;
            cornerY = y;
        } else if (angle >= 90 && angle < 180) {
            cornerX = x + width;
            cornerY = y;
        } else if (angle >= 180 && angle < 270) {
            cornerX = x + width;
            cornerY = y + height;
        } else {
            cornerX = x;
            cornerY = y + height;
        }
        
        // Calculate perpendicular direction
        double perpX = -sinA;
        double perpY = cosA;
        
        // Generate lines covering the whole box
        double startDistance = -diagonal / 2;
        
        for (int i = 0; i < numLines; i++) {
            double offset = startDistance + i * spacing;
            
            // Starting point offset perpendicularly from the corner
            double startX = cornerX + offset * perpX;
            double startY = cornerY + offset * perpY;
            
            // Endpoint is in the line's direction, far enough to cross the whole box
            double endX = startX + diagonal * 2 * cosA;
            double endY = startY + diagonal * 2 * sinA;
            
            // Clip line to box boundaries
            if (auto clipped = clipLineToRect(startX, startY, endX, endY, x, y, width, height)) {
                paths.push_back(*clipped);
            }
        }
    }
    
    return paths;
}

// Check if line segments can connect
bool BitmapConverter::canConnect(const PathSegment& s1, const PathSegment& s2) {
   const double EPSILON = 1e-10;
   return (std::abs(s1.x2 - s2.x1) < EPSILON && std::abs(s1.y2 - s2.y1) < EPSILON) ||
          (std::abs(s1.x2 - s2.x2) < EPSILON && std::abs(s1.y2 - s2.y2) < EPSILON) ||
          (std::abs(s1.x1 - s2.x1) < EPSILON && std::abs(s1.y1 - s2.y1) < EPSILON) ||
          (std::abs(s1.x1 - s2.x2) < EPSILON && std::abs(s1.y1 - s2.y2) < EPSILON);
}

// Optimize paths by connecting segments
std::vector<OptimizedPath> BitmapConverter::optimizePaths(
   const std::vector<std::vector<PathSegment>>& allPaths, double strokeWidth) {
   std::vector<OptimizedPath> optimizedPaths;
   std::vector<bool> used(allPaths.size(), false);

   for (size_t i = 0; i < allPaths.size(); i++) {
       if (used[i]) continue;

       OptimizedPath currentPath;
       currentPath.strokeWidth = strokeWidth;
       currentPath.segments = allPaths[i];
       used[i] = true;

       bool foundConnection;
       do {
           foundConnection = false;
           for (size_t j = 0; j < allPaths.size(); j++) {
               if (used[j]) continue;
               for (const auto& segment : allPaths[j]) {
                   if (canConnect(currentPath.segments.back(), segment)) {
                       currentPath.segments.push_back(segment);
                       used[j] = true;
                       foundConnection = true;
                       break;
                   }
               }
               if (foundConnection) break;
           }
       } while (foundConnection);

       optimizedPaths.push_back(currentPath);
   }
   return optimizedPaths;
}

// Clip line to rectangle using Cohen-Sutherland
std::optional<std::array<double, 4>> BitmapConverter::clipLineToRect(
    double x1, double y1, double x2, double y2, double rx, double ry, double rw, double rh) {
    
    // Define the edges of the rectangle
    const double xmin = rx;
    const double ymin = ry;
    const double xmax = rx + rw;
    const double ymax = ry + rh;

    // Calculate direction vector
    const double dx = x2 - x1;
    const double dy = y2 - y1;

    // Initialize parameters
    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x1 - xmin, xmax - x1, y1 - ymin, ymax - y1};
    double u1 = 0;
    double u2 = 1;

    for (int i = 0; i < 4; i++) {
        // Parallel line to edge
        if (std::abs(p[i]) < 1e-10) {
            // Line outside the edge
            if (q[i] < 0) {
                return std::nullopt;
            }
            // Otherwise we continue, line is inside or on the edge
        } else {
            double t = q[i] / p[i];
            if (p[i] < 0) {
                // Line entering
                if (t > u2) {
                    return std::nullopt;
                }
                if (t > u1) {
                    u1 = t;
                }
            } else {
                // Line leaving
                if (t < u1) {
                    return std::nullopt;
                }
                if (t < u2) {
                    u2 = t;
                }
            }
        }
    }

    // If we get here, the line segment is at least partially inside
    return std::array<double, 4>{
        x1 + u1 * dx,
        y1 + u1 * dy,
        x1 + u2 * dx,
        y1 + u2 * dy
    };
}

// Check if line intersects rectangle
bool BitmapConverter::lineRectIntersect(
   double x1, double y1, double x2, double y2,
   double rx, double ry, double rw, double rh) {
   return !(std::max(x1, x2) < rx || std::min(x1, x2) > rx + rw ||
            std::max(y1, y2) < ry || std::min(y1, y2) > ry + rh);
}

// Main conversion function
void BitmapConverter::convert(
   const std::string& inputFile, 
   const std::string& outputFile,
   const ConverterParams& params) {
   
   QImage img(QString::fromStdString(inputFile));
   if (img.isNull()) throw std::runtime_error("Failed to load image");
   img = img.convertToFormat(QImage::Format_Mono);
   
   int width = img.width();
   int height = img.height();

   // Calculate sizes in mm
   double pixelSizeMm;
   if (params.units == Units::Inches) {
       pixelSizeMm = params.pixelSize ? inchesToMm(params.pixelSize) : 
                     getPixelSizeFromDocSize(inchesToMm(params.docWidth), 
                                          inchesToMm(params.docHeight), 
                                          width, height);
   } else {
       pixelSizeMm = params.pixelSize ? params.pixelSize : 
                     getPixelSizeFromDocSize(params.docWidth, params.docHeight, 
                                          width, height);
   }

   // Convert to points (72dpi)
   double pixelSize = (pixelSizeMm / 25.4) * 72;
   double strokeWidth = ((params.units == Units::Inches ? 
                         inchesToMm(params.strokeWidth) : 
                         params.strokeWidth) / 25.4) * 72;
   double inset = strokeWidth / 2;

   std::ofstream svg(outputFile);
   if (!svg) throw std::runtime_error("Failed to create output file");

   // Write SVG header
   svg << "<svg width=\"" << width * pixelSize << "px\" "
       << "height=\"" << height * pixelSize << "px\" "
       << "viewBox=\"0 0 " << width * pixelSize << " " << height * pixelSize << "\" "
       << "xmlns=\"http://www.w3.org/2000/svg\">\n";

   // Collect paths
   std::vector<std::vector<PathSegment>> allPaths;
   for (int y = 0; y < height; ++y) {
       for (int x = 0; x < width; ++x) {
           if (img.pixelColor(x, y).value() == 0) {
               double px = x * pixelSize + inset;
               double py = y * pixelSize + inset;
               double pwidth = pixelSize - strokeWidth;
               double pheight = pixelSize - strokeWidth;

               std::vector<PathSegment> pixelPaths;
               // Add outline
               pixelPaths.push_back({px, py, px + pwidth, py});
               pixelPaths.push_back({px + pwidth, py, px + pwidth, py + pheight});
               pixelPaths.push_back({px + pwidth, py + pheight, px, py + pheight});
               pixelPaths.push_back({px, py + pheight, px, py});

               // Add zigzag fill if selected
               if (params.fillType == FillType::Zigzag) {
                   auto zigzags = createZigzagPath(px, py, pwidth, pheight, strokeWidth, params.angle);
                   for (const auto& z : zigzags) {
                       pixelPaths.push_back({z[0], z[1], z[2], z[3]});
                   }
               }
               allPaths.push_back(pixelPaths);
           }
       }
   }

   // Write optimized or unoptimized paths
   if (params.optimize) {
       auto optimizedPaths = optimizePaths(allPaths, strokeWidth);
       
       // Write outline paths
       for (const auto& path : optimizedPaths) {
           bool isOutline = path.segments.size() == 4;
           if (isOutline) {
               svg << "  <path d=\"M " << path.segments[0].x1 << " " << path.segments[0].y1;
               for (const auto& segment : path.segments) {
                   svg << " L " << segment.x2 << " " << segment.y2;
               }
               svg << "\" fill=\"none\" stroke=\"black\" stroke-width=\"" 
                   << path.strokeWidth << "\"/>\n";
           }
       }
       
       // Write zigzag paths
       for (const auto& path : optimizedPaths) {
           bool isOutline = path.segments.size() == 4;
           if (!isOutline) {
               for (const auto& segment : path.segments) {
                   svg << "  <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                       << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                       << "\" stroke=\"black\" stroke-width=\"" << strokeWidth << "\"/>\n";
               }
           }
       }
   } else {
       for (const auto& pixelPaths : allPaths) {
           for (const auto& segment : pixelPaths) {
               svg << "  <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                   << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                   << "\" stroke=\"black\" stroke-width=\"" << strokeWidth << "\"/>\n";
           }
       }
   }

   svg << "</svg>\n";
}