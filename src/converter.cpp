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

// Optimize paths for continuous zigzag pattern to minimize pen lifts
std::vector<OptimizedPath> BitmapConverter::optimizePaths(
   const std::vector<std::vector<PathSegment>>& allPaths, double strokeWidth) {
   std::vector<OptimizedPath> optimizedPaths;
   
   // First pass: Separate outlines and zigzags
   std::vector<std::vector<PathSegment>> outlines;
   std::vector<std::vector<PathSegment>> zigzags;
   
   for (const auto& path : allPaths) {
       if (path.size() == 4) {
           // This is an outline (rectangle with 4 segments)
           outlines.push_back(path);
       } else if (path.size() > 4) {
           // This is a zigzag fill (outline + zigzag lines)
           std::vector<PathSegment> outline(path.begin(), path.begin() + 4);
           std::vector<PathSegment> zigzag(path.begin() + 4, path.end());
           
           outlines.push_back(outline);
           zigzags.push_back(zigzag);
       }
   }
   
   // Optimize outlines using the original algorithm
   std::vector<bool> outlineUsed(outlines.size(), false);
   
   for (size_t i = 0; i < outlines.size(); i++) {
       if (outlineUsed[i]) continue;
       
       OptimizedPath currentPath;
       currentPath.strokeWidth = strokeWidth;
       currentPath.segments = outlines[i];
       outlineUsed[i] = true;
       
       bool foundConnection;
       do {
           foundConnection = false;
           for (size_t j = 0; j < outlines.size(); j++) {
               if (outlineUsed[j]) continue;
               
               // Check if outline j can connect to the current path
               for (const auto& segment : outlines[j]) {
                   if (canConnect(currentPath.segments.back(), segment)) {
                       currentPath.segments.push_back(segment);
                       outlineUsed[j] = true;
                       foundConnection = true;
                       break;
                   }
               }
               if (foundConnection) break;
           }
       } while (foundConnection);
       
       optimizedPaths.push_back(currentPath);
   }
   
   // Special optimization for zigzag fills
   for (size_t i = 0; i < zigzags.size(); i++) {
       const auto& zigzagLines = zigzags[i];
       if (zigzagLines.empty()) continue;
       
       OptimizedPath zigzagPath;
       zigzagPath.strokeWidth = strokeWidth;
       
       // For a continuous zigzag, we need to intelligently order the lines
       // and create connecting segments between them
       
       // Group zigzag lines by their angle (for multiangle cases)
       std::map<double, std::vector<PathSegment>> linesByAngle;
       
       for (const auto& line : zigzagLines) {
           // Calculate angle of the line
           double dx = line.x2 - line.x1;
           double dy = line.y2 - line.y1;
           double angle = std::atan2(dy, dx);
           
           // Group by angle, rounded to 0.01 radians
           double roundedAngle = std::round(angle * 100) / 100.0;
           linesByAngle[roundedAngle].push_back(line);
       }
       
       // Process each angle group
       for (auto& [angle, lines] : linesByAngle) {
           // Determine the primary axis for sorting based on line angle
           bool sortByY = std::abs(std::cos(angle)) < std::abs(std::sin(angle));
           
           // Sort lines by their position
           if (sortByY) {
               std::sort(lines.begin(), lines.end(), [](const PathSegment& a, const PathSegment& b) {
                   return (a.y1 + a.y2) / 2.0 < (b.y1 + b.y2) / 2.0;
               });
           } else {
               std::sort(lines.begin(), lines.end(), [](const PathSegment& a, const PathSegment& b) {
                   return (a.x1 + a.x2) / 2.0 < (b.x1 + b.x2) / 2.0;
               });
           }
           
           // Create a continuous zigzag path with connecting segments
           std::vector<PathSegment> continuousPath;
           bool forward = true;
           
           for (size_t j = 0; j < lines.size(); j++) {
               PathSegment currentLine = lines[j];
               
               // If going backward, reverse the line direction
               if (!forward) {
                   std::swap(currentLine.x1, currentLine.x2);
                   std::swap(currentLine.y1, currentLine.y2);
               }
               
               // Add the current line to the path
               continuousPath.push_back(currentLine);
               
               // If not the last line, add a connector to the next line
               if (j < lines.size() - 1) {
                   PathSegment nextLine = lines[j + 1];
                   
                   // Create connecting segment
                   PathSegment connector;
                   connector.x1 = currentLine.x2;
                   connector.y1 = currentLine.y2;
                   
                   // Determine which end of the next line to connect to
                   if (forward) {
                       // Going from current end to next start
                       connector.x2 = nextLine.x1;
                       connector.y2 = nextLine.y1;
                   } else {
                       // Going from current end to next end (since we'll traverse next line backward)
                       connector.x2 = nextLine.x2;
                       connector.y2 = nextLine.y2;
                   }
                   
                   continuousPath.push_back(connector);
                   
                   // Flip direction for next line (bidirectional zigzag)
                   forward = !forward;
               }
           }
           
           // Add all segments from this angle group to the path
           zigzagPath.segments.insert(zigzagPath.segments.end(), continuousPath.begin(), continuousPath.end());
       }
       
       // Add the optimized zigzag path
       if (!zigzagPath.segments.empty()) {
           optimizedPaths.push_back(zigzagPath);
       }
   }
   
   return optimizedPaths;
}

// Clip line to rectangle using Liang-Barsky algorithm
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

// Format a path for Inkscape using SVG path commands
std::string BitmapConverter::formatPathForInkscape(const std::vector<PathSegment>& segments, double strokeWidth) {
    std::stringstream ss;
    
    if (segments.empty()) return "";
    
    // Start the path
    ss << "<path d=\"M " << segments[0].x1 << "," << segments[0].y1;
    
    // Add line segments
    for (const auto& seg : segments) {
        ss << " L " << seg.x2 << "," << seg.y2;
    }
    
    // Close the path attributes
    ss << "\" fill=\"none\" stroke=\"black\" stroke-width=\"" << strokeWidth << "\" />";
    
    return ss.str();
}

// Write SVG with optimizations for Inkscape
void BitmapConverter::writeOptimizedSvg(std::ofstream& svg, const std::vector<OptimizedPath>& optimizedPaths, bool forInkscape) {
    if (forInkscape) {
        // Group outlines
        svg << "  <g id=\"outlines\">\n";
        for (const auto& path : optimizedPaths) {
            bool isOutline = path.segments.size() == 4;
            if (isOutline) {
                svg << "    " << formatPathForInkscape(path.segments, path.strokeWidth) << "\n";
            }
        }
        svg << "  </g>\n";
        
        // Group zigzag fills (if any)
        bool hasZigzags = false;
        for (const auto& path : optimizedPaths) {
            if (path.segments.size() > 4) {
                hasZigzags = true;
                break;
            }
        }
        
        if (hasZigzags) {
            svg << "  <g id=\"fills\">\n";
            for (const auto& path : optimizedPaths) {
                if (path.segments.size() > 4) {
                    // Check if this is a continuous zigzag path (segments with shared endpoints)
                    bool isContinuous = true;
                    for (size_t i = 1; i < path.segments.size(); i++) {
                        if (std::abs(path.segments[i].x1 - path.segments[i-1].x2) > 1e-10 ||
                            std::abs(path.segments[i].y1 - path.segments[i-1].y2) > 1e-10) {
                            isContinuous = false;
                            break;
                        }
                    }
                    
                    if (isContinuous) {
                        // For continuous zigzag, use a single polyline which is very efficient
                        svg << "    <polyline points=\"" << path.segments[0].x1 << "," << path.segments[0].y1;
                        
                        for (const auto& segment : path.segments) {
                            svg << " " << segment.x2 << "," << segment.y2;
                        }
                        svg << "\" fill=\"none\" stroke=\"black\" stroke-width=\"" << path.strokeWidth << "\" />\n";
                    } else {
                        // For non-continuous zigzag, use a polyline for each segment
                        for (const auto& segment : path.segments) {
                            svg << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                                << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                                << "\" stroke=\"black\" stroke-width=\"" << path.strokeWidth << "\" />\n";
                        }
                    }
                }
            }
            svg << "  </g>\n";
        }
    } else {
        // Original output format with improved zigzag handling
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
                // Check if this is a continuous zigzag path (segments with shared endpoints)
                bool isContinuous = true;
                for (size_t i = 1; i < path.segments.size(); i++) {
                    if (std::abs(path.segments[i].x1 - path.segments[i-1].x2) > 1e-10 ||
                        std::abs(path.segments[i].y1 - path.segments[i-1].y2) > 1e-10) {
                        isContinuous = false;
                        break;
                    }
                }
                
                if (isContinuous) {
                    // For continuous zigzag, use a single polyline for efficiency
                    svg << "  <polyline points=\"" << path.segments[0].x1 << "," << path.segments[0].y1;
                    
                    for (const auto& segment : path.segments) {
                        svg << " " << segment.x2 << "," << segment.y2;
                    }
                    svg << "\" fill=\"none\" stroke=\"black\" stroke-width=\"" 
                        << path.strokeWidth << "\"/>\n";
                } else {
                    // For non-continuous zigzag, use individual lines
                    for (const auto& segment : path.segments) {
                        svg << "  <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                            << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                            << "\" stroke=\"black\" stroke-width=\"" << path.strokeWidth << "\"/>\n";
                    }
                }
            }
        }
    }
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

   // Write SVG header with additional metadata for Inkscape
   svg << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
   if (params.optimizeForInkscape) {
       svg << "<!-- Created with Bitmap Converter (Inkscape optimized) -->\n";
       svg << "<svg xmlns:svg=\"http://www.w3.org/2000/svg\" "
           << "xmlns=\"http://www.w3.org/2000/svg\" "
           << "xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
           << "width=\"" << width * pixelSize << "px\" "
           << "height=\"" << height * pixelSize << "px\" "
           << "viewBox=\"0 0 " << width * pixelSize << " " << height * pixelSize << "\" "
           << "version=\"1.1\">\n";
       svg << "<title>Bitmap Conversion</title>\n";
       svg << "<desc>Converted from " << inputFile << "</desc>\n";
   } else {
       svg << "<svg width=\"" << width * pixelSize << "px\" "
           << "height=\"" << height * pixelSize << "px\" "
           << "viewBox=\"0 0 " << width * pixelSize << " " << height * pixelSize << "\" "
           << "xmlns=\"http://www.w3.org/2000/svg\">\n";
   }

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
       writeOptimizedSvg(svg, optimizedPaths, params.optimizeForInkscape);
   } else {
       // Non-optimized path output
       if (params.optimizeForInkscape) {
           // For Inkscape optimization without path optimization,
           // we still group similar elements together
           
           // Group all outline segments
           svg << "  <g id=\"outlines\">\n";
           for (const auto& pixelPaths : allPaths) {
               for (size_t i = 0; i < std::min<size_t>(4, pixelPaths.size()); i++) {
                   const auto& segment = pixelPaths[i];
                   svg << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                       << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                       << "\" stroke=\"black\" stroke-width=\"" << strokeWidth << "\"/>\n";
               }
           }
           svg << "  </g>\n";
           
           // Group all fill segments
           bool hasFills = false;
           for (const auto& pixelPaths : allPaths) {
               if (pixelPaths.size() > 4) {
                   hasFills = true;
                   break;
               }
           }
           
           if (hasFills) {
               svg << "  <g id=\"fills\">\n";
               for (const auto& pixelPaths : allPaths) {
                   if (pixelPaths.size() > 4) {
                       for (size_t i = 4; i < pixelPaths.size(); i++) {
                           const auto& segment = pixelPaths[i];
                           svg << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                               << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                               << "\" stroke=\"black\" stroke-width=\"" << strokeWidth << "\"/>\n";
                       }
                   }
               }
               svg << "  </g>\n";
           }
       } else {
           // Original non-optimized output format
           for (const auto& pixelPaths : allPaths) {
               for (const auto& segment : pixelPaths) {
                   svg << "  <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                       << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                       << "\" stroke=\"black\" stroke-width=\"" << strokeWidth << "\"/>\n";
               }
           }
       }
   }

   svg << "</svg>\n";
}