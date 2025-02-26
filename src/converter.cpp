#include "converter.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <QImage>
#include <QDebug>
#include <QElapsedTimer>
#include <iostream>
#include <future>

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

// NEW FUNCTION: Optimize a chunk of paths for multi-threaded optimization
std::vector<OptimizedPath> BitmapConverter::optimizePathChunk(
    const std::vector<std::vector<PathSegment>>& chunkPaths, double strokeWidth,
    int chunkIndex, int totalChunks) {
    
    QElapsedTimer chunkTimer;
    chunkTimer.start();
    
    std::vector<OptimizedPath> optimizedChunk;
    
    // First pass: Separate outlines and zigzags
    std::vector<std::vector<PathSegment>> outlines;
    std::vector<std::vector<PathSegment>> zigzags;
    
    for (const auto& path : chunkPaths) {
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
        
        optimizedChunk.push_back(currentPath);
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
            optimizedChunk.push_back(zigzagPath);
        }
    }
    
    qint64 elapsed = chunkTimer.elapsed();
    std::cout << "Path optimization chunk " << chunkIndex << "/" << totalChunks 
              << " complete. Time: " << elapsed << " ms, "
              << "Paths: " << chunkPaths.size() << " -> " << optimizedChunk.size() << std::endl;
    
    return optimizedChunk;
}

// NEW FUNCTION: Optimize paths for a single color (for multi-threaded color optimization)
std::vector<OptimizedPath> BitmapConverter::optimizeColorPaths(
    const std::vector<std::vector<PathSegment>>& colorPaths, double strokeWidth,
    const QString& colorName) {
    
    QElapsedTimer colorTimer;
    colorTimer.start();
    
    // Use the same optimization logic as for monochrome paths
    auto result = optimizePathChunk(colorPaths, strokeWidth, 0, 1);
    
    qint64 elapsed = colorTimer.elapsed();
    std::cout << "Optimized paths for color " << colorName.toStdString() 
              << " in " << elapsed << " ms. Paths: " 
              << colorPaths.size() << " -> " << result.size() << std::endl;
    
    return result;
}

// Original optimizePaths function is now a wrapper that distributes work to multiple threads
std::vector<OptimizedPath> BitmapConverter::optimizePaths(
   const std::vector<std::vector<PathSegment>>& allPaths, double strokeWidth, int numThreads) {
    
    if (numThreads <= 1 || allPaths.size() < 100) {
        // For small path counts or single-threading, use the direct approach
        return optimizePathChunk(allPaths, strokeWidth, 0, 1);
    }
    
    // Divide paths into chunks for parallel processing
    std::vector<std::vector<std::vector<PathSegment>>> chunks;
    
    // Limit threads based on path count
    int adjustedThreads = std::min(numThreads, static_cast<int>(allPaths.size() / 50));
    adjustedThreads = std::max(1, adjustedThreads);
    
    if (adjustedThreads < numThreads) {
        std::cout << "Reduced optimization threads from " << numThreads 
                  << " to " << adjustedThreads << " based on path count" << std::endl;
    }
    
    // Create chunks of paths
    chunks.resize(adjustedThreads);
    for (size_t i = 0; i < allPaths.size(); i++) {
        chunks[i % adjustedThreads].push_back(allPaths[i]);
    }
    
    std::cout << "Multi-threaded path optimization with " << adjustedThreads 
              << " threads for " << allPaths.size() << " paths" << std::endl;
    
    // Process each chunk in parallel
    std::vector<std::future<std::vector<OptimizedPath>>> futures;
    for (int i = 0; i < adjustedThreads; i++) {
        futures.push_back(std::async(std::launch::async, 
                          &BitmapConverter::optimizePathChunk, this,
                          chunks[i], strokeWidth, i, adjustedThreads));
    }
    
    // Collect results
    std::vector<OptimizedPath> result;
    for (auto& future : futures) {
        auto chunkResult = future.get();
        result.insert(result.end(), chunkResult.begin(), chunkResult.end());
    }
    
    return result;
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

// Extract unique colors from an image
std::vector<ColorInfo> BitmapConverter::extractUniqueColors(const QImage& img) {
    std::map<QRgb, ColorInfo> uniqueColors;
    
    // Scan image to find all unique non-transparent colors
    for (int y = 0; y < img.height(); ++y) {
        for (int x = 0; x < img.width(); ++x) {
            QRgb pixel = img.pixel(x, y);
            if (qAlpha(pixel) > 0) { // Skip fully transparent pixels
                if (uniqueColors.find(pixel) == uniqueColors.end()) {
                    ColorInfo colorInfo;
                    colorInfo.color = pixel;
                    colorInfo.svgColor = rgbToSvgColor(pixel);
                    // Generate layer ID based on color
                    int colorIndex = uniqueColors.size();
                    colorInfo.layerId = QString("color_layer_%1").arg(colorIndex);
                    uniqueColors[pixel] = colorInfo;
                }
            }
        }
    }
    
    // Convert map to vector
    std::vector<ColorInfo> result;
    for (const auto& pair : uniqueColors) {
        result.push_back(pair.second);
    }
    
    return result;
}

// Convert RGB color to SVG color string
QString BitmapConverter::rgbToSvgColor(QRgb color) {
    return QString("#%1%2%3")
        .arg(qRed(color), 2, 16, QChar('0'))
        .arg(qGreen(color), 2, 16, QChar('0'))
        .arg(qBlue(color), 2, 16, QChar('0'));
}

// Write SVG with color layers
void BitmapConverter::writeColorLayersSvg(std::ofstream& svg, 
                                       const std::map<ColorInfo, std::vector<OptimizedPath>>& colorPathsMap,
                                       bool forInkscape) {
    // Write each color as a separate layer
    for (const auto& pair : colorPathsMap) {
        const ColorInfo& colorInfo = pair.first;
        const std::vector<OptimizedPath>& paths = pair.second;
        
        // Create a layer group for this color
        svg << "  <g id=\"" << colorInfo.layerId.toStdString() << "\" "
            << "fill=\"none\" stroke=\"" << colorInfo.svgColor.toStdString() << "\">\n";
        
        // Use existing method for generating SVG content
        if (forInkscape) {
            // Group outlines
            bool hasOutlines = false;
            for (const auto& path : paths) {
                if (path.segments.size() == 4) {
                    hasOutlines = true;
                    break;
                }
            }
            
            if (hasOutlines) {
                svg << "    <g id=\"" << colorInfo.layerId.toStdString() << "_outlines\">\n";
                for (const auto& path : paths) {
                    if (path.segments.size() == 4) {
                        svg << "      " << formatPathForInkscape(path.segments, path.strokeWidth) << "\n";
                    }
                }
                svg << "    </g>\n";
            }
            
            // Group zigzag fills (if any)
            bool hasZigzags = false;
            for (const auto& path : paths) {
                if (path.segments.size() > 4) {
                    hasZigzags = true;
                    break;
                }
            }
            
            if (hasZigzags) {
                svg << "    <g id=\"" << colorInfo.layerId.toStdString() << "_fills\">\n";
                for (const auto& path : paths) {
                    if (path.segments.size() > 4) {
                        // Check if this is a continuous zigzag path
                        bool isContinuous = true;
                        for (size_t i = 1; i < path.segments.size(); i++) {
                            if (std::abs(path.segments[i].x1 - path.segments[i-1].x2) > 1e-10 ||
                                std::abs(path.segments[i].y1 - path.segments[i-1].y2) > 1e-10) {
                                isContinuous = false;
                                break;
                            }
                        }
                        
                        if (isContinuous) {
                            // For continuous zigzag, use a single polyline
                            svg << "      <polyline points=\"" << path.segments[0].x1 << "," << path.segments[0].y1;
                            for (const auto& segment : path.segments) {
                                svg << " " << segment.x2 << "," << segment.y2;
                            }
                            svg << "\" fill=\"none\" stroke-width=\"" << path.strokeWidth << "\" />\n";
                        } else {
                            // For non-continuous zigzag, use individual lines
                            for (const auto& segment : path.segments) {
                                svg << "      <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                                    << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                                    << "\" stroke-width=\"" << path.strokeWidth << "\" />\n";
                            }
                        }
                    }
                }
                svg << "    </g>\n";
            }
        } else {
            // Non-Inkscape optimized format
            // Write outline paths
            for (const auto& path : paths) {
                bool isOutline = path.segments.size() == 4;
                if (isOutline) {
                    svg << "    <path d=\"M " << path.segments[0].x1 << " " << path.segments[0].y1;
                    for (const auto& segment : path.segments) {
                        svg << " L " << segment.x2 << " " << segment.y2;
                    }
                    svg << "\" fill=\"none\" stroke-width=\"" << path.strokeWidth << "\"/>\n";
                }
            }
            
            // Write zigzag paths
            for (const auto& path : paths) {
                bool isOutline = path.segments.size() == 4;
                if (!isOutline) {
                    // Check if this is a continuous zigzag path
                    bool isContinuous = true;
                    for (size_t i = 1; i < path.segments.size(); i++) {
                        if (std::abs(path.segments[i].x1 - path.segments[i-1].x2) > 1e-10 ||
                            std::abs(path.segments[i].y1 - path.segments[i-1].y2) > 1e-10) {
                            isContinuous = false;
                            break;
                        }
                    }
                    
                    if (isContinuous) {
                        // For continuous zigzag, use a single polyline
                        svg << "    <polyline points=\"" << path.segments[0].x1 << "," << path.segments[0].y1;
                        for (const auto& segment : path.segments) {
                            svg << " " << segment.x2 << "," << segment.y2;
                        }
                        svg << "\" fill=\"none\" stroke-width=\"" << path.strokeWidth << "\"/>\n";
                    } else {
                        // For non-continuous zigzag, use individual lines
                        for (const auto& segment : path.segments) {
                            svg << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                                << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                                << "\" stroke-width=\"" << path.strokeWidth << "\"/>\n";
                        }
                    }
                }
            }
        }
        
        svg << "  </g>\n";
    }
}

// Process a chunk of the image in a separate thread - modified to ignore white in color mode
void BitmapConverter::processChunk(
    const QImage& img, int startY, int endY, int width, 
    double pixelSize, double strokeWidth, double inset,
    const ConverterParams& params, ThreadContext& context) {
    
    // Count processed pixels for reporting
    int processedPixels = 0;
    int skippedWhitePixels = 0;
    QElapsedTimer threadTimer;
    threadTimer.start();
    
    // Process all pixels in this chunk
    for (int y = startY; y < endY; ++y) {
        for (int x = 0; x < width; ++x) {
            if (params.colorMode == ColorMode::Monochrome) {
                // Process monochrome pixels
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
                    
                    // Thread-safe add to the shared paths collection
                    {
                        std::lock_guard<std::mutex> lock(context.mutex);
                        context.paths.push_back(pixelPaths);
                    }
                    
                    processedPixels++;
                }
            } else {
                // Process color pixels
                QRgb pixelColor = img.pixel(x, y);
                
                // Skip transparent pixels
                if (qAlpha(pixelColor) == 0) continue;
                
                // Skip white or near-white pixels in color mode
                int r = qRed(pixelColor);
                int g = qGreen(pixelColor);
                int b = qBlue(pixelColor);
                
                // Define whiteness threshold (can be adjusted)
                // Higher threshold means more off-white colors will be ignored
                const int whiteThreshold = 245; 
                
                if (r >= whiteThreshold && g >= whiteThreshold && b >= whiteThreshold) {
                    skippedWhitePixels++;
                    continue;
                }
                
                // Create color info
                ColorInfo currentColor;
                currentColor.color = pixelColor;
                currentColor.svgColor = rgbToSvgColor(pixelColor);
                
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
                
                // Thread-safe add to the shared color paths collection
                {
                    std::lock_guard<std::mutex> lock(context.mutex);
                    
                    // Check if we already have this color
                    bool colorFound = false;
                    for (auto& pair : context.colorPaths) {
                        if (pair.first.color == currentColor.color) {
                            // Add to existing color
                            pair.second.push_back(pixelPaths);
                            colorFound = true;
                            break;
                        }
                    }
                    
                    if (!colorFound) {
                        // Generate layer ID for new color
                        int colorIndex = context.colorPaths.size();
                        currentColor.layerId = QString("color_layer_%1").arg(colorIndex);
                        
                        // Create new color entry
                        context.colorPaths[currentColor].push_back(pixelPaths);
                    }
                }
                
                processedPixels++;
            }
        }
    }
    
    qint64 elapsed = threadTimer.elapsed();
    std::cout << "Thread processing rows " << startY << "-" << (endY-1) 
              << " finished in " << elapsed << " ms, processed " 
              << processedPixels << " pixels";
    
    if (params.colorMode == ColorMode::PreserveColors) {
        std::cout << ", skipped " << skippedWhitePixels << " white pixels";
    }
    
    std::cout << " (" << (processedPixels * 1000.0 / elapsed) << " pixels/sec)" << std::endl;
}

// Main conversion function with multithreading support
void BitmapConverter::convert(
   const std::string& inputFile, 
   const std::string& outputFile,
   const ConverterParams& params) {
   
   // Start overall timing
   QElapsedTimer totalTimer;
   totalTimer.start();
   
   std::cout << "\n========== Starting conversion ==========\n";
   std::cout << "Input file: " << inputFile << std::endl;
   std::cout << "Output file: " << outputFile << std::endl;
   
   QImage img(QString::fromStdString(inputFile));
   if (img.isNull()) throw std::runtime_error("Failed to load image");
   
   std::cout << "Image loaded successfully. Dimensions: " << img.width() << "x" << img.height() << std::endl;
   
   // Handle image format based on color mode
   if (params.colorMode == ColorMode::Monochrome) {
       img = img.convertToFormat(QImage::Format_Mono);
       std::cout << "Using monochrome mode" << std::endl;
   } else {
       // For color mode, ensure we have a format that preserves colors
       img = img.convertToFormat(QImage::Format_ARGB32);
       std::cout << "Using color mode - preserving colors" << std::endl;
   }
   
   int width = img.width();
   int height = img.height();

   // Calculate sizes in mm
   double pixelSizeMm;
   if (params.units == Units::Inches) {
       pixelSizeMm = params.pixelSize ? inchesToMm(params.pixelSize) : 
                     getPixelSizeFromDocSize(inchesToMm(params.docWidth), 
                                          inchesToMm(params.docHeight), 
                                          width, height);
       std::cout << "Units: Inches" << std::endl;
   } else {
       pixelSizeMm = params.pixelSize ? params.pixelSize : 
                     getPixelSizeFromDocSize(params.docWidth, params.docHeight, 
                                          width, height);
       std::cout << "Units: Millimeters" << std::endl;
   }

   // Convert to points (72dpi)
   double pixelSize = (pixelSizeMm / 25.4) * 72;
   double strokeWidth = ((params.units == Units::Inches ? 
                         inchesToMm(params.strokeWidth) : 
                         params.strokeWidth) / 25.4) * 72;
   double inset = strokeWidth / 2;

   // Determine number of threads to use
   int numThreads = params.numThreads;
   if (numThreads <= 0) {
       // Auto-detect: use available hardware concurrency, but at least 1
       numThreads = std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
       std::cout << "Auto-detected available threads: " << numThreads << std::endl;
   } else {
       std::cout << "Using user-specified thread count: " << numThreads << std::endl;
   }
   
   // Limit threads to reasonable number
   int originalThreadCount = numThreads;
   numThreads = std::min(numThreads, 32);
   if (originalThreadCount != numThreads) {
       std::cout << "Limited thread count to 32" << std::endl;
   }
   
   // Create thread context for collecting results
   ThreadContext context;
   
   // Divide the image into horizontal strips for each thread
   std::vector<std::thread> threads;
   int rowsPerThread = height / numThreads;
   
   // If the number of rows is less than the number of threads, adjust
   if (rowsPerThread == 0) {
       rowsPerThread = 1;
       numThreads = height;
       std::cout << "Adjusted thread count to match image height: " << numThreads << std::endl;
   }
   
   std::cout << "Processing with " << numThreads << " threads, " 
             << rowsPerThread << " rows per thread" << std::endl;
   std::cout << "Fill type: " << (params.fillType == FillType::Zigzag ? "Zigzag" : "Solid") << std::endl;
   std::cout << "Optimization: " << (params.optimize ? "Enabled" : "Disabled") << std::endl;
   
   // Start processing timer
   QElapsedTimer processTimer;
   processTimer.start();
   
   // Launch threads to process chunks
   for (int t = 0; t < numThreads; t++) {
       int startY = t * rowsPerThread;
       int endY = (t == numThreads - 1) ? height : (t + 1) * rowsPerThread;
       
       std::cout << "Starting thread " << t << " to process rows " << startY << " to " << endY - 1 << std::endl;
       
       threads.push_back(std::thread(&BitmapConverter::processChunk, this,
                                  std::ref(img), startY, endY, width,
                                  pixelSize, strokeWidth, inset,
                                  std::ref(params), std::ref(context)));
   }
   
   // Wait for all threads to complete
   for (int t = 0; t < threads.size(); t++) {
       threads[t].join();
       std::cout << "Thread " << t << " completed" << std::endl;
   }
   
   qint64 processingTime = processTimer.elapsed();
   std::cout << "All threads finished. Processing time: " << processingTime << " ms" << std::endl;
   
   // Start SVG writing timer
   QElapsedTimer svgTimer;
   svgTimer.start();
   
   std::cout << "Writing SVG output..." << std::endl;
   
   // Prepare to write SVG output
   std::ofstream svg(outputFile);
   if (!svg) throw std::runtime_error("Failed to create output file");

   // Write SVG header
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

   // Start optimization timer
   QElapsedTimer optimizeTimer;
   optimizeTimer.start();

   // Process based on color mode
   if (params.colorMode == ColorMode::Monochrome) {
       // Process monochrome paths
       std::vector<std::vector<PathSegment>>& allPaths = context.paths;
       std::cout << "Monochrome paths generated: " << allPaths.size() << std::endl;
       
       // Write optimized or unoptimized paths
       if (params.optimize) {
           std::cout << "Optimizing paths with " << numThreads << " threads..." << std::endl;
           auto optimizedPaths = optimizePaths(allPaths, strokeWidth, numThreads);
           qint64 optimizeTime = optimizeTimer.elapsed();
           std::cout << "Path optimization complete. Time: " << optimizeTime << " ms" << std::endl;
           std::cout << "Original paths: " << allPaths.size() << ", Optimized paths: " << optimizedPaths.size() << std::endl;
           writeOptimizedSvg(svg, optimizedPaths, params.optimizeForInkscape);
       } else {
           // Code for non-optimized path output (unchanged)
           if (params.optimizeForInkscape) {
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
   } else {
       // Handle color processing
       std::map<ColorInfo, std::vector<std::vector<PathSegment>>>& pathsByColor = context.colorPaths;
       std::cout << "Color mode - unique colors found: " << pathsByColor.size() << std::endl;
       
       // Optimize paths by color and write to SVG
       std::map<ColorInfo, std::vector<OptimizedPath>> optimizedPathsByColor;
       
       if (params.optimize) {
           std::cout << "Optimizing paths for multiple colors using " << numThreads << " threads..." << std::endl;
           
           // Use multiple threads to optimize paths for different colors in parallel
           if (pathsByColor.size() > 1 && numThreads > 1) {
               // Create futures for each color
               std::vector<std::future<std::pair<ColorInfo, std::vector<OptimizedPath>>>> futures;
               
               for (const auto& pair : pathsByColor) {
                   futures.push_back(std::async(std::launch::async, 
                                   [this, &pair, strokeWidth]() {
                       return std::make_pair(
                           pair.first, 
                           this->optimizeColorPaths(pair.second, strokeWidth, pair.first.svgColor)
                       );
                   }));
               }
               
               // Collect results
               for (auto& future : futures) {
                   auto result = future.get();
                   optimizedPathsByColor[result.first] = result.second;
               }
           } else {
               // Process colors sequentially if there's only one color or thread
               for (const auto& pair : pathsByColor) {
                   const ColorInfo& color = pair.first;
                   const auto& colorPaths = pair.second;
                   
                   QElapsedTimer colorOptimizeTimer;
                   colorOptimizeTimer.start();
                   std::cout << "Optimizing paths for color: " << color.svgColor.toStdString() << "..." << std::endl;
                   optimizedPathsByColor[color] = optimizePaths(colorPaths, strokeWidth, numThreads);
                   std::cout << "Color paths optimization complete. Time: " << colorOptimizeTimer.elapsed() << " ms" << std::endl;
               }
           }
       } else {
           // For non-optimized paths, we still create OptimizedPath objects
           // for consistent handling
           for (const auto& pair : pathsByColor) {
               std::vector<OptimizedPath> simplePaths;
               for (const auto& path : pair.second) {
                   OptimizedPath optimizedPath;
                   optimizedPath.segments = path;
                   optimizedPath.strokeWidth = strokeWidth;
                   simplePaths.push_back(optimizedPath);
               }
               optimizedPathsByColor[pair.first] = simplePaths;
           }
       }
       
       qint64 optimizeTime = optimizeTimer.elapsed();
       std::cout << "Color path optimization complete. Time: " << optimizeTime << " ms" << std::endl;
       
       // Write all color layers to SVG
       writeColorLayersSvg(svg, optimizedPathsByColor, params.optimizeForInkscape);
   }

   svg << "</svg>\n";
   
   qint64 svgWriteTime = svgTimer.elapsed();
   std::cout << "SVG writing complete. Time: " << svgWriteTime << " ms" << std::endl;
   
   qint64 totalTime = totalTimer.elapsed();
   std::cout << "\n========== Conversion Summary ==========\n";
   std::cout << "Total conversion time: " << totalTime << " ms\n";
   std::cout << "- Processing time: " << processingTime << " ms (" 
             << (processingTime * 100 / totalTime) << "%)\n";
   std::cout << "- Optimization time: " << optimizeTimer.elapsed() << " ms ("
             << (optimizeTimer.elapsed() * 100 / totalTime) << "%)\n";
   std::cout << "- SVG writing time: " << svgWriteTime << " ms (" 
             << (svgWriteTime * 100 / totalTime) << "%)\n";
   std::cout << "Threads used: " << numThreads << "\n";
   std::cout << "Output file: " << outputFile << "\n";
   std::cout << "========================================\n";
}