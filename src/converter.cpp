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
#include <algorithm>
#include <functional>
#include <limits>

// Unit conversions
double BitmapConverter::mmToInches(double mm) { return mm / 25.4; }
double BitmapConverter::inchesToMm(double inches) { return inches * 25.4; }

// Calculate pixel size to fit document dimensions
double BitmapConverter::getPixelSizeFromDocSize(double docWidth, double docHeight, int pixelWidth, int pixelHeight) {
   double pixSize = std::min(docWidth / pixelWidth, docHeight / pixelHeight);
   std::cout << "getPixelSizeFromDocSize: " << docWidth << "x" << docHeight 
             << " document for " << pixelWidth << "x" << pixelHeight 
             << " pixels = " << pixSize << " size" << std::endl;
   return pixSize;
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

// Optimize a chunk of paths for multi-threaded optimization
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
    
    // Special optimization for zigzag fills with enhanced bidirectional handling
    for (size_t i = 0; i < zigzags.size(); i++) {
        const auto& zigzagLines = zigzags[i];
        if (zigzagLines.empty()) continue;
        
        OptimizedPath zigzagPath;
        zigzagPath.strokeWidth = strokeWidth;
        
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
        
        // Process each angle group with enhanced bidirectional optimization
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
            
            // Create a continuous zigzag path with optimized bidirectional traversal
            std::vector<PathSegment> continuousPath;
            
            // Enhanced zigzag with bidirectional traversal that minimizes travel distance
            for (size_t j = 0; j < lines.size(); j++) {
                PathSegment currentLine = lines[j];
                
                // Add the current line to the path
                continuousPath.push_back(currentLine);
                
                // If not the last line, add optimized connector to the next line
                if (j < lines.size() - 1) {
                    PathSegment nextLine = lines[j + 1];
                    
                    // Determine which connection (start-to-start, start-to-end, etc) is shortest
                    double dist1 = std::pow(currentLine.x2 - nextLine.x1, 2) + std::pow(currentLine.y2 - nextLine.y1, 2); // end-to-start
                    double dist2 = std::pow(currentLine.x2 - nextLine.x2, 2) + std::pow(currentLine.y2 - nextLine.y2, 2); // end-to-end
                    double dist3 = std::pow(currentLine.x1 - nextLine.x1, 2) + std::pow(currentLine.y1 - nextLine.y1, 2); // start-to-start
                    double dist4 = std::pow(currentLine.x1 - nextLine.x2, 2) + std::pow(currentLine.y1 - nextLine.y2, 2); // start-to-end
                    
                    // Find minimum distance connection
                    double minDist = std::min({dist1, dist2, dist3, dist4});
                    
                    PathSegment connector;
                    // Adjust line directions and create connector based on minimum distance
                    if (minDist == dist1) {
                        // Current end to next start (default case)
                        connector.x1 = currentLine.x2;
                        connector.y1 = currentLine.y2;
                        connector.x2 = nextLine.x1;
                        connector.y2 = nextLine.y1;
                        // Next line direction stays as is
                    } else if (minDist == dist2) {
                        // Current end to next end
                        connector.x1 = currentLine.x2;
                        connector.y1 = currentLine.y2;
                        connector.x2 = nextLine.x2;
                        connector.y2 = nextLine.y2;
                        // Reverse next line direction
                        std::swap(nextLine.x1, nextLine.x2);
                        std::swap(nextLine.y1, nextLine.y2);
                        lines[j+1] = nextLine; // Update for future iterations
                    } else if (minDist == dist3) {
                        // Current start to next start (need to reverse current line first)
                        std::swap(currentLine.x1, currentLine.x2);
                        std::swap(currentLine.y1, currentLine.y2);
                        continuousPath.back() = currentLine; // Update the last added line
                        
                        connector.x1 = currentLine.x2; // Now this is the original start
                        connector.y1 = currentLine.y2;
                        connector.x2 = nextLine.x1;
                        connector.y2 = nextLine.y1;
                    } else { // dist4 is minimum
                        // Current start to next end (need to reverse both lines)
                        std::swap(currentLine.x1, currentLine.x2);
                        std::swap(currentLine.y1, currentLine.y2);
                        continuousPath.back() = currentLine; // Update the last added line
                        
                        connector.x1 = currentLine.x2; // Now this is the original start
                        connector.y1 = currentLine.y2;
                        connector.x2 = nextLine.x2;
                        connector.y2 = nextLine.y2;
                        
                        // Reverse next line direction
                        std::swap(nextLine.x1, nextLine.x2);
                        std::swap(nextLine.y1, nextLine.y2);
                        lines[j+1] = nextLine; // Update for future iterations
                    }
                    
                    continuousPath.push_back(connector);
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

// Optimize paths for a single color (for multi-threaded color optimization)
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

// Path reordering to minimize travel distance (new function)
std::vector<std::vector<PathSegment>> BitmapConverter::reorderPathsForMinimalTravel(
    const std::vector<std::vector<PathSegment>>& paths) {
    
    if (paths.size() <= 1) return paths;
    
    std::vector<std::vector<PathSegment>> result;
    std::vector<bool> used(paths.size(), false);
    
    // Start with the first path
    result.push_back(paths[0]);
    used[0] = true;
    
    // Nearest neighbor algorithm to find next closest path
    while (result.size() < paths.size()) {
        const auto& lastPath = result.back();
        double minDistance = std::numeric_limits<double>::max();
        int bestIndex = -1;
        
        // Find the closest unused path
        for (size_t i = 0; i < paths.size(); i++) {
            if (!used[i]) {
                double distance = calculatePathDistance(lastPath, paths[i]);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestIndex = i;
                }
            }
        }
        
        if (bestIndex >= 0) {
            result.push_back(paths[bestIndex]);
            used[bestIndex] = true;
        }
    }
    
    return result;
}

double BitmapConverter::calculatePathDistance(
    const std::vector<PathSegment>& path1, 
    const std::vector<PathSegment>& path2) {
    
    if (path1.empty() || path2.empty()) return std::numeric_limits<double>::max();
    
    // Get the endpoint of the first path
    const auto& lastSegment = path1.back();
    double endX = lastSegment.x2;
    double endY = lastSegment.y2;
    
    // Get the start point of the second path
    const auto& firstSegment = path2.front();
    double startX = firstSegment.x1;
    double startY = firstSegment.y1;
    
    // Calculate Euclidean distance
    return std::sqrt(std::pow(endX - startX, 2) + std::pow(endY - startY, 2));
}

// Path simplification functions (new)
std::vector<PathSegment> BitmapConverter::simplifyPath(
    const std::vector<PathSegment>& path, double tolerance) {
    
    // For very short paths, no simplification needed
    if (path.size() <= 2) return path;
    
    // Convert segments to points for simplification
    auto points = segmentsToPoints(path);
    
    // Implementation of Douglas-Peucker algorithm
    std::vector<bool> keepPoint(points.size(), false);
    
    // Always keep first and last points
    keepPoint[0] = true;
    keepPoint[points.size() - 1] = true;
    
    // Recursive function to mark points to keep
    std::function<void(int, int)> simplifySegment = [&](int start, int end) {
        if (end - start <= 1) return;
        
        double maxDistance = 0;
        int maxIndex = start;
        
        // Line from start to end
        double lineX1 = points[start].first;
        double lineY1 = points[start].second;
        double lineX2 = points[end].first;
        double lineY2 = points[end].second;
        
        // Calculate line length
        double lineLength = std::sqrt(std::pow(lineX2 - lineX1, 2) + std::pow(lineY2 - lineY1, 2));
        
        // Find point with maximum distance from line
        for (int i = start + 1; i < end; i++) {
            double px = points[i].first;
            double py = points[i].second;
            
            // Calculate perpendicular distance
            double distance = 0;
            if (lineLength > 0) {
                distance = std::abs((lineY2 - lineY1) * px - (lineX2 - lineX1) * py + 
                                  lineX2 * lineY1 - lineY2 * lineX1) / lineLength;
            }
            
            if (distance > maxDistance) {
                maxDistance = distance;
                maxIndex = i;
            }
        }
        
        // If max distance is greater than tolerance, keep this point and recurse
        if (maxDistance > tolerance) {
            keepPoint[maxIndex] = true;
            simplifySegment(start, maxIndex);
            simplifySegment(maxIndex, end);
        }
    };
    
    // Start simplification
    simplifySegment(0, points.size() - 1);
    
    // Build new path with only kept points
    std::vector<std::pair<double, double>> simplifiedPoints;
    for (size_t i = 0; i < points.size(); i++) {
        if (keepPoint[i]) {
            simplifiedPoints.push_back(points[i]);
        }
    }
    
    // Convert back to segments
    return pointsToSegments(simplifiedPoints);
}

std::vector<std::pair<double, double>> BitmapConverter::segmentsToPoints(
    const std::vector<PathSegment>& segments) {
    
    std::vector<std::pair<double, double>> points;
    
    if (segments.empty()) return points;
    
    // Add first point
    points.push_back(std::make_pair(segments[0].x1, segments[0].y1));
    
    // Add each endpoint
    for (const auto& segment : segments) {
        points.push_back(std::make_pair(segment.x2, segment.y2));
    }
    
    return points;
}

std::vector<PathSegment> BitmapConverter::pointsToSegments(
    const std::vector<std::pair<double, double>>& points) {
    
    std::vector<PathSegment> segments;
    
    if (points.size() < 2) return segments;
    
    // Create segments between consecutive points
    for (size_t i = 0; i < points.size() - 1; i++) {
        PathSegment segment = {
            points[i].first, points[i].second,
            points[i+1].first, points[i+1].second
        };
        segments.push_back(segment);
    }
    
    return segments;
}

// Optimized optimizePaths function with travel minimization
std::vector<OptimizedPath> BitmapConverter::optimizePaths(
   const std::vector<std::vector<PathSegment>>& allPaths, double strokeWidth, int numThreads) {
    
    if (numThreads <= 1 || allPaths.size() < 100) {
        // For small path counts or single-threading, use the direct approach
        auto result = optimizePathChunk(allPaths, strokeWidth, 0, 1);
        
        // Apply path reordering if requested
        std::vector<OptimizedPath> reorderedResult;
        
        // Separate outlines and fill paths
        std::vector<OptimizedPath> outlinePaths;
        std::vector<OptimizedPath> fillPaths;
        
        for (const auto& path : result) {
            if (path.segments.size() == 4) {
                outlinePaths.push_back(path);
            } else {
                fillPaths.push_back(path);
            }
        }
        
        // Reorder outline paths for minimal travel
        if (outlinePaths.size() > 1) {
            std::vector<std::vector<PathSegment>> segments;
            for (const auto& path : outlinePaths) {
                segments.push_back(path.segments);
            }
            
            auto reorderedSegments = reorderPathsForMinimalTravel(segments);
            outlinePaths.clear();
            
            for (const auto& seg : reorderedSegments) {
                OptimizedPath path;
                path.segments = seg;
                path.strokeWidth = strokeWidth;
                outlinePaths.push_back(path);
            }
        }
        
        // Combine the reordered paths - keep outlines first, then fills
        reorderedResult.insert(reorderedResult.end(), outlinePaths.begin(), outlinePaths.end());
        reorderedResult.insert(reorderedResult.end(), fillPaths.begin(), fillPaths.end());
        
        return reorderedResult;
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
    
    // Apply path simplification to zigzag fills only if requested
    for (auto& path : result) {
        // Only simplify paths with more than 4 segments (zigzag fills)
        if (path.segments.size() > 4) {
            // Use a tolerance that is a fraction of the stroke width
            double tolerance = strokeWidth * 0.1;
            path.segments = simplifyPath(path.segments, tolerance);
        }
    }
    
    // Apply path reordering for minimal travel
    std::vector<OptimizedPath> reorderedResult;
        
    // Separate outlines and fill paths
    std::vector<OptimizedPath> outlinePaths;
    std::vector<OptimizedPath> fillPaths;
    
    for (const auto& path : result) {
        if (path.segments.size() == 4) {
            outlinePaths.push_back(path);
        } else {
            fillPaths.push_back(path);
        }
    }
    
    // Reorder outline paths for minimal travel
    if (outlinePaths.size() > 1) {
        std::vector<std::vector<PathSegment>> segments;
        for (const auto& path : outlinePaths) {
            segments.push_back(path.segments);
        }
        
        auto reorderedSegments = reorderPathsForMinimalTravel(segments);
        outlinePaths.clear();
        
        for (const auto& seg : reorderedSegments) {
            OptimizedPath path;
            path.segments = seg;
            path.strokeWidth = strokeWidth;
            outlinePaths.push_back(path);
        }
    }
    
    // Combine the reordered paths - keep outlines first, then fills
    reorderedResult.insert(reorderedResult.end(), outlinePaths.begin(), outlinePaths.end());
    reorderedResult.insert(reorderedResult.end(), fillPaths.begin(), fillPaths.end());
    
    return reorderedResult;
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
    
    // Close the path attributes with vector-effect for non-scaling strokes
    ss << "\" fill=\"none\" stroke=\"black\" stroke-width=\"" << strokeWidth 
       << "\" style=\"vector-effect:non-scaling-stroke\" />";
    
    return ss.str();
}

// Write SVG with optimizations for Inkscape
void BitmapConverter::writeOptimizedSvg(std::ofstream& svg, const std::vector<OptimizedPath>& optimizedPaths, 
                                      bool /* forInkscape - no longer used */, const std::string& strokeColor) {
    // Use the specified stroke color, or default to black if not provided
    std::string color = !strokeColor.empty() ? strokeColor : "black";
    
    // Group outlines
    svg << "  <g id=\"outlines\" inkscape:groupmode=\"layer\" inkscape:label=\"Outlines\">\n";
    for (const auto& path : optimizedPaths) {
        bool isOutline = path.segments.size() == 4;
        if (isOutline) {
            std::string pathStr = formatPathForInkscape(path.segments, path.strokeWidth);
            pathStr.replace(pathStr.find(" stroke=\"black\""), 15, " stroke=\"" + color + "\"");
            svg << "    " << pathStr << "\n";
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
        svg << "  <g id=\"fills\" inkscape:groupmode=\"layer\" inkscape:label=\"Fills\">\n";
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
                    svg << "\" fill=\"none\" stroke=\"" << color 
                        << "\" stroke-width=\"" << path.strokeWidth
                        << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                } else {
                    // For non-continuous zigzag, use a polyline for each segment
                    for (const auto& segment : path.segments) {
                        svg << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                            << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                            << "\" stroke=\"" << color 
                            << "\" stroke-width=\"" << path.strokeWidth
                            << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                    }
                }
            }
        }
        svg << "  </g>\n";
    }
}

// Color handling functions (new)
int BitmapConverter::rgbToLuminosity(QRgb color) {
    // Calculate luminosity using standard formula (perceived brightness)
    // Y = 0.299*R + 0.587*G + 0.114*B
    return static_cast<int>(0.299 * qRed(color) + 0.587 * qGreen(color) + 0.114 * qBlue(color));
}

std::vector<ColorInfo> BitmapConverter::sortColorsByLuminosity(
    const std::map<ColorInfo, std::vector<std::vector<PathSegment>>>& colorMap) {
    
    std::vector<ColorInfo> sortedColors;
    
    // Extract colors from map
    for (const auto& pair : colorMap) {
        sortedColors.push_back(pair.first);
    }
    
    // Sort colors from dark to light (darkest drawn first)
    std::sort(sortedColors.begin(), sortedColors.end(), 
              [this](const ColorInfo& a, const ColorInfo& b) {
        return rgbToLuminosity(a.color) < rgbToLuminosity(b.color);
    });
    
    return sortedColors;
}

// Group similar colors to reduce pen changes
std::map<ColorInfo, std::vector<std::vector<PathSegment>>> BitmapConverter::groupSimilarColors(
    const std::map<ColorInfo, std::vector<std::vector<PathSegment>>>& colorPaths,
    double threshold) {
    
    if (colorPaths.size() <= 1 || threshold <= 0) {
        return colorPaths;
    }
    
    // Create a copy of the input map for grouping
    std::map<ColorInfo, std::vector<std::vector<PathSegment>>> result;
    
    // Convert map to vector for easier manipulation
    std::vector<std::pair<ColorInfo, std::vector<std::vector<PathSegment>>>> colorVec(
        colorPaths.begin(), colorPaths.end());
    
    // Sort colors by luminosity for more predictable grouping
    std::sort(colorVec.begin(), colorVec.end(), 
              [this](const auto& a, const auto& b) {
        return rgbToLuminosity(a.first.color) < rgbToLuminosity(b.first.color);
    });
    
    // Track which colors have been grouped
    std::vector<bool> processed(colorVec.size(), false);
    
    // Process each color
    for (size_t i = 0; i < colorVec.size(); i++) {
        if (processed[i]) continue;
        
        // This color becomes a representative for a group
        ColorInfo groupColor = colorVec[i].first;
        auto& groupPaths = result[groupColor];
        groupPaths = colorVec[i].second;
        processed[i] = true;
        
        // Check other colors to see if they should be grouped with this one
        for (size_t j = i + 1; j < colorVec.size(); j++) {
            if (processed[j]) continue;
            
            // Calculate color distance using Euclidean distance in RGB space
            int rDiff = qRed(groupColor.color) - qRed(colorVec[j].first.color);
            int gDiff = qGreen(groupColor.color) - qGreen(colorVec[j].first.color);
            int bDiff = qBlue(groupColor.color) - qBlue(colorVec[j].first.color);
            
            double distance = std::sqrt(rDiff*rDiff + gDiff*gDiff + bDiff*bDiff);
            
            // If colors are similar enough, group them
            if (distance <= threshold) {
                // Add paths to the group
                groupPaths.insert(groupPaths.end(), 
                                 colorVec[j].second.begin(), 
                                 colorVec[j].second.end());
                processed[j] = true;
            }
        }
    }
    
    return result;
}

// Convert RGB color to SVG color string - Updated for transparency
QString BitmapConverter::rgbToSvgColor(QRgb color) {
    // Check if we need to handle alpha (transparency)
    if (qAlpha(color) < 255) {
        // Include alpha channel for semi-transparent colors
        return QString("rgba(%1,%2,%3,%4)")
            .arg(qRed(color))
            .arg(qGreen(color))
            .arg(qBlue(color))
            .arg(qAlpha(color)/255.0, 0, 'f', 2); // Alpha as 0.00-1.00
    } else {
        // For fully opaque colors, use hex format which is more common
        return QString("#%1%2%3")
            .arg(qRed(color), 2, 16, QChar('0'))
            .arg(qGreen(color), 2, 16, QChar('0'))
            .arg(qBlue(color), 2, 16, QChar('0'));
    }
}

// Write SVG with color layers - Enhanced for better Inkscape compatibility
void BitmapConverter::writeColorLayersSvg(std::ofstream& svg, 
                                       const std::map<ColorInfo, std::vector<OptimizedPath>>& colorPathsMap,
                                       bool /* forInkscape - no longer used */) {
    std::cout << "Writing SVG with " << colorPathsMap.size() << " color layers..." << std::endl;
    
    // Create a vector of colors for ordering
    std::vector<ColorInfo> colorOrder;
    for (const auto& pair : colorPathsMap) {
        colorOrder.push_back(pair.first);
    }
    
    // Sort colors by luminosity (darkest to lightest) for optimal plotting
    std::sort(colorOrder.begin(), colorOrder.end(), 
              [this](const ColorInfo& a, const ColorInfo& b) {
        return rgbToLuminosity(a.color) < rgbToLuminosity(b.color);
    });
    
    // Write each color as a separate layer in order of luminosity
    for (const auto& colorInfo : colorOrder) {
        const auto& paths = colorPathsMap.at(colorInfo);
        
        // Create a layer group for this color with Inkscape layer attributes
        svg << "  <g id=\"" << colorInfo.layerId.toStdString() << "\" "
            << "inkscape:groupmode=\"layer\" "
            << "inkscape:label=\"" << colorInfo.layerId.toStdString() << " (" 
            << rgbToLuminosity(colorInfo.color) << ")\" "
            << "style=\"display:inline\" "
            << "fill=\"none\" stroke=\"" << colorInfo.svgColor.toStdString() << "\">\n";
        
        // Always use Inkscape-optimized format
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
                    // Use explicit color for each path to ensure color is preserved
                    std::string pathStr = formatPathForInkscape(path.segments, path.strokeWidth);
                    // Remove the default stroke color and add our color
                    pathStr.replace(pathStr.find(" stroke=\"black\""), 15, 
                                   " stroke=\"" + colorInfo.svgColor.toStdString() + "\"");
                    svg << "      " << pathStr << "\n";
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
                        svg << "\" fill=\"none\" stroke=\"" << colorInfo.svgColor.toStdString() 
                            << "\" stroke-width=\"" << path.strokeWidth 
                            << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                    } else {
                        // For non-continuous zigzag, use individual lines
                        for (const auto& segment : path.segments) {
                            svg << "      <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                                << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                                << "\" stroke=\"" << colorInfo.svgColor.toStdString() 
                                << "\" stroke-width=\"" << path.strokeWidth 
                                << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                        }
                    }
                }
            }
            svg << "    </g>\n";
        }
        
        svg << "  </g>\n";
    }
    
    std::cout << "Finished writing color layers" << std::endl;
}

// Process a chunk of the image in a separate thread - Updated with size validation
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
                    // Calculate pixel position - ensure positive sizes
                    double px = x * pixelSize + inset; 
                    double py = y * pixelSize + inset;
                    double pwidth = std::max(0.001, pixelSize - 2 * inset);  // Ensure positive width
                    double pheight = std::max(0.001, pixelSize - 2 * inset); // Ensure positive height

                    // Add debug - only for the first few pixels
                    if (x < 3 && y < 3) {
                        std::cout << "Pixel at (" << x << "," << y << ") positioned at (" 
                                << px << "," << py << ") with size " << pwidth << "x" << pheight 
                                << " and stroke width " << strokeWidth << std::endl;
                    }

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
                // Process color pixels with similar size validation
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
                
                // Calculate pixel position - ensure positive sizes
                double px = x * pixelSize + inset; 
                double py = y * pixelSize + inset;
                double pwidth = std::max(0.001, pixelSize - 2 * inset);  // Ensure positive width
                double pheight = std::max(0.001, pixelSize - 2 * inset); // Ensure positive height
                
                // Add debug - only for the first few pixels
                if (x < 3 && y < 3) {
                    std::cout << "Color Pixel at (" << x << "," << y << ") positioned at (" 
                            << px << "," << py << ") with size " << pwidth << "x" << pheight 
                            << " and stroke width " << strokeWidth << std::endl;
                }

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

// Main conversion function with multithreading support and enhanced optimizations
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
   
   // Debug document size
   std::cout << "\n========== Document Size Debug ==========\n";
   std::cout << "Document size from parameters: " << params.docWidth << "x" << params.docHeight 
             << " " << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
   std::cout << "Margin from parameters: " << params.margin 
             << " " << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
   std::cout << "Pixel size from parameters: " << params.pixelSize 
             << " " << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
   
   // Debug parameter info
   std::cout << "\nInput parameters:" << std::endl;
   std::cout << "Units: " << (params.units == Units::Inches ? "Inches" : "MM") << std::endl;
   std::cout << "Fill type: " << (params.fillType == FillType::Zigzag ? "Zigzag" : "Solid") << std::endl;
   std::cout << "Color mode: " << (params.colorMode == ColorMode::Monochrome ? "Monochrome" : "Color") << std::endl;
   std::cout << "Stroke width: " << params.strokeWidth << " " << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
   
   if (params.pixelSize > 0.001) {
       std::cout << "Pixel size: " << params.pixelSize << " " << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
   } else {
       std::cout << "Document size: " << params.docWidth << "x" << params.docHeight << " " 
                 << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
   }
   
   std::cout << "Margin: " << params.margin << " " << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
   std::cout << "Optimize: " << (params.optimize ? "Yes" : "No") << std::endl;
   std::cout << "Thread count: " << params.numThreads << " (" << (params.numThreads == 0 ? "Auto" : "Manual") << ")" << std::endl;
   std::cout << "Minimize travel: " << (params.minimizeTravel ? "Yes" : "No") << std::endl;
   std::cout << "Simplify paths: " << (params.simplifyPaths ? "Yes" : "No") << std::endl;
   
   if (params.colorMode == ColorMode::PreserveColors) {
       std::cout << "Optimize color order: " << (params.optimizeColorOrder ? "Yes" : "No") << std::endl;
       std::cout << "Group similar colors: " << (params.groupSimilarColors ? "Yes" : "No") << std::endl;
       if (params.groupSimilarColors) {
           std::cout << "Color similarity threshold: " << params.colorSimilarityThreshold << std::endl;
       }
   }
   
   QImage img(QString::fromStdString(inputFile));
   if (img.isNull()) throw std::runtime_error("Failed to load image");
   
   std::cout << "Image loaded successfully. Dimensions: " << img.width() << "x" << img.height() << " pixels" << std::endl;
   
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

   // Create local copies of document dimensions that we can modify
   double docWidth = params.docWidth;
   double docHeight = params.docHeight;
   
   // Convert document dimensions to points (72dpi)
   double docWidthPoints = 0, docHeightPoints = 0;
   
   // Always use document dimensions from params (when available) - FIXED
   if (params.pixelSize <= 0.001) { // Only if pixel size is not explicitly set
       // Validate document dimensions - minimum size is 1 inch
       if (docWidth < 1 || docHeight < 1) {
           std::cout << "WARNING: Document dimensions too small: " << docWidth << "x" << docHeight 
                     << " " << (params.units == Units::Inches ? "inches" : "mm") << std::endl;
           std::cout << "Using default letter size (8.5x11 inches) instead\n";
           
           // Use standard letter size as default
           if (params.units == Units::Inches) {
               docWidth = 8.5;
               docHeight = 11.0;
           } else {
               // Convert from inches to mm
               docWidth = 215.9;  // 8.5 inches in mm
               docHeight = 279.4; // 11 inches in mm
           }
       }
       
       // Convert document dimensions to points
       if (params.units == Units::Inches) {
           docWidthPoints = docWidth * 72;
           docHeightPoints = docHeight * 72;
       } else {
           docWidthPoints = (docWidth / 25.4) * 72;
           docHeightPoints = (docHeight / 25.4) * 72;
       }
       
       std::cout << "Using document dimensions: " << docWidth << "x" << docHeight
                 << " " << (params.units == Units::Inches ? "inches" : "mm")
                 << " (" << docWidthPoints << "x" << docHeightPoints << " points)\n";
   }

   // Calculate sizes in mm - FIXED to handle tiny pixel sizes
   double pixelSizeMm;
   bool useExplicitPixelSize = (params.pixelSize > 0.001); // Only use explicit pixel size if it's reasonable
   
   if (params.units == Units::Inches) {
       // If pixel size is directly specified and reasonable
       if (useExplicitPixelSize) {
           std::cout << "Using explicit pixel size: " << params.pixelSize << " inches\n";
           pixelSizeMm = inchesToMm(params.pixelSize);
       } 
       // If document size is specified, subtract margins first to get printable area
       else {
           // Calculate available space after margins
           double availableWidth = docWidth - 2 * params.margin;
           double availableHeight = docHeight - 2 * params.margin;
           // Ensure we have positive dimensions
           availableWidth = std::max(0.1, availableWidth);
           availableHeight = std::max(0.1, availableHeight);
           std::cout << "Available content area: " << availableWidth << "x" << availableHeight << " inches\n";
           
           // Calculate pixel size to fit within available space
           pixelSizeMm = getPixelSizeFromDocSize(inchesToMm(availableWidth), 
                                              inchesToMm(availableHeight), 
                                              width, height);
       }
       std::cout << "Units: Inches" << std::endl;
       std::cout << "Document size: " << docWidth << "x" << docHeight << " inches" << std::endl;
       std::cout << "Margin: " << params.margin << " inches" << std::endl;
   } else {
       // Same logic for mm units
       if (useExplicitPixelSize) {
           std::cout << "Using explicit pixel size: " << params.pixelSize << " mm\n";
           pixelSizeMm = params.pixelSize;
       } else {
           double availableWidth = docWidth - 2 * params.margin;
           double availableHeight = docHeight - 2 * params.margin;
           availableWidth = std::max(0.1, availableWidth);
           availableHeight = std::max(0.1, availableHeight);
           std::cout << "Available content area: " << availableWidth << "x" << availableHeight << " mm\n";
           
           pixelSizeMm = getPixelSizeFromDocSize(availableWidth, availableHeight, 
                                              width, height);
       }
       std::cout << "Units: Millimeters" << std::endl;
       std::cout << "Document size: " << docWidth << "x" << docHeight << " mm" << std::endl;
       std::cout << "Margin: " << params.margin << " mm" << std::endl;
   }

   // Convert to points (72dpi)
   double pixelSize = (pixelSizeMm / 25.4) * 72;
   
   // Validate pixel size - ensure it's not too small
   if (pixelSize < 0.1) {
       std::cout << "WARNING: Calculated pixel size too small: " << pixelSize << " points" << std::endl;
       // Recalculate based on document dimensions and margin
       double availableWidthPoints = docWidthPoints - 2 * (params.units == Units::Inches ? params.margin * 72 : (params.margin / 25.4) * 72);
       double availableHeightPoints = docHeightPoints - 2 * (params.units == Units::Inches ? params.margin * 72 : (params.margin / 25.4) * 72);
       
       if (availableWidthPoints < 10 || availableHeightPoints < 10) {
           std::cout << "ERROR: Available area too small for content. Using default document size." << std::endl;
           docWidthPoints = 14 * 72; // 14 inches
           docHeightPoints = 11 * 72; // 11 inches
           availableWidthPoints = docWidthPoints - 2 * 72; // 1 inch margins
           availableHeightPoints = docHeightPoints - 2 * 72;
       }
       
       pixelSize = std::min(availableWidthPoints / width, availableHeightPoints / height);
       std::cout << "Recalculated pixel size: " << pixelSize << " points" << std::endl;
   }
   
   double strokeWidth = ((params.units == Units::Inches ? 
                         inchesToMm(params.strokeWidth) : 
                         params.strokeWidth) / 25.4) * 72;
   
   double inset = strokeWidth / 2;

   // Convert margin to points (72dpi)
   double marginPoints = 0;
   if (params.units == Units::Inches) {
       marginPoints = params.margin * 72;  // 72 points per inch
   } else {
       marginPoints = (params.margin / 25.4) * 72;  // Convert mm to inches, then to points
   }

   std::cout << "Pixel size: " << pixelSize << " points" << std::endl;
   std::cout << "Applying margin: " << params.margin << " " 
             << (params.units == Units::Inches ? "inches" : "mm") 
             << " (" << marginPoints << " points)" << std::endl;

   // Calculate content dimensions (without margin)
   double contentWidth = width * pixelSize;
   double contentHeight = height * pixelSize;
   
   std::cout << "Content dimensions before adjustment: " << contentWidth << "x" << contentHeight << " points" << std::endl;

   // Determine the total document dimensions - FIXED logic
   if (useExplicitPixelSize) {
       // When pixel size is explicitly specified, calculate document size from content
       docWidthPoints = contentWidth + 2 * marginPoints;
       docHeightPoints = contentHeight + 2 * marginPoints;
   } else {
       // When document size is specified in the params, honor that size
       // docWidthPoints and docHeightPoints already set above
   }

   // Validate final document dimensions - ensure they're not too small
   if (docWidthPoints < 72 || docHeightPoints < 72) { // Less than 1 inch
       std::cout << "WARNING: Final document dimensions too small. Setting to default 14x11 inches." << std::endl;
       docWidthPoints = 14 * 72; // 14 inches
       docHeightPoints = 11 * 72; // 11 inches
   }

   // Use docWidthPoints and docHeightPoints as the total dimensions
   double totalWidth = docWidthPoints;
   double totalHeight = docHeightPoints;

   std::cout << "Content dimensions: " << contentWidth << "x" << contentHeight << " points" << std::endl;
   std::cout << "Total document dimensions: " << totalWidth << "x" << totalHeight << " points" << std::endl;
   std::cout << "Total document in inches: " << (totalWidth/72) << "x" << (totalHeight/72) << " inches" << std::endl;

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
   
   // Group similar colors if requested
   if (params.colorMode == ColorMode::PreserveColors && params.groupSimilarColors) {
       QElapsedTimer groupTimer;
       groupTimer.start();
       
       std::cout << "Grouping similar colors with threshold: " << params.colorSimilarityThreshold << std::endl;
       std::cout << "Before grouping: " << context.colorPaths.size() << " unique colors" << std::endl;
       
       context.colorPaths = groupSimilarColors(context.colorPaths, params.colorSimilarityThreshold);
       
       std::cout << "After grouping: " << context.colorPaths.size() << " unique colors" << std::endl;
       std::cout << "Color grouping time: " << groupTimer.elapsed() << " ms" << std::endl;
   }
   
   // Start SVG writing timer
   QElapsedTimer svgTimer;
   svgTimer.start();
   
   std::cout << "Writing SVG output..." << std::endl;
   
   // Prepare to write SVG output
   std::ofstream svg(outputFile);
   if (!svg) throw std::runtime_error("Failed to create output file");

   // Write enhanced SVG header with total dimensions including margin and explicit units
   svg << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
   
   // Calculate dimensions in inches for Inkscape compatibility
   double docWidthInches = totalWidth / 72.0;
   double docHeightInches = totalHeight / 72.0;
   
   // Enhanced SVG header with additional Inkscape-specific attributes
   svg << "<!-- Created with Bitmap Converter (Optimized for Inkscape and efficient plotting) -->\n";
   svg << "<svg xmlns:svg=\"http://www.w3.org/2000/svg\" "
       << "xmlns=\"http://www.w3.org/2000/svg\" "
       << "xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
       << "xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\" "
       << "xmlns:sodipodi=\"http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd\" "
       << "width=\"" << docWidthInches << "in\" "
       << "height=\"" << docHeightInches << "in\" "
       << "viewBox=\"0 0 " << totalWidth << " " << totalHeight << "\" "
       << "inkscape:version=\"1.2\" "
       << "inkscape:document-units=\"in\" "
       << "version=\"1.1\">\n";

   // Enhanced metadata for Inkscape
   svg << "  <sodipodi:namedview "
       << "pagecolor=\"#ffffff\" "
       << "bordercolor=\"#666666\" "
       << "borderopacity=\"1.0\" "
       << "inkscape:pageopacity=\"0.0\" "
       << "inkscape:pageshadow=\"2\" "
       << "inkscape:document-units=\"in\" "
       << "units=\"in\" "
       << "showgrid=\"false\" "
       << "fit-margin-top=\"0\" "
       << "fit-margin-left=\"0\" "
       << "fit-margin-right=\"0\" "
       << "fit-margin-bottom=\"0\" "
       << "inkscape:pagecheckerboard=\"0\" "
       << "inkscape:deskcolor=\"#d1d1d1\" "
       << "/>\n";

   // Add metadata to ensure proper document size
   svg << "<metadata>\n";
   svg << "  <rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" "
       << "xmlns:dc=\"http://purl.org/dc/elements/1.1/\" "
       << "xmlns:cc=\"http://creativecommons.org/ns#\">\n";
   svg << "    <rdf:Description>\n";
   svg << "      <dc:format>image/svg+xml</dc:format>\n";
   svg << "      <dc:type rdf:resource=\"http://purl.org/dc/dcmitype/StillImage\"/>\n";
   svg << "    </rdf:Description>\n";
   svg << "  </rdf:RDF>\n";
   svg << "</metadata>\n";

   // Debug output to help understand the issue
   std::cout << "\nDEBUG TRANSFORM:\n";
   std::cout << "Margin Points: " << marginPoints << "\n";
   std::cout << "Content Width/Height: " << contentWidth << "x" << contentHeight << "\n";
   std::cout << "Total Width/Height: " << totalWidth << "x" << totalHeight << "\n";
       
    // Calculate centering offsets
    double xOffset = marginPoints + (totalWidth - contentWidth - 2 * marginPoints) / 2;
    double yOffset = marginPoints + (totalHeight - contentHeight - 2 * marginPoints) / 2;

    // Create a transform that centers content within the margins
    svg << "<g transform=\"translate(" << xOffset << "," << yOffset << ")\" "
        << "inkscape:label=\"Centered Content with Margin\" "
        << "inkscape:groupmode=\"layer\">\n";
   
   // Optional: Add a background rectangle to visualize the content area
   svg << "  <!-- Content area outline for debugging -->\n";
   svg << "  <rect x=\"0\" y=\"0\" width=\"" << contentWidth 
       << "\" height=\"" << contentHeight 
       << "\" fill=\"none\" stroke=\"#cccccc\" stroke-width=\"0.5\" stroke-dasharray=\"2,2\" />\n";

   // Start optimization timer
   QElapsedTimer optimizeTimer;
   optimizeTimer.start();

   // Process based on color mode
   if (params.colorMode == ColorMode::Monochrome) {
       // Process monochrome paths
       std::vector<std::vector<PathSegment>>& allPaths = context.paths;
       std::cout << "Monochrome paths generated: " << allPaths.size() << std::endl;
       
       // Apply path reordering for minimal travel if requested
       if (params.minimizeTravel && !allPaths.empty()) {
           QElapsedTimer travelTimer;
           travelTimer.start();
           
           std::cout << "Reordering paths to minimize travel distance..." << std::endl;
           allPaths = reorderPathsForMinimalTravel(allPaths);
           
           std::cout << "Path reordering complete. Time: " << travelTimer.elapsed() << " ms" << std::endl;
       }
       
       // Write optimized or unoptimized paths
       if (params.optimize) {
           std::cout << "Optimizing paths with " << numThreads << " threads..." << std::endl;
           auto optimizedPaths = optimizePaths(allPaths, strokeWidth, numThreads);
           qint64 optimizeTime = optimizeTimer.elapsed();
           std::cout << "Path optimization complete. Time: " << optimizeTime << " ms" << std::endl;
           std::cout << "Original paths: " << allPaths.size() << ", Optimized paths: " << optimizedPaths.size() << std::endl;
           writeOptimizedSvg(svg, optimizedPaths, true, "black");
       } else {
           // Code for non-optimized path output - always using Inkscape optimized format
           // Group all outline segments
           svg << "  <g id=\"outlines\" inkscape:groupmode=\"layer\" inkscape:label=\"Outlines\">\n";
           for (const auto& pixelPaths : allPaths) {
               for (size_t i = 0; i < std::min<size_t>(4, pixelPaths.size()); i++) {
                   const auto& segment = pixelPaths[i];
                   svg << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                       << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                       << "\" stroke=\"black\" stroke-width=\"" << strokeWidth 
                       << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
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
               svg << "  <g id=\"fills\" inkscape:groupmode=\"layer\" inkscape:label=\"Fills\">\n";
               for (const auto& pixelPaths : allPaths) {
                   if (pixelPaths.size() > 4) {
                       for (size_t i = 4; i < pixelPaths.size(); i++) {
                           const auto& segment = pixelPaths[i];
                           svg << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                               << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                               << "\" stroke=\"black\" stroke-width=\"" << strokeWidth 
                               << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                       }
                   }
               }
               svg << "  </g>\n";
           }
       }
   } else {
       // Handle color processing with enhancements
       std::map<ColorInfo, std::vector<std::vector<PathSegment>>>& pathsByColor = context.colorPaths;
       std::cout << "Color mode - unique colors found: " << pathsByColor.size() << std::endl;
       
       // Apply color path reordering for minimal travel if requested
       if (params.minimizeTravel) {
           QElapsedTimer travelTimer;
           travelTimer.start();
           
           std::cout << "Reordering color paths to minimize travel distance..." << std::endl;
           
           for (auto& pair : pathsByColor) {
               pair.second = reorderPathsForMinimalTravel(pair.second);
           }
           
           std::cout << "Color path reordering complete. Time: " << travelTimer.elapsed() << " ms" << std::endl;
       }
       
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
       
       // Write all color layers to SVG - always using Inkscape optimized format
       writeColorLayersSvg(svg, optimizedPathsByColor, true);
   }

   // Close the transform group
   svg << "</g>\n";
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
   std::cout << "Margin: " << params.margin << " " 
             << (params.units == Units::Inches ? "inches" : "mm") << "\n";
   std::cout << "Total document size: " << (totalWidth/72) << "x" << (totalHeight/72) << " inches\n";
   std::cout << "Optimization: " << (params.optimize ? "Enabled" : "Disabled") << "\n";
   std::cout << "Minimize travel: " << (params.minimizeTravel ? "Enabled" : "Disabled") << "\n";
   std::cout << "Simplify paths: " << (params.simplifyPaths ? "Enabled" : "Disabled") << "\n";
   std::cout << "========================================\n";
}