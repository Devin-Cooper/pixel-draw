#include "geometryutils.h"
#include <cmath>
#include <iostream>

// Generate zigzag pattern for pixel with special case handling for orthogonal angles
std::vector<std::array<double, 4>> GeometryUtils::createZigzagPath(
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

// Clip line to rectangle using Liang-Barsky algorithm
std::optional<std::array<double, 4>> GeometryUtils::clipLineToRect(
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
bool GeometryUtils::lineRectIntersect(
   double x1, double y1, double x2, double y2,
   double rx, double ry, double rw, double rh) {
   return !(std::max(x1, x2) < rx || std::min(x1, x2) > rx + rw ||
            std::max(y1, y2) < ry || std::min(y1, y2) > ry + rh);
}