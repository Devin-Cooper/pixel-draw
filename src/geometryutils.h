// geometryutils.h
#pragma once
#include "converter.h"
#include <array>
#include <optional>
#include <vector>

/**
 * @brief Class providing geometric operations for path manipulation
 */
class GeometryUtils {
public:
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
    std::vector<std::array<double, 4>> createZigzagPath(
        double x, double y, double width, double height, 
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
    std::optional<std::array<double, 4>> clipLineToRect(
        double x1, double y1, double x2, double y2, 
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
    bool lineRectIntersect(
        double x1, double y1, double x2, double y2, 
        double rx, double ry, double rw, double rh);
};