// colorhandler.h
#pragma once
#include "converter.h"
#include <vector>
#include <map>
#include <QRgb>
#include <QString>

/**
 * @brief Class for handling color-related operations
 */
class ColorHandler {
public:
    /**
     * @brief Convert RGB color to SVG color string
     * @param color The RGB color value
     * @return SVG color string in hex or rgba format
     */
    QString rgbToSvgColor(QRgb color);
    
    /**
     * @brief Convert QRgb color to grayscale luminosity level
     * @param color RGB color
     * @return Luminosity value (0-255)
     */
    int rgbToLuminosity(QRgb color);
    
    /**
     * @brief Sort colors by luminosity for optimal pen order
     * @param colorMap Map of colors to paths
     * @return Vector of colors sorted by luminosity (dark to light)
     */
    std::vector<ColorInfo> sortColorsByLuminosity(
        const std::map<ColorInfo, std::vector<std::vector<PathSegment>>>& colorMap);
    
    /**
     * @brief Group similar colors to reduce pen changes
     * @param colorPaths Map of colors to paths
     * @param threshold Similarity threshold
     * @return Grouped color to paths map
     */
    std::map<ColorInfo, std::vector<std::vector<PathSegment>>> groupSimilarColors(
        const std::map<ColorInfo, std::vector<std::vector<PathSegment>>>& colorPaths,
        double threshold);
};