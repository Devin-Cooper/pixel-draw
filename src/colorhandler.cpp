#include "colorhandler.h"
#include <algorithm>
#include <cmath>
#include <QColor>

// Convert RGB color to SVG color string - Updated for transparency
QString ColorHandler::rgbToSvgColor(QRgb color) {
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

// Convert QRgb color to luminosity value
int ColorHandler::rgbToLuminosity(QRgb color) {
    // Calculate luminosity using standard formula (perceived brightness)
    // Y = 0.299*R + 0.587*G + 0.114*B
    return static_cast<int>(0.299 * qRed(color) + 0.587 * qGreen(color) + 0.114 * qBlue(color));
}

// Sort colors by luminosity (darkest to lightest)
std::vector<ColorInfo> ColorHandler::sortColorsByLuminosity(
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
std::map<ColorInfo, std::vector<std::vector<PathSegment>>> ColorHandler::groupSimilarColors(
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