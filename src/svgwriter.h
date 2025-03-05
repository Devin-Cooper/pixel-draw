// svgwriter.h
#pragma once
#include "converter.h"
#include <string>
#include <fstream>
#include <map>

/**
 * @brief Document properties for SVG generation
 */
struct SvgDocumentProperties {
    double totalWidth;       ///< Total document width in points
    double totalHeight;      ///< Total document height in points
    double contentWidth;     ///< Content width in points
    double contentHeight;    ///< Content height in points
    double marginPoints;     ///< Margin size in points
    double strokeWidth;      ///< Stroke width in points
};

/**
 * @brief Class for generating SVG output from optimized paths
 */
class SvgWriter {
public:
    /**
     * @brief Write SVG with optimizations for Inkscape
     * @param outputStream Output stream to write SVG
     * @param optimizedPaths Vector of optimized paths
     * @param docProps Document properties
     * @param strokeColor Optional stroke color to use (defaults to black if empty)
     */
    void writeSvg(std::ofstream& outputStream, 
                 const std::vector<OptimizedPath>& optimizedPaths, 
                 const SvgDocumentProperties& docProps,
                 const std::string& strokeColor = "");
    
    /**
     * @brief Write SVG with color layers
     * @param outputStream Output stream to write SVG
     * @param colorPathsMap Map of color info to optimized paths
     * @param docProps Document properties
     */
    void writeColorSvg(std::ofstream& outputStream, 
                     const std::map<ColorInfo, std::vector<OptimizedPath>>& colorPathsMap,
                     const SvgDocumentProperties& docProps);

private:
    /**
     * @brief Write SVG header with document properties
     * @param outputStream Output stream to write SVG
     * @param docProps Document properties
     */
    void writeSvgHeader(std::ofstream& outputStream, const SvgDocumentProperties& docProps);
    
    /**
     * @brief Write SVG footer
     * @param outputStream Output stream to write SVG
     */
    void writeSvgFooter(std::ofstream& outputStream);
    
    /**
     * @brief Write optimized SVG content
     * @param outputStream Output stream to write SVG
     * @param optimizedPaths Vector of optimized paths
     * @param strokeColor Optional stroke color to use (defaults to black if empty)
     */
    void writeOptimizedSvgContent(std::ofstream& outputStream, 
                                 const std::vector<OptimizedPath>& optimizedPaths,
                                 const std::string& strokeColor = "");
    
    /**
     * @brief Write optimized SVG content with color layers
     * @param outputStream Output stream to write SVG
     * @param colorPathsMap Map of color info to optimized paths
     */
    void writeColorLayersSvgContent(std::ofstream& outputStream, 
                                  const std::map<ColorInfo, std::vector<OptimizedPath>>& colorPathsMap);
};