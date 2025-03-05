// pathoptimizer.h
#pragma once
#include "converter.h"
#include <vector>
#include <map>
#include <QString>

/**
 * @brief Class for optimizing paths in the SVG output
 */
class PathOptimizer {
public:
    /**
     * @brief Optimize paths for continuous zigzag pattern with minimal pen lifts
     * @param allPaths Vector of paths to optimize
     * @param strokeWidth Width of the stroke
     * @param numThreads Number of threads to use for optimization
     * @return Vector of optimized paths
     */
    std::vector<OptimizedPath> optimizePaths(
        const std::vector<std::vector<PathSegment>>& allPaths, 
        double strokeWidth, int numThreads = 1);
    
    /**
     * @brief Optimize a chunk of paths in parallel
     * @param chunkPaths Vector of paths to optimize
     * @param strokeWidth Stroke width
     * @param chunkIndex Index of this chunk
     * @param totalChunks Total number of chunks
     * @return Vector of optimized paths
     */
    std::vector<OptimizedPath> optimizePathChunk(
        const std::vector<std::vector<PathSegment>>& chunkPaths, 
        double strokeWidth, int chunkIndex, int totalChunks);
    
    /**
     * @brief Optimize paths for a single color
     * @param colorPaths Vector of paths for this color
     * @param strokeWidth Stroke width
     * @param colorName Color name for logging
     * @return Vector of optimized paths
     */
    std::vector<OptimizedPath> optimizeColorPaths(
        const std::vector<std::vector<PathSegment>>& colorPaths, 
        double strokeWidth, const QString& colorName);
    
    /**
     * @brief Reorder paths to minimize travel distance using nearest neighbor
     * @param paths Vector of path segments to reorder
     * @return Reordered vector of path segments
     */
    std::vector<std::vector<PathSegment>> reorderPathsForMinimalTravel(
        const std::vector<std::vector<PathSegment>>& paths);
    
    /**
     * @brief Format a path for Inkscape using SVG path syntax
     * @param segments Vector of path segments
     * @param strokeWidth Width of the stroke
     * @return SVG path element as string
     */
    std::string formatPathForInkscape(const std::vector<PathSegment>& segments, double strokeWidth);

private:
    /**
     * @brief Check if two line segments can be connected
     * @param s1 First segment
     * @param s2 Second segment
     * @return True if the segments can be connected
     */
    bool canConnect(const PathSegment& s1, const PathSegment& s2);
    
    /**
     * @brief Calculate distance between two paths
     * @param path1 First path
     * @param path2 Second path
     * @return Distance between end of path1 and start of path2
     */
    double calculatePathDistance(
        const std::vector<PathSegment>& path1, 
        const std::vector<PathSegment>& path2);
    
    /**
     * @brief Simplify path using Douglas-Peucker algorithm
     * @param path Original path segments
     * @param tolerance Simplification tolerance
     * @return Simplified path segments
     */
    std::vector<PathSegment> simplifyPath(
        const std::vector<PathSegment>& path, double tolerance);
    
    /**
     * @brief Convert path segments to point sequence for simplification
     * @param segments Path segments to convert
     * @return Vector of points (x,y pairs)
     */
    std::vector<std::pair<double, double>> segmentsToPoints(
        const std::vector<PathSegment>& segments);
    
    /**
     * @brief Convert point sequence back to path segments
     * @param points Vector of points
     * @return Path segments
     */
    std::vector<PathSegment> pointsToSegments(
        const std::vector<std::pair<double, double>>& points);
};