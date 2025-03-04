#include "pathoptimizer.h"
#include <QElapsedTimer>
#include <iostream>
#include <sstream>
#include <future>
#include <algorithm>
#include <limits>
#include <cmath>
#include <functional>
#include <thread>

// Check if line segments can connect
bool PathOptimizer::canConnect(const PathSegment& s1, const PathSegment& s2) {
   const double EPSILON = 1e-10;
   return (std::abs(s1.x2 - s2.x1) < EPSILON && std::abs(s1.y2 - s2.y1) < EPSILON) ||
          (std::abs(s1.x2 - s2.x2) < EPSILON && std::abs(s1.y2 - s2.y2) < EPSILON) ||
          (std::abs(s1.x1 - s2.x1) < EPSILON && std::abs(s1.y1 - s2.y1) < EPSILON) ||
          (std::abs(s1.x1 - s2.x2) < EPSILON && std::abs(s1.y1 - s2.y2) < EPSILON);
}

// Optimize a chunk of paths for multi-threaded optimization
std::vector<OptimizedPath> PathOptimizer::optimizePathChunk(
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
std::vector<OptimizedPath> PathOptimizer::optimizeColorPaths(
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

// Original Path reordering implementation - kept for backwards compatibility
std::vector<std::vector<PathSegment>> PathOptimizer::reorderPathsForMinimalTravel(
    const std::vector<std::vector<PathSegment>>& paths) {
    
    return reorderPathsForMinimalTravel(paths, 0);
}

// Improved reorderPathsForMinimalTravel with parallel processing
std::vector<std::vector<PathSegment>> PathOptimizer::reorderPathsForMinimalTravel(
    const std::vector<std::vector<PathSegment>>& paths, int numThreads) {
    
    // Early exit for empty or single path
    if (paths.size() <= 1) return paths;

    // Determine number of threads to use
    if (numThreads <= 0) {
        // Auto-detect threads, but limit based on problem size
        numThreads = std::min(
            static_cast<int>(std::thread::hardware_concurrency()), 
            static_cast<int>(paths.size() / 100 + 1)
        );
        // Ensure at least 1 thread and not more than 8 for reasonable overhead
        numThreads = std::max(1, std::min(numThreads, 8)); 
    }
    
    std::cout << "Path reordering using " << numThreads << " threads for " 
              << paths.size() << " paths" << std::endl;
    
    // For small path counts or single threading, use enhanced algorithm directly
    if (numThreads == 1 || paths.size() < 200) {
        // Find optimized path order
        std::vector<int> bestOrder(paths.size());
        for (int i = 0; i < bestOrder.size(); i++) {
            bestOrder[i] = i;
        }
        
        double totalDistance = calculateTotalDistance(paths, bestOrder);
        
        // Try different starting points with enhanced nearest neighbor + 2-opt
        findBetterPathOrdering(paths, 0, nullptr, nullptr, &bestOrder);
        
        // Apply final 2-opt improvement
        apply2OptImprovement(paths, bestOrder, totalDistance);
        
        // Build result using the optimal order
        std::vector<std::vector<PathSegment>> result;
        result.reserve(paths.size());
        for (int idx : bestOrder) {
            result.push_back(paths[idx]);
        }
        return result;
    }
    
    // Parallel approach for larger path counts
    std::vector<std::thread> threads;
    std::mutex mutex;
    std::vector<int> bestOrder(paths.size());
    double bestDistance = std::numeric_limits<double>::max();
    
    // Initialize with sequential ordering
    for (int i = 0; i < bestOrder.size(); i++) {
        bestOrder[i] = i;
    }
    
    // Distribute starting points among threads
    int pathsPerThread = static_cast<int>(paths.size()) / numThreads;
    for (int t = 0; t < numThreads; t++) {
        int startIdx = t * pathsPerThread;
        threads.push_back(std::thread(&PathOptimizer::findBetterPathOrdering, this,
                                      std::ref(paths), startIdx, &mutex, 
                                      &bestDistance, &bestOrder));
    }
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Build result using the optimal order
    std::vector<std::vector<PathSegment>> result;
    result.reserve(paths.size());
    for (int idx : bestOrder) {
        result.push_back(paths[idx]);
    }
    
    return result;
}

// Find better path ordering with enhanced nearest neighbor + 2-opt improvement
void PathOptimizer::findBetterPathOrdering(
    const std::vector<std::vector<PathSegment>>& paths,
    int startIdx,
    std::mutex* mutex,
    double* sharedBestDistance,
    std::vector<int>* sharedBestOrder) {
    
    std::vector<int> order(paths.size());
    std::vector<bool> used(paths.size(), false);
    
    // Start with the specified index
    order[0] = startIdx;
    used[startIdx] = true;
    
    // Enhanced nearest neighbor algorithm
    // Build a path by selecting the closest unused path at each step
    for (size_t pos = 1; pos < paths.size(); pos++) {
        int lastIdx = order[pos-1];
        const auto& lastPath = paths[lastIdx];
        
        double minDistance = std::numeric_limits<double>::max();
        int bestNextIdx = -1;
        
        // Look ahead by considering not just the closest path
        // but also how that choice affects the next step
        for (size_t i = 0; i < paths.size(); i++) {
            if (!used[i]) {
                // Calculate direct distance to this candidate
                double directDist = calculatePathDistance(lastPath, paths[i]);
                
                // Look ahead coefficient reduces with path count to avoid excessive computation
                double lookAheadCoeff = std::min(1.0, 10.0 / paths.size());
                double lookAheadBonus = 0.0;
                
                // Consider each potential next step from this candidate
                if (lookAheadCoeff > 0.01) {  // Only do lookahead if coefficient is significant
                    double minNextDist = std::numeric_limits<double>::max();
                    for (size_t j = 0; j < paths.size(); j++) {
                        if (!used[j] && j != i) {
                            double nextDist = calculatePathDistance(paths[i], paths[j]);
                            minNextDist = std::min(minNextDist, nextDist);
                        }
                    }
                    // Add a bonus for candidates that are close to other unused paths
                    lookAheadBonus = lookAheadCoeff * minNextDist;
                }
                
                // Combine direct distance with lookahead bonus
                double combinedScore = directDist + lookAheadBonus;
                
                if (combinedScore < minDistance) {
                    minDistance = combinedScore;
                    bestNextIdx = i;
                }
            }
        }
        
        if (bestNextIdx >= 0) {
            order[pos] = bestNextIdx;
            used[bestNextIdx] = true;
        }
    }
    
    // Calculate total distance for this ordering
    double totalDistance = calculateTotalDistance(paths, order);
    
    // Apply 2-opt improvement to the path ordering
    bool improved = true;
    int iterations = 0;
    int maxIterations = static_cast<int>(paths.size() * 0.5); // Limit iterations based on problem size
    
    while (improved && iterations < maxIterations) {
        improved = apply2OptImprovement(paths, order, totalDistance);
        iterations++;
    }
    
    // If using parallel processing, update shared best solution if better
    if (mutex && sharedBestDistance && sharedBestOrder) {
        std::lock_guard<std::mutex> lock(*mutex);
        if (totalDistance < *sharedBestDistance) {
            *sharedBestDistance = totalDistance;
            *sharedBestOrder = order;
        }
    } else if (sharedBestOrder) {
        // Single-threaded mode - just update the best order
        *sharedBestOrder = order;
    }
}

// Apply 2-opt improvement to path ordering
bool PathOptimizer::apply2OptImprovement(
    const std::vector<std::vector<PathSegment>>& paths,
    std::vector<int>& order,
    double& totalDistance) {
    
    bool improved = false;
    
    // Determine how many paths to check - for large path counts, check a subset
    // to maintain reasonable performance
    int pathsToCheck = static_cast<int>(paths.size());
    if (paths.size() > 500) {
        pathsToCheck = 500 + static_cast<int>(sqrt(paths.size() - 500));
    }
    
    // Try swapping segments to find improvements
    for (int i = 0; i < pathsToCheck - 1; i++) {
        for (int j = i + 1; j < pathsToCheck; j++) {
            // Skip adjacent paths as swapping them wouldn't change the order
            if (j == i + 1) continue;
            
            // Calculate current segment distances
            double d_before_i = (i > 0) ? 
                calculatePathDistance(paths[order[i-1]], paths[order[i]]) : 0;
            double d_i_next = calculatePathDistance(paths[order[i]], paths[order[i+1]]);
            double d_before_j = calculatePathDistance(paths[order[j-1]], paths[order[j]]);
            double d_j_next = (j < paths.size() - 1) ? 
                calculatePathDistance(paths[order[j]], paths[order[j+1]]) : 0;
            
            // Calculate potential new segment distances
            double d_before_j_first = (i > 0) ? 
                calculatePathDistance(paths[order[i-1]], paths[order[j]]) : 0;
            double d_j_i_next = calculatePathDistance(paths[order[j]], paths[order[i+1]]);
            double d_before_i_next = calculatePathDistance(paths[order[j-1]], paths[order[i]]);
            double d_i_j_next = (j < paths.size() - 1) ? 
                calculatePathDistance(paths[order[i]], paths[order[j+1]]) : 0;
            
            // Calculate change in total distance if we swap these paths
            double currentDist = d_before_i + d_i_next + d_before_j + d_j_next;
            double newDist = d_before_j_first + d_j_i_next + d_before_i_next + d_i_j_next;
            
            // If the new arrangement is better, swap the paths
            if (newDist < currentDist) {
                std::swap(order[i], order[j]);
                totalDistance = totalDistance - currentDist + newDist;
                improved = true;
                
                // If we made a significant improvement, exit early and start the next
                // iteration of the outer loop with this improved path
                if ((currentDist - newDist) / currentDist > 0.1) {
                    return true;
                }
            }
        }
    }
    
    return improved;
}

// Calculate total distance for a given path order
double PathOptimizer::calculateTotalDistance(
    const std::vector<std::vector<PathSegment>>& paths,
    const std::vector<int>& order) {
    
    double totalDistance = 0.0;
    
    for (size_t i = 0; i < order.size() - 1; i++) {
        totalDistance += calculatePathDistance(paths[order[i]], paths[order[i+1]]);
    }
    
    return totalDistance;
}

double PathOptimizer::calculatePathDistance(
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

// Path simplification functions
std::vector<PathSegment> PathOptimizer::simplifyPath(
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

std::vector<std::pair<double, double>> PathOptimizer::segmentsToPoints(
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

std::vector<PathSegment> PathOptimizer::pointsToSegments(
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

// Format a path for Inkscape using SVG path commands
std::string PathOptimizer::formatPathForInkscape(const std::vector<PathSegment>& segments, double strokeWidth) {
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

// Optimized optimizePaths function with travel minimization
std::vector<OptimizedPath> PathOptimizer::optimizePaths(
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
            
            auto reorderedSegments = reorderPathsForMinimalTravel(segments, numThreads);
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
                          &PathOptimizer::optimizePathChunk, this,
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
        
        auto reorderedSegments = reorderPathsForMinimalTravel(segments, numThreads);
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