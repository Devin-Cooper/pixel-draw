#include <future>
#include "converter.h"
#include "pathoptimizer.h"
#include "svgwriter.h"
#include "colorhandler.h"
#include "geometryutils.h"
#include <QElapsedTimer>
#include <QDebug>
#include <iostream>
#include <fstream>
#include <thread>
#include <functional>


// Constructor
BitmapConverter::BitmapConverter() {
    m_pathOptimizer = std::make_unique<PathOptimizer>();
    m_svgWriter = std::make_unique<SvgWriter>();
    m_colorHandler = std::make_unique<ColorHandler>();
    m_geometryUtils = std::make_unique<GeometryUtils>();
}

// Destructor
BitmapConverter::~BitmapConverter() {
    // Smart pointers automatically handle cleanup
}

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

// Process a chunk of the image in a separate thread
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
                    double pwidth = std::max(0.001, pixelSize - 2 * inset);
                    double pheight = std::max(0.001, pixelSize - 2 * inset);

                    // Debug - only for the first few pixels
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
                        auto zigzags = m_geometryUtils->createZigzagPath(px, py, pwidth, pheight, strokeWidth, params.angle);
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
                const int whiteThreshold = 245; 
                
                if (r >= whiteThreshold && g >= whiteThreshold && b >= whiteThreshold) {
                    skippedWhitePixels++;
                    continue;
                }
                
                // Create color info
                ColorInfo currentColor;
                currentColor.color = pixelColor;
                currentColor.svgColor = m_colorHandler->rgbToSvgColor(pixelColor);
                
                // Calculate pixel position - ensure positive sizes
                double px = x * pixelSize + inset; 
                double py = y * pixelSize + inset;
                double pwidth = std::max(0.001, pixelSize - 2 * inset);
                double pheight = std::max(0.001, pixelSize - 2 * inset);
                
                // Debug - only for the first few pixels
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
                    auto zigzags = m_geometryUtils->createZigzagPath(px, py, pwidth, pheight, strokeWidth, params.angle);
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

// Main conversion function
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

   // Calculate sizes in mm
   double pixelSizeMm;
   bool useExplicitPixelSize = (params.pixelSize > 0.001);
   
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

   // Determine the total document dimensions
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
       
       context.colorPaths = m_colorHandler->groupSimilarColors(context.colorPaths, params.colorSimilarityThreshold);
       
       std::cout << "After grouping: " << context.colorPaths.size() << " unique colors" << std::endl;
       std::cout << "Color grouping time: " << groupTimer.elapsed() << " ms" << std::endl;
   }
   
   // Start SVG writing timer
   QElapsedTimer svgTimer;
   svgTimer.start();
   
   std::cout << "Writing SVG output..." << std::endl;
   
   // Prepare SVG document properties
   SvgDocumentProperties docProps;
   docProps.totalWidth = totalWidth;
   docProps.totalHeight = totalHeight;
   docProps.contentWidth = contentWidth;
   docProps.contentHeight = contentHeight;
   docProps.marginPoints = marginPoints;
   docProps.strokeWidth = strokeWidth;

   // Open output file
   std::ofstream outputStream(outputFile);
   if (!outputStream) throw std::runtime_error("Failed to create output file");

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
           // Use the enhanced algorithm with multi-threading
           allPaths = m_pathOptimizer->reorderPathsForMinimalTravel(allPaths, numThreads);
           
           std::cout << "Path reordering complete. Time: " << travelTimer.elapsed() << " ms" << std::endl;
       }
       
       // Optimize paths if requested
       std::vector<OptimizedPath> optimizedPaths;
       if (params.optimize) {
           QElapsedTimer optimizeTimer;
           optimizeTimer.start();
           
           std::cout << "Optimizing paths with " << numThreads << " threads..." << std::endl;
           optimizedPaths = m_pathOptimizer->optimizePaths(allPaths, strokeWidth, numThreads);
           
           qint64 optimizeTime = optimizeTimer.elapsed();
           std::cout << "Path optimization complete. Time: " << optimizeTime << " ms" << std::endl;
           std::cout << "Original paths: " << allPaths.size() << ", Optimized paths: " << optimizedPaths.size() << std::endl;
       } else {
           // Create simple non-optimized paths
           for (const auto& path : allPaths) {
               OptimizedPath simplePath;
               simplePath.segments = path;
               simplePath.strokeWidth = strokeWidth;
               optimizedPaths.push_back(simplePath);
           }
       }
       
       // Write the SVG
       m_svgWriter->writeSvg(outputStream, optimizedPaths, docProps);
   } else {
       // Handle color processing
       std::map<ColorInfo, std::vector<std::vector<PathSegment>>>& pathsByColor = context.colorPaths;
       std::cout << "Color mode - unique colors found: " << pathsByColor.size() << std::endl;
       
       // Apply color path reordering for minimal travel if requested
       if (params.minimizeTravel) {
           QElapsedTimer travelTimer;
           travelTimer.start();
           
           std::cout << "Reordering color paths to minimize travel distance..." << std::endl;
           
           for (auto& pair : pathsByColor) {
               // Use the enhanced algorithm with multi-threading
               pair.second = m_pathOptimizer->reorderPathsForMinimalTravel(pair.second, numThreads);
           }
           
           std::cout << "Color path reordering complete. Time: " << travelTimer.elapsed() << " ms" << std::endl;
       }
       
       // Optimize paths by color
       std::map<ColorInfo, std::vector<OptimizedPath>> optimizedPathsByColor;
       
       if (params.optimize) {
           QElapsedTimer optimizeTimer;
           optimizeTimer.start();
           
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
                           this->m_pathOptimizer->optimizeColorPaths(pair.second, strokeWidth, pair.first.svgColor)
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
                   optimizedPathsByColor[color] = m_pathOptimizer->optimizePaths(colorPaths, strokeWidth, numThreads);
                   std::cout << "Color paths optimization complete. Time: " << colorOptimizeTimer.elapsed() << " ms" << std::endl;
               }
           }
           
           qint64 optimizeTime = optimizeTimer.elapsed();
           std::cout << "Color path optimization complete. Time: " << optimizeTime << " ms" << std::endl;
       } else {
           // For non-optimized paths, we still create OptimizedPath objects for consistent handling
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
       
       // Write the SVG with color layers
       m_svgWriter->writeColorSvg(outputStream, optimizedPathsByColor, docProps);
   }
   
   qint64 svgWriteTime = svgTimer.elapsed();
   std::cout << "SVG writing complete. Time: " << svgWriteTime << " ms" << std::endl;
   
   qint64 totalTime = totalTimer.elapsed();
   std::cout << "\n========== Conversion Summary ==========\n";
   std::cout << "Total conversion time: " << totalTime << " ms\n";
   std::cout << "- Processing time: " << processingTime << " ms (" 
             << (processingTime * 100 / totalTime) << "%)\n";
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