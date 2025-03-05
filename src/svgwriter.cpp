#include "svgwriter.h"
#include <sstream>
#include <iostream>
#include <algorithm>

// Write SVG header with document properties
void SvgWriter::writeSvgHeader(std::ofstream& outputStream, const SvgDocumentProperties& docProps) {
    // Calculate dimensions in inches for Inkscape compatibility
    double docWidthInches = docProps.totalWidth / 72.0;
    double docHeightInches = docProps.totalHeight / 72.0;
    
    // Write SVG header
    outputStream << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
    outputStream << "<!-- Created with Bitmap Converter (Optimized for Inkscape and efficient plotting) -->\n";
    outputStream << "<svg xmlns:svg=\"http://www.w3.org/2000/svg\" "
                << "xmlns=\"http://www.w3.org/2000/svg\" "
                << "xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
                << "xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\" "
                << "xmlns:sodipodi=\"http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd\" "
                << "width=\"" << docWidthInches << "in\" "
                << "height=\"" << docHeightInches << "in\" "
                << "viewBox=\"0 0 " << docProps.totalWidth << " " << docProps.totalHeight << "\" "
                << "inkscape:version=\"1.2\" "
                << "inkscape:document-units=\"in\" "
                << "version=\"1.1\">\n";

    // Enhanced metadata for Inkscape
    outputStream << "  <sodipodi:namedview "
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
    outputStream << "<metadata>\n";
    outputStream << "  <rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" "
                << "xmlns:dc=\"http://purl.org/dc/elements/1.1/\" "
                << "xmlns:cc=\"http://creativecommons.org/ns#\">\n";
    outputStream << "    <rdf:Description>\n";
    outputStream << "      <dc:format>image/svg+xml</dc:format>\n";
    outputStream << "      <dc:type rdf:resource=\"http://purl.org/dc/dcmitype/StillImage\"/>\n";
    outputStream << "    </rdf:Description>\n";
    outputStream << "  </rdf:RDF>\n";
    outputStream << "</metadata>\n";

    // Calculate centering offsets
    double xOffset = docProps.marginPoints + (docProps.totalWidth - docProps.contentWidth - 2 * docProps.marginPoints) / 2;
    double yOffset = docProps.marginPoints + (docProps.totalHeight - docProps.contentHeight - 2 * docProps.marginPoints) / 2;

    // Create a transform that centers content within the margins
    outputStream << "<g transform=\"translate(" << xOffset << "," << yOffset << ")\" "
                << "inkscape:label=\"Centered Content with Margin\" "
                << "inkscape:groupmode=\"layer\">\n";
   
    // Optional: Add a background rectangle to visualize the content area
    outputStream << "  <!-- Content area outline for debugging -->\n";
    outputStream << "  <rect x=\"0\" y=\"0\" width=\"" << docProps.contentWidth 
                << "\" height=\"" << docProps.contentHeight 
                << "\" fill=\"none\" stroke=\"#cccccc\" stroke-width=\"0.5\" stroke-dasharray=\"2,2\" />\n";
}

// Write SVG footer
void SvgWriter::writeSvgFooter(std::ofstream& outputStream) {
    outputStream << "</g>\n";
    outputStream << "</svg>\n";
}

// Write optimized SVG content
void SvgWriter::writeOptimizedSvgContent(std::ofstream& outputStream, 
                                       const std::vector<OptimizedPath>& optimizedPaths,
                                       const std::string& strokeColor) {
    // Use the specified stroke color, or default to black if not provided
    std::string color = !strokeColor.empty() ? strokeColor : "black";
    
    // Group outlines
    outputStream << "  <g id=\"outlines\" inkscape:groupmode=\"layer\" inkscape:label=\"Outlines\">\n";
    for (const auto& path : optimizedPaths) {
        bool isOutline = path.segments.size() == 4;
        if (isOutline) {
            if (path.segments.empty()) continue;
            
            // Generate SVG path
            outputStream << "    <path d=\"M " << path.segments[0].x1 << "," << path.segments[0].y1;
    
            // Add line segments
            for (const auto& seg : path.segments) {
                outputStream << " L " << seg.x2 << "," << seg.y2;
            }
            
            // Close the path attributes
            outputStream << "\" fill=\"none\" stroke=\"" << color 
                << "\" stroke-width=\"" << path.strokeWidth 
                << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
        }
    }
    outputStream << "  </g>\n";
    
    // Group zigzag fills (if any)
    bool hasZigzags = false;
    for (const auto& path : optimizedPaths) {
        if (path.segments.size() > 4) {
            hasZigzags = true;
            break;
        }
    }
    
    if (hasZigzags) {
        outputStream << "  <g id=\"fills\" inkscape:groupmode=\"layer\" inkscape:label=\"Fills\">\n";
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
                    outputStream << "    <polyline points=\"" << path.segments[0].x1 << "," << path.segments[0].y1;
                    
                    for (const auto& segment : path.segments) {
                        outputStream << " " << segment.x2 << "," << segment.y2;
                    }
                    outputStream << "\" fill=\"none\" stroke=\"" << color 
                        << "\" stroke-width=\"" << path.strokeWidth
                        << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                } else {
                    // For non-continuous zigzag, use a polyline for each segment
                    for (const auto& segment : path.segments) {
                        outputStream << "    <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                            << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                            << "\" stroke=\"" << color 
                            << "\" stroke-width=\"" << path.strokeWidth
                            << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                    }
                }
            }
        }
        outputStream << "  </g>\n";
    }
}

// Write optimized SVG content with color layers
void SvgWriter::writeColorLayersSvgContent(std::ofstream& outputStream, 
                                        const std::map<ColorInfo, std::vector<OptimizedPath>>& colorPathsMap) {
    std::cout << "Writing SVG with " << colorPathsMap.size() << " color layers..." << std::endl;
    
    // Create a vector of colors for ordering
    std::vector<ColorInfo> colorOrder;
    for (const auto& pair : colorPathsMap) {
        colorOrder.push_back(pair.first);
    }
    
    // Sort colors by luminosity (darkest to lightest) for optimal plotting
    std::sort(colorOrder.begin(), colorOrder.end(), 
              [](const ColorInfo& a, const ColorInfo& b) {
        // Calculate luminosity using standard formula (perceived brightness)
        // Y = 0.299*R + 0.587*G + 0.114*B
        int lumA = static_cast<int>(0.299 * qRed(a.color) + 0.587 * qGreen(a.color) + 0.114 * qBlue(a.color));
        int lumB = static_cast<int>(0.299 * qRed(b.color) + 0.587 * qGreen(b.color) + 0.114 * qBlue(b.color));
        return lumA < lumB;
    });
    
    // Write each color as a separate layer in order of luminosity
    for (const auto& colorInfo : colorOrder) {
        const auto& paths = colorPathsMap.at(colorInfo);
        
        // Create a layer group for this color with Inkscape layer attributes
        outputStream << "  <g id=\"" << colorInfo.layerId.toStdString() << "\" "
            << "inkscape:groupmode=\"layer\" "
            << "inkscape:label=\"" << colorInfo.layerId.toStdString() << "\" "
            << "style=\"display:inline\" "
            << "fill=\"none\" stroke=\"" << colorInfo.svgColor.toStdString() << "\">\n";
        
        // Group outlines
        bool hasOutlines = false;
        for (const auto& path : paths) {
            if (path.segments.size() == 4) {
                hasOutlines = true;
                break;
            }
        }
        
        if (hasOutlines) {
            outputStream << "    <g id=\"" << colorInfo.layerId.toStdString() << "_outlines\">\n";
            for (const auto& path : paths) {
                if (path.segments.size() == 4) {
                    // Check if path has segments
                    if (path.segments.empty()) continue;
                    
                    // Generate SVG path
                    outputStream << "      <path d=\"M " << path.segments[0].x1 << "," << path.segments[0].y1;
            
                    // Add line segments
                    for (const auto& seg : path.segments) {
                        outputStream << " L " << seg.x2 << "," << seg.y2;
                    }
                    
                    // Close the path attributes with explicit color
                    outputStream << "\" fill=\"none\" stroke=\"" << colorInfo.svgColor.toStdString() 
                        << "\" stroke-width=\"" << path.strokeWidth 
                        << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                }
            }
            outputStream << "    </g>\n";
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
            outputStream << "    <g id=\"" << colorInfo.layerId.toStdString() << "_fills\">\n";
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
                        outputStream << "      <polyline points=\"" << path.segments[0].x1 << "," << path.segments[0].y1;
                        for (const auto& segment : path.segments) {
                            outputStream << " " << segment.x2 << "," << segment.y2;
                        }
                        outputStream << "\" fill=\"none\" stroke=\"" << colorInfo.svgColor.toStdString() 
                            << "\" stroke-width=\"" << path.strokeWidth 
                            << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                    } else {
                        // For non-continuous zigzag, use individual lines
                        for (const auto& segment : path.segments) {
                            outputStream << "      <line x1=\"" << segment.x1 << "\" y1=\"" << segment.y1
                                << "\" x2=\"" << segment.x2 << "\" y2=\"" << segment.y2
                                << "\" stroke=\"" << colorInfo.svgColor.toStdString() 
                                << "\" stroke-width=\"" << path.strokeWidth 
                                << "\" style=\"vector-effect:non-scaling-stroke\" />\n";
                        }
                    }
                }
            }
            outputStream << "    </g>\n";
        }
        
        outputStream << "  </g>\n";
    }
    
    std::cout << "Finished writing color layers" << std::endl;
}

// Write SVG with optimizations for Inkscape
void SvgWriter::writeSvg(std::ofstream& outputStream, 
                       const std::vector<OptimizedPath>& optimizedPaths, 
                       const SvgDocumentProperties& docProps,
                       const std::string& strokeColor) {
    // Write SVG header
    writeSvgHeader(outputStream, docProps);
    
    // Write the optimized SVG content
    writeOptimizedSvgContent(outputStream, optimizedPaths, strokeColor);
    
    // Write SVG footer
    writeSvgFooter(outputStream);
}

// Write SVG with color layers
void SvgWriter::writeColorSvg(std::ofstream& outputStream, 
                            const std::map<ColorInfo, std::vector<OptimizedPath>>& colorPathsMap,
                            const SvgDocumentProperties& docProps) {
    // Write SVG header
    writeSvgHeader(outputStream, docProps);
    
    // Write the color layers SVG content
    writeColorLayersSvgContent(outputStream, colorPathsMap);
    
    // Write SVG footer
    writeSvgFooter(outputStream);
}