#include "converter.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <QImage>

double BitmapConverter::mmToInches(double mm) {
   return mm / 25.4;
}

double BitmapConverter::inchesToMm(double inches) {
   return inches * 25.4;
}

double BitmapConverter::getPixelSizeFromDocSize(double docWidth, double docHeight, int pixelWidth, int pixelHeight) {
   double widthPixelSize = docWidth / pixelWidth;
   double heightPixelSize = docHeight / pixelHeight;
   return std::min(widthPixelSize, heightPixelSize);
}

std::vector<std::array<double, 4>> BitmapConverter::createZigzagPath(
   double x, double y, double width, double height, double strokeWidth, double angle) {
   
   // Convert angle to radians and calculate spacing
   double angleRad = angle * M_PI / 180.0;
   double spacing = strokeWidth;
   
   // Calculate lines needed to fill space
   double diagonal = std::sqrt(width*width + height*height);
   int numLines = static_cast<int>(diagonal / spacing) + 1;
   
   // Calculate perpendicular vector for line offsets
   double perpX = -std::sin(angleRad);
   double perpY = std::cos(angleRad);
   
   std::vector<std::array<double, 4>> paths;
   for (int i = 0; i < numLines; ++i) {
       double offset = i * spacing;
       double startX = x + offset * perpX;
       double startY = y + offset * perpY;
       
       double endX = startX + width * std::cos(angleRad);
       double endY = startY + width * std::sin(angleRad);
       
       auto points = clipLineToRect(startX, startY, endX, endY, x, y, width, height);
       if (points) {
           paths.push_back(*points);
       }
   }
   return paths;
}

std::optional<std::array<double, 4>> BitmapConverter::clipLineToRect(
   double x1, double y1, double x2, double y2, 
   double rx, double ry, double rw, double rh) {
   
   if (!lineRectIntersect(x1, y1, x2, y2, rx, ry, rw, rh)) {
       return std::nullopt;
   }

   // Clip line to rectangle boundaries using Cohen-Sutherland algorithm
   auto clip = [](double& x1, double& y1, double& x2, double& y2,
                 double rx, double ry, double rw, double rh) 
                 -> std::optional<std::array<double, 4>> {
       double tMin = 0, tMax = 1;
       double dx = x2 - x1;
       double dy = y2 - y1;

       auto clipT = [](double denom, double num, double& tMin, double& tMax) {
           if (std::abs(denom) < 1e-10) return denom >= 0;
           double t = num / denom;
           if (denom > 0) {
               tMax = std::min(tMax, t);
           } else {
               tMin = std::max(tMin, t);
           }
           return tMax >= tMin;
       };

       if (clipT(dx, rx - x1, tMin, tMax) &&
           clipT(-dx, x1 - (rx + rw), tMin, tMax) &&
           clipT(dy, ry - y1, tMin, tMax) &&
           clipT(-dy, y1 - (ry + rh), tMin, tMax)) {
           
           double newX1 = x1 + tMin * dx;
           double newY1 = y1 + tMin * dy;
           double newX2 = x1 + tMax * dx;
           double newY2 = y1 + tMax * dy;
           return std::array<double, 4>{newX1, newY1, newX2, newY2};
       }
       return std::nullopt;
   };

   return clip(x1, y1, x2, y2, rx, ry, rw, rh);
}

bool BitmapConverter::lineRectIntersect(
   double x1, double y1, double x2, double y2,
   double rx, double ry, double rw, double rh) {
   return !(std::max(x1, x2) < rx || std::min(x1, x2) > rx + rw ||
            std::max(y1, y2) < ry || std::min(y1, y2) > ry + rh);
}

void BitmapConverter::convert(
   const std::string& inputFile, 
   const std::string& outputFile,
   const ConverterParams& params) {
   
   // Load and convert image to 1-bit
   QImage img(QString::fromStdString(inputFile));
   if (img.isNull()) {
       throw std::runtime_error("Failed to load image");
   }
   img = img.convertToFormat(QImage::Format_Mono);
   
   int width = img.width();
   int height = img.height();

   // Calculate pixel size in mm based on input parameters
   double pixelSizeMm;
   if (params.units == Units::Inches) {
       pixelSizeMm = params.pixelSize ? inchesToMm(params.pixelSize) : 
                     getPixelSizeFromDocSize(inchesToMm(params.docWidth), 
                                          inchesToMm(params.docHeight), 
                                          width, height);
   } else {
       pixelSizeMm = params.pixelSize ? params.pixelSize : 
                     getPixelSizeFromDocSize(params.docWidth, params.docHeight, 
                                          width, height);
   }

   // Convert measurements to points (72dpi)
   double pixelSize = (pixelSizeMm / 25.4) * 72;
   double strokeWidth = ((params.units == Units::Inches ? 
                         inchesToMm(params.strokeWidth) : 
                         params.strokeWidth) / 25.4) * 72;
   double inset = strokeWidth / 2;

   // Create output file
   std::ofstream svg(outputFile);
   if (!svg) {
       throw std::runtime_error("Failed to create output file");
   }

   // Write SVG header
   svg << "<svg width=\"" << width * pixelSize << "px\" "
       << "height=\"" << height * pixelSize << "px\" "
       << "viewBox=\"0 0 " << width * pixelSize << " " << height * pixelSize << "\" "
       << "xmlns=\"http://www.w3.org/2000/svg\">\n";

   // Process each pixel
   for (int y = 0; y < height; ++y) {
       for (int x = 0; x < width; ++x) {
           if (img.pixelColor(x, y).value() == 0) {  // Black pixel
               double px = x * pixelSize + inset;
               double py = y * pixelSize + inset;
               double pwidth = pixelSize - strokeWidth;
               double pheight = pixelSize - strokeWidth;

               // Draw pixel outline
               svg << "  <rect x=\"" << px << "\" y=\"" << py
                   << "\" width=\"" << pwidth << "\" height=\"" << pheight
                   << "\" fill=\"none\" stroke=\"black\" "
                   << "stroke-width=\"" << strokeWidth << "\"/>\n";

               // Add zigzag pattern if specified
               if (params.fillType == FillType::Zigzag) {
                   auto paths = createZigzagPath(px, py, pwidth, pheight, 
                                               strokeWidth, params.angle);
                   for (const auto& p : paths) {
                       svg << "  <line x1=\"" << p[0] << "\" y1=\"" << p[1]
                           << "\" x2=\"" << p[2] << "\" y2=\"" << p[3]
                           << "\" stroke=\"black\" "
                           << "stroke-width=\"" << strokeWidth << "\"/>\n";
                   }
               }
           }
       }
   }

   svg << "</svg>\n";
}