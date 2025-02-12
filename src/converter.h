#pragma once
#include <string>
#include <vector>
#include <array>
#include <optional>

enum class Units { MM, Inches };
enum class FillType { Solid, Zigzag };

struct ConverterParams {
    Units units;
    FillType fillType;
    double strokeWidth;
    double pixelSize;
    double docWidth;
    double docHeight;
    double angle;
};

class BitmapConverter {
public:
    void convert(const std::string& inputFile, const std::string& outputFile, const ConverterParams& params);

private:
    double mmToInches(double mm);
    double inchesToMm(double inches);
    double getPixelSizeFromDocSize(double docWidth, double docHeight, int pixelWidth, int pixelHeight);
    std::vector<std::array<double, 4>> createZigzagPath(double x, double y, double width, double height, double strokeWidth, double angle);
    std::optional<std::array<double, 4>> clipLineToRect(double x1, double y1, double x2, double y2, double rx, double ry, double rw, double rh);
    bool lineRectIntersect(double x1, double y1, double x2, double y2, double rx, double ry, double rw, double rh);
};