// main.cpp
#include "mainwindow.h"
#include <QApplication>
#include <iostream>

int main(int argc, char *argv[]) {
   QApplication app(argc, argv);
   
   // Set application information
   QApplication::setApplicationName("Bitmap to SVG Converter");
   QApplication::setApplicationVersion("2.0");
   QApplication::setOrganizationName("BitmapConverter");
   
   std::cout << "Starting Bitmap to SVG Converter v2.0" << std::endl;
   std::cout << "Enhanced with optimizations for plotting and Inkscape compatibility" << std::endl;
   
   MainWindow window;
   window.show();
   
   return app.exec();
}