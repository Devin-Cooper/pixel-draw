// mainwindow.cpp
#include "mainwindow.h"
#include "../resources/ui_mainwindow.h"
#include "converter.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QRegularExpression>
#include <QButtonGroup>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    
    ui->pixelSizeRadio->setChecked(true);
    ui->mmRadio->setChecked(true);
    ui->solidRadio->setChecked(true);
    ui->optimizeCheckbox->setChecked(true);

    sizeTypeGroup = new QButtonGroup(this);
    sizeTypeGroup->addButton(ui->pixelSizeRadio);
    sizeTypeGroup->addButton(ui->docSizeRadio);

    unitsGroup = new QButtonGroup(this);
    unitsGroup->addButton(ui->mmRadio);
    unitsGroup->addButton(ui->inchesRadio);

    fillTypeGroup = new QButtonGroup(this);
    fillTypeGroup->addButton(ui->solidRadio);
    fillTypeGroup->addButton(ui->zigzagRadio);

    ui->pixelSizeInput->setValue(1.0);
    ui->strokeWidthInput->setValue(0.1);
    ui->angleInput->setValue(45.0);
    ui->docWidthInput->setValue(100.0);
    ui->docHeightInput->setValue(100.0);

    connect(ui->browseButton, &QPushButton::clicked, this, &MainWindow::browseFile);
    connect(ui->convertButton, &QPushButton::clicked, this, &MainWindow::convert);
    connect(sizeTypeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::updateSizeInputs);
    connect(fillTypeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::updateAngleInput);

    updateSizeInputs();
    updateAngleInput();
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::browseFile() {
    inputFile = QFileDialog::getOpenFileName(this, "Select Bitmap", "", "Images (*.bmp *.gif)");
    if (!inputFile.isEmpty()) {
        ui->filePathLabel->setText(inputFile);
    }
}

void MainWindow::convert() {
    try {
        if (inputFile.isEmpty()) {
            throw std::runtime_error("Please select an input file");
        }

        ConverterParams params;
        params.units = unitsGroup->checkedButton()->text() == "mm" ? Units::MM : Units::Inches;
        params.fillType = fillTypeGroup->checkedButton()->text() == "Zigzag" ? FillType::Zigzag : FillType::Solid;
        params.strokeWidth = ui->strokeWidthInput->value();
        params.optimize = ui->optimizeCheckbox->isChecked();

        if (sizeTypeGroup->checkedButton()->text() == "Pixel Size") {
            params.pixelSize = ui->pixelSizeInput->value();
        } else {
            params.docWidth = ui->docWidthInput->value();
            params.docHeight = ui->docHeightInput->value();
        }

        if (params.fillType == FillType::Zigzag) {
            params.angle = ui->angleInput->value();
        }

        QString outputFile = inputFile;
        outputFile.replace(QRegularExpression("\\..[^.]*$"), ".svg");
        
        BitmapConverter converter;
        converter.convert(inputFile.toStdString(), outputFile.toStdString(), params);
        
        QMessageBox::information(this, "Success", "SVG saved as " + outputFile);
    }
    catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", e.what());
    }
}

void MainWindow::updateSizeInputs() {
    QAbstractButton* button = sizeTypeGroup->checkedButton();
    if (button) {
        bool isPixelSize = button->text() == "Pixel Size";
        ui->pixelSizeWidget->setVisible(isPixelSize);
        ui->documentSizeWidget->setVisible(!isPixelSize);
    }
}

void MainWindow::updateAngleInput() {
    QAbstractButton* button = fillTypeGroup->checkedButton();
    if (button) {
        ui->angleWidget->setVisible(button->text() == "Zigzag");
    }
}