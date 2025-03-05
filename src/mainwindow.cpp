// mainwindow.cpp
#include "mainwindow.h"
#include "../resources/ui_mainwindow.h"
#include "converter.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QRegularExpression>
#include <QButtonGroup>
#include <QCloseEvent>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <iostream>

/**
 * @brief Constructor for MainWindow
 * 
 * Sets up the UI, initializes button groups, connects signals/slots,
 * and loads previously saved settings.
 *
 * @param parent Optional parent widget
 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow), 
    settings("BitmapConverter", "BitmapConverterApp") {
    // Setup UI from form
    ui->setupUi(this);
    
    // Create and configure button groups for radio buttons
    sizeTypeGroup = new QButtonGroup(this);
    sizeTypeGroup->addButton(ui->pixelSizeRadio);
    sizeTypeGroup->addButton(ui->docSizeRadio);

    unitsGroup = new QButtonGroup(this);
    unitsGroup->addButton(ui->mmRadio);
    unitsGroup->addButton(ui->inchesRadio);

    fillTypeGroup = new QButtonGroup(this);
    fillTypeGroup->addButton(ui->solidRadio);
    fillTypeGroup->addButton(ui->zigzagRadio);
    
    // Add color mode button group
    colorModeGroup = new QButtonGroup(this);
    colorModeGroup->addButton(ui->monochromeRadio);
    colorModeGroup->addButton(ui->colorRadio);

    // Create a performance settings group box
    QGroupBox *performanceGroupBox = new QGroupBox("Performance Settings", this);
    QVBoxLayout *performanceLayout = new QVBoxLayout(performanceGroupBox);
    
    // Create thread count widget
    QWidget *threadWidget = new QWidget(this);
    QHBoxLayout *threadLayout = new QHBoxLayout(threadWidget);
    threadLayout->setContentsMargins(0, 0, 0, 0);
    
    threadCountLabel = new QLabel("Thread Count (0 = Auto):", threadWidget);
    threadLayout->addWidget(threadCountLabel);
    
    threadCountSpinBox = new QSpinBox(threadWidget);
    threadCountSpinBox->setMinimum(0);
    threadCountSpinBox->setMaximum(32);
    threadCountSpinBox->setValue(0); // Default to auto
    threadCountSpinBox->setToolTip("Number of threads to use (0 = automatic based on CPU cores)");
    threadLayout->addWidget(threadCountSpinBox);
    
    // Add thread widget to the performance layout
    performanceLayout->addWidget(threadWidget);
    
    // Create enhanced optimization settings group
    QGroupBox *enhancedOptimizationGroupBox = new QGroupBox("Enhanced Optimization", this);
    QVBoxLayout *enhancedOptimizationLayout = new QVBoxLayout(enhancedOptimizationGroupBox);
    
    // Create minimize travel checkbox
    minimizeTravelCheckbox = new QCheckBox("Minimize Travel Distance", this);
    minimizeTravelCheckbox->setToolTip("Optimize the order of paths to minimize pen travel");
    minimizeTravelCheckbox->setChecked(true);
    enhancedOptimizationLayout->addWidget(minimizeTravelCheckbox);
    
    // Create path simplification controls
    simplifyPathsCheckbox = new QCheckBox("Simplify Paths", this);
    simplifyPathsCheckbox->setToolTip("Reduce the number of points in paths while preserving appearance");
    simplifyPathsCheckbox->setChecked(true);
    enhancedOptimizationLayout->addWidget(simplifyPathsCheckbox);
    
    QWidget *simplifyToleranceWidget = new QWidget(this);
    QHBoxLayout *simplifyToleranceLayout = new QHBoxLayout(simplifyToleranceWidget);
    simplifyToleranceLayout->setContentsMargins(20, 0, 0, 0);
    
    simplifyToleranceLabel = new QLabel("Simplification Tolerance:", simplifyToleranceWidget);
    simplifyToleranceLayout->addWidget(simplifyToleranceLabel);
    
    simplifyToleranceInput = new QDoubleSpinBox(simplifyToleranceWidget);
    simplifyToleranceInput->setMinimum(0.01);
    simplifyToleranceInput->setMaximum(5.0);
    simplifyToleranceInput->setSingleStep(0.05);
    simplifyToleranceInput->setValue(0.1);
    simplifyToleranceInput->setToolTip("Lower values preserve more detail, higher values reduce file size");
    simplifyToleranceLayout->addWidget(simplifyToleranceInput);
    
    enhancedOptimizationLayout->addWidget(simplifyToleranceWidget);
    
    // Color optimization controls (initially hidden, shown when color mode is selected)
    QGroupBox *colorOptimizationGroupBox = new QGroupBox("Color Optimization", this);
    QVBoxLayout *colorOptimizationLayout = new QVBoxLayout(colorOptimizationGroupBox);
    
    optimizeColorOrderCheckbox = new QCheckBox("Optimize Color Order", this);
    optimizeColorOrderCheckbox->setToolTip("Order colors from dark to light for optimal plotting");
    optimizeColorOrderCheckbox->setChecked(true);
    colorOptimizationLayout->addWidget(optimizeColorOrderCheckbox);
    
    groupSimilarColorsCheckbox = new QCheckBox("Group Similar Colors", this);
    groupSimilarColorsCheckbox->setToolTip("Group similar colors to reduce pen changes");
    groupSimilarColorsCheckbox->setChecked(false);
    colorOptimizationLayout->addWidget(groupSimilarColorsCheckbox);
    
    QWidget *colorSimilarityWidget = new QWidget(this);
    QHBoxLayout *colorSimilarityLayout = new QHBoxLayout(colorSimilarityWidget);
    colorSimilarityLayout->setContentsMargins(20, 0, 0, 0);
    
    colorSimilarityLabel = new QLabel("Color Similarity Threshold:", colorSimilarityWidget);
    colorSimilarityLayout->addWidget(colorSimilarityLabel);
    
    colorSimilarityInput = new QDoubleSpinBox(colorSimilarityWidget);
    colorSimilarityInput->setMinimum(1.0);
    colorSimilarityInput->setMaximum(100.0);
    colorSimilarityInput->setSingleStep(1.0);
    colorSimilarityInput->setValue(20.0);
    colorSimilarityInput->setToolTip("Higher values group more colors together (1-100)");
    colorSimilarityLayout->addWidget(colorSimilarityInput);
    
    colorOptimizationLayout->addWidget(colorSimilarityWidget);
    colorSimilarityWidget->setVisible(false); // Initially hidden until "Group Similar Colors" is checked
    
    // Add the groups to the main layout
    ui->verticalLayout->insertWidget(ui->verticalLayout->indexOf(ui->optimizationGroupBox), performanceGroupBox);
    ui->verticalLayout->insertWidget(ui->verticalLayout->indexOf(ui->optimizationGroupBox), enhancedOptimizationGroupBox);
    ui->verticalLayout->insertWidget(ui->verticalLayout->indexOf(ui->optimizationGroupBox), colorOptimizationGroupBox);
    
    // Log available thread count to help with debugging
    std::cout << "System has " << std::thread::hardware_concurrency() << " hardware threads available." << std::endl;
    std::cout << "Default thread count set to: 0 (Auto)" << std::endl;

    // Set default values (these will be overridden by loadSettings if settings exist)
    ui->pixelSizeRadio->setChecked(true);
    ui->mmRadio->setChecked(true);
    ui->solidRadio->setChecked(true);
    ui->monochromeRadio->setChecked(true); // Default to monochrome mode
    ui->optimizeCheckbox->setChecked(true);
    ui->pixelSizeInput->setValue(1.0);
    ui->strokeWidthInput->setValue(0.1);
    ui->angleInput->setValue(45.0);
    ui->docWidthInput->setValue(100.0);
    ui->docHeightInput->setValue(100.0);
    ui->marginInput->setValue(0.0); // Default to 0 margin

    // Set tooltip for margin input
    ui->marginInput->setToolTip("Margin around the SVG content (in selected units)");

    // Load previously saved settings from persistent storage
    // This must happen BEFORE connecting signals to avoid saving during loading
    loadSettings();

    // Connect buttons to their slots
    connect(ui->browseButton, &QPushButton::clicked, this, &MainWindow::browseFile);
    connect(ui->convertButton, &QPushButton::clicked, this, &MainWindow::convert);
    
    // Connect radio button groups to update UI visibility
    connect(sizeTypeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::updateSizeInputs);
    connect(fillTypeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::updateAngleInput);
    connect(colorModeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::updateColorModeOptions);
    
    // Connect checkboxes to update dependent controls
    connect(simplifyPathsCheckbox, &QCheckBox::stateChanged, this, &MainWindow::updatePathSimplificationOptions);
    connect(groupSimilarColorsCheckbox, &QCheckBox::stateChanged, this, &MainWindow::updateSimilarColorOptions);
    
    // Connect all input widgets to save settings whenever they change
    // For double spin boxes
    connect(ui->pixelSizeInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(ui->strokeWidthInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(ui->angleInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(ui->docWidthInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(ui->docHeightInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(ui->marginInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(simplifyToleranceInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(colorSimilarityInput, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    
    // For checkboxes
    connect(ui->optimizeCheckbox, &QCheckBox::stateChanged, this, &MainWindow::saveSettings);    
    connect(minimizeTravelCheckbox, &QCheckBox::stateChanged, this, &MainWindow::saveSettings);
    connect(simplifyPathsCheckbox, &QCheckBox::stateChanged, this, &MainWindow::saveSettings);
    connect(optimizeColorOrderCheckbox, &QCheckBox::stateChanged, this, &MainWindow::saveSettings);
    connect(groupSimilarColorsCheckbox, &QCheckBox::stateChanged, this, &MainWindow::saveSettings);
    
    // For radio button groups
    connect(sizeTypeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::saveSettings);
    connect(unitsGroup, &QButtonGroup::buttonClicked, this, &MainWindow::saveSettings);
    connect(fillTypeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::saveSettings);
    connect(colorModeGroup, &QButtonGroup::buttonClicked, this, &MainWindow::saveSettings);
    
    // For thread count
    connect(threadCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &MainWindow::saveSettings);
    connect(threadCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int value) {
                std::cout << "Thread count changed to: " << value 
                          << (value == 0 ? " (Auto)" : "") << std::endl;
            });
    
    // Update UI visibility based on loaded settings
    updateSizeInputs();
    updateAngleInput();
    updateColorModeOptions();
    updatePathSimplificationOptions();
    updateSimilarColorOptions();
    
    // Adjust window size to fit all the new controls
    resize(width(), sizeHint().height());
}

/**
 * @brief Destructor for MainWindow
 * 
 * Cleans up UI resources
 */
MainWindow::~MainWindow() {
    // Save settings one last time before closing
    saveSettings();
    delete ui;
}

/**
 * @brief Opens a file dialog to select a bitmap image
 * 
 * Updates the UI with the selected file path and saves it in settings
 */
void MainWindow::browseFile() {
    // Show file dialog to select bitmap files
    inputFile = QFileDialog::getOpenFileName(this, "Select Bitmap", "", "Images (*.bmp *.gif *.png *.jpg *.jpeg)");
    
    // If a file was selected, update the UI and save the path
    if (!inputFile.isEmpty()) {
        ui->filePathLabel->setText(inputFile);
        settings.setValue("lastInputFile", inputFile);
        std::cout << "Selected file: " << inputFile.toStdString() << std::endl;
    }
}

/**
 * @brief Performs the conversion from bitmap to SVG
 * 
 * Gathers parameters from the UI, executes the conversion, and handles errors
 */
void MainWindow::convert() {
    try {
        // Verify that an input file has been selected
        if (inputFile.isEmpty()) {
            throw std::runtime_error("Please select an input file");
        }

        // Gather parameters from UI
        ConverterParams params;
        params.units = unitsGroup->checkedButton()->text() == "mm" ? Units::MM : Units::Inches;
        params.fillType = fillTypeGroup->checkedButton()->text() == "Zigzag" ? FillType::Zigzag : FillType::Solid;
        params.strokeWidth = ui->strokeWidthInput->value();
        params.optimize = ui->optimizeCheckbox->isChecked();
        params.numThreads = threadCountSpinBox->value();
        params.margin = ui->marginInput->value(); // Set margin from UI
        
        // Set enhanced optimization parameters
        params.minimizeTravel = minimizeTravelCheckbox->isChecked();
        params.simplifyPaths = simplifyPathsCheckbox->isChecked();
        params.simplifyTolerance = simplifyToleranceInput->value();
        params.optimizeColorOrder = optimizeColorOrderCheckbox->isChecked();
        params.groupSimilarColors = groupSimilarColorsCheckbox->isChecked();
        params.colorSimilarityThreshold = colorSimilarityInput->value();
        
        std::cout << "\n***** Starting conversion with parameters *****" << std::endl;
        std::cout << "Thread count: " << params.numThreads 
                  << (params.numThreads == 0 ? " (Auto)" : "") << std::endl;
        std::cout << "Fill type: " << (params.fillType == FillType::Zigzag ? "Zigzag" : "Solid") << std::endl;
        std::cout << "Stroke width: " << params.strokeWidth << std::endl;
        std::cout << "Margin: " << params.margin << " " 
                  << (params.units == Units::MM ? "mm" : "inches") << std::endl;
        std::cout << "Optimize: " << (params.optimize ? "Yes" : "No") << std::endl;
        std::cout << "Minimize travel: " << (params.minimizeTravel ? "Yes" : "No") << std::endl;
        std::cout << "Simplify paths: " << (params.simplifyPaths ? "Yes" : "No") << std::endl;
        if (params.simplifyPaths) {
            std::cout << "Simplification tolerance: " << params.simplifyTolerance << std::endl;
        }
        
        // Set color mode
        params.colorMode = colorModeGroup->checkedButton()->text() == "Preserve Colors" ? 
                         ColorMode::PreserveColors : ColorMode::Monochrome;
        std::cout << "Color mode: " << (params.colorMode == ColorMode::PreserveColors ? 
                                      "Preserve Colors" : "Monochrome") << std::endl;
                                      
        if (params.colorMode == ColorMode::PreserveColors) {
            std::cout << "Optimize color order: " << (params.optimizeColorOrder ? "Yes" : "No") << std::endl;
            std::cout << "Group similar colors: " << (params.groupSimilarColors ? "Yes" : "No") << std::endl;
            if (params.groupSimilarColors) {
                std::cout << "Color similarity threshold: " << params.colorSimilarityThreshold << std::endl;
            }
        }

        // Set either pixel size or document size based on selection
        if (sizeTypeGroup->checkedButton()->text() == "Pixel Size") {
            params.pixelSize = ui->pixelSizeInput->value();
            std::cout << "Using Pixel Size: " << params.pixelSize << std::endl;
        } else {
            params.docWidth = ui->docWidthInput->value();
            params.docHeight = ui->docHeightInput->value();
            std::cout << "Using Document Size: " << params.docWidth << " x " << params.docHeight << std::endl;
        }

        // Set angle for zigzag fill if selected
        if (params.fillType == FillType::Zigzag) {
            params.angle = ui->angleInput->value();
            std::cout << "Zigzag angle: " << params.angle << std::endl;
        }

        // Create output filename by replacing extension with .svg
        QString outputFile = inputFile;
        outputFile.replace(QRegularExpression("\\..[^.]*$"), ".svg");
        std::cout << "Output file will be: " << outputFile.toStdString() << std::endl;
        
        // Perform the conversion
        std::cout << "Starting converter..." << std::endl;
        BitmapConverter converter;
        converter.convert(inputFile.toStdString(), outputFile.toStdString(), params);
        
        // Show success message
        QMessageBox::information(this, "Success", "SVG saved as " + outputFile);
        std::cout << "Conversion completed successfully" << std::endl;
        
        // Save all settings after successful conversion
        saveSettings();
    }
    catch (const std::exception& e) {
        // Show error message if conversion fails
        QMessageBox::critical(this, "Error", e.what());
        std::cerr << "Error during conversion: " << e.what() << std::endl;
    }
}

/**
 * @brief Updates visibility of pixel size vs document size inputs
 * 
 * Shows/hides appropriate UI elements based on the selected size type
 */
void MainWindow::updateSizeInputs() {
    QAbstractButton* button = sizeTypeGroup->checkedButton();
    if (button) {
        bool isPixelSize = button->text() == "Pixel Size";
        ui->pixelSizeWidget->setVisible(isPixelSize);
        ui->documentSizeWidget->setVisible(!isPixelSize);
    }
}

/**
 * @brief Updates visibility of the angle input
 * 
 * Shows/hides the angle input based on whether zigzag fill is selected
 */
void MainWindow::updateAngleInput() {
    QAbstractButton* button = fillTypeGroup->checkedButton();
    if (button) {
        ui->angleWidget->setVisible(button->text() == "Zigzag");
    }
}

/**
 * @brief Updates visibility of color optimization options based on color mode
 */
void MainWindow::updateColorModeOptions() {
    QAbstractButton* button = colorModeGroup->checkedButton();
    if (button) {
        bool isColorMode = button->text() == "Preserve Colors";
        optimizeColorOrderCheckbox->parentWidget()->setVisible(isColorMode);
        updateSimilarColorOptions(); // Update visibility of color similarity controls
    }
}

/**
 * @brief Updates visibility of path simplification options
 */
void MainWindow::updatePathSimplificationOptions() {
    bool simplifyEnabled = simplifyPathsCheckbox->isChecked();
    simplifyToleranceLabel->parentWidget()->setVisible(simplifyEnabled);
}

/**
 * @brief Updates visibility of similar color grouping options
 */
void MainWindow::updateSimilarColorOptions() {
    bool colorMode = colorModeGroup->checkedButton()->text() == "Preserve Colors";
    bool groupEnabled = groupSimilarColorsCheckbox->isChecked();
    colorSimilarityLabel->parentWidget()->setVisible(colorMode && groupEnabled);
}

/**
 * @brief Saves current UI settings to persistent storage
 * 
 * This is called automatically whenever UI elements are changed
 */
void MainWindow::saveSettings() {
    // Save radio button states
    settings.setValue("pixelSizeRadio", ui->pixelSizeRadio->isChecked());
    settings.setValue("mmRadio", ui->mmRadio->isChecked());
    settings.setValue("zigzagRadio", ui->zigzagRadio->isChecked());
    settings.setValue("colorRadio", ui->colorRadio->isChecked()); // Save color mode
    
    // Save numeric values
    settings.setValue("pixelSize", ui->pixelSizeInput->value());
    settings.setValue("strokeWidth", ui->strokeWidthInput->value());
    settings.setValue("angle", ui->angleInput->value());
    settings.setValue("docWidth", ui->docWidthInput->value());
    settings.setValue("docHeight", ui->docHeightInput->value());
    settings.setValue("margin", ui->marginInput->value()); 
    settings.setValue("simplifyTolerance", simplifyToleranceInput->value());
    settings.setValue("colorSimilarity", colorSimilarityInput->value());
    
    // Save checkbox states
    settings.setValue("optimize", ui->optimizeCheckbox->isChecked());
    settings.setValue("minimizeTravel", minimizeTravelCheckbox->isChecked());
    settings.setValue("simplifyPaths", simplifyPathsCheckbox->isChecked());
    settings.setValue("optimizeColorOrder", optimizeColorOrderCheckbox->isChecked());
    settings.setValue("groupSimilarColors", groupSimilarColorsCheckbox->isChecked());
    
    // Save thread count
    settings.setValue("threadCount", threadCountSpinBox->value());
    
    // Explicitly sync to ensure all settings are written immediately
    settings.sync();
}

/**
 * @brief Loads settings from persistent storage
 * 
 * Called during application startup to restore previous settings
 */
void MainWindow::loadSettings() {
    // Load last used filename
    inputFile = settings.value("lastInputFile", "").toString();
    if (!inputFile.isEmpty()) {
        ui->filePathLabel->setText(inputFile);
        std::cout << "Loaded previous file path: " << inputFile.toStdString() << std::endl;
    }
    
    // First load the numeric values
    ui->pixelSizeInput->setValue(settings.value("pixelSize", 1.0).toDouble());
    ui->strokeWidthInput->setValue(settings.value("strokeWidth", 0.1).toDouble());
    ui->angleInput->setValue(settings.value("angle", 45.0).toDouble());
    ui->docWidthInput->setValue(settings.value("docWidth", 100.0).toDouble());
    ui->docHeightInput->setValue(settings.value("docHeight", 100.0).toDouble());
    ui->marginInput->setValue(settings.value("margin", 0.0).toDouble());
    simplifyToleranceInput->setValue(settings.value("simplifyTolerance", 0.1).toDouble());
    colorSimilarityInput->setValue(settings.value("colorSimilarity", 20.0).toDouble());
    
    // Then load checkbox states
    ui->optimizeCheckbox->setChecked(settings.value("optimize", true).toBool());
    minimizeTravelCheckbox->setChecked(settings.value("minimizeTravel", true).toBool());
    simplifyPathsCheckbox->setChecked(settings.value("simplifyPaths", true).toBool());
    optimizeColorOrderCheckbox->setChecked(settings.value("optimizeColorOrder", true).toBool());
    groupSimilarColorsCheckbox->setChecked(settings.value("groupSimilarColors", false).toBool());
    
    // Load thread count
    int threadCount = settings.value("threadCount", 0).toInt();
    threadCountSpinBox->setValue(threadCount);
    std::cout << "Loaded thread count setting: " << threadCount 
              << (threadCount == 0 ? " (Auto)" : "") << std::endl;
    
    // Finally, load radio button states (this triggers visibility updates)
    bool usePixelSize = settings.value("pixelSizeRadio", true).toBool();
    bool useMM = settings.value("mmRadio", true).toBool();
    bool useZigzag = settings.value("zigzagRadio", false).toBool();
    bool useColor = settings.value("colorRadio", false).toBool(); // Load color mode
    
    ui->pixelSizeRadio->setChecked(usePixelSize);
    ui->docSizeRadio->setChecked(!usePixelSize);
    ui->mmRadio->setChecked(useMM);
    ui->inchesRadio->setChecked(!useMM);
    ui->zigzagRadio->setChecked(useZigzag);
    ui->solidRadio->setChecked(!useZigzag);
    ui->colorRadio->setChecked(useColor);
    ui->monochromeRadio->setChecked(!useColor);
}