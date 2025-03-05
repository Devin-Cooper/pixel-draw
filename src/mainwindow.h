#pragma once
#include <QMainWindow>
#include <QButtonGroup>
#include <QSettings>
#include <QSpinBox>
#include <QLabel>
#include <QCheckBox>
#include <QDoubleSpinBox>

QT_BEGIN_NAMESPACE
namespace Ui { 
    class MainWindow; 
}
QT_END_NAMESPACE

/**
 * @brief MainWindow class for the Bitmap Converter application
 * 
 * This class handles the main user interface and interactions for converting
 * bitmap images to SVG. It includes functionality to persist user settings
 * between application sessions.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    /**
     * @brief Constructor for the MainWindow
     * @param parent Optional parent widget
     */
    MainWindow(QWidget *parent = nullptr);
    
    /**
     * @brief Destructor for the MainWindow
     */
    ~MainWindow();

private slots:
    /**
     * @brief Opens a file dialog to browse for an input bitmap file
     */
    void browseFile();
    
    /**
     * @brief Performs the conversion from bitmap to SVG
     */
    void convert();
    
    /**
     * @brief Updates UI visibility based on whether pixel size or document size is selected
     */
    void updateSizeInputs();
    
    /**
     * @brief Updates angle input visibility based on the fill type
     */
    void updateAngleInput();
    
    /**
     * @brief Updates visibility of controls based on the color mode
     */
    void updateColorModeOptions();
    
    /**
     * @brief Updates visibility of path simplification options
     */
    void updatePathSimplificationOptions();
    
    /**
     * @brief Updates visibility of similar color grouping options
     */
    void updateSimilarColorOptions();
    
    /**
     * @brief Saves current UI settings to persistent storage
     * This is called automatically when UI elements change
     */
    void saveSettings();

private:
    /**
     * @brief Loads previously saved settings from persistent storage
     * This is called during application startup
     */
    void loadSettings();
    
    Ui::MainWindow *ui;          ///< UI elements defined in the form
    QString inputFile;           ///< Path to the input file
    QButtonGroup *sizeTypeGroup; ///< Group for pixel size vs document size radio buttons
    QButtonGroup *unitsGroup;    ///< Group for mm vs inches radio buttons
    QButtonGroup *fillTypeGroup; ///< Group for solid vs zigzag fill type radio buttons
    QButtonGroup *colorModeGroup; ///< Group for monochrome vs color mode radio buttons
    QSettings settings;          ///< Persistent settings storage
    QSpinBox *threadCountSpinBox; ///< Control for thread count
    QLabel *threadCountLabel;    ///< Label for thread count control
    
    // New UI elements for enhanced optimization
    QCheckBox *minimizeTravelCheckbox;      ///< Checkbox for minimizing travel
    QCheckBox *simplifyPathsCheckbox;       ///< Checkbox for path simplification
    QDoubleSpinBox *simplifyToleranceInput; ///< Input for simplification tolerance
    QLabel *simplifyToleranceLabel;         ///< Label for simplification tolerance
    
    // Color-specific optimization controls
    QCheckBox *optimizeColorOrderCheckbox;  ///< Checkbox for optimizing color order
    QCheckBox *groupSimilarColorsCheckbox;  ///< Checkbox for grouping similar colors
    QDoubleSpinBox *colorSimilarityInput;   ///< Input for color similarity threshold
    QLabel *colorSimilarityLabel;           ///< Label for color similarity threshold
};