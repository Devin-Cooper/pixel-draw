#pragma once
#include <QMainWindow>
#include <QButtonGroup>
#include <QSettings>

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
    QSettings settings;          ///< Persistent settings storage
};