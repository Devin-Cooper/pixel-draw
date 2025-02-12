#pragma once
#include <QMainWindow>
#include <QButtonGroup>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void browseFile();
    void convert();
    void updateSizeInputs();
    void updateAngleInput();

private:
    Ui::MainWindow *ui;
    QString inputFile;
    QButtonGroup *sizeTypeGroup;
    QButtonGroup *unitsGroup;
    QButtonGroup *fillTypeGroup;
};