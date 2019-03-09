/*! \mainpage SpaceCloud
 *
 * \section intro_sec Introduction
 *
 * 深圳智绘科技研发部学习材料
 *
 * some demo developed by Shenzhen Zhihui Technology Co., Ltd
 *
 * 软件由C++在Linux下编写，利用Qt和OSG搭建界面
 *
 * The software is written by C++ under Linux, with Qt and OSG.
 *
 * \section prerequisites_sec Prerequisites
 *  1. OpenSceneGraph 3.2
 *  2. Qt version 5.5 or newer
 *  Make sure you have a C++11 compliant compiler (gcc 5.5+ / clang)
 *
 * \section install_sec Installation
 * \subsection step1 Step 1: Clone the project
 * \subsection step2 Step 2: Build
 * \subsection step3 Step 3: Run
 *
 * etc...
 */

#include <QtWidgets/QApplication>
#include <QtCore/QTranslator>

#include "MainWindow.h"


int main(int argc, char *argv[]) {
    LOG_INFO << "pid = " << getpid();
    qputenv("QT_STYLE_OVERRIDE", ""); // suppress the qt style warning
    QApplication app(argc, argv);

    QTranslator translator;
    if (!app.arguments().contains("en"))
        if (translator.load(QLocale("zh"), QLatin1String("lpd"), QLatin1String("_"), app.applicationDirPath() + "/tr"))
            app.installTranslator(&translator);

    MainWindow mainwindow;
    mainwindow.setMinimumSize(800, 600);  //avoid graphic context bugs!
    mainwindow.show();

    return app.exec();
}