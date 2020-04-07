/********************************************************************************
** Form generated from reading UI file 'Point_Gui.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINT_GUI_H
#define UI_POINT_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_Point_GuiClass
{
public:
    QWidget *centralWidget;
    QVTKWidget *qvtkWidget;
    QTextBrowser *textBrowser;
    QWidget *layoutWidget;
    QFormLayout *formLayout;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Point_GuiClass)
    {
        if (Point_GuiClass->objectName().isEmpty())
            Point_GuiClass->setObjectName(QStringLiteral("Point_GuiClass"));
        Point_GuiClass->resize(600, 418);
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Point_GuiClass->sizePolicy().hasHeightForWidth());
        Point_GuiClass->setSizePolicy(sizePolicy);
        Point_GuiClass->setMinimumSize(QSize(600, 418));
        Point_GuiClass->setMaximumSize(QSize(600, 418));
        Point_GuiClass->setSizeIncrement(QSize(20, 20));
        Point_GuiClass->setToolButtonStyle(Qt::ToolButtonTextOnly);
        centralWidget = new QWidget(Point_GuiClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setEnabled(true);
        qvtkWidget->setGeometry(QRect(160, 10, 413, 341));
        qvtkWidget->setMinimumSize(QSize(413, 341));
        qvtkWidget->setMaximumSize(QSize(800, 800));
        qvtkWidget->setSizeIncrement(QSize(10, 10));
        textBrowser = new QTextBrowser(centralWidget);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(10, 100, 141, 221));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(34, 30, 83, 54));
        formLayout = new QFormLayout(layoutWidget);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_2 = new QPushButton(layoutWidget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        formLayout->setWidget(0, QFormLayout::LabelRole, pushButton_2);

        pushButton = new QPushButton(layoutWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        formLayout->setWidget(1, QFormLayout::LabelRole, pushButton);

        Point_GuiClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Point_GuiClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        Point_GuiClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Point_GuiClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Point_GuiClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Point_GuiClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Point_GuiClass->setStatusBar(statusBar);

        retranslateUi(Point_GuiClass);

        QMetaObject::connectSlotsByName(Point_GuiClass);
    } // setupUi

    void retranslateUi(QMainWindow *Point_GuiClass)
    {
        Point_GuiClass->setWindowTitle(QApplication::translate("Point_GuiClass", "dianyunPCL demo", Q_NULLPTR));
        textBrowser->setHtml(QApplication::translate("Point_GuiClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">\350\277\231\346\230\257\344\270\200\344\270\252\345\234\250VS2015\344\270\213\344\275\277\347\224\250QT5.8\345\201\232\344\272\206\344\270\200\344\270\252\347\256\200\345\215\225\347\232\204demo,\345\256\236\347\216\260\344\272\206\346\211\223\345\274\200\344\270\200\344\270\252PCD\346\226\207\344\273\266\357\274\214\345\271\266\345\234\250QT\344\270\255\345\256\236\347\216\260\344\272\206\345\217\257\350\247\206\345\214\226\357\274\214\344\276\235\350\265\226\351\241\271\344\270\273\350\246\201\346\234\211PCL1.8.1\344\273\245\345\217"
                        "\212\351\234\200\350\246\201\351\207\215\346\226\260\344\270\216QT\350\201\224\345\220\210\347\274\226\350\257\221\347\232\204VTK\343\200\202</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:10pt;\"><br /></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:10pt;\"><br /></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:10pt;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">dianyunPCL</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt;\">\346"
                        "\234\254\347\250\213\345\272\217\347\224\2612020-1-10  particle</span></p></body></html>", Q_NULLPTR));
        pushButton_2->setText(QApplication::translate("Point_GuiClass", "OpneFile", Q_NULLPTR));
        pushButton->setText(QApplication::translate("Point_GuiClass", "RandomColor", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Point_GuiClass: public Ui_Point_GuiClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINT_GUI_H
