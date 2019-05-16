/********************************************************************************
** Form generated from reading UI file 'mainwindowL14987.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MAINWINDOWL14987_H
#define MAINWINDOWL14987_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGroupBox *groupBox;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_next;
    QPushButton *pushButton_continue;
    QSpacerItem *verticalSpacer;
    QSpinBox *spinBox_nFrame;
    QPushButton *pushButton_pre_frame;
    QPushButton *pushButton_next_frame;
    QPushButton *pushButton_save_frame;
    QGroupBox *groupBox_2;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_load_pcd;
    QComboBox *comboBox_pcd;
    QSpacerItem *verticalSpacer_2;
    QPushButton *pushButton_show_pcd;
    QPushButton *pushButton_next_pcd;
    QPushButton *pushButton_pre_pcd;
    QTextEdit *textEdit_msg;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(265, 410);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 20, 111, 271));
        verticalLayoutWidget = new QWidget(groupBox);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(0, 20, 101, 251));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_next = new QPushButton(verticalLayoutWidget);
        pushButton_next->setObjectName(QStringLiteral("pushButton_next"));

        verticalLayout->addWidget(pushButton_next);

        pushButton_continue = new QPushButton(verticalLayoutWidget);
        pushButton_continue->setObjectName(QStringLiteral("pushButton_continue"));

        verticalLayout->addWidget(pushButton_continue);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        spinBox_nFrame = new QSpinBox(verticalLayoutWidget);
        spinBox_nFrame->setObjectName(QStringLiteral("spinBox_nFrame"));
        spinBox_nFrame->setMinimum(1);
        spinBox_nFrame->setSingleStep(1);
        spinBox_nFrame->setValue(10);

        verticalLayout->addWidget(spinBox_nFrame);

        pushButton_pre_frame = new QPushButton(verticalLayoutWidget);
        pushButton_pre_frame->setObjectName(QStringLiteral("pushButton_pre_frame"));

        verticalLayout->addWidget(pushButton_pre_frame);

        pushButton_next_frame = new QPushButton(verticalLayoutWidget);
        pushButton_next_frame->setObjectName(QStringLiteral("pushButton_next_frame"));

        verticalLayout->addWidget(pushButton_next_frame);

        pushButton_save_frame = new QPushButton(verticalLayoutWidget);
        pushButton_save_frame->setObjectName(QStringLiteral("pushButton_save_frame"));

        verticalLayout->addWidget(pushButton_save_frame);

        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(120, 19, 111, 271));
        verticalLayoutWidget_2 = new QWidget(groupBox_2);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 20, 101, 251));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        pushButton_load_pcd = new QPushButton(verticalLayoutWidget_2);
        pushButton_load_pcd->setObjectName(QStringLiteral("pushButton_load_pcd"));

        verticalLayout_2->addWidget(pushButton_load_pcd);

        comboBox_pcd = new QComboBox(verticalLayoutWidget_2);
        comboBox_pcd->setObjectName(QStringLiteral("comboBox_pcd"));

        verticalLayout_2->addWidget(comboBox_pcd);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        pushButton_show_pcd = new QPushButton(verticalLayoutWidget_2);
        pushButton_show_pcd->setObjectName(QStringLiteral("pushButton_show_pcd"));

        verticalLayout_2->addWidget(pushButton_show_pcd);

        pushButton_next_pcd = new QPushButton(verticalLayoutWidget_2);
        pushButton_next_pcd->setObjectName(QStringLiteral("pushButton_next_pcd"));

        verticalLayout_2->addWidget(pushButton_next_pcd);

        pushButton_pre_pcd = new QPushButton(verticalLayoutWidget_2);
        pushButton_pre_pcd->setObjectName(QStringLiteral("pushButton_pre_pcd"));

        verticalLayout_2->addWidget(pushButton_pre_pcd);

        textEdit_msg = new QTextEdit(centralwidget);
        textEdit_msg->setObjectName(QStringLiteral("textEdit_msg"));
        textEdit_msg->setGeometry(QRect(10, 300, 221, 51));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 265, 25));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Play Tool (with driver)", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "control node:", 0));
        pushButton_next->setText(QApplication::translate("MainWindow", "next", 0));
        pushButton_continue->setText(QApplication::translate("MainWindow", "continue", 0));
        pushButton_pre_frame->setText(QApplication::translate("MainWindow", "pre frame", 0));
        pushButton_next_frame->setText(QApplication::translate("MainWindow", "next frame", 0));
        pushButton_save_frame->setText(QApplication::translate("MainWindow", "save", 0));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "replay node:", 0));
        pushButton_load_pcd->setText(QApplication::translate("MainWindow", "load", 0));
        pushButton_show_pcd->setText(QApplication::translate("MainWindow", "show", 0));
        pushButton_next_pcd->setText(QApplication::translate("MainWindow", "next", 0));
        pushButton_pre_pcd->setText(QApplication::translate("MainWindow", "pre", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MAINWINDOWL14987_H
