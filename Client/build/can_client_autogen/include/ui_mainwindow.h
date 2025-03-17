/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
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
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *serverIpEdit;
    QLabel *label_2;
    QLineEdit *serverPortEdit;
    QPushButton *connectButton;
    QPushButton *disconnectButton;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QLineEdit *canIdEdit;
    QLabel *label_4;
    QLineEdit *canDataEdit;
    QPushButton *sendButton;
    QGroupBox *presetGroupBox;
    QGridLayout *presetGridLayout;
    QPushButton *presetButton1;
    QPushButton *presetButton2;
    QPushButton *presetButton3;
    QPushButton *presetButton4;
    QPushButton *presetButton5;
    QPushButton *presetButton6;
    QPushButton *presetButton7;
    QPushButton *presetButton8;
    QPushButton *presetButton9;
    QPushButton *presetButton10;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *collapseCheckBox;
    QSpinBox *collapseIntervalSpinBox;
    QSpacerItem *horizontalSpacer_2;
    QTextEdit *monitorText;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *monitorButton;
    QSpacerItem *horizontalSpacer;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        label = new QLabel(centralwidget);
        label->setObjectName("label");

        horizontalLayout->addWidget(label);

        serverIpEdit = new QLineEdit(centralwidget);
        serverIpEdit->setObjectName("serverIpEdit");

        horizontalLayout->addWidget(serverIpEdit);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName("label_2");

        horizontalLayout->addWidget(label_2);

        serverPortEdit = new QLineEdit(centralwidget);
        serverPortEdit->setObjectName("serverPortEdit");

        horizontalLayout->addWidget(serverPortEdit);

        connectButton = new QPushButton(centralwidget);
        connectButton->setObjectName("connectButton");

        horizontalLayout->addWidget(connectButton);

        disconnectButton = new QPushButton(centralwidget);
        disconnectButton->setObjectName("disconnectButton");

        horizontalLayout->addWidget(disconnectButton);


        verticalLayout->addLayout(horizontalLayout);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName("groupBox");
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setObjectName("verticalLayout_2");
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        label_3 = new QLabel(groupBox);
        label_3->setObjectName("label_3");

        horizontalLayout_2->addWidget(label_3);

        canIdEdit = new QLineEdit(groupBox);
        canIdEdit->setObjectName("canIdEdit");

        horizontalLayout_2->addWidget(canIdEdit);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName("label_4");

        horizontalLayout_2->addWidget(label_4);

        canDataEdit = new QLineEdit(groupBox);
        canDataEdit->setObjectName("canDataEdit");

        horizontalLayout_2->addWidget(canDataEdit);

        sendButton = new QPushButton(groupBox);
        sendButton->setObjectName("sendButton");

        horizontalLayout_2->addWidget(sendButton);


        verticalLayout_2->addLayout(horizontalLayout_2);

        presetGroupBox = new QGroupBox(groupBox);
        presetGroupBox->setObjectName("presetGroupBox");
        presetGridLayout = new QGridLayout(presetGroupBox);
        presetGridLayout->setObjectName("presetGridLayout");
        presetButton1 = new QPushButton(presetGroupBox);
        presetButton1->setObjectName("presetButton1");

        presetGridLayout->addWidget(presetButton1, 0, 0, 1, 1);

        presetButton2 = new QPushButton(presetGroupBox);
        presetButton2->setObjectName("presetButton2");

        presetGridLayout->addWidget(presetButton2, 0, 1, 1, 1);

        presetButton3 = new QPushButton(presetGroupBox);
        presetButton3->setObjectName("presetButton3");

        presetGridLayout->addWidget(presetButton3, 0, 2, 1, 1);

        presetButton4 = new QPushButton(presetGroupBox);
        presetButton4->setObjectName("presetButton4");

        presetGridLayout->addWidget(presetButton4, 0, 3, 1, 1);

        presetButton5 = new QPushButton(presetGroupBox);
        presetButton5->setObjectName("presetButton5");

        presetGridLayout->addWidget(presetButton5, 0, 4, 1, 1);

        presetButton6 = new QPushButton(presetGroupBox);
        presetButton6->setObjectName("presetButton6");

        presetGridLayout->addWidget(presetButton6, 1, 0, 1, 1);

        presetButton7 = new QPushButton(presetGroupBox);
        presetButton7->setObjectName("presetButton7");

        presetGridLayout->addWidget(presetButton7, 1, 1, 1, 1);

        presetButton8 = new QPushButton(presetGroupBox);
        presetButton8->setObjectName("presetButton8");

        presetGridLayout->addWidget(presetButton8, 1, 2, 1, 1);

        presetButton9 = new QPushButton(presetGroupBox);
        presetButton9->setObjectName("presetButton9");

        presetGridLayout->addWidget(presetButton9, 1, 3, 1, 1);

        presetButton10 = new QPushButton(presetGroupBox);
        presetButton10->setObjectName("presetButton10");

        presetGridLayout->addWidget(presetButton10, 1, 4, 1, 1);


        verticalLayout_2->addWidget(presetGroupBox);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName("groupBox_2");
        verticalLayout_3 = new QVBoxLayout(groupBox_2);
        verticalLayout_3->setObjectName("verticalLayout_3");
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        collapseCheckBox = new QCheckBox(groupBox_2);
        collapseCheckBox->setObjectName("collapseCheckBox");

        horizontalLayout_4->addWidget(collapseCheckBox);

        collapseIntervalSpinBox = new QSpinBox(groupBox_2);
        collapseIntervalSpinBox->setObjectName("collapseIntervalSpinBox");
        collapseIntervalSpinBox->setMinimum(100);
        collapseIntervalSpinBox->setMaximum(10000);
        collapseIntervalSpinBox->setValue(1000);

        horizontalLayout_4->addWidget(collapseIntervalSpinBox);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);


        verticalLayout_3->addLayout(horizontalLayout_4);

        monitorText = new QTextEdit(groupBox_2);
        monitorText->setObjectName("monitorText");
        monitorText->setReadOnly(true);

        verticalLayout_3->addWidget(monitorText);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        monitorButton = new QPushButton(groupBox_2);
        monitorButton->setObjectName("monitorButton");

        horizontalLayout_3->addWidget(monitorButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout_3);


        verticalLayout->addWidget(groupBox_2);

        MainWindow->setCentralWidget(centralwidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName("statusBar");
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "CAN Client", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Server IP:", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Port:", nullptr));
        connectButton->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        disconnectButton->setText(QCoreApplication::translate("MainWindow", "Disconnect", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "CAN Message", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "CAN ID:", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Data:", nullptr));
        sendButton->setText(QCoreApplication::translate("MainWindow", "Send", nullptr));
        presetGroupBox->setTitle(QCoreApplication::translate("MainWindow", "Preset Messages", nullptr));
        presetButton1->setText(QCoreApplication::translate("MainWindow", "Preset 1", nullptr));
        presetButton2->setText(QCoreApplication::translate("MainWindow", "Preset 2", nullptr));
        presetButton3->setText(QCoreApplication::translate("MainWindow", "Preset 3", nullptr));
        presetButton4->setText(QCoreApplication::translate("MainWindow", "Preset 4", nullptr));
        presetButton5->setText(QCoreApplication::translate("MainWindow", "Preset 5", nullptr));
        presetButton6->setText(QCoreApplication::translate("MainWindow", "Preset 6", nullptr));
        presetButton7->setText(QCoreApplication::translate("MainWindow", "Preset 7", nullptr));
        presetButton8->setText(QCoreApplication::translate("MainWindow", "Preset 8", nullptr));
        presetButton9->setText(QCoreApplication::translate("MainWindow", "Preset 9", nullptr));
        presetButton10->setText(QCoreApplication::translate("MainWindow", "Preset 10", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("MainWindow", "Monitor", nullptr));
        collapseCheckBox->setText(QCoreApplication::translate("MainWindow", "Collapse Messages", nullptr));
        collapseIntervalSpinBox->setSuffix(QCoreApplication::translate("MainWindow", " ms", nullptr));
        monitorButton->setText(QCoreApplication::translate("MainWindow", "Start Monitor", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
