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
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_5;
    QTabWidget *tabWidget;
    QWidget *canTab;
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
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_5;
    QLineEdit *canDataSplit1;
    QLineEdit *canDataSplit2;
    QLineEdit *canDataSplit3;
    QLineEdit *canDataSplit4;
    QSpacerItem *horizontalSpacer_3;
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
    QCheckBox *showTimeCheckBox;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *monitorLayout;
    QGroupBox *txGroupBox;
    QVBoxLayout *verticalLayout_4;
    QTextEdit *txMonitorText;
    QGroupBox *rxGroupBox;
    QVBoxLayout *verticalLayout_5;
    QTextEdit *rxMonitorText;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *monitorButton;
    QSpacerItem *horizontalSpacer;
    QWidget *jointTab;
    QVBoxLayout *verticalLayout_41;
    QGroupBox *joint1Group;
    QHBoxLayout *horizontalLayout_6;
    QSlider *joint1Slider;
    QSpinBox *joint1SpinBox;
    QGroupBox *joint2Group;
    QHBoxLayout *horizontalLayout_7;
    QSlider *joint2Slider;
    QSpinBox *joint2SpinBox;
    QGroupBox *joint3Group;
    QHBoxLayout *horizontalLayout_8;
    QSlider *joint3Slider;
    QSpinBox *joint3SpinBox;
    QGroupBox *joint4Group;
    QHBoxLayout *horizontalLayout_9;
    QSlider *joint4Slider;
    QSpinBox *joint4SpinBox;
    QSpacerItem *verticalSpacer;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1080, 720);
        MainWindow->setMinimumSize(QSize(1080, 720));
        MainWindow->setStyleSheet(QString::fromUtf8("* {\n"
"    color: #000000;\n"
"}\n"
"QMainWindow {\n"
"    background-color: #f0f0f0;\n"
"}\n"
"QGroupBox {\n"
"    border: 2px solid #cccccc;\n"
"    border-radius: 6px;\n"
"    margin-top: 1ex;\n"
"    font-weight: bold;\n"
"    background-color: #ffffff;\n"
"}\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 10px;\n"
"    padding: 0 3px;\n"
"    color: #000000;\n"
"}\n"
"QPushButton {\n"
"    background-color: #0078d4;\n"
"    color: white;\n"
"    border: none;\n"
"    border-radius: 4px;\n"
"    padding: 5px 15px;\n"
"    min-height: 25px;\n"
"    font-weight: bold;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: #1084d8;\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: #006cbd;\n"
"}\n"
"QLineEdit {\n"
"    padding: 4px;\n"
"    border: 1px solid #cccccc;\n"
"    border-radius: 4px;\n"
"    background-color: white;\n"
"    color: #000000;\n"
"}\n"
"QTextEdit {\n"
"    border: 1px solid #cccccc;\n"
"    border-radius: 4px;\n"
"    background-color: white;\n"
"    c"
                        "olor: #000000;\n"
"}\n"
"QLabel {\n"
"    color: #000000;\n"
"    font-weight: bold;\n"
"}\n"
"QCheckBox {\n"
"    color: #000000;\n"
"}\n"
"QSpinBox {\n"
"    color: #000000;\n"
"    background-color: white;\n"
"    padding: 4px;\n"
"    border: 1px solid #cccccc;\n"
"    border-radius: 4px;\n"
"}\n"
"QSlider::groove:horizontal {\n"
"    border: 1px solid #999999;\n"
"    height: 8px;\n"
"    background: #ffffff;\n"
"    margin: 2px 0;\n"
"    border-radius: 4px;\n"
"}\n"
"QSlider::handle:horizontal {\n"
"    background: #0078d4;\n"
"    border: none;\n"
"    width: 18px;\n"
"    margin: -5px 0;\n"
"    border-radius: 9px;\n"
"}\n"
"QSlider::handle:horizontal:hover {\n"
"    background: #1084d8;\n"
"}\n"
"QTabWidget::pane {\n"
"    border: 1px solid #cccccc;\n"
"    border-radius: 4px;\n"
"    background-color: white;\n"
"}\n"
"QTabBar::tab {\n"
"    background-color: #f0f0f0;\n"
"    border: 1px solid #cccccc;\n"
"    border-bottom: none;\n"
"    border-top-left-radius: 4px;\n"
"    border-top-right-radius: "
                        "4px;\n"
"    padding: 8px 16px;\n"
"    margin-right: 2px;\n"
"    color: #000000;\n"
"}\n"
"QTabBar::tab:selected {\n"
"    background-color: white;\n"
"    color: #000000;\n"
"    font-weight: bold;\n"
"}\n"
"QTabBar::tab:hover {\n"
"    background-color: #e5e5e5;\n"
"}\n"
"QStatusBar {\n"
"    color: #000000;\n"
"}\n"
"QPushButton[objectName^=\"presetButton\"] {\n"
"    background-color: #5c2d91;\n"
"    color: white;\n"
"    min-width: 100px;\n"
"    font-weight: bold;\n"
"}\n"
"QPushButton[objectName^=\"presetButton\"]:hover {\n"
"    background-color: #6b3a9e;\n"
"}\n"
"QPushButton[objectName^=\"presetButton\"]:pressed {\n"
"    background-color: #4c2277;\n"
"}"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        horizontalLayout_5 = new QHBoxLayout(centralwidget);
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName("tabWidget");
        canTab = new QWidget();
        canTab->setObjectName("canTab");
        verticalLayout = new QVBoxLayout(canTab);
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        label = new QLabel(canTab);
        label->setObjectName("label");

        horizontalLayout->addWidget(label);

        serverIpEdit = new QLineEdit(canTab);
        serverIpEdit->setObjectName("serverIpEdit");

        horizontalLayout->addWidget(serverIpEdit);

        label_2 = new QLabel(canTab);
        label_2->setObjectName("label_2");

        horizontalLayout->addWidget(label_2);

        serverPortEdit = new QLineEdit(canTab);
        serverPortEdit->setObjectName("serverPortEdit");

        horizontalLayout->addWidget(serverPortEdit);

        connectButton = new QPushButton(canTab);
        connectButton->setObjectName("connectButton");

        horizontalLayout->addWidget(connectButton);

        disconnectButton = new QPushButton(canTab);
        disconnectButton->setObjectName("disconnectButton");

        horizontalLayout->addWidget(disconnectButton);


        verticalLayout->addLayout(horizontalLayout);

        groupBox = new QGroupBox(canTab);
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

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName("horizontalLayout_10");
        label_5 = new QLabel(groupBox);
        label_5->setObjectName("label_5");

        horizontalLayout_10->addWidget(label_5);

        canDataSplit1 = new QLineEdit(groupBox);
        canDataSplit1->setObjectName("canDataSplit1");
        canDataSplit1->setMaximumWidth(60);
        canDataSplit1->setMaxLength(4);

        horizontalLayout_10->addWidget(canDataSplit1);

        canDataSplit2 = new QLineEdit(groupBox);
        canDataSplit2->setObjectName("canDataSplit2");
        canDataSplit2->setMaximumWidth(60);
        canDataSplit2->setMaxLength(4);

        horizontalLayout_10->addWidget(canDataSplit2);

        canDataSplit3 = new QLineEdit(groupBox);
        canDataSplit3->setObjectName("canDataSplit3");
        canDataSplit3->setMaximumWidth(60);
        canDataSplit3->setMaxLength(4);

        horizontalLayout_10->addWidget(canDataSplit3);

        canDataSplit4 = new QLineEdit(groupBox);
        canDataSplit4->setObjectName("canDataSplit4");
        canDataSplit4->setMaximumWidth(60);
        canDataSplit4->setMaxLength(4);

        horizontalLayout_10->addWidget(canDataSplit4);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_3);


        verticalLayout_2->addLayout(horizontalLayout_10);

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

        groupBox_2 = new QGroupBox(canTab);
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

        showTimeCheckBox = new QCheckBox(groupBox_2);
        showTimeCheckBox->setObjectName("showTimeCheckBox");
        showTimeCheckBox->setChecked(false);

        horizontalLayout_4->addWidget(showTimeCheckBox);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);


        verticalLayout_3->addLayout(horizontalLayout_4);

        monitorLayout = new QHBoxLayout();
        monitorLayout->setObjectName("monitorLayout");
        txGroupBox = new QGroupBox(groupBox_2);
        txGroupBox->setObjectName("txGroupBox");
        verticalLayout_4 = new QVBoxLayout(txGroupBox);
        verticalLayout_4->setObjectName("verticalLayout_4");
        txMonitorText = new QTextEdit(txGroupBox);
        txMonitorText->setObjectName("txMonitorText");
        txMonitorText->setReadOnly(true);

        verticalLayout_4->addWidget(txMonitorText);


        monitorLayout->addWidget(txGroupBox);

        rxGroupBox = new QGroupBox(groupBox_2);
        rxGroupBox->setObjectName("rxGroupBox");
        verticalLayout_5 = new QVBoxLayout(rxGroupBox);
        verticalLayout_5->setObjectName("verticalLayout_5");
        rxMonitorText = new QTextEdit(rxGroupBox);
        rxMonitorText->setObjectName("rxMonitorText");
        rxMonitorText->setReadOnly(true);

        verticalLayout_5->addWidget(rxMonitorText);


        monitorLayout->addWidget(rxGroupBox);


        verticalLayout_3->addLayout(monitorLayout);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        monitorButton = new QPushButton(groupBox_2);
        monitorButton->setObjectName("monitorButton");

        horizontalLayout_3->addWidget(monitorButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout_3);


        verticalLayout->addWidget(groupBox_2);

        tabWidget->addTab(canTab, QString());
        jointTab = new QWidget();
        jointTab->setObjectName("jointTab");
        verticalLayout_41 = new QVBoxLayout(jointTab);
        verticalLayout_41->setObjectName("verticalLayout_41");
        joint1Group = new QGroupBox(jointTab);
        joint1Group->setObjectName("joint1Group");
        horizontalLayout_6 = new QHBoxLayout(joint1Group);
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        joint1Slider = new QSlider(joint1Group);
        joint1Slider->setObjectName("joint1Slider");
        joint1Slider->setMinimum(-90);
        joint1Slider->setMaximum(90);
        joint1Slider->setOrientation(Qt::Orientation::Horizontal);
        joint1Slider->setTickPosition(QSlider::TickPosition::TicksBelow);

        horizontalLayout_6->addWidget(joint1Slider);

        joint1SpinBox = new QSpinBox(joint1Group);
        joint1SpinBox->setObjectName("joint1SpinBox");
        joint1SpinBox->setMinimum(-90);
        joint1SpinBox->setMaximum(90);

        horizontalLayout_6->addWidget(joint1SpinBox);


        verticalLayout_41->addWidget(joint1Group);

        joint2Group = new QGroupBox(jointTab);
        joint2Group->setObjectName("joint2Group");
        horizontalLayout_7 = new QHBoxLayout(joint2Group);
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        joint2Slider = new QSlider(joint2Group);
        joint2Slider->setObjectName("joint2Slider");
        joint2Slider->setMinimum(-90);
        joint2Slider->setMaximum(90);
        joint2Slider->setOrientation(Qt::Orientation::Horizontal);
        joint2Slider->setTickPosition(QSlider::TickPosition::TicksBelow);

        horizontalLayout_7->addWidget(joint2Slider);

        joint2SpinBox = new QSpinBox(joint2Group);
        joint2SpinBox->setObjectName("joint2SpinBox");
        joint2SpinBox->setMinimum(-90);
        joint2SpinBox->setMaximum(90);

        horizontalLayout_7->addWidget(joint2SpinBox);


        verticalLayout_41->addWidget(joint2Group);

        joint3Group = new QGroupBox(jointTab);
        joint3Group->setObjectName("joint3Group");
        horizontalLayout_8 = new QHBoxLayout(joint3Group);
        horizontalLayout_8->setObjectName("horizontalLayout_8");
        joint3Slider = new QSlider(joint3Group);
        joint3Slider->setObjectName("joint3Slider");
        joint3Slider->setMinimum(-90);
        joint3Slider->setMaximum(90);
        joint3Slider->setOrientation(Qt::Orientation::Horizontal);
        joint3Slider->setTickPosition(QSlider::TickPosition::TicksBelow);

        horizontalLayout_8->addWidget(joint3Slider);

        joint3SpinBox = new QSpinBox(joint3Group);
        joint3SpinBox->setObjectName("joint3SpinBox");
        joint3SpinBox->setMinimum(-90);
        joint3SpinBox->setMaximum(90);

        horizontalLayout_8->addWidget(joint3SpinBox);


        verticalLayout_41->addWidget(joint3Group);

        joint4Group = new QGroupBox(jointTab);
        joint4Group->setObjectName("joint4Group");
        horizontalLayout_9 = new QHBoxLayout(joint4Group);
        horizontalLayout_9->setObjectName("horizontalLayout_9");
        joint4Slider = new QSlider(joint4Group);
        joint4Slider->setObjectName("joint4Slider");
        joint4Slider->setMinimum(-90);
        joint4Slider->setMaximum(90);
        joint4Slider->setOrientation(Qt::Orientation::Horizontal);
        joint4Slider->setTickPosition(QSlider::TickPosition::TicksBelow);

        horizontalLayout_9->addWidget(joint4Slider);

        joint4SpinBox = new QSpinBox(joint4Group);
        joint4SpinBox->setObjectName("joint4SpinBox");
        joint4SpinBox->setMinimum(-90);
        joint4SpinBox->setMaximum(90);

        horizontalLayout_9->addWidget(joint4SpinBox);


        verticalLayout_41->addWidget(joint4Group);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout_41->addItem(verticalSpacer);

        tabWidget->addTab(jointTab, QString());

        horizontalLayout_5->addWidget(tabWidget);

        MainWindow->setCentralWidget(centralwidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName("statusBar");
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "LittleArm Controller", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Server IP:", nullptr));
        serverIpEdit->setText(QCoreApplication::translate("MainWindow", "192.168.66.202", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Port:", nullptr));
        serverPortEdit->setText(QCoreApplication::translate("MainWindow", "5000", nullptr));
        connectButton->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        disconnectButton->setText(QCoreApplication::translate("MainWindow", "Disconnect", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "CAN Message", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "CAN ID:", nullptr));
        canIdEdit->setText(QCoreApplication::translate("MainWindow", "027", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Data:", nullptr));
        canDataEdit->setText(QCoreApplication::translate("MainWindow", "0800000000000000", nullptr));
        sendButton->setText(QCoreApplication::translate("MainWindow", "Send", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Split Data:", nullptr));
        canDataSplit1->setText(QCoreApplication::translate("MainWindow", "0800", nullptr));
        canDataSplit2->setText(QCoreApplication::translate("MainWindow", "0000", nullptr));
        canDataSplit3->setText(QCoreApplication::translate("MainWindow", "0000", nullptr));
        canDataSplit4->setText(QCoreApplication::translate("MainWindow", "0000", nullptr));
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
        showTimeCheckBox->setText(QCoreApplication::translate("MainWindow", "Show Time", nullptr));
        txGroupBox->setTitle(QCoreApplication::translate("MainWindow", "TX (Sent)", nullptr));
        txMonitorText->setStyleSheet(QString());
        rxGroupBox->setTitle(QCoreApplication::translate("MainWindow", "RX (Received)", nullptr));
        rxMonitorText->setStyleSheet(QString());
        monitorButton->setText(QCoreApplication::translate("MainWindow", "Start Monitor", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(canTab), QCoreApplication::translate("MainWindow", "CAN Monitor", nullptr));
        joint1Group->setTitle(QCoreApplication::translate("MainWindow", "Joint 1 (Base)", nullptr));
        joint1SpinBox->setSuffix(QCoreApplication::translate("MainWindow", "\302\260", nullptr));
        joint2Group->setTitle(QCoreApplication::translate("MainWindow", "Joint 2 (Shoulder)", nullptr));
        joint2SpinBox->setSuffix(QCoreApplication::translate("MainWindow", "\302\260", nullptr));
        joint3Group->setTitle(QCoreApplication::translate("MainWindow", "Joint 3 (Elbow)", nullptr));
        joint3SpinBox->setSuffix(QCoreApplication::translate("MainWindow", "\302\260", nullptr));
        joint4Group->setTitle(QCoreApplication::translate("MainWindow", "Joint 4 (Wrist)", nullptr));
        joint4SpinBox->setSuffix(QCoreApplication::translate("MainWindow", "\302\260", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(jointTab), QCoreApplication::translate("MainWindow", "Joint Control", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
