#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDateTime>
#include <QSettings>
#include <QFile>
#include <QDir>
#include <QMenu>
#include <QAction>
#include <QStyle>
#include <QStyleFactory>
#include <QRegularExpression>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    socket(new QTcpSocket(this)),
    monitorTimer(new QTimer(this)),
    isMonitoring(false),
    isCollapsing(false),
    collapseInterval(1000)
{
    setupUi();
    setupConnections();
    loadPresetMessages();
    setupPresetButtons();
    setupStyles();
}

MainWindow::~MainWindow()
{
    savePresetMessages();
    delete ui;
}

void MainWindow::setupUi()
{
    ui->setupUi(this);
    
    // 设置默认值
    ui->serverIpEdit->setText("192.168.66.202");  // 替换为树莓派的IP
    ui->serverPortEdit->setText("5000");
    ui->canIdEdit->setText("027");
    ui->canDataEdit->setText("0800000000000000");
    
    // 禁用发送按钮，直到连接建立
    ui->sendButton->setEnabled(false);
    ui->monitorButton->setEnabled(false);

    // 设置默认折叠间隔
    ui->collapseIntervalSpinBox->setValue(collapseInterval);
}

void MainWindow::setupConnections()
{
    // 连接按钮信号
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::connectToServer);
    connect(ui->disconnectButton, &QPushButton::clicked, this, &MainWindow::disconnectFromServer);
    connect(ui->sendButton, &QPushButton::clicked, this, &MainWindow::sendCANMessage);
    connect(ui->monitorButton, &QPushButton::clicked, this, &MainWindow::startMonitoring);
    
    // 消息折叠相关信号
    connect(ui->collapseCheckBox, &QCheckBox::toggled, this, &MainWindow::toggleMessageCollapse);
    connect(ui->collapseIntervalSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MainWindow::updateCollapseInterval);
    
    // Socket信号
    connect(socket, &QTcpSocket::connected, [this]() {
        ui->statusBar->showMessage("Connected to server");
        ui->sendButton->setEnabled(true);
        ui->monitorButton->setEnabled(true);
    });
    
    connect(socket, &QTcpSocket::disconnected, [this]() {
        ui->statusBar->showMessage("Disconnected from server");
        ui->sendButton->setEnabled(false);
        ui->monitorButton->setEnabled(false);
        stopMonitoring();
    });
    
    connect(socket, &QTcpSocket::readyRead, this, &MainWindow::handleSocketData);
    
    // 添加Qt5和Qt6兼容支持
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this, &MainWindow::handleSocketError);
#else
    connect(socket, &QTcpSocket::errorOccurred, this, &MainWindow::handleSocketError);
#endif
    
    // 监控定时器
    monitorTimer->setInterval(100);  // 100ms
    connect(monitorTimer, &QTimer::timeout, this, &MainWindow::updateMonitor);

    // 设置关节控制连接
    setupJointConnections();

    // 添加显示时间复选框的连接
    connect(ui->showTimeCheckBox, &QCheckBox::toggled, [this](bool checked) {
        ui->txMonitorText->clear();
        ui->rxMonitorText->clear();
    });
}

void MainWindow::setupPresetButtons()
{
    // 连接预设按钮信号
    QList<QPushButton*> presetButtons = {
        ui->presetButton1, ui->presetButton2, ui->presetButton3, ui->presetButton4, ui->presetButton5,
        ui->presetButton6, ui->presetButton7, ui->presetButton8, ui->presetButton9, ui->presetButton10
    };

    for (int i = 0; i < presetButtons.size(); ++i) {
        QPushButton* button = presetButtons[i];
        
        // 左键点击发送预设消息
        connect(button, &QPushButton::clicked, [this, i]() { sendPresetMessage(i + 1); });
        
        // 右键菜单设置预设
        button->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(button, &QPushButton::customContextMenuRequested, [this, i, button](const QPoint& pos) {
            QMenu menu(button);
            QAction* setAction = menu.addAction("设置为预设");
            QAction* clearAction = menu.addAction("清除预设");
            
            QAction* selectedAction = menu.exec(button->mapToGlobal(pos));
            
            if (selectedAction == setAction) {
                QString canId = ui->canIdEdit->text();
                QString canData = ui->canDataEdit->text();
                if (!canId.isEmpty() && !canData.isEmpty()) {
                    QString message = QString("%1 # %2").arg(canId).arg(formatCanData(canData));
                    presetMessages[i + 1] = message;
                    button->setText(QString("Preset %1: %2").arg(i + 1).arg(canId));
                    savePresetMessages();
                    QMessageBox::information(this, "成功", "预设消息已保存");
                } else {
                    QMessageBox::warning(this, "错误", "CAN ID和数据不能为空");
                }
            } else if (selectedAction == clearAction) {
                presetMessages.remove(i + 1);
                button->setText(QString("Preset %1").arg(i + 1));
                savePresetMessages();
                QMessageBox::information(this, "成功", "预设消息已清除");
            }
        });

        // 更新按钮文本
        if (presetMessages.contains(i + 1)) {
            QString message = presetMessages[i + 1];
            QStringList parts = message.split('#');
            if (parts.size() == 2) {
                button->setText(QString("Preset %1: %2").arg(i + 1).arg(parts[0].trimmed()));
            }
        }
    }
}

void MainWindow::loadPresetMessages()
{
    QSettings settings("CANClient", "PresetMessages");
    for (int i = 1; i <= 10; ++i) {
        QString key = QString("Preset%1").arg(i);
        QString message = settings.value(key).toString();
        if (!message.isEmpty()) {
            presetMessages[i] = message;
        }
    }
}

void MainWindow::savePresetMessages()
{
    QSettings settings("CANClient", "PresetMessages");
    for (auto it = presetMessages.begin(); it != presetMessages.end(); ++it) {
        QString key = QString("Preset%1").arg(it.key());
        settings.setValue(key, it.value());
    }
}

void MainWindow::sendPresetMessage(int presetIndex)
{
    if (presetMessages.contains(presetIndex)) {
        QString message = presetMessages[presetIndex];
        QStringList parts = message.split('#');
        if (parts.size() == 2) {
            ui->canIdEdit->setText(parts[0].trimmed());
            ui->canDataEdit->setText(parts[1].trimmed());
            sendCANMessage();
        }
    }
}

void MainWindow::toggleMessageCollapse(bool checked)
{
    isCollapsing = checked;
    if (!checked) {
        messageCount.clear();
        lastMessage.clear();
    }
}

void MainWindow::updateCollapseInterval(int value)
{
    collapseInterval = value;
}

QString MainWindow::formatCollapsedMessage(const QString& message, int count)
{
    if (ui->showTimeCheckBox->isChecked()) {
        return QString("[%1] %2 (repeated %3 times)")
                .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
                .arg(message)
                .arg(count);
    } else {
        return QString("%1 (repeated %2 times)")
                .arg(message)
                .arg(count);
    }
}

QString MainWindow::formatCanData(const QString &data)
{
    QString formatted;
    for (int i = 0; i < data.length(); i += 2) {
        if (i > 0) formatted += " ";
        formatted += data.mid(i, 2);
    }
    return formatted;
}

void MainWindow::handleSocketData()
{
    QByteArray data = socket->readAll();
    QString message = QString(data);
    
    // 使用QRegularExpression替换QRegExp
    QRegularExpression rx("\\s*can0\\s+(\\d{3})\\s+\\[8\\]\\s+([0-9A-F]{2}\\s+[0-9A-F]{2}\\s+[0-9A-F]{2}\\s+[0-9A-F]{2}\\s+[0-9A-F]{2}\\s+[0-9A-F]{2}\\s+[0-9A-F]{2}\\s+[0-9A-F]{2})");
    QRegularExpressionMatch match = rx.match(message);
    
    if (match.hasMatch()) {
        QString canId = match.captured(1);
        QString canData = match.captured(2);
        
        // 格式化显示消息
        QString formattedMessage;
        if (ui->showTimeCheckBox->isChecked()) {
            formattedMessage = QString("[%1] %2 # %3")
                .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
                .arg(canId)
                .arg(canData);
        } else {
            formattedMessage = QString("%1 # %2")
                .arg(canId)
                .arg(canData);
        }

        // 更新CAN ID分类
        if (!canIdCategories.contains(canId)) {
            // 新的CAN ID，添加到顺序列表
            canIdOrder.append(canId);
            canIdCategories[canId] = QStringList();
        }

        // 更新该CAN ID的消息列表
        QStringList &messages = canIdCategories[canId];
        messages.append(formattedMessage);
        while (messages.size() > 3) {
            messages.removeFirst();
        }

        // 清空显示区域
        ui->rxMonitorText->clear();

        // 按顺序显示所有CAN ID的消息
        for (const QString &id : canIdOrder) {
            ui->rxMonitorText->append(QString("=== CAN ID: %1 ===").arg(id));
            const QStringList &idMessages = canIdCategories[id];
            
            // 显示最多3条消息，不足的用空行填充
            for (int i = 0; i < 3; i++) {
                if (i < idMessages.size()) {
                    ui->rxMonitorText->append(idMessages[i]);
                } else {
                    ui->rxMonitorText->append("");
                }
            }
            ui->rxMonitorText->append(""); // 添加一个空行作为分隔
        }
    } else if (message.startsWith("SEND:")) {
        // 处理发送的消息
        message = message.mid(5);
        QStringList parts = message.split('#');
        if (parts.size() == 2) {
            message = QString("%1 # %2").arg(parts[0]).arg(formatCanData(parts[1]));
            appendTxMessage(message);
        }
    }
}

void MainWindow::connectToServer()
{
    QString ip = ui->serverIpEdit->text();
    int port = ui->serverPortEdit->text().toInt();
    
    socket->connectToHost(ip, port);
}

void MainWindow::disconnectFromServer()
{
    socket->disconnectFromHost();
}

void MainWindow::sendCANMessage()
{
    if (socket->state() != QAbstractSocket::ConnectedState) {
        return;
    }
    
    QString canId = ui->canIdEdit->text();
    QString canData = ui->canDataEdit->text();
    
    // 显示格式化的消息
    QString displayMessage = QString("%1 # %2").arg(canId).arg(formatCanData(canData));
    appendTxMessage(displayMessage);
    
    // 发送到服务器时使用紧凑格式
    socket->write(QString("SEND:%1#%2").arg(canId).arg(canData).toUtf8());
}

void MainWindow::startMonitoring()
{
    if (!isMonitoring) {
        isMonitoring = true;
        ui->monitorButton->setText("Stop Monitor");
        monitorTimer->start();
    } else {
        stopMonitoring();
    }
}

void MainWindow::stopMonitoring()
{
    isMonitoring = false;
    ui->monitorButton->setText("Start Monitor");
    monitorTimer->stop();
}

void MainWindow::updateMonitor()
{
    if (socket->state() == QAbstractSocket::ConnectedState) {
        socket->write("MONITOR");
    }
}

void MainWindow::handleSocketError(QAbstractSocket::SocketError error)
{
    QString errorMsg = QString("Socket error: %1").arg(socket->errorString());
    ui->statusBar->showMessage(errorMsg);
    QMessageBox::warning(this, "Connection Error", errorMsg);
}

void MainWindow::setupJointConnections()
{
    // Joint 1
    connect(ui->joint1Slider, &QSlider::valueChanged, this, &MainWindow::onJoint1SliderChanged);
    connect(ui->joint1SpinBox, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &MainWindow::onJoint1SpinBoxChanged);

    // Joint 2
    connect(ui->joint2Slider, &QSlider::valueChanged, this, &MainWindow::onJoint2SliderChanged);
    connect(ui->joint2SpinBox, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &MainWindow::onJoint2SpinBoxChanged);

    // Joint 3
    connect(ui->joint3Slider, &QSlider::valueChanged, this, &MainWindow::onJoint3SliderChanged);
    connect(ui->joint3SpinBox, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &MainWindow::onJoint3SpinBoxChanged);

    // Joint 4
    connect(ui->joint4Slider, &QSlider::valueChanged, this, &MainWindow::onJoint4SliderChanged);
    connect(ui->joint4SpinBox, QOverload<int>::of(&QSpinBox::valueChanged), 
            this, &MainWindow::onJoint4SpinBoxChanged);
}

// Joint 1 控制
void MainWindow::onJoint1SliderChanged(int value)
{
    if (ui->joint1SpinBox->value() != value) {
        ui->joint1SpinBox->setValue(value);
        // TODO: 发送关节1控制命令
    }
}

void MainWindow::onJoint1SpinBoxChanged(int value)
{
    if (ui->joint1Slider->value() != value) {
        ui->joint1Slider->setValue(value);
        // TODO: 发送关节1控制命令
    }
}

// Joint 2 控制
void MainWindow::onJoint2SliderChanged(int value)
{
    if (ui->joint2SpinBox->value() != value) {
        ui->joint2SpinBox->setValue(value);
        // TODO: 发送关节2控制命令
    }
}

void MainWindow::onJoint2SpinBoxChanged(int value)
{
    if (ui->joint2Slider->value() != value) {
        ui->joint2Slider->setValue(value);
        // TODO: 发送关节2控制命令
    }
}

// Joint 3 控制
void MainWindow::onJoint3SliderChanged(int value)
{
    if (ui->joint3SpinBox->value() != value) {
        ui->joint3SpinBox->setValue(value);
        // TODO: 发送关节3控制命令
    }
}

void MainWindow::onJoint3SpinBoxChanged(int value)
{
    if (ui->joint3Slider->value() != value) {
        ui->joint3Slider->setValue(value);
        // TODO: 发送关节3控制命令
    }
}

// Joint 4 控制
void MainWindow::onJoint4SliderChanged(int value)
{
    if (ui->joint4SpinBox->value() != value) {
        ui->joint4SpinBox->setValue(value);
        // TODO: 发送关节4控制命令
    }
}

void MainWindow::onJoint4SpinBoxChanged(int value)
{
    if (ui->joint4Slider->value() != value) {
        ui->joint4Slider->setValue(value);
        // TODO: 发送关节4控制命令
    }
}

void MainWindow::setupStyles()
{
    // 设置窗口大小
    resize(1080, 720);
    setMinimumSize(1080, 720);

    // 设置全局样式
    QString styleSheet = R"(
        QMainWindow {
            background-color: #f0f0f0;
        }
        QGroupBox {
            border: 2px solid #cccccc;
            border-radius: 6px;
            margin-top: 1ex;
            font-weight: bold;
            background-color: #ffffff;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px;
            color: #333333;
        }
        QPushButton {
            background-color: #0078d4;
            color: white;
            border: none;
            border-radius: 4px;
            padding: 5px 15px;
            min-height: 25px;
        }
        QPushButton:hover {
            background-color: #1084d8;
        }
        QPushButton:pressed {
            background-color: #006cbd;
        }
        QLineEdit {
            padding: 4px;
            border: 1px solid #cccccc;
            border-radius: 4px;
            background-color: white;
        }
        QTextEdit {
            border: 1px solid #cccccc;
            border-radius: 4px;
            background-color: white;
            font-family: "Consolas", "Monaco", monospace;
        }
        QSlider::groove:horizontal {
            border: 1px solid #999999;
            height: 8px;
            background: #ffffff;
            margin: 2px 0;
            border-radius: 4px;
        }
        QSlider::handle:horizontal {
            background: #0078d4;
            border: none;
            width: 18px;
            margin: -5px 0;
            border-radius: 9px;
        }
        QSlider::handle:horizontal:hover {
            background: #1084d8;
        }
        QSpinBox {
            padding: 4px;
            border: 1px solid #cccccc;
            border-radius: 4px;
            background-color: white;
        }
        QTabWidget::pane {
            border: 1px solid #cccccc;
            border-radius: 4px;
            background-color: white;
        }
        QTabBar::tab {
            background-color: #f0f0f0;
            border: 1px solid #cccccc;
            border-bottom: none;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
            padding: 8px 16px;
            margin-right: 2px;
        }
        QTabBar::tab:selected {
            background-color: white;
            border-bottom: none;
        }
        QTabBar::tab:hover {
            background-color: #e5e5e5;
        }
    )";

    setStyleSheet(styleSheet);

    // 设置预设按钮的特殊样式
    QString presetButtonStyle = R"(
        QPushButton[objectName^="presetButton"] {
            background-color: #5c2d91;
            min-width: 100px;
        }
        QPushButton[objectName^="presetButton"]:hover {
            background-color: #6b3a9e;
        }
        QPushButton[objectName^="presetButton"]:pressed {
            background-color: #4c2277;
        }
    )";

    ui->presetGroupBox->setStyleSheet(presetButtonStyle);

    // 设置监控文本框的字体
    QFont monitorFont("Consolas", 10);
    ui->txMonitorText->setFont(monitorFont);
    ui->rxMonitorText->setFont(monitorFont);

    // 设置滑块的刻度
    ui->joint1Slider->setTickInterval(10);
    ui->joint2Slider->setTickInterval(10);
    ui->joint3Slider->setTickInterval(10);
    ui->joint4Slider->setTickInterval(10);

    // 设置分组框的间距
    ui->joint1Group->layout()->setContentsMargins(10, 15, 10, 10);
    ui->joint2Group->layout()->setContentsMargins(10, 15, 10, 10);
    ui->joint3Group->layout()->setContentsMargins(10, 15, 10, 10);
    ui->joint4Group->layout()->setContentsMargins(10, 15, 10, 10);
}

void MainWindow::appendTxMessage(const QString &message)
{
    QString formattedMessage;
    if (ui->showTimeCheckBox->isChecked()) {
        formattedMessage = QString("[%1] %2")
            .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
            .arg(message);
    } else {
        formattedMessage = message;
    }
    ui->txMonitorText->append(formattedMessage);
}

void MainWindow::appendRxMessage(const QString &message)
{
    QString formattedMessage;
    if (ui->showTimeCheckBox->isChecked()) {
        formattedMessage = QString("[%1] %2")
            .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
            .arg(message);
    } else {
        formattedMessage = message;
    }
    ui->rxMonitorText->append(formattedMessage);
} 