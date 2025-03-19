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
#include <QInputDialog>
#include <QQmlEngine>
#include <QQmlContext>
#include <QCoreApplication>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    socket(new QTcpSocket(this)),
    monitorTimer(new QTimer(this)),
    isMonitoring(false),
    isCollapsing(false),
    collapseInterval(1000),
    robot3dView(nullptr),
    qmlEngine(nullptr),
    qmlContext(nullptr),
    rootObject(nullptr)
{
    setupUi();
    setupConnections();
    loadPresetMessages();
    setupPresetButtons();
    setupStyles();
    setup3DView();
}

MainWindow::~MainWindow()
{
    savePresetMessages();
    delete ui;
}

void MainWindow::setupUi()
{
    ui->setupUi(this);
    
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
    
    // GIM控制页面信号连接
    connect(ui->gimInputEdit, &QLineEdit::returnPressed, this, &MainWindow::onGimInputEditReturnPressed);
    
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

    // 添加数据同步连接
    connect(ui->canDataEdit, &QLineEdit::textChanged,
            this, &MainWindow::onCanDataEditChanged);
    
    connect(ui->canDataSplit1, &QLineEdit::textChanged,
            this, &MainWindow::onCanDataSplitChanged);
    connect(ui->canDataSplit2, &QLineEdit::textChanged,
            this, &MainWindow::onCanDataSplitChanged);
    connect(ui->canDataSplit3, &QLineEdit::textChanged,
            this, &MainWindow::onCanDataSplitChanged);
    connect(ui->canDataSplit4, &QLineEdit::textChanged,
            this, &MainWindow::onCanDataSplitChanged);
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
            QAction* editDisplayAction = menu.addAction("修改显示");
            
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
            } else if (selectedAction == editDisplayAction) {
                bool ok;
                QString currentText = button->text();
                QString newText = QInputDialog::getText(this, "修改显示",
                    "请输入新的显示内容:", QLineEdit::Normal, currentText, &ok);
                if (ok && !newText.isEmpty()) {
                    button->setText(newText);
                    // 保存显示文本到设置中
                    QSettings settings("CANClient", "PresetDisplay");
                    settings.setValue(QString("Preset%1Display").arg(i + 1), newText);
                }
            }
        });

        // 更新按钮文本
        if (presetMessages.contains(i + 1)) {
            QString message = presetMessages[i + 1];
            QStringList parts = message.split('#');
            if (parts.size() == 2) {
                // 尝试从设置中读取自定义显示文本
                QSettings settings("CANClient", "PresetDisplay");
                QString customDisplay = settings.value(QString("Preset%1Display").arg(i + 1)).toString();
                if (!customDisplay.isEmpty()) {
                    button->setText(customDisplay);
                } else {
                    button->setText(QString("Preset %1: %2").arg(i + 1).arg(parts[0].trimmed()));
                }
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
            ui->canDataEdit->setText(parts[1].trimmed().remove(" "));
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
        updateRobotModel();
    }
}

void MainWindow::onJoint1SpinBoxChanged(int value)
{
    if (ui->joint1Slider->value() != value) {
        ui->joint1Slider->setValue(value);
    }
}

// Joint 2 控制
void MainWindow::onJoint2SliderChanged(int value)
{
    if (ui->joint2SpinBox->value() != value) {
        ui->joint2SpinBox->setValue(value);
        updateRobotModel();
    }
}

void MainWindow::onJoint2SpinBoxChanged(int value)
{
    if (ui->joint2Slider->value() != value) {
        ui->joint2Slider->setValue(value);
    }
}

// Joint 3 控制
void MainWindow::onJoint3SliderChanged(int value)
{
    if (ui->joint3SpinBox->value() != value) {
        ui->joint3SpinBox->setValue(value);
        updateRobotModel();
    }
}

void MainWindow::onJoint3SpinBoxChanged(int value)
{
    if (ui->joint3Slider->value() != value) {
        ui->joint3Slider->setValue(value);
    }
}

// Joint 4 控制
void MainWindow::onJoint4SliderChanged(int value)
{
    if (ui->joint4SpinBox->value() != value) {
        ui->joint4SpinBox->setValue(value);
        updateRobotModel();
    }
}

void MainWindow::onJoint4SpinBoxChanged(int value)
{
    if (ui->joint4Slider->value() != value) {
        ui->joint4Slider->setValue(value);
    }
}

void MainWindow::setupStyles()
{
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

void MainWindow::onCanDataEditChanged(const QString &text)
{
    if (text.length() == 16 && isValidHexString(text)) {
        updateSplitDataFromMain();
    }
}

void MainWindow::onCanDataSplitChanged()
{
    QString split1 = ui->canDataSplit1->text().toUpper();
    QString split2 = ui->canDataSplit2->text().toUpper();
    QString split3 = ui->canDataSplit3->text().toUpper();
    QString split4 = ui->canDataSplit4->text().toUpper();

    // 检查所有分段是否都是有效的4位十六进制数
    if (split1.length() == 4 && split2.length() == 4 && 
        split3.length() == 4 && split4.length() == 4 &&
        isValidHexString(split1) && isValidHexString(split2) &&
        isValidHexString(split3) && isValidHexString(split4)) {
        updateMainDataFromSplit();
    }
}

void MainWindow::updateSplitDataFromMain()
{
    QString mainData = ui->canDataEdit->text().toUpper();
    if (mainData.length() == 16) {
        ui->canDataSplit1->setText(mainData.mid(0, 4));
        ui->canDataSplit2->setText(mainData.mid(4, 4));
        ui->canDataSplit3->setText(mainData.mid(8, 4));
        ui->canDataSplit4->setText(mainData.mid(12, 4));
    }
}

void MainWindow::updateMainDataFromSplit()
{
    QString combinedData = ui->canDataSplit1->text() +
                          ui->canDataSplit2->text() +
                          ui->canDataSplit3->text() +
                          ui->canDataSplit4->text();
    ui->canDataEdit->setText(combinedData.toUpper());
}

bool MainWindow::isValidHexString(const QString &str)
{
    QRegularExpression hexRegex("^[0-9A-Fa-f]+$");
    return hexRegex.match(str).hasMatch();
}

void MainWindow::setup3DView()
{
    // 使用UI中已存在的QQuickWidget
    robot3dView = ui->robot3dView;
    robot3dView->setResizeMode(QQuickWidget::SizeRootObjectToView);

    qmlEngine = robot3dView->engine();
    qmlContext = robot3dView->rootContext();

    // 从资源文件加载QML
    robot3dView->setSource(QUrl("qrc:/src/robot3d.qml"));

    if (robot3dView->status() == QQuickWidget::Error) {
        qDebug() << "QML加载错误:";
        for (const QQmlError &error : robot3dView->errors()) {
            qDebug() << error.toString();
        }
    } else {
        qDebug() << "QML加载成功";
        rootObject = robot3dView->rootObject();
        if (!rootObject) {
            qDebug() << "无法获取根对象";
        }
    }
}

void MainWindow::updateRobotModel()
{
    if (!rootObject) return;
    
    // 更新关节角度
    rootObject->setProperty("joint1Angle", ui->joint1Slider->value());
    rootObject->setProperty("joint2Angle", ui->joint2Slider->value());
    rootObject->setProperty("joint3Angle", ui->joint3Slider->value());
    rootObject->setProperty("joint4Angle", ui->joint4Slider->value());
}

void MainWindow::onGimInputEditReturnPressed()
{
    // 获取输入的浮点数
    QString input = ui->gimInputEdit->text().trimmed();
    bool ok;
    float floatValue = input.toFloat(&ok);
    
    if (!ok) {
        ui->gimOutputText->setText("错误：请输入有效的浮点数");
        return;
    }
    
    // 将浮点数转换为小端序十六进制
    union {
        float f;
        quint32 i;
    } converter;
    
    converter.f = floatValue;
    
    // 获取小端序字节
    quint8 bytes[4];
    bytes[0] = converter.i & 0xFF;          // 最低位字节
    bytes[1] = (converter.i >> 8) & 0xFF;   // 次低位字节
    bytes[2] = (converter.i >> 16) & 0xFF;  // 次高位字节
    bytes[3] = (converter.i >> 24) & 0xFF;  // 最高位字节
    
    // 格式化为十六进制字符串
    QString hexString = QString("%1%2%3%4")
        .arg(bytes[0], 2, 16, QChar('0'))
        .arg(bytes[1], 2, 16, QChar('0'))
        .arg(bytes[2], 2, 16, QChar('0'))
        .arg(bytes[3], 2, 16, QChar('0'));
    
    // 显示结果
    QString result = QString("浮点数: %1\n十六进制小端序: 0x%2\n")
        .arg(floatValue)
        .arg(hexString.toUpper());
    
    // 添加字节顺序说明
    result += QString("字节顺序: %1 %2 %3 %4")
        .arg(QString("0x%1").arg(bytes[0], 2, 16, QChar('0')).toUpper())
        .arg(QString("0x%1").arg(bytes[1], 2, 16, QChar('0')).toUpper())
        .arg(QString("0x%1").arg(bytes[2], 2, 16, QChar('0')).toUpper())
        .arg(QString("0x%1").arg(bytes[3], 2, 16, QChar('0')).toUpper());
    
    ui->gimOutputText->setText(result);
}
