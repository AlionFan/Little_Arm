#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDateTime>
#include <QSettings>
#include <QFile>
#include <QDir>
#include <QMenu>
#include <QAction>

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
                    QString message = QString("%1#%2").arg(canId).arg(canData);
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
                button->setText(QString("Preset %1: %2").arg(i + 1).arg(parts[0]));
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
            ui->canIdEdit->setText(parts[0]);
            ui->canDataEdit->setText(parts[1]);
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
    return QString("[%1] %2 (repeated %3 times)")
            .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
            .arg(message)
            .arg(count);
}

void MainWindow::handleSocketData()
{
    QByteArray data = socket->readAll();
    QString message = QString(data);
    
    if (isCollapsing) {
        QString key = message;
        if (messageCount.contains(key)) {
            messageCount[key]++;
            if (messageCount[key] % 10 == 0) {  // 每10次更新一次显示
                QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
                ui->monitorText->append(formatCollapsedMessage(message, messageCount[key]));
            }
        } else {
            messageCount[key] = 1;
            lastMessage[key] = message;
            ui->monitorText->append(QString("[%1] [RX] %2")
                    .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
                    .arg(message));
        }
    } else {
        ui->monitorText->append(QString("[%1] [RX] %2")
                .arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"))
                .arg(message));
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
    QString message = QString("SEND:%1#%2").arg(canId).arg(canData);
    
    // 显示发送的消息
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString displayMessage = QString("[%1] [TX] %2").arg(timestamp).arg(message);
    ui->monitorText->append(displayMessage);
    
    socket->write(message.toUtf8());
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