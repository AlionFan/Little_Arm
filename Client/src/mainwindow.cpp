#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDateTime>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    socket(new QTcpSocket(this)),
    monitorTimer(new QTimer(this)),
    isMonitoring(false)
{
    setupUi();
    setupConnections();
}

MainWindow::~MainWindow()
{
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
}

void MainWindow::setupConnections()
{
    // 连接按钮信号
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::connectToServer);
    connect(ui->disconnectButton, &QPushButton::clicked, this, &MainWindow::disconnectFromServer);
    connect(ui->sendButton, &QPushButton::clicked, this, &MainWindow::sendCANMessage);
    connect(ui->monitorButton, &QPushButton::clicked, this, &MainWindow::startMonitoring);
    
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

void MainWindow::handleSocketData()
{
    QByteArray data = socket->readAll();
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    ui->monitorText->append(QString("[%1] %2").arg(timestamp).arg(QString(data)));
}

void MainWindow::handleSocketError(QAbstractSocket::SocketError error)
{
    QString errorMsg = QString("Socket error: %1").arg(socket->errorString());
    ui->statusBar->showMessage(errorMsg);
    QMessageBox::warning(this, "Connection Error", errorMsg);
} 