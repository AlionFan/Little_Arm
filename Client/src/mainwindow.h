#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void connectToServer();
    void disconnectFromServer();
    void sendCANMessage();
    void startMonitoring();
    void stopMonitoring();
    void handleSocketData();
    void handleSocketError(QAbstractSocket::SocketError error);
    void updateMonitor();

private:
    Ui::MainWindow *ui;
    QTcpSocket *socket;
    QTimer *monitorTimer;
    bool isMonitoring;

    void setupUi();
    void setupConnections();
};

#endif // MAINWINDOW_H 