#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QMap>
#include <QString>

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
    void sendPresetMessage(int presetIndex);
    void toggleMessageCollapse(bool checked);
    void updateCollapseInterval(int value);

private:
    Ui::MainWindow *ui;
    QTcpSocket *socket;
    QTimer *monitorTimer;
    bool isMonitoring;
    bool isCollapsing;
    int collapseInterval;
    QMap<QString, int> messageCount;
    QMap<QString, QString> lastMessage;
    QMap<int, QString> presetMessages;

    void setupUi();
    void setupConnections();
    void loadPresetMessages();
    void savePresetMessages();
    void setupPresetButtons();
    QString formatCollapsedMessage(const QString& message, int count);
};

#endif // MAINWINDOW_H 