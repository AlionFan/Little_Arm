#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QMap>
#include <QString>
#include <QStringList>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // CAN通信相关槽
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

    // 关节控制相关槽
    void onJoint1SliderChanged(int value);
    void onJoint2SliderChanged(int value);
    void onJoint3SliderChanged(int value);
    void onJoint4SliderChanged(int value);
    void onJoint1SpinBoxChanged(int value);
    void onJoint2SpinBoxChanged(int value);
    void onJoint3SpinBoxChanged(int value);
    void onJoint4SpinBoxChanged(int value);

    // 数据同步相关槽
    void onCanDataEditChanged(const QString &text);
    void onCanDataSplitChanged();

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

    // 新增：用于跟踪CAN ID分类
    QMap<QString, QStringList> canIdCategories;  // 存储每个CAN ID的最近3条消息
    QStringList canIdOrder;  // 保持CAN ID的显示顺序

    void setupUi();
    void setupConnections();
    void setupJointConnections();
    void setupPresetButtons();
    void setupStyles();
    void loadPresetMessages();
    void savePresetMessages();
    void appendTxMessage(const QString &message);
    void appendRxMessage(const QString &message);
    QString formatCollapsedMessage(const QString& message, int count);
    QString formatCanData(const QString &data);

    // 新增：数据同步辅助函数
    void updateSplitDataFromMain();
    void updateMainDataFromSplit();
    bool isValidHexString(const QString &str);
};

#endif // MAINWINDOW_H 