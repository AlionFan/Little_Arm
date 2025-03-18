#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QQuickWidget>
#include <QQmlContext>
#include <QQmlEngine>
#include <QSettings>
#include <QQuickItem>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
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
    void setupConnections();
    void setupPresetButtons();
    void setup3DView();
    void updateRobotModel();
    bool isValidHexString(const QString &str);
    void updateSplitDataFromMain();
    void updateMainDataFromSplit();
    void savePresetMessage(int index);
    void loadPresetMessage(int index);
    void clearPresetMessage(int index);
    void editPresetDisplay(int index);

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

    // 新增：3D视图相关
    QQuickWidget *robot3dView;
    QQmlEngine *qmlEngine;
    QQmlContext *qmlContext;
    QQuickItem *rootObject;
    QSettings *settings;

    void setupUi();
    void setupJointConnections();
    void setupStyles();
    void loadPresetMessages();
    void savePresetMessages();
    void appendTxMessage(const QString &message);
    void appendRxMessage(const QString &message);
    QString formatCollapsedMessage(const QString& message, int count);
    QString formatCanData(const QString &data);
};

#endif // MAINWINDOW_H 