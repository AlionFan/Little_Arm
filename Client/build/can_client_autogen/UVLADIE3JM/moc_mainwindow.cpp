/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.8.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/mainwindow.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {

#ifdef QT_MOC_HAS_STRINGDATA
struct qt_meta_stringdata_CLASSMainWindowENDCLASS_t {};
constexpr auto qt_meta_stringdata_CLASSMainWindowENDCLASS = QtMocHelpers::stringData(
    "MainWindow",
    "connectToServer",
    "",
    "disconnectFromServer",
    "sendCANMessage",
    "startMonitoring",
    "stopMonitoring",
    "handleSocketData",
    "handleSocketError",
    "QAbstractSocket::SocketError",
    "error",
    "updateMonitor",
    "sendPresetMessage",
    "presetIndex",
    "toggleMessageCollapse",
    "checked",
    "updateCollapseInterval",
    "value",
    "onJoint1SliderChanged",
    "onJoint2SliderChanged",
    "onJoint3SliderChanged",
    "onJoint4SliderChanged",
    "onJoint1SpinBoxChanged",
    "onJoint2SpinBoxChanged",
    "onJoint3SpinBoxChanged",
    "onJoint4SpinBoxChanged",
    "onCanDataEditChanged",
    "text",
    "onCanDataSplitChanged"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_CLASSMainWindowENDCLASS[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
      21,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  140,    2, 0x08,    1 /* Private */,
       3,    0,  141,    2, 0x08,    2 /* Private */,
       4,    0,  142,    2, 0x08,    3 /* Private */,
       5,    0,  143,    2, 0x08,    4 /* Private */,
       6,    0,  144,    2, 0x08,    5 /* Private */,
       7,    0,  145,    2, 0x08,    6 /* Private */,
       8,    1,  146,    2, 0x08,    7 /* Private */,
      11,    0,  149,    2, 0x08,    9 /* Private */,
      12,    1,  150,    2, 0x08,   10 /* Private */,
      14,    1,  153,    2, 0x08,   12 /* Private */,
      16,    1,  156,    2, 0x08,   14 /* Private */,
      18,    1,  159,    2, 0x08,   16 /* Private */,
      19,    1,  162,    2, 0x08,   18 /* Private */,
      20,    1,  165,    2, 0x08,   20 /* Private */,
      21,    1,  168,    2, 0x08,   22 /* Private */,
      22,    1,  171,    2, 0x08,   24 /* Private */,
      23,    1,  174,    2, 0x08,   26 /* Private */,
      24,    1,  177,    2, 0x08,   28 /* Private */,
      25,    1,  180,    2, 0x08,   30 /* Private */,
      26,    1,  183,    2, 0x08,   32 /* Private */,
      28,    0,  186,    2, 0x08,   34 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   13,
    QMetaType::Void, QMetaType::Bool,   15,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::Int,   17,
    QMetaType::Void, QMetaType::QString,   27,
    QMetaType::Void,

       0        // eod
};

Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_CLASSMainWindowENDCLASS.offsetsAndSizes,
    qt_meta_data_CLASSMainWindowENDCLASS,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_CLASSMainWindowENDCLASS_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<MainWindow, std::true_type>,
        // method 'connectToServer'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'disconnectFromServer'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'sendCANMessage'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'startMonitoring'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'stopMonitoring'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'handleSocketData'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'handleSocketError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QAbstractSocket::SocketError, std::false_type>,
        // method 'updateMonitor'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'sendPresetMessage'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'toggleMessageCollapse'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'updateCollapseInterval'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint1SliderChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint2SliderChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint3SliderChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint4SliderChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint1SpinBoxChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint2SpinBoxChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint3SpinBoxChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onJoint4SpinBoxChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>,
        // method 'onCanDataEditChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'onCanDataSplitChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>
    >,
    nullptr
} };

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->connectToServer(); break;
        case 1: _t->disconnectFromServer(); break;
        case 2: _t->sendCANMessage(); break;
        case 3: _t->startMonitoring(); break;
        case 4: _t->stopMonitoring(); break;
        case 5: _t->handleSocketData(); break;
        case 6: _t->handleSocketError((*reinterpret_cast< std::add_pointer_t<QAbstractSocket::SocketError>>(_a[1]))); break;
        case 7: _t->updateMonitor(); break;
        case 8: _t->sendPresetMessage((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 9: _t->toggleMessageCollapse((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 10: _t->updateCollapseInterval((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 11: _t->onJoint1SliderChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 12: _t->onJoint2SliderChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 13: _t->onJoint3SliderChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 14: _t->onJoint4SliderChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 15: _t->onJoint1SpinBoxChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 16: _t->onJoint2SpinBoxChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 17: _t->onJoint3SpinBoxChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 18: _t->onJoint4SpinBoxChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 19: _t->onCanDataEditChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 20: _t->onCanDataSplitChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 6:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QAbstractSocket::SocketError >(); break;
            }
            break;
        }
    }
}

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CLASSMainWindowENDCLASS.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 21)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 21;
    }
    return _id;
}
QT_WARNING_POP
