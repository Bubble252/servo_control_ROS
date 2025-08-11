/****************************************************************************
** Meta object code from reading C++ file 'qt_servo_control.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "include/servo_qt_node/qt_servo_control.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qt_servo_control.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QtServoControl_t {
    QByteArrayData data[7];
    char stringdata0[77];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QtServoControl_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QtServoControl_t qt_meta_stringdata_QtServoControl = {
    {
QT_MOC_LITERAL(0, 0, 14), // "QtServoControl"
QT_MOC_LITERAL(1, 15, 16), // "onSlider1Changed"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 5), // "value"
QT_MOC_LITERAL(4, 39, 16), // "onSlider2Changed"
QT_MOC_LITERAL(5, 56, 7), // "rosSpin"
QT_MOC_LITERAL(6, 64, 12) // "updateCharts"

    },
    "QtServoControl\0onSlider1Changed\0\0value\0"
    "onSlider2Changed\0rosSpin\0updateCharts"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QtServoControl[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x08 /* Private */,
       4,    1,   37,    2, 0x08 /* Private */,
       5,    0,   40,    2, 0x08 /* Private */,
       6,    0,   41,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void QtServoControl::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<QtServoControl *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onSlider1Changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->onSlider2Changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->rosSpin(); break;
        case 3: _t->updateCharts(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject QtServoControl::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_QtServoControl.data,
    qt_meta_data_QtServoControl,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *QtServoControl::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QtServoControl::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QtServoControl.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int QtServoControl::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
