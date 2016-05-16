/****************************************************************************
** Meta object code from reading C++ file 'gui.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "gui.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GUI[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
       5,    4,    4,    4, 0x05,

 // slots: signature, parameters, type, tag, flags
      24,    4,    4,    4, 0x0a,
      48,    4,    4,    4, 0x0a,
      81,   77,    4,    4, 0x0a,
     120,    4,    4,    4, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GUI[] = {
    "GUI\0\0signal_updateGUI()\0on_updateGUI_recieved()\0"
    "on_buttonStopRobot_clicked()\0v,w\0"
    "on_update_canvas_recieved(float,float)\0"
    "on_checks_changed()\0"
};

void GUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GUI *_t = static_cast<GUI *>(_o);
        switch (_id) {
        case 0: _t->signal_updateGUI(); break;
        case 1: _t->on_updateGUI_recieved(); break;
        case 2: _t->on_buttonStopRobot_clicked(); break;
        case 3: _t->on_update_canvas_recieved((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 4: _t->on_checks_changed(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GUI::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GUI::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_GUI,
      qt_meta_data_GUI, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GUI::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GUI::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GUI))
        return static_cast<void*>(const_cast< GUI*>(this));
    return QWidget::qt_metacast(_clname);
}

int GUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void GUI::signal_updateGUI()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
