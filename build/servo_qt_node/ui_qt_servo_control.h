/********************************************************************************
** Form generated from reading UI file 'qt_servo_control.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QT_SERVO_CONTROL_H
#define UI_QT_SERVO_CONTROL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QtServoControl
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label1;
    QSlider *slider1;
    QLabel *label2;
    QSlider *slider2;
    QWidget *positionChartPlaceholder;
    QWidget *speedChartPlaceholder;
    QWidget *effortChartPlaceholder;

    void setupUi(QWidget *QtServoControl)
    {
        if (QtServoControl->objectName().isEmpty())
            QtServoControl->setObjectName(QString::fromUtf8("QtServoControl"));
        QtServoControl->resize(800, 600);
        verticalLayout = new QVBoxLayout(QtServoControl);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label1 = new QLabel(QtServoControl);
        label1->setObjectName(QString::fromUtf8("label1"));

        verticalLayout->addWidget(label1);

        slider1 = new QSlider(QtServoControl);
        slider1->setObjectName(QString::fromUtf8("slider1"));
        slider1->setOrientation(Qt::Horizontal);
        slider1->setMinimum(-314);
        slider1->setMaximum(314);
        slider1->setValue(0);
        slider1->setSingleStep(1);

        verticalLayout->addWidget(slider1);

        label2 = new QLabel(QtServoControl);
        label2->setObjectName(QString::fromUtf8("label2"));

        verticalLayout->addWidget(label2);

        slider2 = new QSlider(QtServoControl);
        slider2->setObjectName(QString::fromUtf8("slider2"));
        slider2->setOrientation(Qt::Horizontal);
        slider2->setMinimum(-314);
        slider2->setMaximum(314);
        slider2->setValue(0);
        slider2->setSingleStep(1);

        verticalLayout->addWidget(slider2);

        positionChartPlaceholder = new QWidget(QtServoControl);
        positionChartPlaceholder->setObjectName(QString::fromUtf8("positionChartPlaceholder"));
        positionChartPlaceholder->setMinimumHeight(150);

        verticalLayout->addWidget(positionChartPlaceholder);

        speedChartPlaceholder = new QWidget(QtServoControl);
        speedChartPlaceholder->setObjectName(QString::fromUtf8("speedChartPlaceholder"));
        speedChartPlaceholder->setMinimumHeight(150);

        verticalLayout->addWidget(speedChartPlaceholder);

        effortChartPlaceholder = new QWidget(QtServoControl);
        effortChartPlaceholder->setObjectName(QString::fromUtf8("effortChartPlaceholder"));
        effortChartPlaceholder->setMinimumHeight(150);

        verticalLayout->addWidget(effortChartPlaceholder);


        retranslateUi(QtServoControl);

        QMetaObject::connectSlotsByName(QtServoControl);
    } // setupUi

    void retranslateUi(QWidget *QtServoControl)
    {
        label1->setText(QApplication::translate("QtServoControl", "Servo 1 Angle (rad): 0.00", nullptr));
        label2->setText(QApplication::translate("QtServoControl", "Servo 2 Angle (rad): 0.00", nullptr));
        positionChartPlaceholder->setStyleSheet(QApplication::translate("QtServoControl", "background-color: #f0f0f0; border: 1px solid #ccc;", nullptr));
#ifndef QT_NO_TOOLTIP
        positionChartPlaceholder->setToolTip(QApplication::translate("QtServoControl", "Position Chart Placeholder (rad)", nullptr));
#endif // QT_NO_TOOLTIP
        speedChartPlaceholder->setStyleSheet(QApplication::translate("QtServoControl", "background-color: #f0f0f0; border: 1px solid #ccc;", nullptr));
#ifndef QT_NO_TOOLTIP
        speedChartPlaceholder->setToolTip(QApplication::translate("QtServoControl", "Speed Chart Placeholder (rad/s)", nullptr));
#endif // QT_NO_TOOLTIP
        effortChartPlaceholder->setStyleSheet(QApplication::translate("QtServoControl", "background-color: #f0f0f0; border: 1px solid #ccc;", nullptr));
#ifndef QT_NO_TOOLTIP
        effortChartPlaceholder->setToolTip(QApplication::translate("QtServoControl", "Current Chart Placeholder (A)", nullptr));
#endif // QT_NO_TOOLTIP
        Q_UNUSED(QtServoControl);
    } // retranslateUi

};

namespace Ui {
    class QtServoControl: public Ui_QtServoControl {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_SERVO_CONTROL_H
