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
    QWidget *currentChartPlaceholder;

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
        slider1->setMinimum(0);
        slider1->setMaximum(180);
        slider1->setValue(90);

        verticalLayout->addWidget(slider1);

        label2 = new QLabel(QtServoControl);
        label2->setObjectName(QString::fromUtf8("label2"));

        verticalLayout->addWidget(label2);

        slider2 = new QSlider(QtServoControl);
        slider2->setObjectName(QString::fromUtf8("slider2"));
        slider2->setOrientation(Qt::Horizontal);
        slider2->setMinimum(0);
        slider2->setMaximum(180);
        slider2->setValue(90);

        verticalLayout->addWidget(slider2);

        positionChartPlaceholder = new QWidget(QtServoControl);
        positionChartPlaceholder->setObjectName(QString::fromUtf8("positionChartPlaceholder"));
        positionChartPlaceholder->setMinimumHeight(150);

        verticalLayout->addWidget(positionChartPlaceholder);

        speedChartPlaceholder = new QWidget(QtServoControl);
        speedChartPlaceholder->setObjectName(QString::fromUtf8("speedChartPlaceholder"));
        speedChartPlaceholder->setMinimumHeight(150);

        verticalLayout->addWidget(speedChartPlaceholder);

        currentChartPlaceholder = new QWidget(QtServoControl);
        currentChartPlaceholder->setObjectName(QString::fromUtf8("currentChartPlaceholder"));
        currentChartPlaceholder->setMinimumHeight(150);

        verticalLayout->addWidget(currentChartPlaceholder);


        retranslateUi(QtServoControl);

        QMetaObject::connectSlotsByName(QtServoControl);
    } // setupUi

    void retranslateUi(QWidget *QtServoControl)
    {
        label1->setText(QApplication::translate("QtServoControl", "Servo 1 Angle: 90", nullptr));
        label2->setText(QApplication::translate("QtServoControl", "Servo 2 Angle: 90", nullptr));
        positionChartPlaceholder->setStyleSheet(QApplication::translate("QtServoControl", "background-color: #f0f0f0; border: 1px solid #ccc;", nullptr));
#ifndef QT_NO_TOOLTIP
        positionChartPlaceholder->setToolTip(QApplication::translate("QtServoControl", "Position Chart Placeholder", nullptr));
#endif // QT_NO_TOOLTIP
        speedChartPlaceholder->setStyleSheet(QApplication::translate("QtServoControl", "background-color: #f0f0f0; border: 1px solid #ccc;", nullptr));
#ifndef QT_NO_TOOLTIP
        speedChartPlaceholder->setToolTip(QApplication::translate("QtServoControl", "Speed Chart Placeholder", nullptr));
#endif // QT_NO_TOOLTIP
        currentChartPlaceholder->setStyleSheet(QApplication::translate("QtServoControl", "background-color: #f0f0f0; border: 1px solid #ccc;", nullptr));
#ifndef QT_NO_TOOLTIP
        currentChartPlaceholder->setToolTip(QApplication::translate("QtServoControl", "Current Chart Placeholder", nullptr));
#endif // QT_NO_TOOLTIP
        Q_UNUSED(QtServoControl);
    } // retranslateUi

};

namespace Ui {
    class QtServoControl: public Ui_QtServoControl {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_SERVO_CONTROL_H
