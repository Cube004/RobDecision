
#ifndef UITOOL_H
#define UITOOL_H

#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QLabel>

struct colorStyleSheet{
    QString red = "color: rgb(255, 81, 105);";
    QString grey = "color: rgb(153, 153, 153);";
    QString green = "color: rgb(0, 193, 139);";
    QString blue = "color: rgb(15, 114, 239);";
    QString black = "color: rgb(58, 58, 58);";
};


void bindButton(QPushButton *button, void (*func)(void)){ 
    QObject::connect(button, &QPushButton::clicked, func);
}

void bindslider(QSlider *slider, void (*func)(int)){
    QObject::connect(slider, &QSlider::valueChanged, func);
}

void bindQlineEdit(QLineEdit *lineEdit, void (*func)(QString)){
    QObject::connect(lineEdit, &QLineEdit::textChanged, func);
}

void changeLabelText(QLabel *label, QString text, QString color){
    label->setText(text);
    label->setStyleSheet(color);
    //"color: rgb(255, 81, 105);"
}

#endif // UITOOL_H