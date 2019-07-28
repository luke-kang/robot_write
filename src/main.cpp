#include <iostream>
#include <QApplication>

#include "MainWindows.h"

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    windows window;
    window.show();

    return app.exec();
}
