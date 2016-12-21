TEMPLATE = app
TARGET = MV
QT += core gui opengl
CONFIG += console debug

QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -std=c++11 -O3

QMAKE_CXXFLAGS_DEBUG += -std=c++11

RESOURCES += shaders.qrc
DEPENDPATH = ./
HEADERS += *.h
SOURCES += *.cpp
CONFIG += c++11
DISTFILES +=

