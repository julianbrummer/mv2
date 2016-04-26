TEMPLATE = app
TARGET = MV
QT += core gui opengl
CONFIG += console debug

unix:QMAKE_CXXFLAGS_RELEASE -= -O2
unix:QMAKE_CXXFLAGS_RELEASE += -std=c++11 -O3

unix:QMAKE_CXXFLAGS_DEBUG += -std=c++11

RESOURCES += shaders.qrc
DEPENDPATH = ./
HEADERS += *.h
SOURCES += *.cpp
CONFIG += c++11
#QMAKE_LFLAGS += -Wl,--large-address-aware
DISTFILES +=

