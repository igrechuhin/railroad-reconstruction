#ifndef TESTWRITER_H
#define TESTWRITER_H

#include <QObject>

namespace experiment {

class TestWriter : public QObject
{
    Q_OBJECT
public:
    explicit TestWriter(QObject *parent = 0);

    Q_INVOKABLE void writePCD();
signals:

public slots:

};

}

#endif // TESTWRITER_H
