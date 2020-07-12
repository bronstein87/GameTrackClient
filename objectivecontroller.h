#ifndef OBJECTIVECONTROLLER_H
#define OBJECTIVECONTROLLER_H

#include <QObject>
#include <QSerialPort>
#include <QString>

class ObjectiveController : public QObject
{
    Q_OBJECT
public:
    explicit ObjectiveController(const QString& pattern, qint32 port = -1,  QObject *parent = 0);

    void setDiaphragmLevel(const QString& value);

    void setFocusing(const QString& value);

    QString getCurrentFocusing();

    QString currentError() const;

    void connectToController(const QString& ptrn, qint32 port = -1);

private:

    bool testControllerActive();
    QString pattern;
    QSerialPort objective;
    QString error;
    static constexpr const qint16 focusCommandSize = 6;
    static constexpr const qint16 diaphragmCommandSize = 4;
};

#endif // OBJECTIVECONTROLLER_H
