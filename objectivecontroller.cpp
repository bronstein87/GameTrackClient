#include "objectivecontroller.h"
#include <QDebug>
#include <QSerialPortInfo>


ObjectiveController::ObjectiveController(const QString& pattern, qint32 port, QObject *parent) : pattern(pattern),
    QObject(parent)
{
    connectToController(pattern, port);
}

bool ObjectiveController::testControllerActive()
{
    if (!objective.isOpen())
    {
        qDebug() << "контроллер не активирован";
        return false;
    }
    return true;
}

void ObjectiveController::setDiaphragmLevel(const QString& value)
{
    error.clear();
    if (testControllerActive())
    {
        if (value.size() == diaphragmCommandSize && value.at(0) == 'A' && value.at(value.size() - 1) == '#')
        {
            if (objective.write(value.toLocal8Bit())  == -1)
            {
                error = objective.errorString();
                qDebug() << error;
            }
        }
        else
        {
            error = "Wrong command format";
            qDebug() << error;
        }
    }
}

void ObjectiveController::setFocusing(const QString& value)
{
    error.clear();
    if (testControllerActive())
    { 
        if (value.size() == focusCommandSize && value.at(0) == 'M' && value.at(value.size() - 1) == '#')
        {
            if (objective.write(value.toLocal8Bit()) == -1)
            {
                error = objective.errorString();
                qDebug() << error;
            }
        }
        else
        {
            error = "Wrong command format";
            qDebug() << error;
        }
    }
}

QString ObjectiveController::getCurrentFocusing()
{
    error.clear();
    if (testControllerActive())
    {
        if (objective.write(QString("P#").toLocal8Bit()))
        {
            if (objective.waitForReadyRead(1000))
            {
                QByteArray array = objective.read(objective.bytesAvailable());
                return QString::fromLatin1(array.data());
            }
            else
            {
                error = "Can't get answer";
                qDebug() << error;
            }
        }
        else
        {
            error = "Wrong command format";
            qDebug() << error;
        }
    }
    return QString();
}


void ObjectiveController::connectToController(const QString& ptrn, qint32 port)
{
    pattern = ptrn;
    error.clear();
    objective.close();
    if (port == -1)
    {
        auto ports = QSerialPortInfo::availablePorts();
        for (auto& i : ports)
        {
            qDebug() << i.description() << i.manufacturer();
        }
        for (qint32 i = 0; i < ports.size(); ++i)
        {
           // qDebug() << ports.size() << ports[i].description() << ports[i].manufacturer();
            if (ports[i].description().contains(pattern))
            {
                port = i;
                break;
            }
        }
    }
    if (port != -1)
    {
        objective.setPort(QSerialPortInfo::availablePorts().at(port));
        if (objective.open(QIODevice::ReadWrite))
        {
            objective.setBaudRate(38400);
            qDebug() << "controller connected";
        }
        else
        {
            error = "Can't open connection on current port " + objective.errorString();
            qDebug() << error;
        }
    }
    else
    {
        error = "Invalid port ";
        qDebug() << error;
    }
}
