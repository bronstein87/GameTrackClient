#include "logger.h"



void Logger::write(Logger::LogType type, const QString &from, const QString &message)
{
    if (!logFile.isOpen())
    {
        logFile.setFileName(QString("/logs/log_%1.txt")
                            .arg(QDateTime::currentDateTime().toString("dd MM yyyy hh mm ss")));
        Q_ASSERT(logFile.open(QIODevice::WriteOnly));
        out.setDevice(&logFile);
    }

    if (type == Error)
    {
        emit errorLogReady(message, from);
    }
    else if (type == Debug)
    {
        emit errorLogReady(message, from);
    }

    QString typeStr = (type == Error) ? "ERROR" : (type == Debug) ? "DEBUG" : "WARN";
    QMutexLocker lock(&mutex);
    out << typeStr << "\n" << from << "\n" << message << endl;
}
