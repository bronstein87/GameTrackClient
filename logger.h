#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QString>
#include <QFile>
#include <QDateTime>
#include <QTextStream>
#include <QMutex>
#include <QDebug>

class Logger : public QObject
{
    Q_OBJECT
public:
    enum LogType
    {
        Error,
        Debug,
        Warning,
        Report
    };



private:

    Logger() = default;

    Logger(const Logger&) = delete;

    Logger(Logger&& ) = delete;

    Logger& operator=(const Logger&) = delete;

    Logger& operator=(Logger&&) = delete;

    ~Logger() = default;

public:

    static Logger& instance() noexcept
    {
        static Logger g;
        return g;
    }

   void write(LogType type, const QString &from, const QString& message);

signals:
       void errorLogReady(const QString& message, const QString &from);

       void debugLogReady(const QString& message, const QString& from);
private:

       QFile logFile;
       QTextStream out;
       QMutex mutex;

};

#endif // LOGGER_H
