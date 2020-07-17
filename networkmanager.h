
#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <QObject>
#include <QMap>
#include <functional>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <google/protobuf/message.h>
#include <limits>


class NetworkManager : public QObject
{
    Q_OBJECT

public:

    struct MessageData
    {
        MessageData(QTcpSocket* _socket = nullptr, QByteArray _msg = QByteArray()) : socket(_socket), msg(_msg) {}
        QTcpSocket* socket = nullptr;
        QByteArray msg;
    };
    enum EndPointType
    {
        Server,
        Client
    };

    enum ReservedCommands
    {
        OnDisconnect = -1,
        OnConnect = -2,
        Invalid = std::numeric_limits<qint32>::min()
    };

    explicit NetworkManager(EndPointType t, const QString& address, bool init = true, QObject *parent = nullptr);

    void send(qint32 messageId, const google::protobuf::Message *message, QTcpSocket* socket = nullptr);

    void setHandler(qint32 id, std::function <void (const MessageData&)> handler);

    void initialize(){initSocket(address);}

    ~NetworkManager();

signals:

    void messageReady(const QString& message);

private:

    struct RawBuffer
    {
        qint32 id = ReservedCommands::Invalid;
        qint32 needToRead = headerSize;
        QByteArray buffer;
    };

    void initSocket(const QString& address);

    void handle(QObject* endPointData);

    static constexpr const qint32 headerSize = 8;
    QMap <qint32, QVector <std::function <void (const MessageData&)>>> messageHandlers;
    QObject* endPoint;
    QMap <QObject*, RawBuffer> rawBuffers;
    EndPointType type;
    QString address;


};

#endif // NETWORKMANAGER_H
