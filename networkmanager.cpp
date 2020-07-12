#include "networkmanager.h"

NetworkManager::NetworkManager(EndPointType type, const QString &adresss, QObject *parent) : QObject(parent)
{

}

void NetworkManager::send(qint32 messageId, google::protobuf::Message* message, QTcpSocket* socket)
{
    if (type == Server && socket == nullptr)
    {
        qWarning("incorrect socket");
        return;
    }
    if (type == Client)
    {
        socket = static_cast <QTcpSocket*> (endPoint);
    }
    qint32 size = message->ByteSize();
    QByteArray array(size, '\0');
    message->SerializeToArray(array.data(), size);
    array.prepend(size);
    array.prepend(messageId);
    socket->write(array);
    bool flush = true;
    while (flush)
    {
        flush = socket->flush();
    }

}

void NetworkManager::setHandler(qint32 id, std::function<void (const MessageData&)> handler)
{
    if (messageHandlers.contains(id))
    {
        messageHandlers[id].append(handler);
    }
    else
    {
        messageHandlers.insert(id, {handler});
    }
}

void NetworkManager::init(const QString &address)
{
    QStringList ipPort = address.split(":");
    if (type == Server)
    {
        endPoint = new QTcpServer(this);
        QTcpServer* server = static_cast <QTcpServer*> (endPoint);
        connect(server, &QTcpServer::newConnection, this, [this, server]() {

            QTcpSocket* clientConnection = server->nextPendingConnection();
            rawBuffers.insert(clientConnection, RawBuffer());
            if (messageHandlers.contains(OnConnect))
            {
                for (auto& i : messageHandlers[OnConnect])
                {
                    i(MessageData(clientConnection));
                }
            }

            QHostAddress address = clientConnection->peerAddress();
            quint16 port = clientConnection->peerPort();
            connect(clientConnection, &QAbstractSocket::disconnected,
                    this, [this, clientConnection]()
            {
                if (messageHandlers.contains(OnDisconnect))
                {
                    for (auto& i : messageHandlers[OnDisconnect])
                    {
                        i(MessageData(clientConnection));
                    }
                }
                rawBuffers.remove(clientConnection);
                clientConnection->deleteLater();
            });

            connect(clientConnection, &QTcpSocket::readyRead, this, [this, clientConnection]() {handle(clientConnection);});
            qDebug() << clientConnection->peerAddress().toString().remove("::ffff:");
            emit messageReady(QString("New connection: %1 : %2\n")
                              .arg(address.toString())
                              .arg(port));
        });
        connect(server, &QTcpServer::acceptError, this, [server, this](auto errorCode)
        {
            qDebug() << errorCode;
            server->errorString();
            emit messageReady(server->errorString());
        });

        if (!server->listen(QHostAddress(ipPort.first()), ipPort.last().toUInt()))
        {
            emit messageReady(QString("Unable to start the server: %1.")
                              .arg(server->errorString()));
        }
    }
    else
    {
        endPoint = new QTcpSocket(this);
        QTcpSocket* client = static_cast <QTcpSocket*> (endPoint);
        rawBuffers.insert(client, RawBuffer());
        client->connectToHost(QHostAddress(ipPort.first()), ipPort.last().toUInt());
        QSharedPointer <QTimer> timer(new QTimer);
        connect(timer.data(), &QTimer::timeout, [timer, client, ipPort]()  mutable
        {
            if (client->state() != QAbstractSocket::SocketState::ConnectedState)
            {
                client->connectToHost(QHostAddress(ipPort.first()), ipPort.last().toUInt());
            }
            else
            {
                timer.reset();
            }

        });
        connect(client, &QTcpSocket::connected, [this, client]()
        {
            if (messageHandlers.contains(OnConnect))
            {
                for (auto& i : messageHandlers[OnConnect])
                {
                    i(MessageData(client));
                }
            };
        });
        connect(client, &QTcpSocket::disconnected, [this, client]()
        {
            if (messageHandlers.contains(OnDisconnect))
            {
                for (auto& i : messageHandlers[OnConnect])
                {
                    i(MessageData(client));
                }
            };
        });
        connect(client, &QTcpSocket::readyRead, this, [this, client]() {handle(client);});
        connect(client, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error), this, [client, this](auto socketError)
        {
            QString errorMessage;
            switch (socketError) {
            case QAbstractSocket::RemoteHostClosedError:
                errorMessage = "Remote host closed.";
                break;
            case QAbstractSocket::HostNotFoundError:
                errorMessage = "The host was not found. Please check the "
                               "host name and port settings.";
                break;
            case QAbstractSocket::ConnectionRefusedError:
                errorMessage = "The connection was refused by the peer. "
                               "Make sure the server is running, "
                               "and check that the host name and port "
                               "settings are correct.";
                break;
            default:
                errorMessage = QString("The following error occurred: %1.")
                        .arg(client->errorString());
            }
            emit messageReady(errorMessage);
        });
    }
}

void NetworkManager::handle(QObject* endPointData)
{
    RawBuffer& rawBuffer = rawBuffers[endPointData];
    QTcpSocket* source = static_cast <QTcpSocket*> (endPointData);
    bool read = source->bytesAvailable() > rawBuffer.needToRead;
    while (read)
    {
        rawBuffer.buffer = source->read(rawBuffer.needToRead);
        if (rawBuffer.id == ReservedCommands::Invalid)
        {
            memcpy(&rawBuffer.id, rawBuffer.buffer.data(), sizeof(qint32));
            memcpy(&rawBuffer.needToRead, rawBuffer.buffer.data() + sizeof(qint32), sizeof(qint32));
            rawBuffer.buffer.clear();
        }
        else
        {
            for (auto& i : messageHandlers[rawBuffer.id])
            {
                i(MessageData(source,&rawBuffer.buffer));
            }
            rawBuffer.id = ReservedCommands::Invalid;
            rawBuffer.needToRead = headerSize;
            rawBuffer.buffer.clear();
        }
        read = source->bytesAvailable() > rawBuffer.needToRead;
    }
}

