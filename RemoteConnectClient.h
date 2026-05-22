/*!
 *@file     RemoteConnectClientThread.h
 *@brief    线虫跟踪控制系统远程连接模块-客户端模块
 *@author   yangshuhao
 *@date     2024/08/20
 *@remarks
 *@version  1.1
 */

#pragma once
#include <iostream>
#include <QThread>
#include <QTcpSocket>
#include <QString>

class RemoteConnectClient : public QThread
{
	Q_OBJECT

public:
	RemoteConnectClient();
	~RemoteConnectClient();

	void init();

signals:
	void connected();
	void error(const QString& errorString);

private:
	QString connectHost;   //连接地址
	quint16 connectPort;   //连接端口
	QTcpSocket* clientSocket;
};
typedef std::shared_ptr<RemoteConnectClient> RemoteConnectClientPtr;
