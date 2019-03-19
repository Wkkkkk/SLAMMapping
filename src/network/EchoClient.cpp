/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 3/7/19.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#include <muduo/base/Logging.h>

#include "EchoClient.h"
#include "mes.ud.pb.h"

EchoClient::EchoClient(EventLoop *loop, const InetAddress &listenAddr, size_t size)
        : loop_(loop),
          client_(loop, listenAddr, "EchoClient"),
          message_(size, 'H')
{
    client_.setConnectionCallback(
            std::bind(&EchoClient::onConnection, this, _1));
    client_.setMessageCallback(
            std::bind(&EchoClient::onMessage, this, _1, _2, _3));
    client_.enableRetry();
}

void EchoClient::connect() {
    client_.connect();
}

void EchoClient::onConnection(const TcpConnectionPtr &conn) {
    LOG_TRACE << conn->localAddress().toIpPort() << " -> "
              << conn->peerAddress().toIpPort() << " is "
              << (conn->connected() ? "UP" : "DOWN");

    if (conn->connected())
    {
        conn->setTcpNoDelay(true);
//        conn->send(message_);
        conn_ = conn;
    }
    else
    {
        conn_.reset();
//        loop_->quit();
    }
}

void EchoClient::onMessage(const TcpConnectionPtr &conn, Buffer *buf, Timestamp time) {
    muduo::string msg(buf->retrieveAllAsString());

    message::status status;
    status.ParseFromString(msg);

    LOG_INFO << conn->name() << " " << status.GetTypeName() << " with " << msg.size() << " bytes, "
             << "data received at " << time.toString();
}

void EchoClient::send(const std::string &str) {
    conn_->send(str);
}

