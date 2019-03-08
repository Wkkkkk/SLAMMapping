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

#include "EchoServer.h"
#include "../messages/lm.helloworld.pb.h"

#include <muduo/base/Logging.h>
#include <muduo/net/EventLoop.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

 using namespace muduo;
 using namespace muduo::net;

EchoServer::EchoServer(muduo::net::EventLoop* loop,
                       const muduo::net::InetAddress& listenAddr)
        : server_(loop, listenAddr, "EchoServer")
{
    server_.setConnectionCallback(
            std::bind(&EchoServer::onConnection, this, _1));
    server_.setMessageCallback(
            std::bind(&EchoServer::onMessage, this, _1, _2, _3));

    loop->runEvery(1.0, std::bind(&EchoServer::onTimer, this));
}

void EchoServer::start()
{
    server_.start();
}

void EchoServer::onConnection(const muduo::net::TcpConnectionPtr& conn)
{
    LOG_INFO << "EchoServer - " << conn->peerAddress().toIpPort() << " -> "
             << conn->localAddress().toIpPort() << " is "
             << (conn->connected() ? "UP" : "DOWN");

    if (conn->connected())
    {
        Node node;
        node.lastReceiveTime = Timestamp::now();
        connectionList_.push_back(conn);
        node.position = --connectionList_.end();
        conn->setContext(node);
    }
    else
    {
        assert(!conn->getContext().empty());
        const Node& node = boost::any_cast<const Node&>(conn->getContext());
        connectionList_.erase(node.position);
    }
}

void EchoServer::onMessage(const muduo::net::TcpConnectionPtr& conn,
                           muduo::net::Buffer* buf,
                           muduo::Timestamp time)
{
    muduo::string msg(buf->retrieveAllAsString());
    LOG_INFO << conn->name() << " echo " << msg.size() << " bytes, "
             << "data received at " << time.toString();
    conn->send(msg);
}

void EchoServer::onTimer() {
    for (auto it = connectionList_.begin();
         it != connectionList_.end();)
    {
        TcpConnectionPtr conn = it->lock();
        if (conn)
        {
            lm::helloworld msg1;
            msg1.set_id(101);
            msg1.set_str("abcd");

            std::string msg = msg1.str();
            conn->send(msg);
            ++it;
        }
        else
        {
            LOG_WARN << "Expired";
            it = connectionList_.erase(it);
        }
    }

}


