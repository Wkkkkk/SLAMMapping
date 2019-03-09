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

#ifndef PROTYPE_ECHOSERVER_H
#define PROTYPE_ECHOSERVER_H
#include <list>

#include <muduo/net/TcpConnection.h>
#include <muduo/net/TcpServer.h>

#include "ThreadPool.h"

// RFC 862
class EchoServer
{
public:
    EchoServer(muduo::net::EventLoop* loop,
               const muduo::net::InetAddress& listenAddr);

    void start();  // calls server_.start();

private:
    void onConnection(const muduo::net::TcpConnectionPtr& conn);

    void onMessage(const muduo::net::TcpConnectionPtr& conn,
                   muduo::net::Buffer* buf,
                   muduo::Timestamp time);

    void onTimer();

    typedef std::weak_ptr<muduo::net::TcpConnection> WeakTcpConnectionPtr;
    typedef std::list<WeakTcpConnectionPtr> WeakConnectionList;

    struct Node : public muduo::copyable
    {
        size_t task_id;
        muduo::Timestamp lastReceiveTime;
        WeakConnectionList::iterator position;

        Node() : task_id(0) {}
    };

    ThreadPool pool_;

    muduo::net::TcpServer server_;
    WeakConnectionList connectionList_;
};

#endif //PROTYPE_ECHOSERVER_H
