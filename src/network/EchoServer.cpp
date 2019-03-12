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
#include "mes.ud.pb.h"

#include "Singleton.h"
#include "EchoServer.h"

#include <muduo/base/Logging.h>
#include <muduo/net/EventLoop.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace muduo;
using namespace muduo::net;

typedef std::pair<int, int> TaskStatus;

void onTask(const std::weak_ptr<muduo::net::TcpConnection> &conn, const message::task &task) {
    int id = task.task_id();
    LOG_INFO << "onTaskReceive: " << task.GetTypeName() << " with id: " << id;

    for (int i = 0; i < 10; ++i) {
        auto tcp_id = new message::tcp_id;
        tcp_id->CopyFrom(task.tcp_id());

        message::status status;
        status.set_task_id(id);
        status.set_percentage(i * 10);
        status.set_allocated_tcp_id(tcp_id);

        status.set_allocated_last_updated(new message::timestamp);
        status.mutable_last_updated()->set_seconds(muduo::Timestamp::now().microSecondsSinceEpoch());

        //some time-cost calculate
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::string msg = status.SerializeAsString();
        auto connection = conn.lock();
        if (connection) {
            connection->send(msg);
            LOG_INFO << "send: " << status.GetTypeName();
        }
    }
}

void print(int id, const std::string &str) {
    for (int i = 0; i < 10; ++i) {
        TaskStatus task_status = {id, i * 10};
        Singleton<TaskStatus>::getInstance()->update(Key<TaskStatus>(id), task_status);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int calculate(int a, int b) {
    int result = a + b;
    std::this_thread::sleep_for(std::chrono::seconds(4));

    return result;
}

void do_some_thing() {
    std::this_thread::sleep_for(std::chrono::seconds(5));
}

EchoServer::EchoServer(muduo::net::EventLoop* loop,
                       const muduo::net::InetAddress& listenAddr)
        : server_(loop, listenAddr, "EchoServer")
{
    server_.setConnectionCallback(
            std::bind(&EchoServer::onConnection, this, _1));
    server_.setMessageCallback(
            std::bind(&EchoServer::onMessage, this, _1, _2, _3));

//    loop->runEvery(1.0, std::bind(&EchoServer::onTimer, this));
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
        auto node = boost::any_cast<const Node &>(conn->getContext());
        connectionList_.erase(node.position);
    }
}

void EchoServer::onMessage(const muduo::net::TcpConnectionPtr& conn,
                           muduo::net::Buffer* buf,
                           muduo::Timestamp time)
{
    std::string msg(buf->retrieveAllAsString());

    message::task task;
    if (!task.ParseFromString(msg)) {
        LOG_WARN << "Discarding Unknown Message";
        return;
    }

    LOG_INFO << conn->name() << " get task: " << task.task_id() << " from: " << task.tcp_id().pid()
             << " " << task.tcp_id().ip() << ":" << task.tcp_id().port() << " with: " << task.jobs_size()
             << " jobs at " << time.toString();

    WeakTcpConnectionPtr weak_conn(conn);
    pool_.submit(std::bind(onTask, std::move(weak_conn), std::move(task)));
}

void EchoServer::onTimer() {
    for (auto it = connectionList_.begin();
         it != connectionList_.end();)
    {
        TcpConnectionPtr conn = it->lock();
        if (conn)
        {
            std::string msg;
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


