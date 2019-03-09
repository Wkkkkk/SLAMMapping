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
#include "cmd.task.pb.h"
#include "lm.helloworld.pb.h"
#include "Singleton.h"

#include <muduo/base/Logging.h>
#include <muduo/net/EventLoop.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace muduo;
using namespace muduo::net;

typedef std::pair<int, int> TaskStatus;

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
        auto node = boost::any_cast<const Node &>(conn->getContext());
        connectionList_.erase(node.position);
    }
}

void EchoServer::onMessage(const muduo::net::TcpConnectionPtr& conn,
                           muduo::net::Buffer* buf,
                           muduo::Timestamp time)
{
    muduo::string msg(buf->retrieveAllAsString());
    cmd::task task;
    task.ParseFromString(msg);

    LOG_INFO << conn->name() << " get task: " << task.id() << " from: " << task.pid()
             << " " << task.ip() << ":" << task.port() << " with: " << task.jobs_size()
             << " jobs at " << time.toString();
    if (!task.IsInitialized()) return;

    auto node = boost::any_cast<const Node &>(conn->getContext());
    node.task_id = task.id();
    conn->setContext(node);

    for (int i = 0; i < task.jobs_size(); ++i) {
        cmd::task_job job;
        job = task.jobs(i);

        LOG_INFO << "job: " << i << " 's type is " << job.type() << " " << job.description();
        switch (job.type()) {
            case cmd::task_JobType_CALCULATE : {
                auto f = std::bind(&calculate, 10, 5);
                pool_.submit(f);

                break;
            }
            case cmd::task_JobType_FILE_TRANSFER : {
                auto f = std::bind(&print, task.id(), job.file_path());
                pool_.submit(f);

                break;
            }
            case cmd::task_JobType_END : {
                auto f = std::bind(&do_some_thing);
                pool_.submit(f);

                break;
            }
            default:
                LOG_WARN << "get unexpected job: " << job.description();
        }
    }
}

void EchoServer::onTimer() {
    for (auto it = connectionList_.begin();
         it != connectionList_.end();)
    {
        TcpConnectionPtr conn = it->lock();
        if (conn)
        {
            auto node = boost::any_cast<const Node &>(conn->getContext());
            if (node.task_id == 0) {
                ++it;
                continue;
            }

            size_t id = node.task_id;
            TaskStatus task = Singleton<TaskStatus>::getInstance()->findByID(Key<TaskStatus>(id));

            lm::helloworld msg1;
            msg1.set_id(id);
            msg1.set_str(std::to_string(task.second));

            std::string msg = msg1.SerializeAsString();
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


