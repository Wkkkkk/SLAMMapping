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
#include "../../network/EchoClient.h"

#include <muduo/net/TcpClient.h>

#include <muduo/base/Logging.h>
#include <muduo/base/Thread.h>
#include <muduo/net/EventLoop.h>
#include <muduo/net/InetAddress.h>

#include <utility>

#include <stdio.h>
#include <unistd.h>

// using namespace muduo;
// using namespace muduo::net;

int main(int argc, char* argv[])
{
    LOG_INFO << "pid = " << getpid() << ", tid = " << CurrentThread::tid();
    if (argc > 1)
    {
        EventLoop loop;
        InetAddress serverAddr(argv[1], 2000);
        LOG_INFO << "connect to: " << serverAddr.toIpPort();

        int size = 256;
        if (argc > 2)
        {
            size = atoi(argv[2]);
        }

        EchoClient client(&loop, serverAddr, size);
        client.connect();
        loop.loop();
    }
    else
    {
        printf("Usage: %s host_ip [msg_size]\n", argv[0]);
    }
}
