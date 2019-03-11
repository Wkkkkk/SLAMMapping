/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 3/11/19.
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

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <draco/point_cloud/point_cloud.h>
#include <draco/core/encoder_buffer.h>

#include <muduo/base/Logging.h>

#include "UDPSender.h"
#include "DracoHelper.h"

using namespace DracoHelper;

UDPSender::UDPSender(const std::string &server_ip, uint16_t port)
        : point_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>) {
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET; //设置为IP通信
    remote_addr_.sin_addr.s_addr = ::inet_addr(server_ip.c_str());
    remote_addr_.sin_port = ::htons(port);

    /*创建客户端套接字--IPv4协议，面向无连接通信，UDP协议*/
    client_sockfd_ = ::socket(PF_INET, SOCK_DGRAM, 0);
    if (client_sockfd_ < 0) {
        LOG_WARN << "socket error";
        return;
    }
}

void UDPSender::readPCDFileFromPath(const std::string &file_path) {
    pcl::PCDReader reader;
    reader.read(file_path, *point_cloud_ptr_);
}

UDPSender::~UDPSender() {
    ::close(client_sockfd_);
}

void UDPSender::send() {
    if (!point_cloud_ptr_) {
        LOG_WARN << "socket error";
        return;
    }

    const int batch_size = 1000;

    size_t num_points = point_cloud_ptr_->points.size();
    int batch = num_points / batch_size;
    for (int i = 0; i < batch; ++i) {
        std::unique_ptr<draco::PointCloud> pc = convert2draco(point_cloud_ptr_, i * batch_size, batch_size);
        std::unique_ptr<draco::EncoderBuffer> eb = encode2buffer(*pc);

        auto len = eb->size();
//        char buf[len];
//        memcpy(buf, eb->data(), len);

        send(eb->data(), len);
        LOG_INFO << "send point cloud buf with length: " << len;
        sleep(3);
    }
}

void UDPSender::send(const char *data, size_t length) {
    if (::sendto(client_sockfd_, data, length, 0, (struct sockaddr *) &remote_addr_, sizeof(struct sockaddr)) < 0) {
        LOG_FATAL << "failed to send buf!";
        return;
    }
}
