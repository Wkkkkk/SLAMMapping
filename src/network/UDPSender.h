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

#ifndef PROTYPE_UDPSENDER_H
#define PROTYPE_UDPSENDER_H

#include <netinet/in.h>

#include <string>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>


class UDPSender {
public:
    explicit UDPSender(const std::string &server_ip = "127.0.0.1", uint16_t port = 8000);

    ~UDPSender();

    void readPCDFileFromPath(const std::string &file_path);

    void send();

private:
    void send(const char *data, size_t length);

    int client_sockfd_;
    struct sockaddr_in remote_addr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr_;
};


#endif //PROTYPE_UDPSENDER_H
