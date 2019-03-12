/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 3/12/19.
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

#ifndef PROTYPE_PROTOBUFDISPATCHER_H
#define PROTYPE_PROTOBUFDISPATCHER_H

#include <functional>

#include "mes.ud.pb.h"

using namespace std;

class ProtobufDispatcher {
public:
    typedef std::function<void(google::protobuf::Message *message)> ProtobufMessageCallback;

    explicit ProtobufDispatcher(const ProtobufMessageCallback &defaultCb);

    std::function<void()> onMessage(google::protobuf::Message *message);

    void registerMessageCallback(const google::protobuf::Descriptor *desc, const ProtobufMessageCallback &callback);

private:
    typedef std::map<const google::protobuf::Descriptor *, ProtobufMessageCallback> CallbackMap;

    CallbackMap callbacks_;
    ProtobufMessageCallback defaultCallback_;
};

void onTaskReceive(google::protobuf::Message *message);

void onStatusUpdate(google::protobuf::Message *message);

void onUnknownMessageType(google::protobuf::Message *message);


#endif //PROTYPE_PROTOBUFDISPATCHER_H
