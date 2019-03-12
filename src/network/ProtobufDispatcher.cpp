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

#include <muduo/base/Logging.h>

#include "ProtobufDispatcher.h"

ProtobufDispatcher::ProtobufDispatcher(const ProtobufDispatcher::ProtobufMessageCallback &defaultCb)
        : defaultCallback_(defaultCb) {
}

std::function<void()> ProtobufDispatcher::onMessage(google::protobuf::Message *message) {
    auto it = callbacks_.find(message->GetDescriptor());

    std::function<void()> function;
    if (it != callbacks_.end()) {
//        it->second(message);
        function = std::bind(it->second, message);
    } else {
//        defaultCallback_(message);
        function = std::bind(defaultCallback_, message);
    }
    return function;
}

void ProtobufDispatcher::registerMessageCallback(const google::protobuf::Descriptor *desc,
                                                 const ProtobufDispatcher::ProtobufMessageCallback &callback) {
    callbacks_[desc] = callback;
}

void onTaskReceive(google::protobuf::Message *message) {
    LOG_INFO << "onTaskReceive: " << message->GetTypeName();
    auto query = dynamic_cast<message::task *>(message);
    assert(query != nullptr);


}

void onStatusUpdate(google::protobuf::Message *message) {
    LOG_INFO << "onStatusUpdate: " << message->GetTypeName();
    auto answer = dynamic_cast<message::status *>(message);
    assert(answer != nullptr);


}

void onUnknownMessageType(google::protobuf::Message *message) {
    LOG_WARN << "Discarding " << message->GetTypeName();
}
