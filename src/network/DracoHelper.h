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

#ifndef PROTYPE_DRACO_H
#define PROTYPE_DRACO_H

#include <memory>

namespace draco {
    class PointCloud;

    class EncoderBuffer;
}

namespace DracoHelper {
    std::unique_ptr<draco::PointCloud>
    convert2draco(const pcl::PointCloud<pcl::PointXYZ>::Ptr &, uint origin, uint size);

    std::unique_ptr<draco::EncoderBuffer> encode2buffer(const draco::PointCloud &pc);

};


#endif //PROTYPE_DRACO_H
