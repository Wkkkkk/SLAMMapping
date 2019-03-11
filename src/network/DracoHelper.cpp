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

#include <draco/io/ply_encoder.h>
#include <draco/io/ply_decoder.h>
#include <draco/io/point_cloud_io.h>
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <pcl/io/pcd_io.h>

#include "DracoHelper.h"

std::unique_ptr<draco::PointCloud>
DracoHelper::convert2draco(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_ptr, uint origin, uint size) {
    uint num_points = pc_ptr->points.size();

    draco::PointCloudBuilder builder;
    // Initialize the builder for a given number of points (required).
    builder.Start(num_points);
    // Specify desired attributes.
    int pos_att_id =
            builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    // Add attribute values.
    for (uint i = 0; i < size && origin + i < num_points; ++i) {
        auto point = pc_ptr->points.at(origin + i);
        builder.SetAttributeValueForPoint(pos_att_id, draco::PointIndex(origin + i),
                                          draco::Vector3f(point.x, point.y, point.z).data());
        //if(i % 500 == 0) std::cout << i << ": " << point.x << " " << point.y << " " << point.z << std::endl;
    }

    // Get the final PointCloud.
    constexpr bool deduplicate_points = false;
    std::unique_ptr<draco::PointCloud> pc = builder.Finalize(deduplicate_points);

    return pc;
}

std::unique_ptr<draco::EncoderBuffer> DracoHelper::encode2buffer(const draco::PointCloud &pc) {
    std::unique_ptr<draco::EncoderBuffer> out_buffer(new draco::EncoderBuffer);

    draco::Encoder encoder;
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);

    const draco::Status status = encoder.EncodePointCloudToBuffer(pc, out_buffer.get());
    if (!status.ok()) {
        printf("Failed to encode the point cloud.\n");
    }

    return out_buffer;
}
