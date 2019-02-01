/*
 * Copyright (c) 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * All rights reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 */

#ifndef ipa_rtabmap_interface_NODE
#define ipa_rtabmap_interface_NODE



#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdint>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/NodeData.h>
#include <rtabmap_ros/MapGraph.h>

#include <rtabmap/core/util3d.h>
#include <rtabmap_ros/MsgConversion.h>

#include <find_object_2d/ObjectsStamped.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap_ros/UserData.h>
#include <rtabmap_ros/MsgConversion.h>


#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap_ros/GetMap.h>



#include <algorithm>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <image_transport/image_transport.h>
#include <iostream>

#include<boost/bind.hpp>




#include <algorithm>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <image_transport/image_transport.h>
#include <iostream>

#include <dynamic_reconfigure/server.h>
#include <ipa_rtabmap_interface/dynReconfigureConfig.h>   //dyn reconfigure params
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class rtabInterfaceNode
{
public:
    rtabInterfaceNode(ros::NodeHandle node_handle);
    //static void dynamicReconfigure(ipa_rtabmap_interface::dynReconfigureConfig &config,  uint32_t level);

private:
    ros::NodeHandle node_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_cam_;
     void mapCallback(const rtabmap_ros::MapData& map_in);

};

#endif // ipa_rtabmap_interface_NODE
