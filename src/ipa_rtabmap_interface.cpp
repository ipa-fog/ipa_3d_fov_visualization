/*
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * All rights reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 */

/**
 * /author
 *    Author: Florenz Graf, email: florenz.graf@gmx.de
 *
 * /Filedescription
 *
 */

#include "ipa_rtabmap_interface/ipa_rtabmap_interface.h"


//  === CONSTRUCTOR ===
rtabInterfaceNode::rtabInterfaceNode(ros::NodeHandle node_handle):
    node_(node_handle)
{
     pub_cam_ = node_.advertise<sensor_msgs::PointCloud2>("/pointcloud_converted",10);
     sub_map_ = node_.subscribe("/rtabmap/mapData", 10, &rtabInterfaceNode::mapCallback, this);
}


void rtabInterfaceNode::mapCallback(const rtabmap_ros::MapData& map)
{


    std::map<int, rtabmap::Transform> poses;
    for(unsigned int i=0; i<map.graph.posesId.size() && i<map.graph.poses.size(); ++i)
    {
        poses.insert(std::make_pair(map.graph.posesId[i], rtabmap_ros::transformFromPoseMsg(map.graph.poses[i])));
    }

    // Add new clouds...
    for(unsigned int i=0; i<map.nodes.size() && i<map.nodes.size(); ++i)
    {
        int id = map.nodes[i].id;

        // Always refresh the cloud if there are data
        rtabmap::Signature s = rtabmap_ros::nodeDataFromROS(map.nodes[i]);
        if(!s.sensorData().imageCompressed().empty() &&
           !s.sensorData().depthOrRightCompressed().empty() &&
           (s.sensorData().cameraModels().size() || s.sensorData().stereoCameraModel().isValidForProjection()))
        {
            cv::Mat image, depth;
            s.sensorData().uncompressData(&image, &depth, 0);


            if(!s.sensorData().imageRaw().empty() && !s.sensorData().depthOrRightRaw().empty())
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_unorganized;   // unorganized point cloud
                pcl::IndicesPtr validIndices(new std::vector<int>);

                cloud_unorganized = rtabmap::util3d::cloudRGBFromSensorData(
                        s.sensorData(),
                        1,
                        5,
                        0,
                        validIndices.get());

                // using indices to organize point cloud
                pcl::PointCloud<pcl::PointXYZRGB> cloud_organized;
                cloud_organized.width = (int)map.nodes[i].width[0]; // Image-like organized structure, with 480 rows and 640 columns,
                cloud_organized.height = (int)map.nodes[i].height[0];

                ROS_WARN("cloud_organized %i %i ", cloud_organized.height, cloud_organized.width);


                std::vector<int> indi = *validIndices.get();
                for(int i = 0; i< 100; ++i)
                {
                   //ROS_INFO("indi %i", indi[i]);
                }
                ROS_WARN("indi %lu", indi.size());

                //cloud.setIndices(validIndices.get());

                 //cloud = rtabmap::util3d::voxelize(cloud, validIndices, 0.01);


                if(cloud_unorganized->size())
                {

                        // convert in /odom frame



                    sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
                    pcl::toROSMsg(*cloud_unorganized, *cloudMsg);
                    cloudMsg->header = map.header;
                    cloudMsg->header.frame_id = "base_footprint";
                    pub_cam_.publish(*cloudMsg);


        /*            rtabmap_ros::CloudInfoPtr info(new CloudInfo);
                    info->message_ = cloudMsg;
                    info->pose_ = rtabmap::Transform::getIdentity();
                    info->id_ = id;

                    if (transformCloud(info, true))
                    {
                        boost::mutex::scoped_lock lock(new_clouds_mutex_);
                        new_cloud_infos_.erase(id);
                        new_cloud_infos_.insert(std::make_pair(id, info));
                    }*/
                }
            }
        }
    }

}

//  MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ipa_rtabmap_interface_node");

    ros::NodeHandle node_handle;
    rtabInterfaceNode ipa_rtabmap_interface(node_handle);

   /* dynamic_reconfigure::Server<ipa_rtabmap_interface::dynReconfigureConfig> server;
    dynamic_reconfigure::Server<ipa_rtabmap_interface::dynReconfigureConfig>::CallbackType f;
    f = boost::bind(&(rtabInterfaceNode::dynamicReconfigure), _1, _2);
    server.setCallback(f);*/

    ROS_INFO("Node is spinning...");
    ros::spin();
    return 0;
}
