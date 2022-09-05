/**
 * @file
 * @brief Example node to publish raw sensor output on ROS topics
 *
 * ROS Parameters
 * sensor_hostname: hostname or IP in dotted decimal form of the sensor
 * udp_dest: hostname or IP where the sensor will send data packets
 * lidar_port: port to which the sensor should send lidar data
 * imu_port: port to which the sensor should send imu data
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <string>

#include "ouster/build.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

#include "std_msgs/String.h"
#include "ouster/packet.h"

using PacketMsg = ouster_ros::PacketMsg;
using OSConfigSrv = ouster_ros::OSConfigSrv;
using nonstd::optional;
namespace sensor = ouster::sensor;

#define PI 3.141592653589793238762

// fill in values that could not be parsed from metadata
void populate_metadata_defaults(sensor::sensor_info& info,
                                sensor::lidar_mode specified_lidar_mode) {
    if (!info.name.size()) info.name = "UNKNOWN";

    if (!info.sn.size()) info.sn = "UNKNOWN";

    ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
    if (v == ouster::util::invalid_version)
        ROS_WARN("Unknown sensor firmware version; output may not be reliable");
    else if (v < sensor::min_version)
        ROS_WARN("Firmware < %s not supported; output may not be reliable",
                 to_string(sensor::min_version).c_str());

    if (!info.mode) {
        ROS_WARN(
            "Lidar mode not found in metadata; output may not be reliable");
        info.mode = specified_lidar_mode;
    }

    if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        ROS_WARN("Beam angles not found in metadata; using design values");
        info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
        info.beam_altitude_angles = sensor::gen1_altitude_angles;
    }
}

// try to write metadata file
bool write_metadata(const std::string& meta_file, const std::string& metadata) {
    std::ofstream ofs;
    ofs.open(meta_file);
    ofs << metadata << std::endl;
    ofs.close();
    if (ofs) {
        ROS_INFO("Wrote metadata to %s", meta_file.c_str());
    } else {
        ROS_WARN(
            "Failed to write metadata to %s; check that the path is valid. If "
            "you provided a relative path, please note that the working "
            "directory of all ROS nodes is set by default to $ROS_HOME",
            meta_file.c_str());
        return false;
    }
    return true;
}

int connection_loop(ros::NodeHandle& nh, sensor::client& cli,
                    const sensor::sensor_info& info) {
    auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
    auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);

    auto pf = sensor::get_format(info);

    // 추가------------------------------------------------------
    ros::Publisher pos_pub1 = nh.advertise<std_msgs::String>("angle_topic1", 1);
    ros::Publisher pos_pub2 = nh.advertise<std_msgs::String>("angle_topic2", 1);
    ros::Publisher pos_pub3 = nh.advertise<std_msgs::String>("angle_topic3", 1);
    ros::Publisher pos_pub4 = nh.advertise<std_msgs::String>("angle_topic4", 1);
    ros::Publisher pos_pub5 = nh.advertise<std_msgs::String>("angle_topic5", 1);
    ros::Publisher pos_pub6 = nh.advertise<std_msgs::String>("angle_topic6", 1);

    ros::Rate rate(10);

    uint64_t how_many_h_lidar_call = 0;

    //---------------------------------------------------

    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(pf.lidar_packet_size + 1);
    imu_packet.buf.resize(pf.imu_packet_size + 1);

    while (ros::ok()) {

    	std_msgs::String angle_is_30;                                                   // publish하기 위한 data (현재는 ROSTIME을 위해 String으로 publish함)
    	std::stringstream ss;                                                           // data를 topic에 저장할 때, sstream을 이용함


        auto state = sensor::poll_client(cli);
        if (state == sensor::EXIT) {
            ROS_INFO("poll_client: caught signal, exiting");
            return EXIT_SUCCESS;
        }
        if (state & sensor::CLIENT_ERROR) {
            ROS_ERROR("poll_client: returned error");
            return EXIT_FAILURE;
        }
        if (state & sensor::LIDAR_DATA) {
            if (sensor::read_lidar_packet(cli, lidar_packet.buf.data(), pf))
                lidar_packet_pub.publish(lidar_packet);
        }
        if (state & sensor::IMU_DATA) {
            if (sensor::read_imu_packet(cli, imu_packet.buf.data(), pf))
                imu_packet_pub.publish(imu_packet);
        }

        //auto lidar_col_0_ts = ouster::col_timestamp(lidar_packet.buf.data());                 // os1_node에서의 lidar_packet을 이용하여 각도에 대한 정보를 반환해올 수 있음
                                                                                                  // 첨부된 ouster lidar software maunal의 17page와 os1_packet.h를 참고하면 알 수 있음
        auto lidar_col_0_h_angle = (ouster::col_h_angle(lidar_packet.buf.data()) * 180) / PI; // os1_packet.h의 라디안 return값을 degree로 변환하여 계산에 용이하도록 변환함

        if(lidar_col_0_h_angle >= 359 && lidar_col_0_h_angle <= 2) {
            //ss << "(174.0 to 186.0) camera shutter should be opearted by " << how_many_h_lidar_call << " and angle is : " << lidar_col_0_h_angle << "\ntime : " << lidar_col_0_ts;
            ss << ros::Time::now().nsec;
            angle_is_30.data = ss.str();
            pos_pub1.publish(angle_is_30);
            std::cout << "ros::Time::now().sec : " << ros::Time::now().sec << std:: endl;
            std::cout << "ros::Time::now().nsec : " << ros::Time::now().nsec << std:: endl;
        //   std::cout << (unsigned long)ros::Time::now().sec * 1000000000ull << std:: endl;
        //   std::cout << (unsigned long)ros::Time::now().nsec << std:: endl;
        //   ROS_INFO("SUHTTER SIGNAL IS PUBLISHED");
        //    std::cout << ss.str() << std::endl;
            std::cout << "n : " << how_many_h_lidar_call << std::endl;
        //   std::cout << "timestamp : " << lidar_col_0_ts << std::endl;
        //    std::cout << "col_h_angle : " << lidar_col_0_h_angle << std::endl;
            std::cout << "===============================================" << std::endl;

            //FILE* fp = fopen("/home/vialab/jjh_ouster_ws/os1_output.txt", "a+");
            //fprintf(fp, "%d\t%lf\n",how_many_h_lidar_call, ros::Time::now().toSec());
            //fclose(fp);
            how_many_h_lidar_call++;
            ros::spinOnce();
        }
        else if(lidar_col_0_h_angle >= 44.0 && lidar_col_0_h_angle <= 46.0) {
            ss << ros::Time::now().nsec;
            angle_is_30.data = ss.str();
            pos_pub2.publish(angle_is_30);
            std::cout << "ros::Time::now().sec : " << ros::Time::now().sec << std:: endl;
            std::cout << "ros::Time::now().nsec : " << ros::Time::now().nsec << std:: endl;
            std::cout << "n : " << how_many_h_lidar_call << std::endl;
            std::cout << "===============================================" << std::endl;

            //FILE* fp = fopen("/home/vialab/jjh_ouster_ws/os1_output.txt", "a+");
            //fprintf(fp, "%d\t%lf\n",how_many_h_lidar_call, ros::Time::now().toSec());
            //fclose(fp);
            how_many_h_lidar_call++;
            ros::spinOnce();
        }
        else if(lidar_col_0_h_angle >= 134.0 && lidar_col_0_h_angle <= 136.0) {
            ss << ros::Time::now().nsec;
            angle_is_30.data = ss.str();
            pos_pub3.publish(angle_is_30);
            std::cout << "ros::Time::now().sec : " << ros::Time::now().sec << std:: endl;
            std::cout << "ros::Time::now().nsec : " << ros::Time::now().nsec << std:: endl;
            std::cout << "n : " << how_many_h_lidar_call << std::endl;
            std::cout << "===============================================" << std::endl;
            how_many_h_lidar_call++;
            ros::spinOnce();
        }
        else if(lidar_col_0_h_angle >= 179.0 && lidar_col_0_h_angle <= 181.0) {
            ss << ros::Time::now().nsec;
            angle_is_30.data = ss.str();
            pos_pub4.publish(angle_is_30);
            std::cout << "ros::Time::now().sec : " << ros::Time::now().sec << std:: endl;
            std::cout << "ros::Time::now().nsec : " << ros::Time::now().nsec << std:: endl;
            std::cout << "n : " << how_many_h_lidar_call << std::endl;
            std::cout << "===============================================" << std::endl;
            how_many_h_lidar_call++;
            ros::spinOnce();
        }
        else if(lidar_col_0_h_angle >= 224.0 && lidar_col_0_h_angle <= 226.0) {
            ss << ros::Time::now().nsec;
            angle_is_30.data = ss.str();
            pos_pub5.publish(angle_is_30);
            std::cout << "ros::Time::now().sec : " << ros::Time::now().sec << std:: endl;
            std::cout << "ros::Time::now().nsec : " << ros::Time::now().nsec << std:: endl;
            std::cout << "n : " << how_many_h_lidar_call << std::endl;
            std::cout << "===============================================" << std::endl;
            how_many_h_lidar_call++;
            ros::spinOnce();
        }
        else if(lidar_col_0_h_angle >= 314.0 && lidar_col_0_h_angle <= 316.0) {
            ss << ros::Time::now().nsec;
            angle_is_30.data = ss.str();
            pos_pub6.publish(angle_is_30);
            std::cout << "ros::Time::now().sec : " << ros::Time::now().sec << std:: endl;
            std::cout << "ros::Time::now().nsec : " << ros::Time::now().nsec << std:: endl;
            std::cout << "n : " << how_many_h_lidar_call << std::endl;
            std::cout << "===============================================" << std::endl;
            how_many_h_lidar_call++;
            ros::spinOnce();
        }
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "os_node");
    ros::NodeHandle nh("~");

    std::string published_metadata;
    auto srv = nh.advertiseService<OSConfigSrv::Request, OSConfigSrv::Response>(
        "os_config", [&](OSConfigSrv::Request&, OSConfigSrv::Response& res) {
            if (published_metadata.size()) {
                res.metadata = published_metadata;
                return true;
            } else
                return false;
        });

    // empty indicates "not set" since roslaunch xml can't optionally set params
    auto hostname = nh.param("sensor_hostname", std::string{});
    auto udp_dest = nh.param("udp_dest", std::string{});
    auto lidar_port = nh.param("lidar_port", 0);
    auto imu_port = nh.param("imu_port", 0);
    auto replay = nh.param("replay", false);
    auto lidar_mode_arg = nh.param("lidar_mode", std::string{});
    auto timestamp_mode_arg = nh.param("timestamp_mode", std::string{});

    std::string udp_profile_lidar_arg;
    nh.param<std::string>("udp_profile_lidar", udp_profile_lidar_arg, "");

    optional<sensor::UDPProfileLidar> udp_profile_lidar;
    if (udp_profile_lidar_arg.size()) {
        if (replay)
            ROS_WARN("UDP Profile Lidar set in replay mode. Will be ignored.");

        // set lidar profile from param
        udp_profile_lidar =
            sensor::udp_profile_lidar_of_string(udp_profile_lidar_arg);
        if (!udp_profile_lidar) {
            ROS_ERROR("Invalid udp profile lidar: %s",
                      udp_profile_lidar_arg.c_str());
            return EXIT_FAILURE;
        }
    }

    // set lidar mode from param
    sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
    if (lidar_mode_arg.size()) {
        if (replay) ROS_WARN("Lidar mode set in replay mode. May be ignored");

        lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
        if (!lidar_mode) {
            ROS_ERROR("Invalid lidar mode %s", lidar_mode_arg.c_str());
            return EXIT_FAILURE;
        }
    }

    // set timestamp mode from param
    sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
    if (timestamp_mode_arg.size()) {
        if (replay)
            ROS_WARN("Timestamp mode set in replay mode. Will be ignored");

        timestamp_mode = sensor::timestamp_mode_of_string(timestamp_mode_arg);
        if (!timestamp_mode) {
            ROS_ERROR("Invalid timestamp mode %s", timestamp_mode_arg.c_str());
            return EXIT_FAILURE;
        }
    }

    auto meta_file = nh.param("metadata", std::string{});
    if (!meta_file.size()) {
        if (replay) {
            ROS_ERROR("Must specify metadata file in replay mode");
        } else {
            ROS_ERROR("Must specify path for metadata output");
        }
        return EXIT_FAILURE;
    }

    if (!replay && !hostname.size()) {
        ROS_ERROR("Must specify a sensor hostname");
        return EXIT_FAILURE;
    }

    ROS_INFO("Client version: %s", ouster::CLIENT_VERSION_FULL);

    if (replay) {
        ROS_INFO("Running in replay mode");

        // populate info for config service
        try {
            auto info = sensor::metadata_from_json(meta_file);
            published_metadata = to_string(info);

            ROS_INFO("Using lidar_mode: %s",
                     sensor::to_string(info.mode).c_str());
            ROS_INFO("%s sn: %s firmware rev: %s", info.prod_line.c_str(),
                     info.sn.c_str(), info.fw_rev.c_str());

            // just serve config service
            ros::spin();
            return EXIT_SUCCESS;
        } catch (const std::runtime_error& e) {
            ROS_ERROR("Error when running in replay mode: %s", e.what());
        }
    } else {
        ROS_INFO("Waiting for sensor %s to initialize ...", hostname.c_str());

        // use no-config version of init_client to allow for random ports
        auto cli = sensor::init_client(hostname, lidar_port, imu_port);
        if (!cli) {
            ROS_ERROR("Failed to initialize sensor at: %s", hostname.c_str());
            return EXIT_FAILURE;
        }

        sensor::sensor_config config;
        config.udp_port_imu = get_imu_port(*cli);
        config.udp_port_lidar = get_lidar_port(*cli);
        config.udp_profile_lidar = udp_profile_lidar;
        config.operating_mode = sensor::OPERATING_NORMAL;
        if (lidar_mode) config.ld_mode = lidar_mode;
        if (timestamp_mode) config.ts_mode = timestamp_mode;

        uint8_t config_flags = 0;

        if (udp_dest.size()) {
            ROS_INFO("Sending UDP data to %s", udp_dest.c_str());
            config.udp_dest = udp_dest;
        } else {
            ROS_INFO("Using automatic UDP destination");
            config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        }

        try {
            set_config(hostname, config, config_flags);
            ROS_INFO("Sensor configured successfully");
        } catch (const std::runtime_error& e) {
            ROS_ERROR("Errror setting config:  %s", e.what());
            return EXIT_FAILURE;
        } catch (const std::invalid_argument& ia) {
            ROS_ERROR("Error setting config: %s", ia.what());
            return EXIT_FAILURE;
        }

        // fetch metadata for client after setting configs
        auto metadata = sensor::get_metadata(*cli);

        ROS_INFO("Sensor initialized successfully");

        // write metadata file. If metadata_path is relative, will use cwd
        // (usually ~/.ros)
        if (!write_metadata(meta_file, metadata)) {
            ROS_ERROR("Exiting because of failure to write metadata path");
            return EXIT_FAILURE;
        }

        // populate sensor info
        auto info = sensor::parse_metadata(metadata);
        populate_metadata_defaults(info, sensor::MODE_UNSPEC);
        published_metadata = to_string(info);

        ROS_INFO("Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
        ROS_INFO("%s sn: %s firmware rev: %s", info.prod_line.c_str(),
                 info.sn.c_str(), info.fw_rev.c_str());

        // publish packet messages from the sensor
        return connection_loop(nh, *cli, info);
    }
}
