#pragma once

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <visualization_msgs/Marker.h>
#include <unitree/idl/go2/HeightMap_.hpp>
#include <mutex>
#include <random>


class HeightMapHandler
{
public:
    using SharedPtr = std::shared_ptr<HeightMapHandler>;

    HeightMapHandler() {}

    HeightMapHandler(ros::NodeHandle& nh)
    {
        // elevation map client.
        elevationMapClient_ = nh.serviceClient<grid_map_msgs::GetGridMap>("/elevation_mapping/get_raw_submap");
        // marker publisher.
        markerPub_ = nh.advertise<visualization_msgs::Marker>("/elevation_mapping/foot_scan", 0);

        // grid-map service message.
        gridMapSrv_.request.length_x = 6.0;
        gridMapSrv_.request.length_y = 6.0;
        gridMapSrv_.request.position_x = 0.0;
        gridMapSrv_.request.position_y = 0.0;
        // base height.
        baseHeight_ = 0.0;

        isSimulation_ = nh.param<bool>("simulation", false);
        measurementPoints_pub_ = std::make_unique<MeasurementPoints_t>("rt/measurementpoints");
    }

    ~HeightMapHandler()
    {
        terminateMapThread_ = true;
        if (mapThread_ && mapThread_->joinable()) {
            mapThread_->join();
        }
    }

    void reset() {
        mapInitialized_ = false;
    }

    void init(YAML::Node cfg)
    {
        basePosition_ << 0.0, 0.0, 0.76;
        layerName_ = cfg["layer_name"].as<std::string>();
        odomFrameId_ = cfg["frame_id"].as<std::string>();
        gridMapSrv_.request.frame_id = odomFrameId_;

        const std::vector<float> measurementPointsX = cfg["measurement_points"]["x"].as<std::vector<float>>();
        const std::vector<float> measurementPointsY = cfg["measurement_points"]["y"].as<std::vector<float>>();
        int Nx = measurementPointsX.size();
        int Ny = measurementPointsY.size();
        numPoints_ = Nx * Ny;
        measurementPoints_ = Eigen::MatrixXd::Zero(numPoints_, 2);
        measurementHeights_ = Eigen::VectorXd::Zero(numPoints_);
        simulationHeights_ = Eigen::VectorXd::Zero(numPoints_);
        heightMeasurementsVisualizer_ = Eigen::MatrixXd::Zero(numPoints_, 4);
        const std::string scanOrdering = cfg["scan_ordering"].as<std::string>();
        if (scanOrdering == "xy") {
            for (size_t j = 0; j < Ny; ++j) {
                for (size_t i = 0; i < Nx; ++i) {
                    size_t k = j * Nx + i;
                    measurementPoints_(k, 0) = measurementPointsX[i];
                    measurementPoints_(k, 1) = measurementPointsY[j];
                    measurementIndices_.push_back(k);
                }
            }
        } else {
            for (size_t i = 0; i < Nx; ++i) {
                for (size_t j = 0; j < Ny; ++j) {
                    size_t k = i * Ny + j;
                    measurementPoints_(k, 0) = measurementPointsX[i];
                    measurementPoints_(k, 1) = measurementPointsY[j];
                    measurementIndices_.push_back(j * Nx + i);
                }
            }
        }
        if (isSimulation_ && measurementPoints_pub_->trylock()) {
            measurementPoints_pub_->msg_.resolution() = (measurementPointsX.back() - measurementPointsX.front()) / (Nx - 1);
            measurementPoints_pub_->msg_.width() = Nx;
            measurementPoints_pub_->msg_.height() = Ny;
            measurementPoints_pub_->msg_.origin() = {measurementPointsX.front(), measurementPointsY.front()};
            measurementPoints_pub_->unlockAndPublish();
        }
    }

    void start() {
        initializeScanMarkers();
        terminateMapThread_ = false;
        reset();
        if (mapThread_) {
            mapThread_->join();
        }
        mapThread_.reset(new std::thread(&HeightMapHandler::mapThreadCallback, this));
    }

    void update(const double x, const double y, const double z) {
        gridMapSrv_.request.position_x = x;
        gridMapSrv_.request.position_y = y;
        baseHeight_ = z;

        if (elevationMapClient_.call(gridMapSrv_)) {
            grid_map::GridMapRosConverter::fromMessage(gridMapSrv_.response.map, gridMapBuffer_);
            processGridMap(gridMapBuffer_);
        } else {
            ROS_WARN_THROTTLE(0.1, "Couldn't call the map service");
            return;
        }

        {
            std::lock_guard<std::mutex> guard(mapMutex_);
            gridMap_ = gridMapBuffer_;
            mapInitialized_ = true;
        }
    }

    Eigen::VectorXd getMeasurementHeights(Eigen::Quaterniond& quat) {
        // keep only yaw rotation
        quat.x() = 0.;
        quat.y() = 0.;
        quat.normalize();
        Eigen::Matrix3d R_b_yaw = quat.toRotationMatrix();
        {
            std::lock_guard<std::mutex> guard(mapMutex_);
            for (int i = 0; i < measurementPoints_.rows(); i++) {
                Eigen::Vector2d point = R_b_yaw.topLeftCorner(2, 2) * measurementPoints_.row(i).transpose() + basePosition_.head(2);
                float height;
                if (isSimulation_) {
                    height = simulationHeights_(i);
                } else {
                    try {
                    // height = gridMap_.atPosition(layerName_, {point(0), point(1)});
                    height = getHeight(point(0), point(1));
                    } catch (...) {
                    height = 0.f;
                    ROS_INFO_STREAM("[RLController] gridMap_.atPosition failed !");
                    }
                }
                measurementHeights_(i) = std::max(std::min(basePosition_(2) - 0.5 - height, 1.0), -1.0);
                heightMeasurementsVisualizer_(i, 0) = point(0);
                heightMeasurementsVisualizer_(i, 1) = point(1);
                heightMeasurementsVisualizer_(i, 2) = height;
                heightMeasurementsVisualizer_(i, 3) = basePosition_(2);
            }
        }
        return measurementHeights_;
    }

    double getHeight(double x, double y) {
        double h = 0.0;
        try {
            h = gridMap_.atPosition(layerName_, {x, y});
            if (!mapInitialized_) {
            // std::normal_distribution<double> normDist(0., 1.);
            h = baseHeight_ - 0.76;  //+ normDist(mt_) * 0.10;
            ROS_WARN_THROTTLE(1.0, "map is not initialized. filling random values");
            }
        } catch (...) {
            // std::normal_distribution<double> normDist(0., 1.);
            h = baseHeight_ - 0.76;  // + normDist(mt_) * 0.10;
            ROS_WARN_THROTTLE(0.2, "Got error at getting heigt from map. filling random values");
        }
        return h;
    }

    void initializeScanMarkers() {
        int numberOfPoints = static_cast<int>(measurementPoints_.rows());
        // add height map markers
        marker_.header.frame_id = "odom";
        marker_.header.stamp = ros::Time();
        marker_.ns = "rl_controller";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.scale.x = 0.05;
        marker_.scale.y = 0.05;
        marker_.scale.z = 0.05;
        marker_.color.a = 1.0;
        marker_.color.r = 1.0;
        marker_.color.g = 0.0;
        marker_.color.b = 0.0;
        marker_.points.resize(numberOfPoints);
        marker_.colors.resize(numberOfPoints);
        marker_.pose.orientation.w = 1.0;
    }

    void updateScanMarker(const Eigen::MatrixX4d scanPoints) {
        int cnt = 0;
        for (int i = 0; i < scanPoints.rows(); i++) {
            // position
            geometry_msgs::Point obj;
            obj.x = scanPoints(i, 0);
            obj.y = scanPoints(i, 1);
            obj.z = scanPoints(i, 2);
            marker_.points[cnt] = obj;
            // color
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;
            marker_.colors[cnt] = color;
            cnt++;
        }
        marker_.header.stamp = ros::Time::now();
        markerPub_.publish(marker_);
    }

    void mapThreadCallback() {
        while (!terminateMapThread_) {
            if (!isSimulation_) {
                update(basePosition_[0], basePosition_[1], basePosition_[2]);
            }
            updateScanMarker(heightMeasurementsVisualizer_);
            usleep(10000);
        }
    }

    std::mutex mapMutex_;
    int numPoints_;
    Eigen::Vector3d basePosition_;
    bool terminateMapThread_ = true;
    std::unique_ptr<std::thread> mapThread_;
    std::vector<size_t> measurementIndices_;
    Eigen::VectorXd simulationHeights_;

private:

    void processGridMap(grid_map::GridMap& gridMap) {
        Eigen::MatrixXf& map = gridMap.get(layerName_);
        std::mt19937 mt{std::random_device{}()};
        std::normal_distribution<double> normDist(0., 1.);
        float baseHeight = 0.0;
        for (size_t i = 0; i < map.cols(); i++) {
            for (size_t j = 0; j < map.rows(); j++) {
            if (std::isnan(map(j, i)))
                map(j, i) = baseHeight;
            else
                baseHeight = map(j, i);
            }
        }
        gridMap.add(layerName_, map);
    }


    void updateTransform(std::string mapFrameId, ros::Time timeStamp) {
        tf::StampedTransform transformTf;
        try {
            transformListener_.waitForTransform(odomFrameId_, mapFrameId, timeStamp, ros::Duration(1.0));
            transformListener_.lookupTransform(odomFrameId_, mapFrameId, timeStamp, transformTf);
            tf::poseTFToEigen(transformTf, transformationOdomToMap_);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }

    // heightmap message parameters
    std::string layerName_;
    std::string odomFrameId_;

    // measurement points
    Eigen::MatrixX2d measurementPoints_;
    Eigen::VectorXd measurementHeights_;
    Eigen::MatrixX4d heightMeasurementsVisualizer_;

    // ROS services
    ros::ServiceClient elevationMapClient_;
    ros::Publisher markerPub_;
    tf::TransformListener transformListener_;

    // Map thread termination flag and initialization flag.
    bool mapInitialized_ = false;

    // Scan buffers.
    double baseHeight_{0.0};
    std::vector<size_t> footFrameIndicies_;

    // Grid map buffers.
    grid_map_msgs::GetGridMap gridMapSrv_;
    grid_map::GridMap gridMap_;
    grid_map::GridMap gridMapBuffer_;

    // Transformation between the odom and map frames.
    Eigen::Isometry3d transformationOdomToMap_;

    // Visualization marker.
    visualization_msgs::Marker marker_;

    // Random number generator.
    std::mt19937 mt_{std::random_device{}()};

    // Simulation flag.
    bool isSimulation_;

    using MeasurementPoints_t = unitree::robot::RealTimePublisher<unitree_go::msg::dds_::HeightMap_>;
    std::unique_ptr<MeasurementPoints_t> measurementPoints_pub_;
};
