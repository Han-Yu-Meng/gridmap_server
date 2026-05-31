/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#pragma once

#include <fins/node.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

class GridMapServer : public fins::Node {
public:
    void define() override {
        set_name("GridMapServer");
        set_description("Static Grid Map Server");
        set_category("Navigation");

        register_output<nav_msgs::msg::OccupancyGrid>("global_grid_map");

        register_parameter<std::string>("map_yaml_path", &GridMapServer::on_map_path_changed, "");
    }

    void initialize() override {
        logger->info("Initializing GridMapServer...");
        
        if (!map_path_.empty()) {
            if (load_map_from_yaml(map_path_)) {
                map_loaded_ = true;
            }
        } else {
            logger->warn("No map path provided during initialization.");
        }
    }

    void run() override {
        if (map_loaded_) {
            logger->info("Publishing map once on run.");
            send("global_grid_map", cached_map_msg_, fins::now());
        }
    }

private:
    void on_map_path_changed(std::string new_path) {
        if (new_path == map_path_) return;
        logger->info("Map path changed to: {}. Reloading...", new_path);
        if (load_map_from_yaml(new_path)) {
            map_path_ = new_path;
            map_loaded_ = true;
            send("global_grid_map", cached_map_msg_, fins::now());
        }
    }

    bool load_map_from_yaml(const std::string& yaml_path) {
        try {
            if (!std::filesystem::exists(yaml_path)) {
                logger->error("Map YAML file does not exist: {}", yaml_path);
                return false;
            }

            YAML::Node config = YAML::LoadFile(yaml_path);

            std::string image_file = config["image"].as<std::string>();
            double resolution = config["resolution"].as<double>();
            std::vector<double> origin = config["origin"].as<std::vector<double>>();
            double occupied_thresh = config["occupied_thresh"].as<double>(0.65);
            double free_thresh = config["free_thresh"].as<double>(0.196);
            int negate = config["negate"].as<int>(0);

            std::filesystem::path p(yaml_path);
            std::string full_image_path = (p.parent_path() / image_file).string();

            cv::Mat img = cv::imread(full_image_path, cv::IMREAD_GRAYSCALE);
            if (img.empty()) {
                logger->error("Failed to load map image: {}", full_image_path);
                return false;
            }

            cached_map_msg_.header.frame_id = "map";
            cached_map_msg_.info.resolution = resolution;
            cached_map_msg_.info.width = img.cols;
            cached_map_msg_.info.height = img.rows;
            cached_map_msg_.info.origin.position.x = origin[0];
            cached_map_msg_.info.origin.position.y = origin[1];
            cached_map_msg_.info.origin.orientation.z = sin(origin[2] / 2.0);
            cached_map_msg_.info.origin.orientation.w = cos(origin[2] / 2.0);

            cached_map_msg_.data.resize(img.cols * img.rows);

            for (int y = 0; y < img.rows; ++y) {
                for (int x = 0; x < img.cols; ++x) {
                    uint8_t pixel = img.at<uint8_t>(img.rows - 1 - y, x);
                    double occ = (negate) ? (pixel / 255.0) : (1.0 - pixel / 255.0);

                    if (occ > occupied_thresh) {
                        cached_map_msg_.data[y * img.cols + x] = 100; // 占据
                    } else if (occ < free_thresh) {
                        cached_map_msg_.data[y * img.cols + x] = 0;   // 空闲
                    } else {
                        cached_map_msg_.data[y * img.cols + x] = -1;  // 未知
                    }
                }
            }

            logger->info("Map loaded successfully: {}x{} @ {} m/pix", img.cols, img.rows, resolution);
            return true;

        } catch (const std::exception& e) {
            logger->error("Error parsing map YAML: {}", e.what());
            return false;
        }
    }

    std::string map_path_;
    bool map_loaded_ = false;
    nav_msgs::msg::OccupancyGrid cached_map_msg_;
};

EXPORT_NODE(GridMapServer)