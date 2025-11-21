// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <memory>
#include <iomanip>
#include <ros/package.h>

/* ---------- logger ---------- */
namespace spdlog
{
inline void create_logger(std::string log_path)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, 5 * 1024 * 1024, 5);

    std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("unitree", sinks.begin(), sinks.end());

    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v");
    logger->flush_on(spdlog::level::info);

    spdlog::set_default_logger(logger);
}

} // namespace spdlog


namespace param
{
inline std::string VERSION = "1.0.0.1";
inline std::filesystem::path proj_dir;
inline std::filesystem::path config_dir;
inline YAML::Node config;

/* ---------- config.yaml ---------- */
inline void load_config_file()
{
        proj_dir = ros::package::getPath("g1_controller");
        config_dir = proj_dir / "config";

    try {
        std::string config_file = (config_dir / "config.yaml").string();
        if(std::filesystem::exists(config_file))
        {
            config = YAML::LoadFile(config_file);
        }
    } catch (const YAML::BadFile& e) {
        spdlog::error("Failed to load config.yaml: {}", e.what());
        exit(1);
    }
}

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv) 
{
    load_config_file();

    po::options_description desc("Unitree Controller");
    desc.add_options()
        ("help,h", "produce help message")
        ("version,v", "show version")
        ("log", "record log file")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }
    if (vm.count("version"))
    {
        std::cout << "Version: " << VERSION << std::endl;
        exit(0);
    }

#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#else
    spdlog::set_level(spdlog::level::info);
#endif
    if(vm.count("log"))
    {
        std::filesystem::create_directories(proj_dir / "log");
        spdlog::create_logger(proj_dir.string() + "/log/log.txt");
    }

    return vm;
}

}