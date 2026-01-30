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
#include <ament_index_cpp/get_package_share_directory.hpp>

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
    try {
        proj_dir = ament_index_cpp::get_package_share_directory("g1_controller");
    } catch (const std::exception& e) {
        spdlog::error("Failed to get package share directory: {}", e.what());
        exit(1);
    }
    
    config_dir = proj_dir / "config";

    try {
        std::string config_file = (config_dir / "config.yaml").string();
        if(std::filesystem::exists(config_file))
        {
            config = YAML::LoadFile(config_file);
        } else {
             spdlog::error("Config file not found: {}", config_file);
             exit(1);
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
    po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
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
        auto log_dir = std::filesystem::current_path() / "log";
        std::filesystem::create_directories(log_dir);
        spdlog::create_logger((log_dir / "log.txt").string());
    }

    return vm;
}

}