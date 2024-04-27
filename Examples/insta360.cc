/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <signal.h>

#include <System.h>
#include <json.h>
#include <CLI11.hpp>

using namespace std;
using nlohmann::json;


void signal_callback_handler(int signum) {
   cout << "Insta360 SLAM caught signal " << signum << endl;
   // Terminate program
   exit(signum);
}


bool LoadInsta360IMU(
    const string &path_to_json_file,
    vector<double> &vImuTimeStamps,
    vector<double> &vImgTimeStamps,
    vector<cv::Point3f> &vAcc,
    vector<cv::Point3f> &vGyro
) {
    std::ifstream file;
    file.open(path_to_json_file.c_str());
    if (!file.is_open()) {
        return false;
    }
    json j;
    file >> j;
    const auto accl = j["accelerometer"];
    const auto gyro = j["gyroscope"];
    const auto imu_time = j["timestamps_ns"];
    const auto img_time = j["img_timestamps_ns"];

    for (const auto &e : accl) {
        cv::Point3f v((float)e[0], (float)e[1], (float)e[2]);
        vAcc.push_back(v);
    }
    for (const auto &e : gyro) {
        cv::Point3f v((float)e[0], (float)e[1], (float)e[2]);
        vGyro.push_back(v);
    }
    for (const auto &e : imu_time) {
        vImuTimeStamps.push_back((double)e * 1e-9);
    }
    for (const auto &e : img_time) {
        vImgTimeStamps.push_back((double)e * 1e-9);
    }

    file.close();
    return true;
}

double ttrack_tot = 0;
int main(int argc, char *argv[])
{
    signal(SIGINT, signal_callback_handler);

    // CLI parsing
    CLI::App app{"Insta360 SLAM"};

    std::string input_video_mp4;
    app.add_option("-i,--input_video", input_video_mp4)->required();

    std::string input_imu_json;
    app.add_option("-j,--input_imu", input_imu_json)->required();

    std::string output_trajectory_csv;
    app.add_option("-o,--output_trajectory_csv", output_trajectory_csv);

    std::string load_map;
    app.add_option("-l,--load_map", load_map);

    std::string save_map;
    app.add_option("-s,--save_map", save_map);
  
    bool disable_gui = false;
    app.add_flag("-g,--disable_gui", disable_gui);

    bool visual_only = false;
    app.add_flag("-v,--visual_only", visual_only);

    bool tracking_only = false;
    app.add_flag("-t,--tracking_only", tracking_only);

    int num_threads = 4;
    app.add_flag("-n,--num_threads", num_threads);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &e) {
        return app.exit(e);
    }
    
    std::string vocabulary_txt = "./Vocabulary/ORBvoc.txt";
    std::string setting_yaml = "./Examples/insta360_fisheye.yaml";

    cv::setNumThreads(num_threads);
    
    // open setting to get image resolution
    cv::FileStorage fsSettings(setting_yaml, cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        cerr << "Failed to open setting file at: " << setting_yaml << endl;
        exit(-1);
    }
    cv::Size img_size(fsSettings["Camera.width"],fsSettings["Camera.height"]);
    fsSettings.release();

    // Load imu
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsCam;
    vector<double> vTimestampsImu;
    LoadInsta360IMU(input_imu_json, vTimestampsImu, vTimestampsCam, vAcc, vGyro);

    // Load video
    cv::VideoCapture cap(input_video_mp4, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
        std::cout << "Error opening video file" << input_video_mp4 << endl;
        return -1;
    }
    int nFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    cout << "Video and IMU data loaded! " << "Number of images: " << nFrames << " FPS: " << fps << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System::eSensor sensor;
    if (visual_only) {
        sensor = ORB_SLAM3::System::MONOCULAR;
    } else {
        sensor = ORB_SLAM3::System::IMU_MONOCULAR;
    }
    ORB_SLAM3::System SLAM(
        vocabulary_txt,
        setting_yaml,
        sensor, 
        !disable_gui,
        load_map, 
        save_map
    );

    if (tracking_only) {
        cout << "Running localization (tracking only) mode!" << endl;
        SLAM.ActivateLocalizationMode();
    }

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    // Find first imu to be considered, supposing imu measurements start first
    int iLastImuIdx = 0;
    while(vTimestampsImu[iLastImuIdx] <= vTimestampsCam[0]) {
        iLastImuIdx++;
    }
    iLastImuIdx--;

    // Main loop
    cv::Mat im;
    for(int iFrame = 0; iFrame < nFrames; iFrame++)
    {
        double tframe = vTimestampsCam[iFrame];
        // Read image from file
        bool success = cap.read(im);
        if (!success) {
            cout << "Failed to read image at frame " << iFrame << endl;
            break;
        }

        // Resize image
        if (im.size() != img_size){
            cv::resize(im, im, img_size);
        }

        // Load imu measurements from previous frame
        vImuMeas.clear();
        if(iFrame > 0) {
            while(vTimestampsImu[iLastImuIdx] <= tframe) {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                    vAcc[iLastImuIdx].x,
                    vAcc[iLastImuIdx].y,
                    vAcc[iLastImuIdx].z,                                    
                    vGyro[iLastImuIdx].x,
                    vGyro[iLastImuIdx].y,
                    vGyro[iLastImuIdx].z,
                    vTimestampsImu[iLastImuIdx]
                ));
                iLastImuIdx++;
            }
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the image to the SLAM system
        if (visual_only) {
            SLAM.TrackMonocular(im, tframe);  
        } else {
            SLAM.TrackMonocular(im, tframe, vImuMeas);
        }
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double dTrackTime = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        if (iFrame % 100 == 0) {
            cout << "Video FPS: " << fps << ", ORB-SLAM 3 running at: " << 1./dTrackTime << " FPS" << endl;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    if (!output_trajectory_csv.empty()) {
        SLAM.SaveTrajectoryCSV(output_trajectory_csv);
    }

    return 0;
}
