#include "real_time_map/BatchSaver.h"
#include "toy_map_viewer/BinSaver.h" // LaneSaver namespace가 없다면 전역 함수 saveLidarToBin 사용
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

BatchSaver::BatchSaver() : nh_("~") {
    // 1. 파라미터 로딩 (launch 파일에서 설정 가능)
    nh_.param<int>("start_index", start_index_, 20000);
    nh_.param<int>("batch_size", batch_size_, 20);   // Batch 단위 프레임 수
    nh_.param<int>("overlap_size", overlap_size_, 5); // Overlap 프레임 수
    nh_.param<std::string>("line_map_output_folder", output_folder_, "data/issue/converted_bin/");

    // 2. 경로 설정 및 생성
    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    output_dir_ = pkg_path + "/" + output_folder_ + std::to_string(start_index_) + "/batch/";

    if(fs::exists(output_dir_)) {
        for (const auto& entry : fs::directory_iterator(output_dir_)) {
            fs::remove_all(entry.path());
        }
        ROS_WARN("Cleared existing files in: %s", output_dir_.c_str());
    } else {
        fs::create_directories(output_dir_);
    }

    // 3. FrameLoader 초기화
    frame_loader_ = std::make_unique<FrameLoader>(nh_);

    ROS_INFO("BatchSaver Initialized.");
    ROS_INFO(" - Start Index: %d", start_index_);
    ROS_INFO(" - Batch Size: %d, Overlap: %d", batch_size_, overlap_size_);
    ROS_INFO(" - Output Dir: %s", output_dir_.c_str());
}

void BatchSaver::run() {
    int current_idx = 0;
    int batch_count = 0;
    int consecutive_fail_count = 0;
    int last_saved_batch_start_idx = -1;
    int last_successful_idx = -1;

    ROS_INFO(">>> Start Processing Batches (Loading from pred_frames)...");

    while (ros::ok()) {
        // 1. 프레임 로드 (이미 변환된 데이터)
        auto cloud = frame_loader_->loadFrame(current_idx);

        if (!cloud || cloud->empty()) {
            consecutive_fail_count++;
            if (consecutive_fail_count > 5) {
                ROS_WARN("No more frames found or read error (Stopped at %d).", current_idx);
                break;
            }
            current_idx++;
            continue;
        }
        consecutive_fail_count = 0;
        last_successful_idx = current_idx;

        // 2. 버퍼에 추가
        cloud_buffer_.push_back(cloud);

        // 3. 배치 크기 도달 시 처리
        if (cloud_buffer_.size() >= batch_size_) {
            // (1) 버퍼의 모든 포인트 합치기
            pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto& frame : cloud_buffer_) {
                *merged_cloud += *frame;
            }

            // (2) 저장
            int batch_start_frame = current_idx - (int)cloud_buffer_.size() + 1;
            saveBatchMap(merged_cloud, batch_count);
            last_saved_batch_start_idx = batch_start_frame;
            
            ROS_INFO("Processed Batch %d (Frames: %d ~ %d), Points: %lu", 
                     batch_count, 
                     current_idx - (int)cloud_buffer_.size() + 1, 
                     current_idx, 
                     merged_cloud->size());

            batch_count++;

            // (3) Overlap 처리: 앞에서부터 (Batch - Overlap) 개수만큼 제거
            // 예: Batch 20, Overlap 5 -> 15개 제거, 뒤쪽 5개 남김
            int remove_count = batch_size_ - overlap_size_;
            for (int i = 0; i < remove_count; ++i) {
                if (!cloud_buffer_.empty()) {
                    cloud_buffer_.pop_front();
                }
            }
        }

        current_idx++;
        
        // 무한 루프 방지용 안전 장치 (데이터 양에 따라 조절)
        if (current_idx > start_index_ + 10000) {
            ROS_INFO("Reached safety limit frame index.");
            break; 
        }
    }
    
    // 남은 버퍼 처리: 마지막 자투리 프레임들도 저장할지 여부
    if (last_successful_idx != -1) {
        int final_batch_start_idx = last_successful_idx - batch_size_ + 1;

        if (final_batch_start_idx < 0) final_batch_start_idx = start_index_;

        // 마지막으로 저장된 배치와 다를 경우에만 저장 (중복 방지)
        if (final_batch_start_idx > last_saved_batch_start_idx) {
            
            ROS_INFO(">>> Generating Final Batch (Forcing size %d from frame %d to %d)...", 
                     batch_size_, final_batch_start_idx, last_successful_idx);

            pcl::PointCloud<pcl::PointXYZI>::Ptr final_merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            
            // 유효한 구간 다시 로드
            for (int i = final_batch_start_idx; i <= last_successful_idx; ++i) {
                auto cloud = frame_loader_->loadFrame(i);
                if (cloud && !cloud->empty()) {
                    *final_merged_cloud += *cloud;
                }
            }

            if (!final_merged_cloud->empty()) {
                saveBatchMap(final_merged_cloud, batch_count);
                ROS_INFO("Processed Final Batch %d (Size: %lu points)", batch_count, final_merged_cloud->size());
            }
        } else {
            ROS_INFO(">>> Last batch already saved perfectly. No extra batch needed.");
        }
    }

    ROS_INFO(">>> BatchSaver Finished.");
}

void BatchSaver::saveBatchMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int batch_id) {
    std::string filename = output_dir_ + "batch_" + std::to_string(batch_id) + ".bin";
    // BinSaver.h의 함수 사용 (Global Map처럼 cluster_num=1로 저장)
    saveLidarToBin(filename, cloud); 
}