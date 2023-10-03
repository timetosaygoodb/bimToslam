#include "bimToslam/common_include.h"
#include <fstream>
#include <thread>
#include <future>
#include <pcl/filters/voxel_grid.h> // 包含PCL的VoxelGrid头文件

using namespace std;
using namespace pcl; // 使用PCL的命名空间

namespace bimToslam {
    vector<Vec3> readPLYAsync(const string& filename, int skipInterval) {
        
        vector<Vec3> points;

        // 异步加载数据
        future<void> loadData = async(launch::async, [&]() {
            ifstream plyFile(filename);
            if (!plyFile.is_open()) {
                cerr << "无法打开PLY文件: " << filename << endl;
                return;
            }

            string line;
            bool dataSection = false;
            int lineCount = 0;

            // 创建一个点云对象
            PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

            while (getline(plyFile, line)) {
                if (line.find("end_header") != string::npos) {
                    dataSection = true;
                    continue;
                }

                if (dataSection) {
                    if (lineCount % skipInterval == 0) { // 仅处理指定间隔的数据点
                        istringstream iss(line);
                        double x, y, z;
                        if (iss >> x >> y >> z) {
                            PointXYZ point;
                            point.x = static_cast<float>(x);
                            point.y = static_cast<float>(y);
                            point.z = static_cast<float>(z);
                            cloud->push_back(point); // 将点加入点云中
                        }
                    }
                    lineCount++;
                }
            }

            // 对点云进行降采样
            VoxelGrid<PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.01f, 0.01f, 0.01f); // 设置降采样的体素大小
            sor.filter(*cloud); // 执行降采样操作

            // 将降采样后的点云数据转换为Vec3形式
            for (const auto& point : cloud->points) {
                Vec3 vec;
                vec << point.x, point.y, point.z;
                points.push_back(vec);
            }

        });

        // 等待异步加载完成
        loadData.wait();

        return points;
    }

    PointCloud<PointXYZ>::Ptr TestreadPLYAsync(const string& filename, int skipInterval) {
        
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

        // 异步加载数据
        future<void> loadData = async(launch::async, [&]() {
            ifstream plyFile(filename);
            if (!plyFile.is_open()) {
                cerr << "无法打开PLY文件: " << filename << endl;
                return;
            }

            string line;
            bool dataSection = false;
            int lineCount = 0;

            // 创建一个点云对象
            
            while (getline(plyFile, line)) {
                if (line.find("end_header") != string::npos) {
                    dataSection = true;
                    continue;
                }

                if (dataSection) {
                    if (lineCount % skipInterval == 0) { // 仅处理指定间隔的数据点
                        istringstream iss(line);
                        double x, y, z;
                        if (iss >> x >> y >> z) {
                            PointXYZ point;
                            point.x = static_cast<float>(x);
                            point.y = static_cast<float>(y);
                            point.z = static_cast<float>(z);
                            cloud->push_back(point); // 将点加入点云中
                        }
                    }
                    lineCount++;
                }
            }

            // 对点云进行降采样
            VoxelGrid<PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.01f, 0.01f, 0.01f); // 设置降采样的体素大小
            sor.filter(*cloud); // 执行降采样操作


            

        });

        // 等待异步加载完成
        loadData.wait();

        return cloud;

    }
}
