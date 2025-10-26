# Lidar Obstacle Detection

> **Note**: This project was originally submitted as part of Udacity's Sensor Fusion Nanodegree in 2020. It has been published on GitHub with improvements and refinements.

![Skills](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)
![PCL](https://img.shields.io/badge/PCL-0078D4?style=flat&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-064F8C?style=flat&logo=cmake&logoColor=white)

**Tags**: `lidar` `point-cloud-processing` `PCL` `3d-segmentation` `RANSAC` `KD-tree` `euclidean-clustering` `obstacle-detection` `autonomous-vehicles` `sensor-fusion` `3d-computer-vision` `voxel-filtering` `region-growing`

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

## Project Overview

This project focuses on using Lidar sensor data for obstacle detection in autonomous driving applications. I implemented a complete point cloud processing pipeline that segments the road surface, detects obstacles, and clusters them into distinct objects - all critical components for understanding the 3D environment around a self-driving car.

### My Contributions

I developed the complete 3D point cloud processing pipeline:

* **Point Cloud Filtering**: Implemented voxel grid filtering to downsample large point clouds while preserving structure
* **RANSAC Segmentation**: Created a custom 3D RANSAC algorithm to separate road plane from obstacles
* **KD-Tree Implementation**: Built a 3D KD-tree data structure from scratch for efficient spatial searching
* **Euclidean Clustering**: Developed a clustering algorithm using the KD-tree to group obstacle points
* **Region of Interest**: Applied crop box filtering to focus on relevant driving areas
* **Real-time Processing**: Optimized the pipeline to process point clouds in real-time

## Project Structure

```
lidar-obstacle-detector/
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
├── media/                      # Media files
│   └── ObstacleDetectionFPS.gif
├── src/
│   ├── environment.cpp         # Main application and visualization
│   ├── processPointClouds.cpp  # Point cloud processing implementation
│   ├── processPointClouds.h    # Processing header
│   ├── quiz/                   # Algorithm implementations
│   │   ├── cluster/            # Clustering algorithms
│   │   │   ├── cluster.cpp     # Euclidean clustering
│   │   │   ├── cluster.h
│   │   │   ├── CMakeLists.txt
│   │   │   └── kdtree.h        # KD-tree implementation
│   │   └── ransac/             # RANSAC algorithms
│   │       ├── CMakeLists.txt
│   │       ├── ransac2d.cpp    # 2D RANSAC (learning)
│   │       └── ransac2d.h
│   ├── render/                 # Visualization tools
│   │   ├── box.h               # Bounding box rendering
│   │   ├── render.cpp
│   │   └── render.h
│   └── sensors/                # Sensor data
│       ├── lidar.h             # Lidar simulation
│       └── data/
│           └── pcd/            # Point cloud dataset
```

## Technical Implementation

### 1. Voxel Grid Filtering

I implemented downsampling using PCL's voxel grid filter to reduce the point cloud density while maintaining structural integrity. This significantly improves processing speed without losing important details.

**Key Parameters**:
- Filter resolution: Controls voxel size for downsampling
- Trade-off: Resolution vs. processing speed

### 2. Region of Interest (ROI) Filtering  

I applied crop box filtering to:
- Focus on the ego lane and immediate surroundings
- Remove far-away points that aren't immediately relevant
- Exclude roof points from the ego vehicle

### 3. RANSAC 3D Plane Segmentation

I implemented a custom 3D RANSAC algorithm to segment the road plane:

**Algorithm**:
1. Randomly sample 3 points from the point cloud
2. Fit a plane equation: `Ax + By + Cz + D = 0`
3. Count inliers within distance threshold
4. Iterate and keep the best plane
5. Separate inliers (road) from outliers (obstacles)

**Why Custom Implementation?** Building RANSAC from scratch provided deeper understanding of robust fitting and outlier rejection.

### 4. KD-Tree for Spatial Search

I built a 3D KD-tree data structure from scratch for efficient nearest-neighbor searches:

**Features**:
- Recursive tree construction
- Balanced partitioning along x, y, z axes
- Efficient radius search for clustering
- O(log n) average search time

**Structure**:
```cpp
struct Node {
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;
};
```

### 5. Euclidean Clustering

I implemented a clustering algorithm using the KD-tree for grouping obstacle points:

**Algorithm**:
1. Use KD-tree for efficient proximity search
2. Perform region-growing from seed points
3. Group nearby points into clusters
4. Filter clusters by size (remove noise and very large objects)
5. Generate bounding boxes for each cluster

**Parameters**:
- Cluster tolerance: Maximum distance between points in a cluster
- Min/Max cluster size: Filter out noise and unrealistic detections

## Results

The system successfully:
- Processes Lidar point clouds in real-time
- Accurately segments the road surface using RANSAC
- Detects and clusters obstacles (vehicles, pedestrians, cyclists)
- Generates bounding boxes for detected objects
- Handles varying point cloud densities and noise levels

## Skills Demonstrated

- **3D Computer Vision**: Point cloud processing and analysis
- **C++ Programming**: Template programming and STL usage
- **Data Structures**: KD-tree implementation from scratch
- **Algorithms**: RANSAC, clustering, spatial partitioning
- **PCL (Point Cloud Library)**: Advanced library usage
- **Computational Geometry**: Plane fitting, distance calculations
- **Real-time Processing**: Performance optimization for autonomous systems

## Installation and Build

### Ubuntu

```bash
sudo apt install libpcl-dev
cd ~
git clone https://github.com/amr-f-ramadan/lidar-obstacle-detector.git
cd lidar-obstacle-detector
mkdir build && cd build
cmake ..
make
./environment
```

### Windows

Install PCL from: http://www.pointclouds.org/downloads/windows.html

### macOS

#### Install via Homebrew
```bash
brew update
brew tap brewsci/science
brew install pcl
```

Then build:
```bash
mkdir build && cd build
cmake ..
make
./environment
```

## Dependencies

- **CMake** >= 3.5
- **PCL** (Point Cloud Library) >= 1.2
- **C++ Compiler** with C++14 support

## Acknowledgments

This project was completed as part of Udacity's Sensor Fusion Engineer Nanodegree program.
