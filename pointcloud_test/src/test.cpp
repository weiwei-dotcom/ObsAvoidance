// #include <iostream>
// #include <opencv2/opencv.hpp>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/common/transforms.h>

// int main()
// {
//     // Load the point cloud data
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile<pcl::PointXYZ>("input_cloud.pcd", *cloud);

//     // Create the segmentation object
//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

//     // Set the segmentation parameters
//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setMaxIterations(100);
//     seg.setDistanceThreshold(0.01);

//     // Segment the largest planar component from the input cloud
//     seg.setInputCloud(cloud);
//     seg.segment(*inliers, *coefficients);

//     // Extract the inliers
//     pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud(cloud);
//     extract.setIndices(inliers);
//     extract.setNegative(false);
//     extract.filter(*plane);

//     // Visualize the plane
//     pcl::visualization::PCLVisualizer viewer("Plane Viewer");
//     viewer.setBackgroundColor(0.0, 0.0, 0.0);
//     viewer.addPointCloud<pcl::PointXYZ>(plane, "plane_cloud");
//     viewer.spin();

//     // Load the point cloud data
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile<pcl::PointXYZ>("input_cloud.pcd", *cloud);

//     // Read the image
//     cv::Mat image = cv::imread("input_image.jpg", cv::IMREAD_GRAYSCALE);

//     // Define the irregular ROI contour as a set of cv::Point2f coordinates
//     std::vector<cv::Point2f> roiContour;
//     roiContour.push_back(cv::Point2f(100, 100));
//     roiContour.push_back(cv::Point2f(200, 150));
//     roiContour.push_back(cv::Point2f(150, 250));
//     roiContour.push_back(cv::Point2f(50, 200));

//     // Create a binary mask of the ROI within the image
//     cv::Mat roiMask = cv::Mat::zeros(image.size(), CV_8UC1);
//     std::vector<cv::Point> roiContourInt(roiContour.size());
//     for (size_t i = 0; i < roiContour.size(); i++)
//     {
//         roiContourInt[i] = cv::Point(roiContour[i].x, roiContour[i].y);
//     }
//     cv::fillConvexPoly(roiMask, roiContourInt.data(), roiContourInt.size(), cv::Scalar(255));

//     // Obtain the camera intrinsic parameters
//     cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
//     cv::Mat distortionCoefficients = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

//     // Project pixels from the irregular ROI onto the fitted plane
//     pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//     for (int y = 0; y < image.rows; y++)
//     {
//         for (int x = 0; x < image.cols; x++)
//         {
//             // Check if the pixel is within the ROI
//             if (roiMask.at<uchar>(y, x) == 255)
//             {
//                 // Compute the 3D point in the camera coordinate system
//                 double depth = cloud->at(x, y).z;
//                 cv::Point2d uv(x, y);
//                 cv::Point3d point3D = cv::Point3d((uv.x - cx) * depth / fx, (uv.y - cy) * depth / fy, depth);

//                 // Transform the 3D point to the fitted plane's coordinate system
//                 pcl::PointXYZ transformedPoint;
//                 transformedPoint.x = point3D.x * coefficients->values[0] +
//                                      point3D.y * coefficients->values[1] +
//                                      point3D.z * coefficients->values[2] +
//                                      coefficients->values[3];
//                 transformedPoint.y = point3D.y * coefficients->values[4] +
//                                      point3D.y * coefficients->values[5] +
//                                      point3D.z * coefficients->values[6] +
//                                      coefficients->values[7];
//                 transformedPoint.z = point3D.z * coefficients->values[8] +
//                                      point3D.y * coefficients->values[9] +
//                                      point3D.z * coefficients->values[10] +
//                                      coefficients->values[11];

//                 // Add the transformed point to the projected point cloud
//                 projectedCloud->push_back(transformedPoint);
//             }
//         }
//     }

//     return 0;
// }