#pragma once
#include <iostream>
#include <vector>
#include <opencv2\opencv.hpp>
#include <glm\glm\vec3.hpp>
#include <glm\glm\vec2.hpp>
#include <glm\glm\glm.hpp>



std::vector<glm::vec3> rotate_3D_points
(cv::Mat& euler_angle, std::vector<glm::vec3>& inputVec3);


std::vector<glm::vec3> scale_3D_points
(double scale_ratio, std::vector<glm::vec3>& inputVec3);

std::vector<glm::vec3> translate_3D_points
(double x, double y, double z, std::vector<glm::vec3>& inputVec3);