#include "Transformation.h"

//원점 기준으로 회전
std::vector<glm::vec3> rotate_3D_points
(cv::Mat& euler_angle, std::vector<glm::vec3>& inputVec3)
{
	std::vector<glm::vec3> tempVec3_set;
	cv::Mat R_mat_X;
	cv::Mat R_mat_Y;
	cv::Mat R_mat_Z;
	double theta_X = euler_angle.at<double>(0) * 3.14 / 180.0;
	double theta_Y = euler_angle.at<double>(1) * 3.14 / 180.0;
	double theta_Z = euler_angle.at<double>(2) * 3.14 / 180.0;
	double R_mat_element_X[9] = { 1,0,0,0,cos(theta_X),-sin(theta_X),0,sin(theta_X),cos(theta_X) };
	double R_mat_element_Y[9] = { cos(theta_Y),0,sin(theta_Y),0,1,0,-sin(theta_Y),0,cos(theta_Y) };
	double R_mat_element_Z[9] = { cos(theta_Z),-sin(theta_Z),0,sin(theta_Z),cos(theta_Z),0,0,0,1 };
	R_mat_X = cv::Mat(3, 3, CV_64FC1, R_mat_element_X);
	R_mat_Y = cv::Mat(3, 3, CV_64FC1, R_mat_element_Y);
	R_mat_Z = cv::Mat(3, 3, CV_64FC1, R_mat_element_Z);

	cv::Mat tempVec3;
	glm::vec3 outVec3;
	for (unsigned int i = 0; i < inputVec3.size(); i++)
	{
		double tempPoint_element[3] = { inputVec3[i].x, inputVec3[i].y ,inputVec3[i].z };
		tempVec3 = cv::Mat(3, 1, CV_64FC1, tempPoint_element);
		tempVec3 = R_mat_X * tempVec3;
		tempVec3 = R_mat_Y * tempVec3;
		tempVec3 = R_mat_Z * tempVec3;
		outVec3.x = tempVec3.at<double>(0);
		outVec3.y = tempVec3.at<double>(1);
		outVec3.z = tempVec3.at<double>(2);
		tempVec3_set.push_back(outVec3);
	}
	return tempVec3_set;
}

std::vector<glm::vec3> scale_3D_points
(double scale_ratio, std::vector<glm::vec3>& inputVec3)
{
	std::vector<glm::vec3> tempVec3_set;
	glm::vec3 outVec3;
	for (unsigned int i = 0; i < inputVec3.size(); i++)
	{
		outVec3.x = inputVec3[i].x * scale_ratio;
		outVec3.y = inputVec3[i].y * scale_ratio;
		outVec3.z = inputVec3[i].z * scale_ratio;
		tempVec3_set.push_back(outVec3);
	}
	return tempVec3_set;
}
std::vector<glm::vec3> translate_3D_points
(double x, double y, double z, std::vector<glm::vec3>& inputVec3)
{
	std::vector<glm::vec3> tempVec3_set;
	glm::vec3 outVec3;
	for (unsigned int i = 0; i < inputVec3.size(); i++)
	{
		outVec3.x = inputVec3[i].x + x;
		outVec3.y = inputVec3[i].y + y;
		outVec3.z = inputVec3[i].z + z;
		tempVec3_set.push_back(outVec3);
	}
	return tempVec3_set;
}