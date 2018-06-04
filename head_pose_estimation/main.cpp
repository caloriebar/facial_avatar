#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <opencv2\opencv.hpp>
#include <GLFW/glfw3.h> 
#include <glm\glm\vec3.hpp>
#include <glm\glm\vec2.hpp>
#include <glm\glm\glm.hpp>
#include <vector>
#include <iostream>

#include "Object.h"
#include "Transformation.h"

#define SKIP_FRAMES 2
using namespace dlib;

enum Matching_method {
	FORMAL,
	CUSTOM
};
enum Charactor_num{
	STANDARD,
};

enum Information {
	TIME,
	ORDERD_NUM,
	DIFF,
	OFF
};



Matching_method matching_m = FORMAL;
Charactor_num charactor_m = STANDARD;
Information infor_m = OFF;


int landmark_num[68] =
{
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0, //0-16
	5540,5544,5548,5552,5536, //17 -21
	1377,1393,1389,1385,1381, //22-26
	2161,3492,621,810, //27-30
	4561,4569,601,346,338, //31-35
	5449,5457,5461,5421,5433,5437, // 36- 41
	1258,1298,1294,1286,1274,1270, // 42-47
	4319,4335,4420,215,167,60,44, //48-54
	56,229,207,4460,4331, // 55-59
	4400,4434,225,181,143, // 60-64
	247,159,4478 //65- 67
};



void ErrorCallback(int error, const char* description)
{
	std::cout << description << std::endl;
}
void InitApp() {
	// 클리어 색상(배경색) 지정
	glClearColor(1, 1, 1, 1);

	//깊이 테스트 사용
	glEnable(GL_DEPTH_TEST);
}
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void WindowSizeChangeCallback(GLFWwindow* window, int width, int height);
void get_euler_angle(std::vector<cv::Point3d>& object_points, std::vector<cv::Point2d>& image_points,
	cv::Mat& cam_matrix, cv::Mat& dist_coeffs, cv::Mat& out_euler_angle);
void matching_object_to_face(std::vector<glm::vec3>& object_points, std::vector<cv::Point2d>& image_points, double& out_err);
void matching_object_to_face2(std::vector<glm::vec3>& object_points, std::vector<cv::Point2d>& image_points, double& out_err);
cv::Mat get_camera_matrix(float focal_length, cv::Point2d center)
{
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	return camera_matrix;
}
void interpolation(character &temp, character target, int num);

double image_cols, image_rows;


std::vector<cv::Point2d> ref_image_pts; //방향 추정에 사용되는 2d face의 몇개 점
std::vector<cv::Point3d> ref_object_pts; //방향 추정에 사용되는 3d model의 몇개 점


full_object_detection shape;

cv::Mat src, temp, temp2;
cv::Mat rotation_vec;                           //3 x 1
cv::Mat rotation_mat;                           //3 x 3 R
cv::Mat translation_vec;                        //3 x 1 T
cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);
cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

std::vector<character> standard;
std::vector<cv::Point2d> image_pts_68; //비교할 2d face 68개 점, 출력에도 사용

std::vector< std::vector<glm::vec3> > ref_object_pts_68; //비교할 3d model 68개 점, 19개 모델
std::vector< std::vector<glm::vec3> > object_pts_68; //ref_object_pts_68을 회전 및 스케일 하기 위한 임시메모리


double noserate = 0.0;

bool first = true;

int main()
{
	double headPose_before[3] = { 0,0,0 };
	double headPose_after[3] = { 0,0,0 };

	int checkCount = 0;

	double fps = 30.0;
	int count = 0;;

	int numOfExpression = 19;
	FILE* fp = fopen("standard_man/list.txt", "r");
	character cha;
	char filename[50];

	for (int i = 0; i < numOfExpression; i++) {
		fscanf(fp, "%s", filename);
		loadOBJ(filename, cha);
		standard.push_back(cha);
		cha.clear();
	}

	fclose(fp);

	lead_ref_68point("ref_obj_ptr_68_2.txt", ref_object_pts_68);


	/*for (int i = 0; i < objects.size(); i++)
	{
	objects[i].print_obj_info();
	}*/

	// vector 할당
	for (unsigned int i = 0; i < numOfExpression; i++)
	{
		std::vector<glm::vec3> alloc_obj;
		object_pts_68.push_back(alloc_obj);
	}

	//open cam
	cv::VideoCapture cap(0);
	if (!cap.isOpened())
	{
		std::cerr << "Unable to connect to camera" << std::endl;
		return EXIT_FAILURE;
	}
	//Load face detection and pose estimation models (dlib).
	frontal_face_detector detector = get_frontal_face_detector();
	shape_predictor predictor;
	deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;


	std::vector<rectangle> faces_dlib;

	//------------------------------

	ref_object_pts.push_back(cv::Point3d(11.362504, 10.029518, 16.806782));     //#17 left brow left corner
	ref_object_pts.push_back(cv::Point3d(2.148770, 10.294258, 20.354479));     //#21 left brow right corner
	ref_object_pts.push_back(cv::Point3d(-2.059052, 10.125916, 20.436745));    //#22 right brow left corner
	ref_object_pts.push_back(cv::Point3d(-11.466078, 10.092777, 16.708063));    //#26 right brow right corner

	ref_object_pts.push_back(cv::Point3d(9.391946, 5.651413, 14.472001));     //#36 left eye left corner
	ref_object_pts.push_back(cv::Point3d(3.931712, 5.236343, 16.249056));     //#39 left eye right corner
	ref_object_pts.push_back(cv::Point3d(-3.928426, 5.249275, 16.249159));    //#42 right eye left corner
	ref_object_pts.push_back(cv::Point3d(-9.429914, 5.649490, 14.520754));    //#45 right eye right corner

	ref_object_pts.push_back(cv::Point3d(3.382630, -4.237745, 19.905993));     //#31 nose left corner
	ref_object_pts.push_back(cv::Point3d(-3.382631, -4.240194, 19.905748));    //#35 nose right corner
																			   
	ref_object_pts.push_back(cv::Point3d(-13.550348, -5.672138, 5.022416));    //#14 
	ref_object_pts.push_back(cv::Point3d(13.550348, -5.672138, 5.022416));    //#2 
	ref_object_pts.push_back(cv::Point3d(-14.937097, 0.861816, 6.108888));    //#15 
	ref_object_pts.push_back(cv::Point3d(14.937097, 0.861816, 6.108888));    //#1 
	ref_object_pts.push_back(cv::Point3d(-14.762725, 6.650970, 6.197021));    //#16
	ref_object_pts.push_back(cv::Point3d(14.762725, 6.650970, 6.197021));    //#0

	ref_object_pts.push_back(cv::Point3d(5.657823, -9.568151, 17.967920));    //#48
	ref_object_pts.push_back(cv::Point3d(-5.633242, -9.524076, 17.906942));    //#54 
	ref_object_pts.push_back(cv::Point3d(0.000000, -10.216482, 20.936403));    //#57

									
	std::vector<cv::Point2d> reprojectdst;
	reprojectdst.resize(8);


	std::ostringstream outtext;


	double t = (double)cv::getTickCount();

	GLFWwindow* window;

	glfwSetErrorCallback(ErrorCallback);

	if (!glfwInit()) {
		exit(EXIT_FAILURE);
	}

	window = glfwCreateWindow(1000, 1000, "Model window", NULL, NULL);
	if (!window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	glfwSetKeyCallback(window, KeyCallback);
	glfwSetWindowSizeCallback(window, WindowSizeChangeCallback);
	InitApp();

	character cha_before;
	character cha_after = standard[0];
	std::vector<character> cha_current;

	GLuint* text2D_standard;
	text2D_standard = new GLuint[standard[0].texName.size()];
	for (int i = 0; i < standard[0].texName.size(); i++) {
		text2D_standard[i] = LoadTexture(standard[0].texName[i]);
	}



	while (!glfwWindowShouldClose(window))
	{

		switch(charactor_m)
		{
		case STANDARD:
			cha_current = standard;
			if (first) {
				cha_after = standard[18];
				first = false;
			}
			break;
		}
		//에러값이 최소인 순서로 정렬된 object의 넘버를 순서대로 저장
		int ordered_o_num[19];


		if (count == 0)
			t = cv::getTickCount();
		// Grab a frame
		cap >> src;
		cv::flip(src, temp, 1);


		//cv::resize(temp, temp, cv::Size(), 0.7, 0.7);

		image_cols = temp.cols;
		image_rows = temp.rows;

		//printf("%lf %lf\n", image_cols, image_rows);


		temp2 = cv::Mat(temp.rows, temp.cols, CV_8UC3, cv::Scalar(0, 0, 0));

		cv::Size size = temp.size();


		//
		cv_image<bgr_pixel> cimg(temp);

		// Detect faces 
		if (count % SKIP_FRAMES == 0)
		{
			faces_dlib = detector(cimg);
		}



		// Find the pose of each face
		if (faces_dlib.size() > 0)
		{
			shape = predictor(cimg, faces_dlib[0]);
			for (unsigned int i = 0; i < 68; ++i)
			{
				cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 255, 0), 0);
			}

			ref_image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
			ref_image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner

			ref_image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
			ref_image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner

			ref_image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
	
			ref_image_pts.push_back(cv::Point2d(shape.part(14).x(), shape.part(14).y())); //#36 left eye left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(2).x(), shape.part(2).y())); //#39 left eye right corner
			ref_image_pts.push_back(cv::Point2d(shape.part(15).x(), shape.part(15).y())); //#42 right eye left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(1).x(), shape.part(1).y())); //#45 right eye right corner
			ref_image_pts.push_back(cv::Point2d(shape.part(16).x(), shape.part(16).y())); //#31 nose left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(0).x(), shape.part(0).y())); //#35 nose right corner

			ref_image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
			ref_image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
			ref_image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner

																						  //fill in cam intrinsics and distortion coefficients
			double focal_length = temp.cols;
			cv::Mat cam_matrix = get_camera_matrix(focal_length, cv::Point2d(temp.cols / 2, temp.rows / 2));
			cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
			get_euler_angle(ref_object_pts, ref_image_pts, cam_matrix, dist_coeffs, euler_angle);


			if (checkCount < 2)
			{
				checkCount++;
			}


			headPose_before[0] = headPose_after[0];
			headPose_before[1] = headPose_after[1];
			headPose_before[2] = headPose_after[2];

			double angle_dif = 5.0;

			if (abs(headPose_before[0] - euler_angle.at<double>(0)) > angle_dif)
			{
				headPose_after[0] = euler_angle.at<double>(0);
			}
			else
				headPose_after[0] = (headPose_after[0] * 0.7 + euler_angle.at<double>(0)*0.3);

			if (abs(headPose_before[1] - euler_angle.at<double>(1)) > angle_dif)
			{
				headPose_after[1] = euler_angle.at<double>(1);
			}
			else
				headPose_after[1] = (headPose_after[1] * 0.7 + euler_angle.at<double>(1)*0.3);

			if (abs(headPose_before[2] - euler_angle.at<double>(2)) > angle_dif)
			{
				headPose_after[2] = euler_angle.at<double>(2);
			}
			else
				headPose_after[2] = (headPose_after[2] * 0.7 + euler_angle.at<double>(2)*0.3);


			ref_image_pts.clear();


			for (unsigned int i = 0; i < 68; i++)
			{
				image_pts_68.push_back(cv::Point2d(shape.part(i).x(), shape.part(i).y()));
			}

			for (unsigned int i = 0; i < 19; i++)
			{
				object_pts_68[i] = ref_object_pts_68[i];
			}

			for (unsigned int i = 0; i < 19; i++)
			{
				object_pts_68[i] = rotate_3D_points(euler_angle, object_pts_68[i]);
				object_pts_68[i] = scale_3D_points(-5.0f, object_pts_68[i]);

			}

			noserate = (35 + 35 * sin(euler_angle.at<double>(0)*3.14 / 180.0)) /
				sqrt((shape.part(27).x() - shape.part(30).x())*(shape.part(27).x() - shape.part(30).x()) +
				(shape.part(27).y() - shape.part(30).y())*(shape.part(27).y() - shape.part(30).y()));

			double difference[19];

			for (unsigned int i = 0; i < 19; i++)
			{
				if (matching_m == FORMAL)
				{
					object_pts_68[i] = 
						translate_3D_points(image_cols / 2, image_rows / 2, 0, object_pts_68[i]);
					matching_object_to_face(object_pts_68[i], image_pts_68, difference[i]);
				}
				else if (matching_m == CUSTOM)
				{
					object_pts_68[i] = translate_3D_points(320 - object_pts_68[i][27].x, 144 - object_pts_68[i][27].y, 0, object_pts_68[i]);
					matching_object_to_face2(object_pts_68[i], image_pts_68, difference[i]);
				}
			}


			double sorted_difference[19]; //차이값 정렬한 배열
			for (unsigned int i = 0; i < 19; i++)
			{
				sorted_difference[i] = difference[i];
			}

			// selection sort
			for (unsigned int i = 0; i < 19; i++)
			{
				for (unsigned int j = i + 1; j < 19; j++)
				{
					if (sorted_difference[i] > sorted_difference[j])
						swap(sorted_difference[i], sorted_difference[j]);
				}
			}

			//정렬된 배열을 통해 object num도 순서대로 저장
			for (unsigned int i = 0; i < 19; i++)
			{
				for (unsigned int j = 0; j < 19; j++)
				{
					if (sorted_difference[i] == difference[j])
					{
						ordered_o_num[i] = j;
					}
				}
			}

			switch (infor_m)
			{
			case TIME:
				std::cout << "Time:";
				std::cout << glfwGetTime() << std::endl;
				std::cout << std::endl;

				break;
			case ORDERD_NUM:
				std::cout << "Ordered Num:";
				for (unsigned int i = 0; i < 19; i++)
					std::cout << ordered_o_num[i] << ",";
				std::cout << std::endl;
				std::cout << std::endl;
				break;
			case DIFF:
				std::cout << "Difference:";
				for (unsigned int i = 0; i < 19; i++)
					std::cout << difference[i] << ",";
				std::cout << std::endl;
				std::cout << std::endl;
				break;
			case OFF:
				break;
			}

			


			cha_before = cha_after;
			//-----
			//에러값에 따라 가중치를 부여하여 만든 새로운 모델을 obj_after 에 넣는다.
			for (int i = 0; i < cha_after.objs.size(); i++) {
				cha_after.objs[i].vertices.clear();
				cha_after.objs[i].normals.clear();
			}


			int n = 3;

			for (unsigned int i = 0; i < 19; i++)
			{
				difference[i] = pow(difference[i], 3);
			}


			double sum_dif = 0;
			for (unsigned int i = 0; i < n; i++)
				sum_dif += 1.0 / difference[ordered_o_num[i]];

			for (unsigned int k = 0; k < cha_current[0].objs.size(); k++) {
				for (unsigned int i = 0; i < cha_current[0].objs[k].vertices.size(); i++)
				{
					glm::vec3 tempV(0.0, 0.0, 0.0);
					for (unsigned int j = 0; j < n; j++)
					{
						tempV.x += ((1.0 / difference[ordered_o_num[j]]) / sum_dif) * cha_current[ordered_o_num[j]].objs[k].vertices[i].x;
						tempV.y += ((1.0 / difference[ordered_o_num[j]]) / sum_dif) * cha_current[ordered_o_num[j]].objs[k].vertices[i].y;
						tempV.z += ((1.0 / difference[ordered_o_num[j]]) / sum_dif) * cha_current[ordered_o_num[j]].objs[k].vertices[i].z;
					}
					cha_after.objs[k].vertices.push_back(tempV);
				}
				for (unsigned int i = 0; i < cha_current[0].objs[k].normals.size(); i++)
				{
					glm::vec3 tempV(0, 0, 0);
					for (unsigned int j = 0; j < n; j++)
					{
						tempV.x += ((1.0 / difference[ordered_o_num[j]]) / sum_dif) * cha_current[ordered_o_num[j]].objs[k].normals[i].x;
						tempV.y += ((1.0 / difference[ordered_o_num[j]]) / sum_dif) * cha_current[ordered_o_num[j]].objs[k].normals[i].y;
						tempV.z += ((1.0 / difference[ordered_o_num[j]]) / sum_dif) * cha_current[ordered_o_num[j]].objs[k].normals[i].z;
					}
					cha_after.objs[k].normals.push_back(tempV);
				}
			}



			for (unsigned int i = 17; i < 68; ++i)
			{
				cv::circle(temp2, image_pts_68[i], 1, cv::Scalar(0, 255, 0), 1);
			}
			for (unsigned int i = 17; i < 68; ++i)
			{
				cv::circle(temp2, cv::Point2d(object_pts_68[ordered_o_num[0]][i].x, object_pts_68[ordered_o_num[0]][i].y), 1, cv::Scalar(0, 100, 255), 1);
			}


			image_pts_68.clear();


			for (unsigned int i = 0; i <19; i++)
			{
				object_pts_68[i].clear();
			}

		}

		outtext << "USER_face_landmark";
		cv::putText(temp2, outtext.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
		outtext.str("");
		outtext << "MODEL_face_landmark";
		cv::putText(temp2, outtext.str(), cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 100, 255));
		outtext.str("");
		

		imshow("User window", temp);
		imshow("Matching window", temp2);

		// WaitKey slows down the runtime quite a lot
		// So check every 15 frames
		if (count % 15 == 0)
		{
			int k = cv::waitKey(1);
			// esc pressed
			if (k == 27)
			{
				return 0;
			}
		}
		count++;
		if (count == 100)
		{
			t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
			fps = 100.0 / t;
			count = 0;
		}

		double headPose_X = 0;
		double headPose_Y = 0;
		double headPose_Z = 0;

		int step = 5;


		if (checkCount == 2 && (faces_dlib.size() > 0) && first == false )
		{
			for (unsigned int i1 = 0; i1 < step; i1++)
			{
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();

				//glEnable(GL_TEXTURE_2D);
				glEnable(GL_LIGHTING);
				glEnable(GL_LIGHT0);
				glEnable(GL_LIGHT1);


				headPose_X = ((double)(step - i1)) / (double)step * headPose_before[0] + ((double)i1 / (double)step) * headPose_after[0];
				headPose_Y = ((double)(step - i1)) / (double)step * headPose_before[1] + ((double)i1 / (double)step) * headPose_after[1];
				headPose_Z = ((double)(step - i1)) / (double)step * headPose_before[2] + ((double)i1 / (double)step) * headPose_after[2];

				float translate_dis;


				switch (charactor_m)
				{
				case STANDARD:
					translate_dis = 0.5f;
					break;
				}

				float model_scale = 0.1f;

				glRotatef(180.f, 0.f, 1.f, 0.f);//angle: 각도'

				GLfloat position0[] = { 5, 1, 5, 1.0 };
				GLfloat position1[] = { -5, 1, 5, 1.0 };
				GLfloat AmbientLightValue[] = { 0.3f, 0.3f, 0.3f, 1.0f };
				GLfloat DiffuseLightValue[] = { 0.7f, 0.7f, 0.7f, 1.0f };
				GLfloat SpecularLightValue[] = { 1.0f, 1.0f, 1.0f, 1.0f };
				GLfloat PositionLightValue[] = { 0.0f, 0.0f, 1.0f, 0.0f };
				glLightfv(GL_LIGHT0, GL_POSITION, position0);
				glLightfv(GL_LIGHT1, GL_POSITION, position1);
				glLightfv(GL_LIGHT1, GL_AMBIENT, AmbientLightValue);
				glLightfv(GL_LIGHT1, GL_DIFFUSE, DiffuseLightValue);
				glLightfv(GL_LIGHT1, GL_SPECULAR, SpecularLightValue);

				glTranslatef(0.f, -translate_dis, 0.f);
				glRotatef(float(headPose_X), 1.f, 0.f, 0.f); //angle: 각도'
				glRotatef(float(headPose_Y), 0.f, 1.f, 0.f); //angle: 각도'
				glRotatef(float(headPose_Z), 0.f, 0.f, 1.f); //angle: 각도'
				glTranslatef(0.f, translate_dis, 0.f);


				switch (charactor_m)
				{
				case STANDARD:
					model_scale = 0.03f;
					break;
				}



				interpolation(cha_before, cha_after, i1);

				switch (charactor_m)
				{
				case STANDARD:
					drawCharacter(cha_before, text2D_standard, model_scale);
					break;
				}

				
				glDisable(GL_LIGHT1);
				glDisable(GL_LIGHT0);
				glDisable(GL_LIGHTING);

				//렌더 버퍼 교체. (그린 결과를 디스플레이하는 명령)
				glfwSwapBuffers(window);
			}
			//윈도우 이벤트(키 스크로크 등) 폴링
			glfwPollEvents();
		}
	}

	//윈도우 제거
	//GLFW 종료
	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);

	return 0;
}
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
	else if (key == GLFW_KEY_1 && action == GLFW_PRESS)
	{
		matching_m = FORMAL;
	}
	else if (key == GLFW_KEY_2 && action == GLFW_PRESS)
	{
		matching_m = CUSTOM;
	}
	else if (key == GLFW_KEY_A && action == GLFW_PRESS)
	{
		charactor_m = STANDARD;
		first = true;
	}
	else if (key == GLFW_KEY_0)
	{
		infor_m = OFF;
	}
	else if (key == GLFW_KEY_I)
	{
		infor_m = TIME;
	}
	else if (key == GLFW_KEY_O)
	{
		infor_m = ORDERD_NUM;
	}
	else if (key == GLFW_KEY_P)
	{
		infor_m = DIFF;
	}
}
void WindowSizeChangeCallback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glOrtho - 원근감이 없는 직교투영
	//gluPerspective - 원근 투영.
	if (width <= height)
		glFrustum(-1.5, 1.5, -1.5*(float)height / (float)width, 1.5*(float)height / (float)width, -1000.0, 1000.0);
	else
		glFrustum(-1.5*(float)width / (float)height, 1.5*(float)width / (float)height, -1.5, 1.5, -1000.0, 1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void get_euler_angle(std::vector<cv::Point3d>& object_points, std::vector<cv::Point2d>& image_points,
	cv::Mat& cam_matrix, cv::Mat& dist_coeffs, cv::Mat& out_euler_angle)
{
	cv::solvePnP(object_points, image_points, cam_matrix, dist_coeffs, rotation_vec, translation_vec);
	cv::Rodrigues(rotation_vec, rotation_mat);
	cv::hconcat(rotation_mat, translation_vec, pose_mat);
	cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), out_euler_angle);
}



void matching_object_to_face(std::vector<glm::vec3>& object_points, std::vector<cv::Point2d>& image_points, double& out_err)
{
	double f_diag_distance[6]; //face 점의 눈썹, 눈, 코, 입의 boundary box의 대각선 길이의 절반
	double o_diag_distance[6]; //object 점의 눈썹, 눈, 코, 입의 boundary box의 대각선 길이의 절반

	double f_part_minX[6] = { 0 }; double f_part_minY[6] = { 0 };
	double f_part_maxX[6] = { 0 }; double f_part_maxY[6] = { 0 };

	double o_part_minX[6] = { 0 }; double o_part_minY[6] = { 0 };
	double o_part_maxX[6] = { 0 }; double o_part_maxY[6] = { 0 };

	double left_brow_rad, right_brow_rad, left_eye_rad, right_eye_rad, nose_rad, mouse_rad;



	left_brow_rad = image_cols / 20;
	right_brow_rad = image_cols / 20;
	left_eye_rad = image_cols / 25;
	right_eye_rad = image_cols / 25;
	nose_rad = image_cols / 15;
	mouse_rad = image_cols / 15;


	for (unsigned int i = 0; i < 6; i++)
	{
		f_part_minX[i] = image_cols; f_part_minY[i] = image_rows;
		f_part_maxX[i] = 0;	f_part_maxY[i] = 0;

		o_part_minX[i] = image_cols; o_part_minY[i] = image_rows;
		o_part_maxX[i] = 0;	o_part_maxY[i] = 0;
	}

	for (unsigned int i = 17; i < 68; i++)
	{

		//left brows
		if (i >= 17 && i <= 21)
		{
			if (image_points[i].x >= f_part_maxX[0]) { f_part_maxX[0] = image_points[i].x; }
			if (image_points[i].y >= f_part_maxY[0]) { f_part_maxY[0] = image_points[i].y; }
			if (image_points[i].x <= f_part_minX[0]) { f_part_minX[0] = image_points[i].x; }
			if (image_points[i].y <= f_part_minY[0]) { f_part_minY[0] = image_points[i].y; }

			if (object_points[i].x >= o_part_maxX[0]) { o_part_maxX[0] = object_points[i].x; }
			if (object_points[i].y >= o_part_maxY[0]) { o_part_maxY[0] = object_points[i].y; }
			if (object_points[i].x <= o_part_minX[0]) { o_part_minX[0] = object_points[i].x; }
			if (object_points[i].y <= o_part_minY[0]) { o_part_minY[0] = object_points[i].y; }
		}
		//right brows
		else if (i >= 22 && i <= 26)
		{
			if (image_points[i].x >= f_part_maxX[1]) { f_part_maxX[1] = image_points[i].x; }
			if (image_points[i].y >= f_part_maxY[1]) { f_part_maxY[1] = image_points[i].y; }
			if (image_points[i].x <= f_part_minX[1]) { f_part_minX[1] = image_points[i].x; }
			if (image_points[i].y <= f_part_minY[1]) { f_part_minY[1] = image_points[i].y; }

			if (object_points[i].x >= o_part_maxX[1]) { o_part_maxX[1] = object_points[i].x; }
			if (object_points[i].y >= o_part_maxY[1]) { o_part_maxY[1] = object_points[i].y; }
			if (object_points[i].x <= o_part_minX[1]) { o_part_minX[1] = object_points[i].x; }
			if (object_points[i].y <= o_part_minY[1]) { o_part_minY[1] = object_points[i].y; }
		}
		// nose
		else if (i >= 27 && i <= 35)
		{
			if (image_points[i].x >= f_part_maxX[2]) { f_part_maxX[2] = image_points[i].x; }
			if (image_points[i].y >= f_part_maxY[2]) { f_part_maxY[2] = image_points[i].y; }
			if (image_points[i].x <= f_part_minX[2]) { f_part_minX[2] = image_points[i].x; }
			if (image_points[i].y <= f_part_minY[2]) { f_part_minY[2] = image_points[i].y; }

			if (object_points[i].x >= o_part_maxX[2]) { o_part_maxX[2] = object_points[i].x; }
			if (object_points[i].y >= o_part_maxY[2]) { o_part_maxY[2] = object_points[i].y; }
			if (object_points[i].x <= o_part_minX[2]) { o_part_minX[2] = object_points[i].x; }
			if (object_points[i].y <= o_part_minY[2]) { o_part_minY[2] = object_points[i].y; }
		}
		//left eye
		else if (i >= 36 && i <= 41)
		{
			if (image_points[i].x >= f_part_maxX[3]) { f_part_maxX[3] = image_points[i].x; }
			if (image_points[i].y >= f_part_maxY[3]) { f_part_maxY[3] = image_points[i].y; }
			if (image_points[i].x <= f_part_minX[3]) { f_part_minX[3] = image_points[i].x; }
			if (image_points[i].y <= f_part_minY[3]) { f_part_minY[3] = image_points[i].y; }

			if (object_points[i].x >= o_part_maxX[3]) { o_part_maxX[3] = object_points[i].x; }
			if (object_points[i].y >= o_part_maxY[3]) { o_part_maxY[3] = object_points[i].y; }
			if (object_points[i].x <= o_part_minX[3]) { o_part_minX[3] = object_points[i].x; }
			if (object_points[i].y <= o_part_minY[3]) { o_part_minY[3] = object_points[i].y; }
		}
		//right eye
		else if (i >= 42 && i <= 47)
		{
			if (image_points[i].x >= f_part_maxX[4]) { f_part_maxX[4] = image_points[i].x; }
			if (image_points[i].y >= f_part_maxY[4]) { f_part_maxY[4] = image_points[i].y; }
			if (image_points[i].x <= f_part_minX[4]) { f_part_minX[4] = image_points[i].x; }
			if (image_points[i].y <= f_part_minY[4]) { f_part_minY[4] = image_points[i].y; }

			if (object_points[i].x >= o_part_maxX[4]) { o_part_maxX[4] = object_points[i].x; }
			if (object_points[i].y >= o_part_maxY[4]) { o_part_maxY[4] = object_points[i].y; }
			if (object_points[i].x <= o_part_minX[4]) { o_part_minX[4] = object_points[i].x; }
			if (object_points[i].y <= o_part_minY[4]) { o_part_minY[4] = object_points[i].y; }
		}
		//mouse
		else if (i >= 48 && i <= 67)
		{
			if (image_points[i].x >= f_part_maxX[5]) { f_part_maxX[5] = image_points[i].x; }
			if (image_points[i].y >= f_part_maxY[5]) { f_part_maxY[5] = image_points[i].y; }
			if (image_points[i].x <= f_part_minX[5]) { f_part_minX[5] = image_points[i].x; }
			if (image_points[i].y <= f_part_minY[5]) { f_part_minY[5] = image_points[i].y; }

			if (object_points[i].x >= o_part_maxX[5]) { o_part_maxX[5] = object_points[i].x; }
			if (object_points[i].y >= o_part_maxY[5]) { o_part_maxY[5] = object_points[i].y; }
			if (object_points[i].x <= o_part_minX[5]) { o_part_minX[5] = object_points[i].x; }
			if (object_points[i].y <= o_part_minY[5]) { o_part_minY[5] = object_points[i].y; }
		}
	}



	//각 바운더리박스의 대각선의 절반 길이를 구함.
	for (unsigned int i = 0; i < 6; i++)
	{
		double tempX = (f_part_maxX[i] + f_part_minX[i]) / 2.0 - f_part_minX[i];
		double tempY = (f_part_maxY[i] + f_part_minY[i]) / 2.0 - f_part_minY[i];
		f_diag_distance[i] = sqrt(tempX * tempX + tempY * tempY);

		tempX = (o_part_maxX[i] + o_part_minX[i]) / 2.0 - o_part_minX[i];
		tempY = (o_part_maxY[i] + o_part_minY[i]) / 2.0 - o_part_minY[i];
		o_diag_distance[i] = sqrt(tempX * tempX + tempY * tempY);
	}

	//scale하기 위해 감싼 사각형의 중심이 0,0으로 가도록 이동.
	for (unsigned int i = 0; i < 68; i++)
	{
		cv::Point2d f_trsl_ori_point;
		glm::vec3 o_trsl_ori_point;
		//left brows
		if (i >= 17 && i <= 21)
		{
			f_trsl_ori_point = cv::Point2d(image_points[i].x + (0 - ((f_part_maxX[0] + f_part_minX[0]) / 2.0)),
				image_points[i].y + (0 - ((f_part_maxY[0] + f_part_minY[0]) / 2.0)));
			image_points[i] = f_trsl_ori_point;

			o_trsl_ori_point.x = object_points[i].x + (0 - ((o_part_maxX[0] + o_part_minX[0]) / 2.0));
			o_trsl_ori_point.y = object_points[i].y + (0 - ((o_part_maxY[0] + o_part_minY[0]) / 2.0));
			o_trsl_ori_point.z = 0;
			object_points[i] = o_trsl_ori_point;
		}
		//right brows
		else if (i >= 22 && i <= 26)
		{
			f_trsl_ori_point = cv::Point2d(image_points[i].x + (0 - ((f_part_maxX[1] + f_part_minX[1]) / 2.0)),
				image_points[i].y + (0 - ((f_part_maxY[1] + f_part_minY[1]) / 2.0)));
			image_points[i] = f_trsl_ori_point;

			o_trsl_ori_point.x = object_points[i].x + (0 - ((o_part_maxX[1] + o_part_minX[1]) / 2.0));
			o_trsl_ori_point.y = object_points[i].y + (0 - ((o_part_maxY[1] + o_part_minY[1]) / 2.0));
			o_trsl_ori_point.z = 0;
			object_points[i] = o_trsl_ori_point;
		}
		// nose
		else if (i >= 27 && i <= 35)
		{
			f_trsl_ori_point = cv::Point2d(image_points[i].x + (0 - ((f_part_maxX[2] + f_part_minX[2]) / 2.0)),
				image_points[i].y + (0 - ((f_part_maxY[2] + f_part_minY[2]) / 2.0)));
			image_points[i] = f_trsl_ori_point;

			o_trsl_ori_point.x = object_points[i].x + (0 - ((o_part_maxX[2] + o_part_minX[2]) / 2.0));
			o_trsl_ori_point.y = object_points[i].y + (0 - ((o_part_maxY[2] + o_part_minY[2]) / 2.0));
			o_trsl_ori_point.z = 0;
			object_points[i] = o_trsl_ori_point;
		}
		//left eye
		else if (i >= 36 && i <= 41)
		{
			f_trsl_ori_point = cv::Point2d(image_points[i].x + (0 - ((f_part_maxX[3] + f_part_minX[3]) / 2.0)),
				image_points[i].y + (0 - ((f_part_maxY[3] + f_part_minY[3]) / 2.0)));
			image_points[i] = f_trsl_ori_point;

			o_trsl_ori_point.x = object_points[i].x + (0 - ((o_part_maxX[3] + o_part_minX[3]) / 2.0));
			o_trsl_ori_point.y = object_points[i].y + (0 - ((o_part_maxY[3] + o_part_minY[3]) / 2.0));
			o_trsl_ori_point.z = 0;
			object_points[i] = o_trsl_ori_point;
		}
		//right eye
		else if (i >= 42 && i <= 47)
		{
			f_trsl_ori_point = cv::Point2d(image_points[i].x + (0 - ((f_part_maxX[4] + f_part_minX[4]) / 2.0)),
				image_points[i].y + (0 - ((f_part_maxY[4] + f_part_minY[4]) / 2.0)));
			image_points[i] = f_trsl_ori_point;

			o_trsl_ori_point.x = object_points[i].x + (0 - ((o_part_maxX[4] + o_part_minX[4]) / 2.0));
			o_trsl_ori_point.y = object_points[i].y + (0 - ((o_part_maxY[4] + o_part_minY[4]) / 2.0));
			o_trsl_ori_point.z = 0;
			object_points[i] = o_trsl_ori_point;
		}
		//mouse
		else if (i >= 48 && i <= 67)
		{
			f_trsl_ori_point = cv::Point2d(image_points[i].x + (0 - ((f_part_maxX[5] + f_part_minX[5]) / 2.0)),
				image_points[i].y + (0 - ((f_part_maxY[5] + f_part_minY[5]) / 2.0)));
			image_points[i] = f_trsl_ori_point;

			o_trsl_ori_point.x = object_points[i].x + (0 - ((o_part_maxX[5] + o_part_minX[5]) / 2.0));
			o_trsl_ori_point.y = object_points[i].y + (0 - ((o_part_maxY[5] + o_part_minY[5]) / 2.0));
			o_trsl_ori_point.z = 0;
			object_points[i] = o_trsl_ori_point;
		}
	}

	// 가둬 놓을 원과 같은 크기로 scale 한다.
	for (unsigned int i = 0; i < 68; i++)
	{
		//left brows
		if (i >= 17 && i <= 21)
		{
			image_points[i] *= (left_brow_rad / f_diag_distance[0]);
			object_points[i] *= (left_brow_rad / o_diag_distance[0]);
		}
		//right brows
		else if (i >= 22 && i <= 26)
		{
			image_points[i] *= (right_brow_rad / f_diag_distance[1]);
			object_points[i] *= (right_brow_rad / o_diag_distance[1]);
		}
		// nose
		else if (i >= 27 && i <= 35)
		{
			image_points[i] *= (nose_rad / f_diag_distance[2]);
			object_points[i] *= (nose_rad / o_diag_distance[2]);
		}
		//left eye
		else if (i >= 36 && i <= 41)
		{
			image_points[i] *= (left_eye_rad / f_diag_distance[3]);
			object_points[i] *= (left_eye_rad / o_diag_distance[3]);
		}
		//right eye
		else if (i >= 42 && i <= 47)
		{
			image_points[i] *= (right_eye_rad / f_diag_distance[4]);
			object_points[i] *= (right_eye_rad / o_diag_distance[4]);
		}
		//mouse
		else if (i >= 48 && i <= 67)
		{
			image_points[i] *= (mouse_rad / f_diag_distance[5]);
			object_points[i] *= (mouse_rad / o_diag_distance[5]);
		}
	}


	// 가둬 놓을 각 원의 중심으로 각각 이동
	for (unsigned int i = 0; i < 68; i++)
	{
		if (i >= 17 && i <= 21)
		{
			image_points[i].x += (image_cols / 3);
			image_points[i].y += (image_rows / 4.5);

			object_points[i].x += (image_cols / 3);
			object_points[i].y += (image_rows / 4.5);
		}
		//right brows
		else if (i >= 22 && i <= 26)
		{
			image_points[i].x += (image_cols / 3 * 2);
			image_points[i].y += (image_rows / 4.5);

			object_points[i].x += (image_cols / 3 * 2);
			object_points[i].y += (image_rows / 4.5);
		}
		// nose
		else if (i >= 27 && i <= 35)
		{
			image_points[i].x += (image_cols / 2);
			image_points[i].y += (image_rows / 2.0);

			object_points[i].x += (image_cols / 2);
			object_points[i].y += (image_rows / 2.0);
		}
		//left eye
		else if (i >= 36 && i <= 41)
		{
			image_points[i].x += (image_cols / 3);
			image_points[i].y += (image_rows / 2.5);

			object_points[i].x += (image_cols / 3);
			object_points[i].y += (image_rows / 2.5);
		}
		//right eye
		else if (i >= 42 && i <= 47)
		{
			image_points[i].x += (image_cols / 3 * 2);
			image_points[i].y += (image_rows / 2.5);

			object_points[i].x += (image_cols / 3 * 2);
			object_points[i].y += (image_rows / 2.5);
		}
		//mouse
		else if (i >= 48 && i <= 67)
		{
			image_points[i].x += (image_cols / 2);
			image_points[i].y += (image_rows / 1.3);

			object_points[i].x += (image_cols / 2);
			object_points[i].y += (image_rows / 1.3);
		}
	}

	//left brows boundary
	cv::circle(temp2, cv::Point2d(image_cols / 3, image_rows / 4.5), left_brow_rad, cv::Scalar(150, 150, 150), 1);
	//right brows boundary
	cv::circle(temp2, cv::Point2d(image_cols / 3 * 2, image_rows / 4.5), right_brow_rad, cv::Scalar(150, 150, 150), 1);

	//left eye boundary
	cv::circle(temp2, cv::Point2d(image_cols / 3, image_rows / 2.5), left_eye_rad, cv::Scalar(150, 150, 150), 1);
	//right eye boundary
	cv::circle(temp2, cv::Point2d(image_cols / 3 * 2, image_rows / 2.5), right_eye_rad, cv::Scalar(150, 150, 150), 1);

	//nose boundary
	cv::circle(temp2, cv::Point2d(image_cols / 2, image_rows / 2.0), nose_rad, cv::Scalar(150, 150, 150), 1);
	//mouse boundary
	cv::circle(temp2, cv::Point2d(image_cols / 2, image_rows / 1.3), mouse_rad, cv::Scalar(150, 150, 150), 1);


	//차이값 구하기
	out_err = 0;
	for (unsigned int i = 37; i < 68; i++)
	{

		//37, 38, 40, 41, 43,44,46,47, 60,62,64,66

		if(i==37 || i==38 || i==40 || i==41 ||
			i==43 || i==44|| i==46||i==47 ||
			i==60||i==62||i==64||i==66)
			out_err += sqrt(pow(image_points[i].x - object_points[i].x, 2.0) + pow(image_points[i].y - object_points[i].y, 2.0));


	}
}


void matching_object_to_face2(std::vector<glm::vec3>& object_points, std::vector<cv::Point2d>& image_points, double& out_err)
{
	double f_part_maxX = 0; double f_part_maxY = 0;
	double f_part_minX = 1000; double f_part_minY = 1000;
	double o_part_maxX = 0; double o_part_maxY = 0;
	double o_part_minX = 1000; double o_part_minY = 1000;
	for (unsigned int i = 0; i < 68; i++)
	{
		image_points[i] = cv::Point2d(shape.part(i).x()*noserate + image_cols / 2 - shape.part(27).x()*noserate,
			shape.part(i).y()*noserate + image_rows / 2 * (0.6f) - shape.part(27).y()*noserate);
	}

	for (unsigned int i = 48; i < 68; i++)
	{
		if (image_points[i].x >= f_part_maxX) { f_part_maxX = image_points[i].x; }
		if (image_points[i].y >= f_part_maxY) { f_part_maxY = image_points[i].y; }
		if (image_points[i].x <= f_part_minX) { f_part_minX = image_points[i].x; }
		if (image_points[i].y <= f_part_minY) { f_part_minY = image_points[i].y; }

		if (object_points[i].x >= o_part_maxX) { o_part_maxX = object_points[i].x; }
		if (object_points[i].y >= o_part_maxY) { o_part_maxY = object_points[i].y; }
		if (object_points[i].x <= o_part_minX) { o_part_minX = object_points[i].x; }
		if (object_points[i].y <= o_part_minY) { o_part_minY = object_points[i].y; }
	}
	double dx = 0.0;
	double dy = 0.0;
	dx = (o_part_maxX + o_part_minX) / 2 - (f_part_maxX + f_part_minX) / 2;
	dy = (o_part_maxY + o_part_minY) / 2 - (f_part_maxY + f_part_minY) / 2;

	out_err = 0;
	for (unsigned int i = 37; i < 68; i++)
	{

		//37, 38, 40, 41, 43,44,46,47, 60,62,64,66

		if (i == 37 || i == 38 || i == 40 || i == 41 ||
			i == 43 || i == 44 || i == 46 || i == 47)
			out_err += sqrt(pow(image_points[i].x - object_points[i].x, 2.0) + pow(image_points[i].y - object_points[i].y, 2.0));
		if (i == 60 || i == 62 || i == 64 || i == 66)
			out_err += sqrt(pow(image_points[i].x + dx - object_points[i].x, 2.0) + pow(image_points[i].y + dy - object_points[i].y, 2.0));
	}
	image_points.clear();
}

void interpolation(character &temp, character target, int num) {
	float a = (1.f / (5.f - (float)num));
	for (int i = 0; i < temp.objs.size(); i++) {
		for (int j = 0; j < temp.objs[i].vertices.size(); j++) {
			temp.objs[i].vertices[j] = a * target.objs[i].vertices[j] + (1 - a) * temp.objs[i].vertices[j];
			temp.objs[i].normals[j] = a * target.objs[i].normals[j] + (1 - a) * temp.objs[i].normals[j];
		}
	}
}
