#pragma once
#include <iostream>
#include <vector>
#include <glm\glm\vec3.hpp>
#include <glm\glm\vec2.hpp>
#include <glm\glm\glm.hpp>
#include <opencv2\opencv.hpp>
#include <GLFW/glfw3.h>
//삼,사각면의 버텍스 정보를 담을 객체
class surface {

public:
	unsigned int* face_vertexIndices;
	unsigned int* face_uvIndices;
	unsigned int* face_normalIndices;
	unsigned int face_angle;

	surface();
	surface(int n);
	void faces_print();
};

//개체의 정보를 담을 객체
class object {
public:
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals;
	std::vector<surface> faces;

	void print_obj_info();
};
//캐릭터 부분을 담을 객체
class character {
public:
	std::vector<object> objs;
	std::vector<std::string> part;
	std::vector<std::string> texName;

	void clear();
};

//텍스쳐 정보를 담는 구조체
struct Texture_Image {
	int width;
	int height;
	int format;
	unsigned char* data;
};

bool loadOBJ(const char* path, character& out_character);

bool lead_ref_68point(const char* path, std::vector< std::vector<glm::vec3> >& object_pts_68);

void moveNV(character& out_character);//노멀 벡터 이동
GLuint LoadTexture(const std::string TextureName); //텍스쳐 로더
void readTex(Texture_Image* tex, const std::string filename); //텍스쳐 정보 읽어오기
void changeRGB(unsigned char* BGR, unsigned char* RGB, int height, int width);//BGR -> RGB
void changeUD(unsigned char* image, int height, int width);//상하 반전
void drawCharacter(character cha, GLuint* text2D, float model_scale);//3D모델 그리기