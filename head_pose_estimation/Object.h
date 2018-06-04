#pragma once
#include <iostream>
#include <vector>
#include <glm\glm\vec3.hpp>
#include <glm\glm\vec2.hpp>
#include <glm\glm\glm.hpp>
#include <opencv2\opencv.hpp>
#include <GLFW/glfw3.h>
//��,�簢���� ���ؽ� ������ ���� ��ü
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

//��ü�� ������ ���� ��ü
class object {
public:
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals;
	std::vector<surface> faces;

	void print_obj_info();
};
//ĳ���� �κ��� ���� ��ü
class character {
public:
	std::vector<object> objs;
	std::vector<std::string> part;
	std::vector<std::string> texName;

	void clear();
};

//�ؽ��� ������ ��� ����ü
struct Texture_Image {
	int width;
	int height;
	int format;
	unsigned char* data;
};

bool loadOBJ(const char* path, character& out_character);

bool lead_ref_68point(const char* path, std::vector< std::vector<glm::vec3> >& object_pts_68);

void moveNV(character& out_character);//��� ���� �̵�
GLuint LoadTexture(const std::string TextureName); //�ؽ��� �δ�
void readTex(Texture_Image* tex, const std::string filename); //�ؽ��� ���� �о����
void changeRGB(unsigned char* BGR, unsigned char* RGB, int height, int width);//BGR -> RGB
void changeUD(unsigned char* image, int height, int width);//���� ����
void drawCharacter(character cha, GLuint* text2D, float model_scale);//3D�� �׸���