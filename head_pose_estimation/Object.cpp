#include "Object.h"
#include <string>
#define MODEL_NUM 19
// surface class
surface::surface()
{
	this->face_angle = 0;
	this->face_vertexIndices = nullptr;
	this->face_uvIndices = nullptr;
	this->face_normalIndices = nullptr;
}
surface::surface(int n)
{
	this->face_angle = n;
	this->face_vertexIndices = new unsigned int[n];
	this->face_uvIndices = new unsigned int[n];
	this->face_normalIndices = new unsigned int[n];
}
void surface::faces_print()
{
	for (int i = 0; i < face_angle; i++)
	{
		printf("%d/%d/%d ", face_vertexIndices[i], face_uvIndices[i], face_normalIndices[i]);
	}
	printf("\n");
}
//


// object class
void character::clear()
{
	objs.clear();
	part.clear();
	texName.clear();
}

void object::print_obj_info()
{
	std::cout << "vertices num: " << vertices.size() << std::endl;
	std::cout << "uvs num: " << uvs.size() << std::endl;
	std::cout << "normals num: " << normals.size() << std::endl;
	std::cout << "faces num: " << faces.size() << std::endl;
}

bool loadOBJ(const char* path, character& out_character)
{
	int verNum = 0;

	std::cout << "Obj file Loading... file : " << path << std::endl;
	FILE* file = fopen(path, "r");
	if (file == NULL) {
		printf("file open failed\n");
		return false;
	}

	//파일의 마지막부분까지 읽는다.
	while (1)
	{
		char lineHeader[128];
		//read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break;

		if (strcmp(lineHeader, "usemtl") == 0) {
			object obj;
			char tempN[50];
			std::string name, textN;

			if (!out_character.objs.empty()) {
				verNum += out_character.objs[out_character.objs.size() - 1].vertices.size();
			}
			fscanf(file, "%s", tempN);
			name = tempN;

			if (name.find("head") != -1 || name.find("wire") != -1 || name.find("base") != -1) {
				textN = "standard_man/skin.jpg";
			}
			else if (name.find("Standard") != -1) {
				if (name.find("eye") != -1) {
					textN = "standard_man/eye.jpg";
				}
				else if (name.find("lent") != -1) {
					textN = "standard_man/eye.jpg";
				}
				else if (name.find("teeths") != -1) {
					textN = "standard_man/teeths.jpg";
				}
				else if (name.find("gums") != -1) {
					textN = "standard_man/gums diffuse.jpg";
				}
			}
			out_character.objs.push_back(obj);
			out_character.part.push_back(name);
			out_character.texName.push_back(textN);
		}
		//vertices
		else if (strcmp(lineHeader, "v") == 0)
		{
			glm::vec3 vertex;
			fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
			//vertex.z *= -1;
			out_character.objs[out_character.objs.size() - 1].vertices.push_back(vertex);
		}
		//texture coordinates
		else if (strcmp(lineHeader, "vt") == 0)
		{
			glm::vec2 uv;
			float x;
			fscanf(file, "%f %f\n", &uv.x, &uv.y);
			out_character.objs[out_character.objs.size() - 1].uvs.push_back(uv);
		}
		//normals
		else if (strcmp(lineHeader, "vn") == 0)
		{
			glm::vec3 normal;
			fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
			out_character.objs[out_character.objs.size() - 1].normals.push_back(normal);
		}
		else if (strcmp(lineHeader, "f") == 0)
		{
			//삼각형, 사각형 면 둘다 처리하기 위해 수정
			unsigned int vertexIndex[4], uvIndex[4], normalIndex[4];
			int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d\n",
				&vertexIndex[0], &uvIndex[0], &normalIndex[0],
				&vertexIndex[1], &uvIndex[1], &normalIndex[1],
				&vertexIndex[2], &uvIndex[2], &normalIndex[2],
				&vertexIndex[3], &uvIndex[3], &normalIndex[3]);
		

			if (matches == 9) //삼각면
			{
				surface face = surface(3);
				for (int i = 0; i < 3; i++)
				{
					face.face_vertexIndices[i] = vertexIndex[i] - verNum - 1;
					face.face_uvIndices[i] = uvIndex[i] - verNum - 1;
					face.face_normalIndices[i] = normalIndex[i] - verNum - 1;

				}

				out_character.objs[out_character.objs.size() - 1].faces.push_back(face);
			}
			else if (matches == 12) //사각면
			{
				surface face = surface(4);
				for (int i = 0; i < 4; i++)
				{
					face.face_vertexIndices[i] = vertexIndex[i] - verNum - 1;
					face.face_uvIndices[i] = uvIndex[i] - verNum - 1;
					face.face_normalIndices[i] = normalIndex[i] - verNum - 1;
				}

				out_character.objs[out_character.objs.size() - 1].faces.push_back(face);
			}
			else
			{
				printf("File can't be read by out simple parser\n");
				return false;
			}
		}
	}
	std::cout << "Obj file Loading complete. " << std::endl;
}

bool lead_ref_68point(const char* path, std::vector< std::vector<glm::vec3> >& object_pts_68)
{
	std::cout << "Ref_object_file Loading... file : " << path << std::endl;
	FILE* file = fopen(path, "r");
	if (file == NULL) {
		printf("file open failed\n");
		return false;
	}
	//파일의 마지막부분까지 읽는다.

	for (int i = 0; i < MODEL_NUM; i++)
	{
		std::vector<glm::vec3> temp;
		object_pts_68.push_back(temp);
	}

	std::string num[19] = { "0","1","2","3","4","5","6","7","8","9",
		"10","11","12","13","14","15","16","17","18" };
	while (1)
	{
		char lineHeader[128];
		//read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break;

		for (int i = 0; i < MODEL_NUM; i++)
		{
			if (num[i].compare(lineHeader) == 0)
			{
				glm::vec3 vertex;
				fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
				object_pts_68[i].push_back(vertex);
			}
		}

	}
	std::cout << "Ref_object_file Loading complete. " << std::endl;
}

GLuint LoadTexture(const std::string TextureName) {
	GLuint Texture;
	Texture_Image* texRec = new Texture_Image;

	readTex(texRec, TextureName);
	glGenTextures(1, &Texture);

	glBindTexture(GL_TEXTURE_2D, Texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texRec->width, texRec->height, 0, GL_RGB, GL_UNSIGNED_BYTE, texRec->data);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	delete texRec->data;
	delete texRec;

	return Texture;
}

void readTex(Texture_Image* tex, const std::string filename) {
	unsigned char* temp;
	cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);

	tex->height = image.rows;
	tex->width = image.cols;

	tex->data = new unsigned char[tex->height * tex->width * 3];
	temp = new unsigned char[tex->height * tex->width * 3];
	for (int h = 0; h < tex->height; h++) {
		for (int w = 0; w < tex->width * 3; w++) {
			temp[h * tex->width * 3 + w] = image.at<unsigned char>(h, w);
		}
	}

	changeRGB(temp, tex->data, tex->height, tex->width);
	changeUD(tex->data, tex->height, tex->width);
}

void changeRGB(unsigned char* BGR, unsigned char* RGB, int height, int width) {
	for (int i = 0; i < height * width * 3; i += 3) {
		RGB[i] = BGR[i + 2];
		RGB[i + 1] = BGR[i + 1];
		RGB[i + 2] = BGR[i];
	}
}

void changeUD(unsigned char* image, int height, int width) {
	int temp;
	for (int h = 0; h < height / 2; h++) {
		for (int w = 0; w < 3 * width; w++) {
			temp = image[h * width * 3 + w];
			image[h * width * 3 + w] = image[(height - h - 1) * width * 3 + w];
			image[(height - h - 1) * width * 3 + w] = temp;
		}
	}
}

void drawCharacter(character cha, GLuint* text2D, float model_scale) {
	for (int i = 0; i < cha.objs.size(); i++)
	{
		//glBindTexture(GL_TEXTURE_2D, text2D[i]);

		for (int j = 0; j < cha.objs[i].faces.size(); j++)
		{
			glBegin(GL_POLYGON);
			for (int k = 0; k < cha.objs[i].faces[j].face_angle; k++) {
				glNormal3f(
					cha.objs[i].normals[cha.objs[i].faces[j].face_vertexIndices[k]].x,
					cha.objs[i].normals[cha.objs[i].faces[j].face_vertexIndices[k]].y,
					cha.objs[i].normals[cha.objs[i].faces[j].face_vertexIndices[k]].z);
				/*glTexCoord2f(
					cha.objs[i].uvs[cha.objs[i].faces[j].face_uvIndices[k]].x,
					cha.objs[i].uvs[cha.objs[i].faces[j].face_uvIndices[k]].y);*/
				glVertex3f(
					cha.objs[i].vertices[cha.objs[i].faces[j].face_vertexIndices[k]].x * model_scale,
					cha.objs[i].vertices[cha.objs[i].faces[j].face_vertexIndices[k]].y * model_scale,
					cha.objs[i].vertices[cha.objs[i].faces[j].face_vertexIndices[k]].z * model_scale);
			}
			glEnd();
		}
	}
}