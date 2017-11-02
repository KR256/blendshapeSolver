#pragma once

#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

//#include "FacialSolverLib.h"

#include <maya/MPxCommand.h>

class SolverCmd : public MPxCommand
{
public:
	virtual MStatus doIt(const MArgList&);
	static void* creator() { return new SolverCmd; }

	int loadC3DFile(int numberMarkers, std::map <std::string, std::vector<Eigen::Vector3d>> &outC3dMarkers,
		std::vector<std::string> &outOrderedMarkers, std::string C3D_PATH);
	int loadShapes(std::vector<std::vector<unsigned short>> &outIndicesAll,
		std::vector<std::vector<glm::vec3>> &outIndexed_verticesAll,
		std::vector<std::vector<glm::vec2>> &outIndexed_uvsAll,
		std::vector<std::vector<glm::vec3>> &outIndexed_normalsAll,
		std::vector<Eigen::MatrixXd> &outShapesVertices,
		std::vector<std::string> &allBlendshapeNames, std::string path);
	void loadBarycentricCoords(std::map <int, Eigen::Vector2d> &outBarycentricFaceCoords, 
		std::vector <int> &outBarycentricKeysReadOrder, std::string BARYCOORDS_PATH);

	std::string openFileDialog();
};
