// FacialSolverTest.cpp : Defines the entry point for the console application.
//
#include <iostream>
#include <tchar.h>
#include <windows.h>
#include <string>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <map>




#include <btkAcquisitionFileReader.h>
#include <btkC3DFileIO.h>
#include <btkAcquisition.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>
GLFWwindow* window;

#include <Eigen/Dense>
#include <iostream>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "objloader.hpp"

#include "AdditionalFiles\controls.hpp"
#include "AdditionalFiles\texture.hpp"
#include "AdditionalFiles\shader.hpp"
#include "AdditionalFiles\vboindexer.hpp"
#include <iostream>

#include "FacialSolverLib.h"
#include "solverCmd.h"
#include <maya/MGlobal.h>

using namespace std;

#ifndef NOMINMAX
#define NOMINMAX
#endif

//Returns the number of frames + everythin else
int SolverCmd::loadC3DFile(int numberMarkers, std::map <std::string, std::vector<Eigen::Vector3d>> &outC3dMarkers, std::vector<std::string> &outOrderedMarkers, std::string C3D_PATH) {

	btk::AcquisitionFileReader::Pointer reader = btk::AcquisitionFileReader::New();
	btk::C3DFileIO::Pointer io = btk::C3DFileIO::New();
	reader->SetAcquisitionIO(io); // Will open the given file as a C3D file.
	reader->SetFilename(C3D_PATH);
	reader->Update();
	btk::Acquisition::Pointer acq = reader->GetOutput();



	int numFrames = acq->GetAnalogFrameNumber();

	int markerCounter = 0;
	for (btk::Acquisition::PointConstIterator it = acq->BeginPoint(); it != acq->EndPoint(); ++it)
	{
		if (markerCounter >= 83)
			break;

		std::vector<Eigen::Vector3d> markerCoordsPerFrame;
		//std::cout << (*it)->GetLabel() << " (" << (*it)->GetDescription() << "): " << std::endl;

		Eigen::Matrix<double, -1, 3> data = acq->GetPoint(markerCounter)->GetValues();

		Eigen::Index numRows = data.rows();
		Eigen::Index numCols = data.cols();

		for (Eigen::Index frameIdx = 0; frameIdx < numRows; frameIdx++) {
			markerCoordsPerFrame.push_back(Eigen::Vector3d(data(frameIdx, 0), data(frameIdx, 1), data(frameIdx, 2)));
		}
		outC3dMarkers.insert(std::pair<std::string, std::vector<Eigen::Vector3d>>((*it)->GetLabel(), markerCoordsPerFrame));
		outOrderedMarkers.push_back((*it)->GetLabel());

		markerCounter++;
	}

	return numFrames;

}


//Returns the index of the neutral pose Idx + everythin else
int SolverCmd::loadShapes(std::vector<std::vector<unsigned short>> &outIndicesAll, 
	std::vector<std::vector<glm::vec3>> &outIndexed_verticesAll, 
	std::vector<std::vector<glm::vec2>> &outIndexed_uvsAll, 
	std::vector<std::vector<glm::vec3>> &outIndexed_normalsAll,
	std::vector<Eigen::MatrixXd> &outShapesVertices,
	std::vector<std::string> &allBlendshapeNames, std::string NEUT_PATH) {


	//std::string path = "C:/Users/Kyle/Documents/DEV/FacialSolver/FacialSolver/FacialSolverLib/FacialSolverTest/ShriBlendshapes";
	std::size_t botDirPos = NEUT_PATH.find_last_of("\\");
	std::string BS_PATH = NEUT_PATH.substr(0, botDirPos);
	std::cout << "BS folder path: " << BS_PATH << std::endl;
	std::vector<std::string> objFiles = FindAllObjInFolder(BS_PATH);

	int numVertices = -1;

	//Just load one of the shapes to get generic info on num vertex and such
	if (objFiles.size() > 0) {
		std::vector<unsigned short> indices;
		std::vector<glm::vec3> indexed_vertices;
		std::vector<glm::vec2> indexed_uvs;
		std::vector<glm::vec3> indexed_normals;

		bool res = loadAssImp(objFiles[0].c_str(), indices, indexed_vertices, indexed_uvs, indexed_normals);
		if (res == true)
			numVertices = indices.size();
	}
	if (numVertices == -1)
	{
		std::cout << "Obj file couldn't be read" << std::endl;
		return -1;
	}

	int neutralShapeIdx = -1;

	int dimensions = 3;
	int shapeIdxStartOffset = 0;
	for (int shapeIdx = 0; shapeIdx < objFiles.size(); shapeIdx++) {
		//for (int shapeIdx = shapeIdxStartOffset; shapeIdx < objFiles.size(); shapeIdx++) {

		std::string fullBSPath = objFiles[shapeIdx];
		std::cout << "OBJ FILE NAME: " << fullBSPath << std::endl;
		fullBSPath.erase(fullBSPath.find_last_of("."), string::npos);
		std::size_t botDirPos = fullBSPath.find_last_of("\\");
		std::string BS_FILE_NAME = fullBSPath.substr(botDirPos + 1, -1);
		allBlendshapeNames.push_back(BS_FILE_NAME);
		if (objFiles[shapeIdx] == NEUT_PATH) {
			std::cout << "There's a Neutral match!" << std::endl;
			neutralShapeIdx = shapeIdx - shapeIdxStartOffset;
		}


		std::vector<unsigned short> temp_indices;
		std::vector<glm::vec3> temp_indexed_vertices;
		std::vector<glm::vec2> temp_indexed_uvs;
		std::vector<glm::vec3> temp_indexed_normals;
		bool res = loadAssImp(objFiles[shapeIdx].c_str(), temp_indices, temp_indexed_vertices, temp_indexed_uvs, temp_indexed_normals);


		Eigen::MatrixXd m(numVertices, dimensions);
		for (int vrtxIdx = 0; vrtxIdx < numVertices; vrtxIdx++) {
			m(vrtxIdx, 0) = temp_indexed_vertices[vrtxIdx].x;
			m(vrtxIdx, 1) = temp_indexed_vertices[vrtxIdx].y;
			m(vrtxIdx, 2) = temp_indexed_vertices[vrtxIdx].z;
		}
		outShapesVertices.push_back(m);

		outIndicesAll.push_back(temp_indices);
		outIndexed_verticesAll.push_back(temp_indexed_vertices);
		outIndexed_uvsAll.push_back(temp_indexed_uvs);
		outIndexed_normalsAll.push_back(temp_indexed_normals);
	}



	return neutralShapeIdx;
}

void SolverCmd::loadBarycentricCoords(std::map <int, Eigen::Vector2d> &outBarycentricFaceCoords, std::vector <int> &outBarycentricKeysReadOrder, std::string BARYCOORDS_PATH) {

	string line;
	ifstream myfile(BARYCOORDS_PATH);
	if (myfile.is_open())
	{
		getline(myfile, line);
		//std::cout << line << '\n';

		std::string token;
		size_t pos = 0;
		pos = line.find("["); //Skip first 
		if (pos == std::string::npos) {
			myfile.close();
			return;
		}
		line.erase(0, pos + 1);

		while ((pos = line.find("[")) != std::string::npos) {
			line.erase(0, pos + 1);

			pos = line.find(",");
			token = line.substr(0, pos);
			int faceID = std::stoi(token);
			outBarycentricKeysReadOrder.push_back(faceID);
			line.erase(0, pos + 1);

			pos = line.find(",");
			token = line.substr(0, pos);
			double xCoord = std::stod(token);
			line.erase(0, pos + 1);

			pos = line.find("]");
			token = line.substr(0, pos);
			double yCoord = std::stod(token);
			line.erase(0, pos + 1);

			outBarycentricFaceCoords.insert(std::pair<int, Eigen::Vector2d>(faceID, Eigen::Vector2d(xCoord, yCoord)));

			if ((pos = line.find(",")) == std::string::npos) {
				break;
			}

		}

		myfile.close();
	}

}



MStatus SolverCmd::doIt(const MArgList&)
{
	//***********************Setup neutral shape markers to c3d makers association
	std::map <int, std::string> shapeNeutralMarkers2captureNeutralMarkersMap;
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(0, "Face_5:lFarEyeBrow"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(1, "Face_5:lInnerEyeBrow"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(2, "Face_5:rInnerEyeBrow"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(3, "Face_5:rFarEyeBrow"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(4, "Face_5:lInnerEyeLid"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(5, "Face_5:lInnerLowerEyeLid"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(6, "Face_5:rInnerEyeLid"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(7, "Face_5:rInnerLowerEyeLid"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(8, "Face_5:lUpperDimple"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(9, "Face_5:rUpperDimple"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(10, "Face_5:lCornerLip"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(11, "Face_5:lUpperLip"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(12, "Face_5:rUpperLip"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(13, "Face_5:rCornerLip"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(14, "Face_5:rLowerLip"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(15, "Face_5:lLowerLip"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(16, "Face_5:rLowerChin"));
	shapeNeutralMarkers2captureNeutralMarkersMap.insert(std::pair<int, std::string>(17, "Face_5:lLowerChin"));

	//The association happens between the int, which corresponds to the idx position in barycentricFaceCoordsKeysOrder, i.e. the order of the barycentric file, 
	//and the name of the marker loaded from c3d


	
	

	//***********************Read c3d file with the markers info
	std::map <std::string, std::vector<Eigen::Vector3d>> c3dMarkers;
	std::vector<std::string> orderedMarkers;
	int numberMarkers = 83;

	cout << "Please enter C3D filename" << endl;
	MGlobal::displayInfo("Please select C3D file ...\n");
	std::string C3D_PATH = openFileDialog();
	cout << "C3D file path: " << C3D_PATH << endl;
	int numFrames = loadC3DFile(numberMarkers, c3dMarkers, orderedMarkers, C3D_PATH);

	


	//***********************Read our .obj file
	//Load all the shapes

	// Get Inputs Paths
	cout << "Please enter Blendshape folder name" << endl;
	MGlobal::displayInfo("Please select Neutral.obj file ...\n");
	std::string NEUT_FILE = openFileDialog();
	cout << "Neutral file path: " << NEUT_FILE << endl;

	std::vector<std::vector<unsigned short>> indicesAll;
	std::vector<std::vector<glm::vec3>> indexed_verticesAll;
	std::vector<std::vector<glm::vec2>> indexed_uvsAll;
	std::vector<std::vector<glm::vec3>> indexed_normalsAll;

	std::vector<Eigen::MatrixXd> shapesOriginalVerticesEigen;
	std::vector<std::string> allBlendshapeNames;

	int neutralShapeIdx = loadShapes(indicesAll, indexed_verticesAll, indexed_uvsAll, indexed_normalsAll, shapesOriginalVerticesEigen, allBlendshapeNames, NEUT_FILE);

	if (neutralShapeIdx < 0) {
		std::cout << "No neutral pose was found" << "\n";
		return MStatus::kFailure;
	}

	int numVertices = shapesOriginalVerticesEigen[neutralShapeIdx].rows();

	//Convert all shapes into offsets from neutral
	std::vector<Eigen::MatrixXd> shapesOffsets;
	for (int shapeIdx = 0; shapeIdx < shapesOriginalVerticesEigen.size(); shapeIdx++) {
		shapesOffsets.push_back(shapesOriginalVerticesEigen[shapeIdx] - shapesOriginalVerticesEigen[neutralShapeIdx]);
	}




	//***********************Read Barycentric coord files
	std::map <int, Eigen::Vector2d> barycentricFaceCoords;
	std::vector <int> barycentricFaceCoordsKeysOrder;

	cout << "Please enter Barycentric coordinates filename" << endl;
	MGlobal::displayInfo("Please select file containing barycentric coordinates ...\n");
	std::string BARYCOORDS_PATH = openFileDialog();
	cout << "Barycentric coordinates file path: " << BARYCOORDS_PATH << endl;

	loadBarycentricCoords(barycentricFaceCoords, barycentricFaceCoordsKeysOrder, BARYCOORDS_PATH);





	//***********************Extract coordinates from barycentric file
	//std::vector<Eigen::Vector3d> neutralShapeMarkers; //Deprecated

	//Get the barycentric coordinates of the relevant markers out of all the shapes
	std::vector<Eigen::Matrix<double, -1, 3>> allShapeMarkers;
	for (int shapeIdx = 0; shapeIdx < shapesOriginalVerticesEigen.size(); shapeIdx++) {

		Eigen::MatrixXd shapeMarkers(barycentricFaceCoordsKeysOrder.size(), 3);

		for (int coordIdx = 0; coordIdx < barycentricFaceCoordsKeysOrder.size(); coordIdx++) {
			int faceId = barycentricFaceCoordsKeysOrder[coordIdx];
			faceId = faceId * 3;

			double xCoord = barycentricFaceCoords[barycentricFaceCoordsKeysOrder[coordIdx]](0);
			double yCoord = barycentricFaceCoords[barycentricFaceCoordsKeysOrder[coordIdx]](1);

			Eigen::Vector3d faceVrtx1(shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId], 0),
				shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId], 1),
				shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId], 2));

			Eigen::Vector3d faceVrtx2(shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId + 1], 0),
				shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId + 1], 1),
				shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId + 1], 2));

			Eigen::Vector3d faceVrtx3(shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId + 2], 0),
				shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId + 2], 1),
				shapesOriginalVerticesEigen[shapeIdx](indicesAll[shapeIdx][faceId + 2], 2));

			Eigen::Vector3d newPt = xCoord * faceVrtx1 + yCoord * faceVrtx2 + (1 - xCoord - yCoord) * faceVrtx3;
			shapeMarkers.row(coordIdx) = newPt;
		}
		allShapeMarkers.push_back(shapeMarkers);
	}

	//Eigen::MatrixXd neutralShapeMarkers(barycentricFaceCoordsKeysOrder.size(), 3);
	Eigen::MatrixXd neutralShapeMarkers = allShapeMarkers[neutralShapeIdx];

	
	//FROM NOW ONWARDS allShapeMarkers are in displacement form!
	//Eigen::Matrix<double, -1, 3> neutralShapeMarkersFinal = allShapeMarkers[neutralShapeIdx];
	for (int shapeIdx = 0; shapeIdx < shapesOriginalVerticesEigen.size(); shapeIdx++) {
		allShapeMarkers[shapeIdx] = allShapeMarkers[shapeIdx] - neutralShapeMarkers;
	}

	//for (int coordIdx = 0; coordIdx < barycentricFaceCoordsKeysOrder.size(); coordIdx++) {
	//	int faceId = barycentricFaceCoordsKeysOrder[coordIdx];
	//	faceId = faceId * 3;

	//	double xCoord = barycentricFaceCoords[barycentricFaceCoordsKeysOrder[coordIdx]](0);
	//	double yCoord = barycentricFaceCoords[barycentricFaceCoordsKeysOrder[coordIdx]](1);

	//	Eigen::Vector3d faceVrtx1(shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId], 0),
	//		shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId], 1),
	//		shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId], 2));

	//	Eigen::Vector3d faceVrtx2(shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId + 1], 0),
	//		shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId + 1], 1),
	//		shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId + 1], 2));

	//	Eigen::Vector3d faceVrtx3(shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId + 2], 0),
	//		shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId + 2], 1),
	//		shapesOriginalVerticesEigen[neutralShapeIdx](indicesAll[neutralShapeIdx][faceId + 2], 2));

	//	Eigen::Vector3d newPt = xCoord * faceVrtx1 + yCoord * faceVrtx2 + (1 - xCoord - yCoord) * faceVrtx3;
	//	neutralShapeMarkers.row(coordIdx) = newPt.transpose();
	//}



	//Procrustes from landmarks in c3d space to landmarks in neutral shape space
	//Eigen::MatrixXd X(neutralShapeMarkers.size(), 3);
	Eigen::MatrixXd X = neutralShapeMarkers;
	Eigen::MatrixXd Y(neutralShapeMarkers.rows(), 3);
	for (int coordIdx = 0; coordIdx < barycentricFaceCoordsKeysOrder.size(); coordIdx++) {
		//X.row(coordIdx) = neutralShapeMarkers[coordIdx].transpose();
		Y.row(coordIdx) = c3dMarkers[shapeNeutralMarkers2captureNeutralMarkersMap[coordIdx]][0].transpose();
	}
	////////std::cout << X << "\n\n";
	////////std::cout << Y << "\n\n";

	//////Eigen::MatrixXd X_test(5, 3);
	//////Eigen::MatrixXd Y_test(5, 3);
	//////X_test << 111.8580, 95.5860, -228.5650,
	//////	58.6818, 152.7400, -207.5090,
	//////	24.3612, 152.2010, -209.8560,
	//////	-27.1694, 97.2519, -241.0860,
	//////	34.5408, 75.7310, -137.3480;


	//////Y_test << 108.6590,  96.6260, -225.4760,
	//////	54.9069, 152.9150, -204.0790,
	//////	20.5252, 151.9380, -206.4230,
	//////	-30.0959, 96.5905, -238.3440,
	//////	31.4888, 74.9023, -134.6600;

	//////Eigen::MatrixXd X_test2(5, 3);
	//////Eigen::MatrixXd Y_test2(5, 3);
	//////X_test2 << 99.1422000000000, 88.2598000000000, -206.555500000000,
	//////	99.1422000000000, 88.2598000000000, -206.555500000000,
	//////	84.3230000000000, 111.779150000000, -190.009000000000,
	//////	53.4750000000000, 115.707150000000, -170.402000000000,
	//////	53.4750000000000, 115.707150000000, -170.402000000000;

	//////Y_test2 << 104.843000000000, 89.3823000000000, -208.705000000000,
	//////	104.843000000000, 89.3823000000000, -208.705000000000,
	//////	86.2915000000000, 115.848000000000, -192.896000000000,
	//////	59.6131000000000, 119.669000000000, -177.945000000000,
	//////	59.6131000000000, 119.669000000000, -177.945000000000;

	//////std::cout << X_test << "\n\n";
	//////std::cout << Y_test << "\n\n";

	//Figure out transform that aligns the neutral markers to the equivalent shape markers
	Eigen::Matrix<double, -1, 3> alignedc3dMarkers;
	FacialSolver::FacialSolver::TransformProc outTransf;
	double d;
	FacialSolver::FacialSolver::Procrustes(X, Y, false, d, alignedc3dMarkers, outTransf);

	//std::cout << "Results" << "\n";
	//////std::cout << d << "\n\n";
	//std::cout << alignedc3dMarkers << "\n\n";

	//////Test code: convert Y to X
	//Eigen::MatrixXd Y_transf = outTransf.scale * Y * outTransf.Rot + outTransf.Trans;
	//std::cout << Y_transf << "\n\n";

	//Move all poses of the c3d according to the transform of procrustes
	//Select only the relevant markers from c3d
	//
	std::vector<Eigen::Matrix<double, -1, 3>> alignedAndReduced_c3dMarkers;

	////////Align all the frames according to rigid markers
	//////std::vector<std::string> rigidMarkers;
	//////rigidMarkers.push_back("Face_5:rTemple");
	//////rigidMarkers.push_back("Face_5:rForeHead");
	//////rigidMarkers.push_back("Face_5:lTemple");
	//////rigidMarkers.push_back("Face_5:lForeHead");
	//////rigidMarkers.push_back("Face_5:Nose");

	//////Eigen::MatrixXd frame0(rigidMarkers.size(), 3);
	//////for (int coordIdx = 0; coordIdx < rigidMarkers.size(); coordIdx++) {
	//////	frame0.row(coordIdx) = c3dMarkers[rigidMarkers[coordIdx]][0].transpose();
	//////}

	//////std::map <std::string, std::vector<Eigen::Vector3d>> c3dMarkers_NoRigidMotions;
	//////std::map <std::string, int> reverseTempMap;
	//////bool initializedReverse = false;
	//////for (int frameIdx = 0; frameIdx < numFrames; frameIdx++) {

	//////	Eigen::MatrixXd temp_Y(rigidMarkers.size(), 3);
	//////	for (int coordIdx = 0; coordIdx < rigidMarkers.size(); coordIdx++) {
	//////		temp_Y.row(coordIdx) = c3dMarkers[rigidMarkers[coordIdx]][frameIdx].transpose();
	//////	}

	//////	Eigen::Matrix<double, -1, 3> alignedc3dMarkers_temp;
	//////	FacialSolver::FacialSolver::TransformProc outTransf_temp;
	//////	double d_temp;
	//////	//Procrustes(X, Y, false, d, alignedc3dMarkers);
	//////	FacialSolver::FacialSolver::Procrustes(frame0, temp_Y, false, d, alignedc3dMarkers, outTransf_temp);

	//////	Eigen::MatrixXd temp_Y_Full(orderedMarkers.size(), 3);
	//////	for (int coordIdx = 0; coordIdx < orderedMarkers.size(); coordIdx++) {

	//////		temp_Y_Full.row(coordIdx) = c3dMarkers[orderedMarkers[coordIdx]][frameIdx].transpose();

	//////		if(!initializedReverse)
	//////			reverseTempMap.insert(std::pair<std::string, int>(orderedMarkers[coordIdx], coordIdx));
	//////	}
	//////	initializedReverse = true;

	//////	Eigen::Matrix<double, -1, 3> newTrans = outTransf_temp.Trans.row(0).replicate(orderedMarkers.size(),1);

	//////	//std::cout << outTransf_temp.Trans << "\n\n";
	//////	//std::cout << temp_Y_Full << "\n\n";
	//////	Eigen::MatrixXd temp_Y_transf = outTransf_temp.scale * temp_Y_Full * outTransf_temp.Rot + newTrans;
	//////	//std::cout << temp_Y_transf << "\n\n";
	//////	alignedAndReduced_c3dMarkers.push_back(temp_Y_transf);

	//////}


	//////for (int frameIdx = 0; frameIdx < numFrames; frameIdx++) {

	//////	Eigen::MatrixXd temp_Y(neutralShapeMarkers.size(), 3);

	//////	for (int coordIdx = 0; coordIdx < barycentricFaceCoordsKeysOrder.size(); coordIdx++) {
	//////		temp_Y.row(coordIdx) = alignedAndReduced_c3dMarkers[frameIdx].row(reverseTempMap[shapeNeutralMarkers2captureNeutralMarkersMap[coordIdx]]);
	//////	}

	//////	Eigen::MatrixXd temp_Y_transf = outTransf.scale * temp_Y * outTransf.Rot + outTransf.Trans;
	//////	alignedAndReduced_c3dMarkers[frameIdx] = temp_Y_transf;
	//////}


	//Use this code if I don't introduce rigid alignment
	for (int frameIdx = 0; frameIdx < numFrames; frameIdx++) {

		Eigen::MatrixXd temp_Y(neutralShapeMarkers.rows(), 3);
		for (int coordIdx = 0; coordIdx < barycentricFaceCoordsKeysOrder.size(); coordIdx++) {
			temp_Y.row(coordIdx) =  c3dMarkers[shapeNeutralMarkers2captureNeutralMarkersMap[coordIdx]][frameIdx].transpose();
		}

		Eigen::MatrixXd temp_Y_transf = outTransf.scale * temp_Y * outTransf.Rot + outTransf.Trans;
		alignedAndReduced_c3dMarkers.push_back(temp_Y_transf);

	}

	////std::cout << "----------" << "\n\n";
	////std::cout << alignedAndReduced_c3dMarkers[0] << "\n\n";

	//Convert to displacement
	Eigen::Matrix<double, -1, 3> neutralPoseC3Dmarkers = alignedAndReduced_c3dMarkers[0];
	for (int frameIdx = 0; frameIdx < numFrames; frameIdx++) {
		alignedAndReduced_c3dMarkers[frameIdx] = alignedAndReduced_c3dMarkers[frameIdx] - neutralPoseC3Dmarkers;

	}

	////std::cout << "----------" << "\n\n";
	////std::cout << neutralPoseC3Dmarkers << "\n\n";
	////std::cout << alignedAndReduced_c3dMarkers[0] << "\n\n";




	//Trimmed c3d, aligned
	//The real coordinates (associated to the barycentric coords) of all the shapes
	numFrames = 300;
	//Eigen::MatrixXd Allweights = Eigen::MatrixXd::Zero(4000, shapesOriginalVerticesEigen.size());


	// Measure speed
	Eigen::MatrixXd prev_weights = Eigen::MatrixXd::Zero(1, shapesOriginalVerticesEigen.size());
	for (int frameIdx = 0; frameIdx < numFrames; frameIdx++) {

		Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(1, shapesOriginalVerticesEigen.size());
		

		Eigen::MatrixXd delta = allShapeMarkers[neutralShapeIdx] - alignedAndReduced_c3dMarkers[0];
		// Target = Target + delta - Neutral_;
		// It just so happens that the target is the first frame, which I'm also using as the neutral pose

		int targetFrameIdx = frameIdx;
		//Eigen::MatrixXd Target = alignedAndReduced_c3dMarkers[targetFrameIdx] + delta - allShapeMarkers[neutralShapeIdx];
		Eigen::MatrixXd Target = alignedAndReduced_c3dMarkers[targetFrameIdx] + delta;

		FacialSolver::FacialSolver::SolveShape(Target, allShapeMarkers, prev_weights, weights);

		/*std::cout << "-----" << "\n";
		std::cout << *w << "\n\n";
		std::cout << "-----" << "\n";
*/
		//std::cout << summary.BriefReport() << "\n";
		//std::cout << summary.FullReport() << std::endl;
		//std::cout << summary.termination_type << std::endl;

		MGlobal::executeCommand(MString("currentTime ") + (frameIdx + 1));
		for (int bsId = 0; bsId < shapesOriginalVerticesEigen.size(); bsId++)
		{
			std::string bsName = allBlendshapeNames[bsId];
			MString MbsName = bsName.c_str();
			auto bsWeight = weights(bsId);
			MGlobal::executeCommand(MString("setAttr \"blendShape1.BS_") + MbsName + "\" " + bsWeight + ";");
			//cout << "Setting " << bsName << ": " << bsWeight << endl;
		}

		MGlobal::executeCommand(MString("setKeyframe blendShape1.w;"));

		if (frameIdx != 0)
			prev_weights = weights;

	}


	//btk::Point::Pointer p = acq->GetPoint(0); // First point in the acquisition
	//std::cout << p->GetLabel() << std::endl;
	////p = acq->GetPoint("lNose"); // Point with the label 'RHEE'. Could throw an exception
	//std::cout << p->GetFrameNumber() << std::endl;
	//btk::Acquisition::PointIterator itP = acq->FindPoint("IS_THERE_A_POINT_WITH_THIS_LABEL"); // Let's try to find this point ...
	//if (itP != acq->EndPoint()) // Point found as the iterator doesn't point to the end of the list
	//	std::cout << (*itP)->GetValues() << std::endl;
	//else
	//	std::cerr << "No point found" << std::endl;


	// Initialise GLFW

	return MStatus::kSuccess;
}

//int SolverCmd::visualise(int neutralShapeId,)
//{
//	if (!glfwInit())
//	{
//		fprintf(stderr, "Failed to initialize GLFW\n");
//		getchar();
//		return -1;
//	}
//
//	glfwWindowHint(GLFW_SAMPLES, 4);
//	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
//	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//
//	// Open a window and create its OpenGL context
//	window = glfwCreateWindow(1024, 768, "Tutorial 09 - Loading with AssImp", NULL, NULL);
//	if (window == NULL) {
//		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
//		getchar();
//		glfwTerminate();
//		return -1;
//	}
//	glfwMakeContextCurrent(window);
//
//	// Initialize GLEW
//	glewExperimental = true; // Needed for core profile
//	if (glewInit() != GLEW_OK) {
//		fprintf(stderr, "Failed to initialize GLEW\n");
//		getchar();
//		glfwTerminate();
//		return MStatus::kFailure;
//	}
//
//	// Ensure we can capture the escape key being pressed below
//	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
//	// Hide the mouse and enable unlimited mouvement
//	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//
//	// Set the mouse at the center of the screen
//	glfwPollEvents();
//	glfwSetCursorPos(window, 1024 / 2, 768 / 2);
//
//	// Dark blue background
//	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);
//
//	// Enable depth test
//	glEnable(GL_DEPTH_TEST);
//	// Accept fragment if it closer to the camera than the former one
//	glDepthFunc(GL_LESS);
//
//	// Cull triangles which normal is not towards the camera
//	glEnable(GL_CULL_FACE);
//
//	GLuint VertexArrayID;
//	glGenVertexArrays(1, &VertexArrayID);
//	glBindVertexArray(VertexArrayID);
//
//	// Create and compile our GLSL program from the shaders
//	GLuint programID = LoadShaders("shaders\\StandardShading.vertexshader", "shaders\\StandardShading.fragmentshader");
//
//	// Get a handle for our "MVP" uniform
//	GLuint MatrixID = glGetUniformLocation(programID, "MVP");
//	GLuint ViewMatrixID = glGetUniformLocation(programID, "V");
//	GLuint ModelMatrixID = glGetUniformLocation(programID, "M");
//
//	// Load the texture
//	//GLuint Texture = loadDDS("ShriBlendshapes\\Shri_Diffuse.DDS");
//	GLuint Texture = loadDDS("ShriBlendshapes\\Shri_Diffuse_Rot.DDS");
//
//	// Get a handle for our "myTextureSampler" uniform
//	GLuint TextureID = glGetUniformLocation(programID, "myTextureSampler");
//
//
//
//
//	//***********************Load marker obj
//	std::vector<unsigned short> markerIndices;
//	std::vector<glm::vec3> markerVertices;
//	std::vector<glm::vec2> markerUvs;
//	std::vector<glm::vec3> markerNormals;
//
//	bool res = loadAssImp("C:/Users/Kyle/Documents/DEV/FacialSolver/FacialSolver/FacialSolverLib/FacialSolverTest/obj\\marker.obj", markerIndices, markerVertices, markerUvs, markerNormals);
//
//
//	GLuint vertexbufferM;
//	glGenBuffers(1, &vertexbufferM);
//	glBindBuffer(GL_ARRAY_BUFFER, vertexbufferM);
//	glBufferData(GL_ARRAY_BUFFER, markerVertices.size() * sizeof(glm::vec3), &markerVertices[0], GL_STATIC_DRAW);
//
//	GLuint uvbufferM;
//	glGenBuffers(1, &uvbufferM);
//	glBindBuffer(GL_ARRAY_BUFFER, uvbufferM);
//	glBufferData(GL_ARRAY_BUFFER, markerUvs.size() * sizeof(glm::vec2), &markerUvs[0], GL_STATIC_DRAW);
//
//	GLuint normalbufferM;
//	glGenBuffers(1, &normalbufferM);
//	glBindBuffer(GL_ARRAY_BUFFER, normalbufferM);
//	glBufferData(GL_ARRAY_BUFFER, markerNormals.size() * sizeof(glm::vec3), &markerNormals[0], GL_STATIC_DRAW);
//
//	// Generate a buffer for the indices as well
//	GLuint elementbufferM;
//	glGenBuffers(1, &elementbufferM);
//	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbufferM);
//	glBufferData(GL_ELEMENT_ARRAY_BUFFER, markerIndices.size() * sizeof(unsigned short), &markerIndices[0], GL_STATIC_DRAW);
//
//
//
//
//
//
//
//
//	//***********************Load neutral obj
//
//	std::vector<unsigned short> indices;
//	std::vector<glm::vec3> indexed_vertices;
//	std::vector<glm::vec2> indexed_uvs;
//	std::vector<glm::vec3> indexed_normals;
//
//	indices = indicesAll[neutralShapeIdx];
//	indexed_vertices = indexed_verticesAll[neutralShapeIdx];
//	indexed_uvs = indexed_uvsAll[neutralShapeIdx];
//	indexed_normals = indexed_normalsAll[neutralShapeIdx];
//
//
//	GLuint vertexbuffer;
//	glGenBuffers(1, &vertexbuffer);
//	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//	glBufferData(GL_ARRAY_BUFFER, indexed_vertices.size() * sizeof(glm::vec3), &indexed_vertices[0], GL_STATIC_DRAW);
//
//	GLuint uvbuffer;
//	glGenBuffers(1, &uvbuffer);
//	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
//	glBufferData(GL_ARRAY_BUFFER, indexed_uvs.size() * sizeof(glm::vec2), &indexed_uvs[0], GL_STATIC_DRAW);
//
//	GLuint normalbuffer;
//	glGenBuffers(1, &normalbuffer);
//	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
//	glBufferData(GL_ARRAY_BUFFER, indexed_normals.size() * sizeof(glm::vec3), &indexed_normals[0], GL_STATIC_DRAW);
//
//	// Generate a buffer for the indices as well
//	GLuint elementbuffer;
//	glGenBuffers(1, &elementbuffer);
//	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
//	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned short), &indices[0], GL_STATIC_DRAW);
//
//	// Get a handle for our "LightPosition" uniform
//	glUseProgram(programID);
//	GLuint LightID = glGetUniformLocation(programID, "LightPosition_worldspace");
//
//	// For speed computation
//	double lastTime = glfwGetTime();
//	int nbFrames = 0;
//
//	int count = 0;
//	int count2 = 0;
//
//	int markerCount = 0;
//	int markerCount2 = 0;
//
//	int showAlignedMarkersCounter = 0;
//
//	int frameCounter = 0;
//
//	do {
//
//		// Measure speed
//		double currentTime = glfwGetTime();
//		nbFrames++;
//		if (currentTime - lastTime >= 1.0) { // If last prinf() was more than 1sec ago
//											 // printf and reset
//			printf("%f ms/frame\n", 1000.0 / double(nbFrames));
//			nbFrames = 0;
//			lastTime += 1.0;
//		}
//
//		// Clear the screen
//		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//		// Use our shader
//		glUseProgram(programID);
//
//		// Compute the MVP matrix from keyboard and mouse input
//		computeMatricesFromInputs();
//		glm::mat4 ProjectionMatrix = getProjectionMatrix();
//		glm::mat4 ViewMatrix = getViewMatrix();
//		glm::mat4 ModelMatrix = glm::mat4(1.0);
//		glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
//
//		// Send our transformation to the currently bound shader, 
//		// in the "MVP" uniform
//		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
//		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
//		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
//
//		glm::vec3 lightPos = glm::vec3(0, 200, 160);
//		//glm::vec3 lightPos = glm::vec3(4, 4, 4);
//		glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);
//
//		// Bind our texture in Texture Unit 0
//		glActiveTexture(GL_TEXTURE0);
//		glBindTexture(GL_TEXTURE_2D, Texture);
//		// Set our "myTextureSampler" sampler to use Texture Unit 0
//		glUniform1i(TextureID, 0);
//
//		//*********************** NEUTRAL POSE
//		//float weight = (count / 40.0f);
//		float weight = 1;
//		int poseIdx = 0;
//		Eigen::MatrixXd newPose = shapesOriginalVerticesEigen[neutralShapeIdx] + shapesOffsets[poseIdx] * weight;
//
//		std::vector<glm::vec3> indexed_vertices_new;
//		for (int vrtxIdx = 0; vrtxIdx < numVertices; vrtxIdx++) {
//			//glm::vec3 tempVrtx;
//			indexed_vertices[vrtxIdx] = glm::vec3(newPose(vrtxIdx, 0) - 70, newPose(vrtxIdx, 1), newPose(vrtxIdx, 2));
//		}
//
//		if (count == 40) {
//
//			count2++;
//			if (count2 > shapesOriginalVerticesEigen.size())
//				count2 = 0;
//			count = 0;
//		}
//		//count++;
//
//		// 1rst attribute buffer : vertices
//		glEnableVertexAttribArray(0);
//		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//		glBufferData(GL_ARRAY_BUFFER, indexed_vertices.size() * sizeof(glm::vec3), &indexed_vertices[0], GL_STATIC_DRAW);
//
//		glVertexAttribPointer(
//			0,                  // attribute
//			3,                  // size
//			GL_FLOAT,           // type
//			GL_FALSE,           // normalized?
//			0,                  // stride
//			(void*)0            // array buffer offset
//		);
//
//		// 2nd attribute buffer : UVs
//		glEnableVertexAttribArray(1);
//		glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
//		glVertexAttribPointer(
//			1,                                // attribute
//			2,                                // size
//			GL_FLOAT,                         // type
//			GL_FALSE,                         // normalized?
//			0,                                // stride
//			(void*)0                          // array buffer offset
//		);
//
//		// 3rd attribute buffer : normals
//		glEnableVertexAttribArray(2);
//		glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
//		glVertexAttribPointer(
//			2,                                // attribute
//			3,                                // size
//			GL_FLOAT,                         // type
//			GL_FALSE,                         // normalized?
//			0,                                // stride
//			(void*)0                          // array buffer offset
//		);
//
//		// Index buffer
//		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
//
//		// Draw the triangles !
//		glDrawElements(
//			GL_TRIANGLES,      // mode
//			indices.size(),    // count
//			GL_UNSIGNED_SHORT,   // type
//			(void*)0           // element array buffer offset
//		);
//
//
//
//
//		markerCount2 = neutralShapeMarkers.rows();
//		if (markerCount == 40) {
//
//			markerCount2++;
//			if (markerCount2 > neutralShapeMarkers.rows())
//				markerCount2 = neutralShapeMarkers.rows();
//			markerCount = 0;
//		}
//		markerCount++;
//
//
//
//		//*********************** Solved Pose
//		////////float weight = (count / 40.0f);
//		//////
//		//Calc final pose
//		Eigen::MatrixXd solvedPose = shapesOriginalVerticesEigen[neutralShapeIdx];
//		for (int shapeIdx = 0; shapeIdx < shapesOriginalVerticesEigen.size(); shapeIdx++) {
//			solvedPose = solvedPose + Allweights(frameCounter, shapeIdx) * shapesOffsets[shapeIdx];
//		}
//
//		//std::vector<glm::vec3> indexed_vertices_new;
//		for (int vrtxIdx = 0; vrtxIdx < numVertices; vrtxIdx++) {
//			//glm::vec3 tempVrtx;
//			indexed_vertices[vrtxIdx] = glm::vec3(solvedPose(vrtxIdx, 0) + 70, solvedPose(vrtxIdx, 1), solvedPose(vrtxIdx, 2));
//		}
//
//		if (count == 40) {
//
//			count2++;
//			if (count2 > shapesOriginalVerticesEigen.size())
//				count2 = 0;
//			count = 0;
//		}
//		//count++;
//
//		// 1rst attribute buffer : vertices
//		glEnableVertexAttribArray(0);
//		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//		glBufferData(GL_ARRAY_BUFFER, indexed_vertices.size() * sizeof(glm::vec3), &indexed_vertices[0], GL_STATIC_DRAW);
//
//		glVertexAttribPointer(
//			0,                  // attribute
//			3,                  // size
//			GL_FLOAT,           // type
//			GL_FALSE,           // normalized?
//			0,                  // stride
//			(void*)0            // array buffer offset
//		);
//
//		// 2nd attribute buffer : UVs
//		glEnableVertexAttribArray(1);
//		glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
//		glVertexAttribPointer(
//			1,                                // attribute
//			2,                                // size
//			GL_FLOAT,                         // type
//			GL_FALSE,                         // normalized?
//			0,                                // stride
//			(void*)0                          // array buffer offset
//		);
//
//		// 3rd attribute buffer : normals
//		glEnableVertexAttribArray(2);
//		glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
//		glVertexAttribPointer(
//			2,                                // attribute
//			3,                                // size
//			GL_FLOAT,                         // type
//			GL_FALSE,                         // normalized?
//			0,                                // stride
//			(void*)0                          // array buffer offset
//		);
//
//		// Index buffer
//		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
//
//		// Draw the triangles !
//		glDrawElements(
//			GL_TRIANGLES,      // mode
//			indices.size(),    // count
//			GL_UNSIGNED_SHORT,   // type
//			(void*)0           // element array buffer offset
//		);
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//		//***********************Draw marker
//		//for (int markerIdx = 0; markerIdx < neutralShapeMarkers.size(); markerIdx++) {
//		//////for (int markerIdx = 0; markerIdx < markerCount2; markerIdx++) {
//		//////	
//		//////	glm::mat4 ModelMatrix2 = glm::mat4(1.0);
//		//////	ModelMatrix2 = glm::translate(ModelMatrix2, glm::vec3(neutralShapeMarkers[markerIdx](0), neutralShapeMarkers[markerIdx](1), neutralShapeMarkers[markerIdx](2)));
//		//////	glm::mat4 MVP2 = ProjectionMatrix * ViewMatrix * ModelMatrix2;
//
//		//////	// Send our transformation to the currently bound shader, 
//		//////	// in the "MVP" uniform
//		//////	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP2[0][0]);
//		//////	glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix2[0][0]);
//
//
//		//////	// The rest is exactly the same as the first object
//
//		//////	// 1rst attribute buffer : vertices
//		//////	glEnableVertexAttribArray(0);
//		//////	glBindBuffer(GL_ARRAY_BUFFER, vertexbufferM);
//		//////	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//		//////	// 2nd attribute buffer : UVs
//		//////	glEnableVertexAttribArray(1);
//		//////	glBindBuffer(GL_ARRAY_BUFFER, uvbufferM);
//		//////	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//		//////	// 3rd attribute buffer : normals
//		//////	glEnableVertexAttribArray(2);
//		//////	glBindBuffer(GL_ARRAY_BUFFER, normalbufferM);
//		//////	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//		//////	// Index buffer
//		//////	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbufferM);
//
//		//////	// Draw the triangles !
//		//////	glDrawElements(GL_TRIANGLES, markerIndices.size(), GL_UNSIGNED_SHORT, (void*)0);
//
//		//////}
//
//		//showAlignedMarkersCounter++;
//		//if (showAlignedMarkersCounter > 800) {
//		//	//***********************Draw Aligned marker
//		//	//for (int markerIdx = 0; markerIdx < neutralShapeMarkers.size(); markerIdx++) {
//		//	for (int markerIdx = 0; markerIdx < markerCount2; markerIdx++) {
//
//		//		glm::mat4 ModelMatrix2 = glm::mat4(1.0);
//		//		ModelMatrix2 = glm::translate(ModelMatrix2, glm::vec3(alignedc3dMarkers(markerIdx,0), alignedc3dMarkers(markerIdx, 1), alignedc3dMarkers(markerIdx, 2)));
//		//		glm::mat4 MVP2 = ProjectionMatrix * ViewMatrix * ModelMatrix2;
//
//		//		// Send our transformation to the currently bound shader, 
//		//		// in the "MVP" uniform
//		//		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP2[0][0]);
//		//		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix2[0][0]);
//
//
//		//		// The rest is exactly the same as the first object
//
//		//		// 1rst attribute buffer : vertices
//		//		glEnableVertexAttribArray(0);
//		//		glBindBuffer(GL_ARRAY_BUFFER, vertexbufferM);
//		//		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//		//		// 2nd attribute buffer : UVs
//		//		glEnableVertexAttribArray(1);
//		//		glBindBuffer(GL_ARRAY_BUFFER, uvbufferM);
//		//		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//		//		// 3rd attribute buffer : normals
//		//		glEnableVertexAttribArray(2);
//		//		glBindBuffer(GL_ARRAY_BUFFER, normalbufferM);
//		//		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//		//		// Index buffer
//		//		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbufferM);
//
//		//		// Draw the triangles !
//		//		glDrawElements(GL_TRIANGLES, markerIndices.size(), GL_UNSIGNED_SHORT, (void*)0);
//
//		//	}
//
//		//}
//
//		frameCounter++;
//		if (frameCounter > numFrames)
//			frameCounter = 0;
//		//frameCounter = 0;
//
//		//Show c3d Aligned
//
//		//***********************Draw Aligned marker
//		//for (int markerIdx = 0; markerIdx < neutralShapeMarkers.size(); markerIdx++) {
//
//		for (int markerIdx = 0; markerIdx < markerCount2; markerIdx++) {
//
//			glm::mat4 ModelMatrix2 = glm::mat4(1.0);
//			//	Eigen::MatrixXd
//			ModelMatrix2 = glm::translate(ModelMatrix2, glm::vec3(neutralPoseC3Dmarkers(markerIdx, 0) + alignedAndReduced_c3dMarkers[frameCounter](markerIdx, 0) + 70,
//				neutralPoseC3Dmarkers(markerIdx, 1) + alignedAndReduced_c3dMarkers[frameCounter](markerIdx, 1),
//				neutralPoseC3Dmarkers(markerIdx, 2) + alignedAndReduced_c3dMarkers[frameCounter](markerIdx, 2)));
//			glm::mat4 MVP2 = ProjectionMatrix * ViewMatrix * ModelMatrix2;
//
//			//	 Send our transformation to the currently bound shader, 
//			//	 in the "MVP" uniform
//			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP2[0][0]);
//			glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix2[0][0]);
//
//
//			//	 The rest is exactly the same as the first object
//
//			//	 1rst attribute buffer : vertices
//			glEnableVertexAttribArray(0);
//			glBindBuffer(GL_ARRAY_BUFFER, vertexbufferM);
//			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//			// 2nd attribute buffer : UVs
//			glEnableVertexAttribArray(1);
//			glBindBuffer(GL_ARRAY_BUFFER, uvbufferM);
//			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//			// 3rd attribute buffer : normals
//			glEnableVertexAttribArray(2);
//			glBindBuffer(GL_ARRAY_BUFFER, normalbufferM);
//			glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
//
//			// Index buffer
//			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbufferM);
//
//			// Draw the triangles !
//			glDrawElements(GL_TRIANGLES, markerIndices.size(), GL_UNSIGNED_SHORT, (void*)0);
//
//		}
//
//
//
//
//
//		glDisableVertexAttribArray(0);
//		glDisableVertexAttribArray(1);
//		glDisableVertexAttribArray(2);
//
//		// Swap buffers
//		glfwSwapBuffers(window);
//		glfwPollEvents();
//
//	} // Check if the ESC key was pressed or the window was closed
//	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
//		glfwWindowShouldClose(window) == 0);
//
//	// Cleanup VBO and shader
//	glDeleteBuffers(1, &vertexbuffer);
//	glDeleteBuffers(1, &uvbuffer);
//	glDeleteBuffers(1, &normalbuffer);
//	glDeleteBuffers(1, &elementbuffer);
//	glDeleteProgram(programID);
//	glDeleteTextures(1, &Texture);
//	glDeleteVertexArrays(1, &VertexArrayID);
//
//	// Close OpenGL window and terminate GLFW
//	glfwTerminate();
//
//	return 0;
//}

std::string SolverCmd::openFileDialog()
{
	wchar_t filename[MAX_PATH];
	std::string str;

	OPENFILENAME ofn;
	ZeroMemory(&filename, sizeof(filename));
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;  // If you have a window to center over, put its HANDLE here
	//ofn.lpstrFilter = _T("Text Files\0*.obj\0Any File\0*.*\0");
	ofn.lpstrFile = filename;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrTitle = _T("Select a File, yo!");
	ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST;

	if (GetOpenFileName(&ofn))
	{
		std::cout << "You chose the file \"" << filename << "\"\n";
		std::wstring ws(filename);
		std::string wstr(ws.begin(), ws.end());

		str = wstr;
	}
	else
	{
		// All this stuff below is to tell you exactly how you messed up above. 
		// Once you've got that fixed, you can often (not always!) reduce it to a 'user cancelled' assumption.
		switch (CommDlgExtendedError())
		{
		case CDERR_DIALOGFAILURE: cout << "CDERR_DIALOGFAILURE\n";   break;
		case CDERR_FINDRESFAILURE: cout << "CDERR_FINDRESFAILURE\n";  break;
		case CDERR_INITIALIZATION: cout << "CDERR_INITIALIZATION\n";  break;
		case CDERR_LOADRESFAILURE: cout << "CDERR_LOADRESFAILURE\n";  break;
		case CDERR_LOADSTRFAILURE: cout << "CDERR_LOADSTRFAILURE\n";  break;
		case CDERR_LOCKRESFAILURE: cout << "CDERR_LOCKRESFAILURE\n";  break;
		case CDERR_MEMALLOCFAILURE: cout << "CDERR_MEMALLOCFAILURE\n"; break;
		case CDERR_MEMLOCKFAILURE: cout << "CDERR_MEMLOCKFAILURE\n";  break;
		case CDERR_NOHINSTANCE: cout << "CDERR_NOHINSTANCE\n";     break;
		case CDERR_NOHOOK: cout << "CDERR_NOHOOK\n";          break;
		case CDERR_NOTEMPLATE: cout << "CDERR_NOTEMPLATE\n";      break;
		case CDERR_STRUCTSIZE: cout << "CDERR_STRUCTSIZE\n";      break;
		case FNERR_BUFFERTOOSMALL: cout << "FNERR_BUFFERTOOSMALL\n";  break;
		case FNERR_INVALIDFILENAME: cout << "FNERR_INVALIDFILENAME\n"; break;
		case FNERR_SUBCLASSFAILURE: cout << "FNERR_SUBCLASSFAILURE\n"; break;
		default: cout << "You cancelled.\n";
		}
	}

	return str;
}

