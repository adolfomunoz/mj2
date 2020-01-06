#pragma once

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include <exception>

class AssimpException : public std::exception {
	std::string w;
public:
	AssimpException(const std::string& w) : w(w) {}
	const char* what() const noexcept override {	return w.c_str(); }
};	

void import_assimp(const std::string& pFile)
{
  // Create an instance of the Importer class
  Assimp::Importer importer;
  // And have it read the given file with some example postprocessing
  // Usually - if speed is not the most important aspect for you - you'll
  // probably to request more postprocessing than we do in this example.
  const aiScene* scene = importer.ReadFile( pFile,
        aiProcess_CalcTangentSpace       |
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);
  // If the import failed, report it
  if( !scene) throw(AssimpException(importer.GetErrorString()));

  std::cout<<pFile<<" contains "<<scene->mNumMeshes<<" meshes, "<<scene->mNumCameras<<" cameras and "<<scene->mNumLights<<" lights."<<std::endl;
  // We're done. Everything will be cleaned up by the importer destructor
}