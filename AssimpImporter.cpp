#include <iostream>

#include "AssimpImporter.h"
#include "AssimpLoader.h"

#include "OgreMesh.h"
#include "OgreMeshManager.h"
#include "OgreMesh2.h"
#include "OgreMeshManager2.h"

namespace Demo
{
namespace assimp
{

	Importer::Importer()
	{
	}

	bool Importer::createMeshV2(Ogre::MeshPtr & pdestmesh, const Ogre::String& meshname, const Ogre::String& filename, int quality)
	{
		// search if mesh is already created (by meshname)
		pdestmesh = Ogre::MeshManager::getSingleton().getByName(meshname);
		if (!pdestmesh.isNull())
		{
			return true;
		}

		// create one manual
		pdestmesh = Ogre::MeshManager::getSingleton().createManual(meshname, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

		AssimpLoader::AssetOptions opts;
		prepareLoadOptions(opts, filename);

		bool retCode = true;
		try
		{
			retCode = mLoader.convertV2(opts, pdestmesh, quality);
		}
		catch (Ogre::Exception& e)
		{
			Ogre::MeshManager::getSingleton().remove(pdestmesh); // remove generated
			pdestmesh.reset();

			std::cerr << "[Assimp] FATAL ERROR: " << e.getDescription() << std::endl;
			std::cerr << "[Assimp] ABORTING loading!" << std::endl;
			retCode = false;
		}

		return retCode;
	}

	void Importer::prepareLoadOptions(AssimpLoader::AssetOptions & opts, const Ogre::String& filename)
	{
		//////////////////options//////////////////////////
		opts.quietMode = false;
		opts.customAnimationName = "";
		opts.dest = "";
		opts.animationSpeedModifier = 1.0;
		opts.lodValue = 250000;
		opts.lodFixed = 0;
		opts.lodPercent = 20;
		opts.numLods = 0; // no level of detail now
		opts.usePercent = true;
		// ignore program name
		char* source = 0;
		char* dest = 0;

		opts.params = AssimpLoader::LP_GENERATE_SINGLE_MESH;
		opts.source = filename;
		dest = ".";

		opts.dest = dest;
		///////////////////////////////////////////////////
	}

}
}