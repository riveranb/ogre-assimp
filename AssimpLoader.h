/*
-----------------------------------------------------------------------------
This source file is part of
                                    _
  ___   __ _ _ __ ___  __ _ ___ ___(_)_ __ ___  _ __
 / _ \ / _` | '__/ _ \/ _` / __/ __| | '_ ` _ \| '_ \
| (_) | (_| | | |  __/ (_| \__ \__ \ | | | | | | |_) |
 \___/ \__, |_|  \___|\__,_|___/___/_|_| |_| |_| .__/
       |___/                                   |_|

For the latest info, see https://bitbucket.org/jacmoe/ogreassimp

Copyright (c) 2011 Jacob 'jacmoe' Moen

Licensed under the MIT license:

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#ifndef _AssimpLoader_H_
#define _AssimpLoader_H_

#include "Ogre.h"

// assimp
#include "assimp/Importer.hpp"
#include "assimp/matrix4x4.h"

struct aiScene;
struct aiNode;
struct aiMesh;
struct aiMaterial;
struct aiBone;
struct aiAnimation;

namespace Demo
{
namespace assimp
{

	//TODO: only need a bool ?
	struct Bonenode
	{
		aiNode* node;
		aiNode* parent;
		bool isNeeded;
	};

	class AssimpLoader
	{
	public:

		struct AssetOptions
		{
			bool usePercent;
			bool quietMode;

			Ogre::String source;
			Ogre::String dest;
			Ogre::String customAnimationName;

			int params;
			Ogre::Real animationSpeedModifier;
			
			size_t lodFixed;
			unsigned short numLods;
			Ogre::Real lodValue;
			Ogre::Real lodPercent;
			Ogre::String lodStrategy;

			AssetOptions()
			{
				source = "";
				dest = "";
				quietMode = false;
				customAnimationName = "";
				params = LP_GENERATE_SINGLE_MESH | LP_GENERATE_MATERIALS_AS_CODE;
				animationSpeedModifier = 1.0;
				numLods = 0;
				lodValue = 250000;
				lodStrategy = "Distance";
				lodPercent = 20;
				lodFixed = 0;
				usePercent = true;
			};
		};

		enum LoaderParams
		{
			LP_GENERATE_SINGLE_MESH = 1 << 0,

			// See the two possible methods for material gneration
			LP_GENERATE_MATERIALS_AS_CODE = 1 << 1,

			// 3ds max exports the animation over a longer time frame than the animation actually plays for
			// this is a fix for that
			LP_CUT_ANIMATION_WHERE_NO_FURTHER_CHANGE = 1 << 2,

			// when 3ds max exports as DAE it gets some of the transforms wrong, get around this by using
			// this option and a prior run with of the model exported as ASE
			LP_USE_LAST_RUN_NODE_DERIVED_TRANSFORMS = 1 << 3,

			// Quiet mode - don't output anything
			LP_QUIET_MODE = 1 << 4,

			// Create simple shader programs if LP_GENERATE_MATERIALS_AS_CODE is used
			LP_GENERATE_SHADER_MATERIALS = 1 << 5
		};

		AssimpLoader();
		virtual ~AssimpLoader();

		bool convertV2(const AssetOptions &options, Ogre::MeshPtr &pmesh, int quality);

		const Ogre::String& getBasename() { return mBasename; }

	private:
		static int _msBoneCount;

		const aiScene* readingAsset(const AssetOptions & opts, int quality);

		bool genSubMeshV2(const Ogre::String& name, int index, const aiNode* node, const aiMesh* mesh, const aiMaterial* mat,
			Ogre::MeshPtr pmeshv2);
		void genMeshDataFromNodeV2(const aiScene* scene, const aiNode* node);

		Ogre::MaterialPtr createMaterial(int index, const aiMaterial* mat, const Ogre::String& dir);
		Ogre::MaterialPtr createMaterialByScript(int index, const aiMaterial* mat);
		void grabNodeNamesFromNode(const aiScene* scene, const aiNode* node);
		void grabBoneNamesFromNode(const aiScene* scene, const aiNode* node);
		void computeNodesDerivedTransform(const aiScene* scene, const aiNode *node, const aiMatrix4x4 accTransf);
		void createBonesFromNode(const aiScene* scene, const aiNode* node);
		void createBoneHiearchy(const aiScene* scene, const aiNode *node);
		void markAllChildNodesAsNeeded(const aiNode *node);
		void flagNodeAsNeeded(const char* name);
		bool isNodeNeeded(const char* name);
		void parseAnimation(const aiScene* scene, int index, aiAnimation* anim);
		
		bool mQuietMode;
		Ogre::Real mTicksPerSecond;
		Ogre::Real mAnimationSpeedModifier;

		int mLoaderParams;
		Ogre::String mBasename;
		Ogre::String mPath;
		Ogre::String mMaterialCode;
		Ogre::String mCustomAnimationName;

		typedef std::map<Ogre::String, Bonenode> mapBonenode;
		mapBonenode mBonenodes;

		typedef std::map<Ogre::String, const aiNode*> mapaiNode;
		mapaiNode mNodesByName;

		typedef std::map<Ogre::String, const aiBone*> mapaiBone;
		mapaiBone mBonesByName;

		typedef std::map<Ogre::String, aiMatrix4x4> mapaiMatrix4;
		mapaiMatrix4 mDerivedTransformsByName;

		typedef std::vector<Ogre::v1::MeshPtr> vecMeshptr1;
		vecMeshptr1 mMeshesV1;

		Ogre::v1::SkeletonPtr mSkeleton;

		Assimp::Importer mImportHandler;
	};

}
}
#endif // __AssimpLoader_h__
