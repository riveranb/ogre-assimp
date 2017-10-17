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

#include "AssimpLoader.h"

#include "Ogre.h"
// v1
#include "OgreMeshManager.h"
#include "OgreSubMesh.h"
#include "OgreOldBone.h"
#include "OgreSkeleton.h"
#include "OgreAnimation.h"
#include "OgreMesh2.h"
#include "OgreSubMesh2.h"

#include "assimp/Importer.hpp"
#include "assimp/DefaultLogger.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"

#include <tuple>

//#include <boost/tuple/tuple.hpp>
//#include <boost/algorithm/string.hpp>

namespace Demo
{
namespace assimp
{

static Ogre::v1::MeshPtr _sV1mesh;
static Ogre::MeshPtr _sV2mesh;

Ogre::String toString(const aiColor4D& colour)
{
    return Ogre::StringConverter::toString(Ogre::Real(colour.r)) + " " +
        Ogre::StringConverter::toString(Ogre::Real(colour.g)) + " " +
        Ogre::StringConverter::toString(Ogre::Real(colour.b)) + " " +
        Ogre::StringConverter::toString(Ogre::Real(colour.a));
}

int AssimpLoader::_msBoneCount = 0;

AssimpLoader::AssimpLoader()
{
}

AssimpLoader::~AssimpLoader()
{
}

const aiScene* AssimpLoader::readingAsset(const AssetOptions & opts, int quality)
{
	mAnimationSpeedModifier = opts.animationSpeedModifier;
	mLoaderParams = opts.params;
	mQuietMode = ((mLoaderParams & LP_QUIET_MODE) == 0) ? false : true;
	mCustomAnimationName = opts.customAnimationName;
	if ((mLoaderParams & LP_USE_LAST_RUN_NODE_DERIVED_TRANSFORMS) == false)
	{
		mDerivedTransformsByName.clear();
	}

	Ogre::String extension;
	Ogre::StringUtil::splitFullFilename(opts.source, mBasename, extension, mPath);
	mBasename = mBasename + "_" + extension;

	if (!opts.dest.empty())
	{
		mPath = opts.dest + "/";
	}

	Assimp::DefaultLogger::create("assimplogger.log", Assimp::Logger::VERBOSE);
	Assimp::DefaultLogger::get()->info("Loading asset ...");
	if (!mQuietMode)
	{
		Ogre::LogManager::getSingleton().logMessage("*** Loading asset file... ***");
		Ogre::LogManager::getSingleton().logMessage("Filename: " + opts.source);
	}

	const aiScene *scene;
	if (quality == 0)
	{
		scene = mImportHandler.ReadFile(opts.source.c_str(),
			aiProcess_TransformUVCoords |
			aiProcess_FlipUVs |
			aiProcess_OptimizeMeshes |
			aiProcess_JoinIdenticalVertices);
	}
	else
	{
		scene = mImportHandler.ReadFile(opts.source.c_str(),
			aiProcessPreset_TargetRealtime_Quality |
			aiProcess_TransformUVCoords |
			aiProcess_FlipUVs);
	}

	// If the import failed, report it
	if (!scene)
	{
		if (!mQuietMode)
		{
			Ogre::LogManager::getSingleton().logMessage("AssImp importer failed with the following message:");
			Ogre::LogManager::getSingleton().logMessage(mImportHandler.GetErrorString());
		}
	}

	return scene;
}

bool AssimpLoader::convertV2(const AssetOptions& options, Ogre::MeshPtr &pmesh, int quality)
{
	const aiScene* inscene = readingAsset(options, quality);
	if (!inscene)
	{
		return false;
	}

	grabNodeNamesFromNode(inscene, inscene->mRootNode);
	grabBoneNamesFromNode(inscene, inscene->mRootNode);
	computeNodesDerivedTransform(inscene, inscene->mRootNode, inscene->mRootNode->mTransformation);

	// TODO: consider corresponding bone/skeleton and animation data

	_sV2mesh = pmesh;
	genMeshDataFromNodeV2(inscene, inscene->mRootNode);

	if (!mQuietMode)
	{
		Ogre::LogManager::getSingleton().logMessage("*** Finished loading asset file ***");
	}
	Assimp::DefaultLogger::kill();

	// clean up
	mMaterialCode = "";
	mCustomAnimationName = "";
	mBonesByName.clear();
	mNodesByName.clear();
	mBonenodes.clear();
	mImportHandler.FreeScene();
	_sV2mesh.reset(); // release unnecessary pointer, or will crash at shutdown

	return true;
}

void AssimpLoader::genMeshDataFromNodeV2(const aiScene* scene, const aiNode *node)
{
	if (_sV2mesh.isNull())
	{
		// TODO: or create one here?
		return;
	}

	if (node->mNumMeshes > 0)
	{
		/*
		if (mLoaderParams & LP_GENERATE_SINGLE_MESH)
		{
		}
		*/

		for (unsigned int idx = 0; idx<node->mNumMeshes; ++idx)
		{
			aiMesh *pAIMesh = scene->mMeshes[node->mMeshes[idx]];
			if (!mQuietMode)
			{
				Ogre::LogManager::getSingleton().logMessage(
					"SubMesh " + Ogre::StringConverter::toString(idx) + " for mesh '" + Ogre::String(node->mName.data) + "'");
			}

			// Create a material instance for the mesh.
			const aiMaterial *pAIMaterial = scene->mMaterials[pAIMesh->mMaterialIndex];
			genSubMeshV2(node->mName.data, idx, node, pAIMesh, pAIMaterial, _sV2mesh);
		}
	}

	// Traverse all child nodes of the current node instance
	for (unsigned int childIdx = 0; childIdx<node->mNumChildren; childIdx++)
	{
		const aiNode *pChildNode = node->mChildren[childIdx];
		genMeshDataFromNodeV2(scene, pChildNode);
	}
}

bool AssimpLoader::genSubMeshV2(const Ogre::String& name, int index, const aiNode* node, const aiMesh* mesh, 
	const aiMaterial* mat, Ogre::MeshPtr pmeshv2)
{
	// if animated all submeshes must have bone weights
	if (mBonesByName.size() && !mesh->HasBones())
	{
		if (!mQuietMode)
		{
			Ogre::LogManager::getSingleton().logMessage("Skipping Mesh " + Ogre::String(mesh->mName.data) + "with no bone weights");
		}
		return false;
	}

	// TODO: Material for v1::Mesh conversion still have problems now.
	/*
	Ogre::MaterialPtr matptr;
	if((mLoaderParams & LP_GENERATE_MATERIALS_AS_CODE) == 0)
	{
	matptr = createMaterial(mesh->mMaterialIndex, mat, dir);
	}
	else
	{
	matptr = createMaterialByScript(mesh->mMaterialIndex, mat);
	}
	*/

	Ogre::VaoManager *vaoManager = Ogre::Root::getSingleton().getRenderSystem()->getVaoManager();
	// create vertex buffer
	unsigned short numreal = 3;
	unsigned short posBytes = 0, normBytes = 0, uvBytes = 0, colBytes = 0;
	posBytes = sizeof(Ogre::Real) * 3;
	Ogre::VertexElement2Vec velements;
	velements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_POSITION));
	if (mesh->HasNormals())
	{
		normBytes = sizeof(Ogre::Real) * 3;
		numreal += 3;
		velements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_NORMAL));
	}
	if (mesh->HasTextureCoords(0)) // currently consider first uv only
	{
		uvBytes = sizeof(Ogre::Real) * 2;
		numreal += 2;
		velements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES));
	}
	if (mesh->HasVertexColors(0)) // assume first color set as diffuse color
	{
		// TODO: can it be uint32 (RGBA)?
		colBytes = sizeof(Ogre::Real) * 3;
		numreal += 3;
		velements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_DIFFUSE));
	}

	Ogre::Real *vertexdata = reinterpret_cast<Ogre::Real *> (OGRE_MALLOC_SIMD(
								sizeof(Ogre::Real) * numreal * mesh->mNumVertices, Ogre::MEMCATEGORY_GEOMETRY));
	// fill vertexdata manually
	// prime pointers to vertex related data
	aiVector3D *vec = mesh->mVertices;
	aiVector3D *norm = mesh->mNormals;
	aiVector3D *uv = mesh->mTextureCoords[0];
	aiColor4D *col = mesh->mColors[0];
	Ogre::Aabb subAABB(Ogre::Vector3(vec[0].x, vec[0].y, vec[0].z), Ogre::Vector3::UNIT_SCALE);
#if defined _DEBUG && 0
	Ogre::LogManager::getSingleton().logMessage("  = Parsed vertuces (positions):");
#endif
	for (unsigned int i = 0, offset = 0; i < mesh->mNumVertices; ++i)
	{
		// position
		memcpy(vertexdata + offset, &vec[i].x, posBytes);
		//vertexdata[offset + 0] = vec[i].x;
		//vertexdata[offset + 1] = vec[i].y;
		//vertexdata[offset + 2] = vec[i].z;
		offset += 3;
		subAABB.merge(Ogre::Vector3(vec[i].x, vec[i].y, vec[i].z));
#if defined _DEBUG && 0
		Ogre::LogManager::getSingleton().logMessage(Ogre::StringConverter::toString(i) + ": " +
											Ogre::StringConverter::toString(vec[i].x) + ",  " +
											Ogre::StringConverter::toString(vec[i].y) + ",  " +
											Ogre::StringConverter::toString(vec[i].z));
#endif

		// normal
		if (mesh->HasNormals())
		{
			memcpy(vertexdata + offset, &norm[i].x, normBytes);
			//vertexdata[offset + 0] = norm[i].x;
			//vertexdata[offset + 1] = norm[i].y;
			//vertexdata[offset + 2] = norm[i].z;
			offset += 3;
		}
		// uv
		if (mesh->HasTextureCoords(0))
		{
			memcpy(vertexdata + offset, &uv[i].x, uvBytes);
			//vertexdata[offset + 0] = uv[i].x;
			//vertexdata[offset + 1] = uv[i].y;
			offset += 2;
		}
		// color
		if (mesh->HasVertexColors(0))
		{
			memcpy(vertexdata + offset, &col[i].r, colBytes);
			//vertex[offset + 0] = col[i].r;
			//vertex[offset + 1] = col[i].g;
			//vertex[offset + 2] = col[i].b;
			offset += 3;
		}
	}

	Ogre::VertexBufferPacked *vertexBuffer = 0;
	try
	{
		//Create the actual vertex buffer.
		vertexBuffer = vaoManager->createVertexBuffer(velements, mesh->mNumVertices,
			Ogre::BT_IMMUTABLE,
			vertexdata, false);
	}
	catch (Ogre::Exception &e)
	{
		// When keepAsShadow = true, the memory will be freed when the index buffer is destroyed.
		// However if for some weird reason there is an exception raised, the memory will
		// not be freed, so it is up to us to do so.
		// The reasons for exceptions are very rare. But we're doing this for correctness.
		OGRE_FREE_SIMD(vertexBuffer, Ogre::MEMCATEGORY_GEOMETRY);
		vertexBuffer = 0;
		throw e;
	}
	//We'll just use one vertex buffer source (multi-source not working yet)
	Ogre::VertexBufferPackedVec vertexBuffers;
	vertexBuffers.push_back(vertexBuffer);

	// currently consider 16-bit indices array only
	Ogre::uint16 * indexdata = reinterpret_cast<Ogre::uint16 *>(OGRE_MALLOC_SIMD(
									sizeof(Ogre::uint16) * 3 * mesh->mNumFaces, Ogre::MEMCATEGORY_GEOMETRY));
	// fill indexdata manually
#if defined _DEBUG && 0
	Ogre::LogManager::getSingleton().logMessage("  = Parsed indices:");
#endif
	for (unsigned int i = 0, offset = 0; i < mesh->mNumFaces; ++i)
	{
		indexdata[offset + 0] = (Ogre::uint16)mesh->mFaces[i].mIndices[0];
		indexdata[offset + 1] = (Ogre::uint16)mesh->mFaces[i].mIndices[1];
		indexdata[offset + 2] = (Ogre::uint16)mesh->mFaces[i].mIndices[2];
#if defined _DEBUG && 0
		Ogre::LogManager::getSingleton().logMessage(Ogre::StringConverter::toString(i) + ": " +
											Ogre::StringConverter::toString(mesh->mFaces[i].mIndices[0]) + ", " +
											Ogre::StringConverter::toString(mesh->mFaces[i].mIndices[1]) + ", " +
											Ogre::StringConverter::toString(mesh->mFaces[i].mIndices[2]));
#endif

		offset += 3;
	}

	Ogre::IndexBufferPacked *indexBuffer = 0;
	try
	{
		indexBuffer = vaoManager->createIndexBuffer(Ogre::IndexBufferPacked::IT_16BIT,
			sizeof(Ogre::uint16) * 3 * mesh->mNumFaces,
			Ogre::BT_IMMUTABLE,
			indexdata, false);
	}
	catch (Ogre::Exception &e)
	{
		// When keepAsShadow = true, the memory will be freed when the index buffer is destroyed.
		// However if for some weird reason there is an exception raised, the memory will
		// not be freed, so it is up to us to do so.
		// The reasons for exceptions are very rare. But we're doing this for correctness.
		OGRE_FREE_SIMD(indexBuffer, Ogre::MEMCATEGORY_GEOMETRY);
		indexBuffer = 0;
		throw e;
	}
	
	Ogre::SubMesh * submesh = pmeshv2->createSubMesh(index);
	Ogre::VertexArrayObject *vao = vaoManager->createVertexArrayObject(
		vertexBuffers, indexBuffer, Ogre::OT_TRIANGLE_LIST);

	//Each Vao pushed to the vector refers to an LOD level.
	//Must be in sync with mesh->mLodValues & mesh->mNumLods if you use more than one level
	submesh->mVao[Ogre::VpNormal].push_back(vao);
	//Use the same geometry for shadow casting.
	submesh->mVao[Ogre::VpShadow].push_back(vao);

	// AABB
	if (index > 0) // not first
	{
		subAABB.merge(pmeshv2->getAabb());
	}
	pmeshv2->_setBounds(subAABB);
	pmeshv2->_setBoundingSphereRadius(subAABB.getRadius());

	return true;
}

//typedef boost::tuple< aiVectorKey*, aiQuatKey*, aiVectorKey* > KeyframeData
typedef std::tuple<aiVectorKey*, aiQuatKey*, aiVectorKey*> tpKeyframeData;
typedef std::map< Ogre::Real, tpKeyframeData > mapKeyframes;

template <int v>
struct Int2Type
{
    enum { value = v };
};

// T should be a Loki::Int2Type<>
template< typename T > void GetInterpolationIterators(mapKeyframes& keyframes, mapKeyframes::iterator it, mapKeyframes::reverse_iterator& front, 
	mapKeyframes::iterator& back)
{
    front = mapKeyframes::reverse_iterator(it);

    front++;
    for(front; front != keyframes.rend(); front++)
    {
        if(std::get< T::value >(front->second) != NULL)
        {
            break;
        }
    }

    back = it;
    back++;
    for(back; back != keyframes.end(); back++)
    {
        if(std::get< T::value >(back->second) != NULL)
        {
            break;
        }
    }
}

aiVector3D getTranslate(aiNodeAnim* nodeanim, mapKeyframes& keyframes, mapKeyframes::iterator it, Ogre::Real ticksPerSecond)
{
    aiVectorKey* translateKey = std::get<0>(it->second);
    aiVector3D vect;
    if(translateKey)
    {
        vect = translateKey->mValue;
    }
    else
    {
        mapKeyframes::reverse_iterator front;
        mapKeyframes::iterator back;

        GetInterpolationIterators< Int2Type<0> > (keyframes, it, front, back);

        mapKeyframes::reverse_iterator rend = keyframes.rend();
        mapKeyframes::iterator end = keyframes.end();
        aiVectorKey* frontKey = NULL;
        aiVectorKey* backKey = NULL;

        if(front != rend)
            frontKey = std::get<0>(front->second);

        if(back != end)
            backKey = std::get<0>(back->second);

        // got 2 keys can interpolate
        if(frontKey && backKey)
        {
            float prop = (float)(((double)it->first - frontKey->mTime) / (backKey->mTime - frontKey->mTime));
            prop /= ticksPerSecond;
            vect = ((backKey->mValue - frontKey->mValue) * prop) + frontKey->mValue;
        }

        else if(frontKey)
        {
            vect = frontKey->mValue;
        }
        else if(backKey)
        {
            vect = backKey->mValue;
        }
    }

    return vect;
}

aiQuaternion getRotate(aiNodeAnim* nodeanim, mapKeyframes& keyframes, mapKeyframes::iterator it, Ogre::Real ticksPerSecond)
{
    aiQuatKey* rotationKey = std::get<1>(it->second);
    aiQuaternion rot;
    if(rotationKey)
    {
        rot = rotationKey->mValue;
    }
    else
    {
		mapKeyframes::reverse_iterator front;
		mapKeyframes::iterator back;

        GetInterpolationIterators< Int2Type<1> > (keyframes, it, front, back);

		mapKeyframes::reverse_iterator rend = keyframes.rend();
		mapKeyframes::iterator end = keyframes.end();
        aiQuatKey* frontKey = NULL;
        aiQuatKey* backKey = NULL;

        if(front != rend)
            frontKey = std::get<1>(front->second);

        if(back != end)
            backKey = std::get<1>(back->second);

        // got 2 keys can interpolate
        if(frontKey && backKey)
        {
            float prop = (float)(((double)it->first - frontKey->mTime) / (backKey->mTime - frontKey->mTime));
            prop /= ticksPerSecond;
            aiQuaternion::Interpolate(rot, frontKey->mValue, backKey->mValue, prop);
        }

        else if(frontKey)
        {
            rot = frontKey->mValue;
        }
        else if(backKey)
        {
            rot = backKey->mValue;
        }
    }

    return rot;
}

// TODO: animation need test and debug.
void AssimpLoader::parseAnimation (const aiScene* scene, int index, aiAnimation* anim)
{
    // DefBonePose a matrix that represents the local bone transform (can build from Ogre bone components)
    // PoseToKey a matrix representing the keyframe translation
    // What assimp stores aiNodeAnim IS the decomposed form of the transform (DefBonePose * PoseToKey)
    // To get PoseToKey which is what Ogre needs we'ed have to build the transform from components in
    // aiNodeAnim and then DefBonePose.Inverse() * aiNodeAnim(generated transform) will be the right transform

    Ogre::String animName;
    if(mCustomAnimationName != "")
    {
        animName = mCustomAnimationName;
        if(index >= 1)
        {
            animName += Ogre::StringConverter::toString(index);
        }
    }
    else
    {
        animName = Ogre::String(anim->mName.data);
    }
    if(animName.length() < 1)
    {
        animName = "Animation" + Ogre::StringConverter::toString(index);
    }

    if(!mQuietMode)
    {
        Ogre::LogManager::getSingleton().logMessage("Animation name = '" + animName + "'");
        Ogre::LogManager::getSingleton().logMessage("duration = " + Ogre::StringConverter::toString(Ogre::Real(anim->mDuration)));
        Ogre::LogManager::getSingleton().logMessage("tick/sec = " + Ogre::StringConverter::toString(Ogre::Real(anim->mTicksPerSecond)));
        Ogre::LogManager::getSingleton().logMessage("channels = " + Ogre::StringConverter::toString(anim->mNumChannels));
    }
    Ogre::v1::Animation* animation;
    mTicksPerSecond = (Ogre::Real)((0 == anim->mTicksPerSecond) ? 24 : anim->mTicksPerSecond);
    mTicksPerSecond *= mAnimationSpeedModifier;

    Ogre::Real cutTime = 0.0;
    if(mLoaderParams & LP_CUT_ANIMATION_WHERE_NO_FURTHER_CHANGE)
    {
        for (int i = 1; i < (int)anim->mNumChannels; i++)
        {
            aiNodeAnim* node_anim = anim->mChannels[i];

            // times of the equality check
            Ogre::Real timePos = 0.0;
            Ogre::Real timeRot = 0.0;

            for(unsigned int i = 1; i < node_anim->mNumPositionKeys; i++)
            {
                if( node_anim->mPositionKeys[i] != node_anim->mPositionKeys[i-1])
                {
                    timePos = (Ogre::Real)node_anim->mPositionKeys[i].mTime;
                    timePos /= mTicksPerSecond;
                }
            }

            for(unsigned int i = 1; i < node_anim->mNumRotationKeys; i++)
            {
                if( node_anim->mRotationKeys[i] != node_anim->mRotationKeys[i-1])
                {
                    timeRot = (Ogre::Real)node_anim->mRotationKeys[i].mTime;
                    timeRot /= mTicksPerSecond;
                }
            }

            if(timePos > cutTime){ cutTime = timePos; }
            if(timeRot > cutTime){ cutTime = timeRot; }
        }

        animation = mSkeleton->createAnimation(Ogre::String(animName), cutTime);
    }
    else
    {
        cutTime = Ogre::Math::POS_INFINITY;
        animation = mSkeleton->createAnimation(Ogre::String(animName), Ogre::Real(anim->mDuration/mTicksPerSecond));
    }

    animation->setInterpolationMode(Ogre::v1::Animation::IM_LINEAR); //FIXME: Is this always true?

    if(!mQuietMode)
    {
        Ogre::LogManager::getSingleton().logMessage("Cut Time " + Ogre::StringConverter::toString(cutTime));
    }

    for (int i = 0; i < (int)anim->mNumChannels; i++)
    {
        Ogre::v1::TransformKeyFrame* keyframe;

        aiNodeAnim* node_anim = anim->mChannels[i];
        if(!mQuietMode)
        {
            Ogre::LogManager::getSingleton().logMessage("Channel " + Ogre::StringConverter::toString(i));
            Ogre::LogManager::getSingleton().logMessage("affecting node: " + Ogre::String(node_anim->mNodeName.data));
            //Ogre::LogManager::getSingleton().logMessage("position keys: " + Ogre::StringConverter::toString(node_anim->mNumPositionKeys));
            //Ogre::LogManager::getSingleton().logMessage("rotation keys: " + Ogre::StringConverter::toString(node_anim->mNumRotationKeys));
            //Ogre::LogManager::getSingleton().logMessage("scaling keys: " + Ogre::StringConverter::toString(node_anim->mNumScalingKeys));
        }

        Ogre::String boneName = Ogre::String(node_anim->mNodeName.data);
        if(mSkeleton->hasBone(boneName))
        {
            Ogre::v1::OldBone* bone = mSkeleton->getBone(boneName);
            Ogre::Matrix4 defBonePoseInv;
            defBonePoseInv.makeInverseTransform(bone->getPosition(), bone->getScale(), bone->getOrientation());

			Ogre::v1::OldNodeAnimationTrack* oldtrack = animation->createOldNodeTrack(i, bone);

            // Ogre needs translate rotate and scale for each keyframe in the track
            mapKeyframes keyframes;
            for(unsigned int i = 0; i < node_anim->mNumPositionKeys; i++)
            {
                keyframes[ (Ogre::Real)node_anim->mPositionKeys[i].mTime / mTicksPerSecond ] = tpKeyframeData( &(node_anim->mPositionKeys[i]), NULL, NULL);
            }

            for(unsigned int i = 0; i < node_anim->mNumRotationKeys; i++)
            {
                mapKeyframes::iterator it = keyframes.find((Ogre::Real)node_anim->mRotationKeys[i].mTime / mTicksPerSecond);
                if(it != keyframes.end())
                {
                    std::get<1>(it->second) = &(node_anim->mRotationKeys[i]);
                }
                else
                {
                    keyframes[ (Ogre::Real)node_anim->mRotationKeys[i].mTime / mTicksPerSecond ] = tpKeyframeData( NULL, &(node_anim->mRotationKeys[i]), NULL );
                }
            }

            for(unsigned int i = 0; i < node_anim->mNumScalingKeys; i++)
            {
                mapKeyframes::iterator it = keyframes.find((Ogre::Real)node_anim->mScalingKeys[i].mTime / mTicksPerSecond);
                if(it != keyframes.end())
                {
                    std::get<2>(it->second) = &(node_anim->mScalingKeys[i]);
                }
                else
                {
                    keyframes[ (Ogre::Real)node_anim->mRotationKeys[i].mTime / mTicksPerSecond ] = tpKeyframeData( NULL, NULL, &(node_anim->mScalingKeys[i]) );
                }
            }

			mapKeyframes::iterator it = keyframes.begin();
			mapKeyframes::iterator it_end = keyframes.end();
            for(it; it != it_end; ++it)
            {
                if(it->first < cutTime)	// or should it be <=
                {
                    aiVector3D aiTrans = getTranslate( node_anim, keyframes, it, mTicksPerSecond);

                    Ogre::Vector3 trans(aiTrans.x, aiTrans.y, aiTrans.z);

                    aiQuaternion aiRot = getRotate(node_anim, keyframes, it, mTicksPerSecond);
                    Ogre::Quaternion rot(aiRot.w, aiRot.x, aiRot.y, aiRot.z);
                    Ogre::Vector3 scale(1,1,1);	// ignore scale for now

                    Ogre::Vector3 transCopy = trans;

                    Ogre::Matrix4 fullTransform;
                    fullTransform.makeTransform(trans, scale, rot);

                    Ogre::Matrix4 poseTokey = defBonePoseInv * fullTransform;
                    poseTokey.decomposition(trans, scale, rot);

                    keyframe = oldtrack->createNodeKeyFrame(Ogre::Real(it->first));

                    // weirdness with the root bone, But this seems to work
                    if(mSkeleton->getRootBone()->getName() == boneName)
                    {
                        trans = transCopy - bone->getPosition();
                    }

                    keyframe->setTranslate(trans);
                    keyframe->setRotation(rot);
                }
            }

        } // if bone exists

    } // loop through channels

    mSkeleton->optimiseAllAnimations();
}

void AssimpLoader::markAllChildNodesAsNeeded(const aiNode *node)
{
    flagNodeAsNeeded(node->mName.data);
    // Traverse all child nodes of the current node instance
    for ( unsigned int childIdx=0; childIdx<node->mNumChildren; ++childIdx )
    {
        const aiNode *pChildNode = node->mChildren[ childIdx ];
        markAllChildNodesAsNeeded(pChildNode);
    }
}

void AssimpLoader::grabNodeNamesFromNode(const aiScene* scene, const aiNode* node)
{
    Bonenode bNode;
    bNode.node = const_cast<aiNode*>(node);
    if(NULL != node->mParent)
    {
        bNode.parent = const_cast<aiNode*>(node->mParent);
    }
    bNode.isNeeded = false;
    mBonenodes.insert(std::pair<Ogre::String, Bonenode>(Ogre::String(node->mName.data), bNode));
    mNodesByName[node->mName.data] = node;
    if(!mQuietMode)
    {
        Ogre::LogManager::getSingleton().logMessage("Node " + Ogre::String(node->mName.data) + " found.");
    }

    // Traverse all child nodes of the current node instance
    for ( unsigned int childIdx=0; childIdx<node->mNumChildren; ++childIdx )
    {
        const aiNode *pChildNode = node->mChildren[ childIdx ];
        grabNodeNamesFromNode(scene, pChildNode);
    }
}

void AssimpLoader::computeNodesDerivedTransform(const aiScene* scene, const aiNode *node, const aiMatrix4x4 accTransf)
{
    if(mDerivedTransformsByName.find(node->mName.data) == mDerivedTransformsByName.end())
    {
		mDerivedTransformsByName[node->mName.data] = accTransf;
    }
    for ( unsigned int childIdx=0; childIdx<node->mNumChildren; ++childIdx )
    {
        const aiNode *pChildNode = node->mChildren[ childIdx ];
        computeNodesDerivedTransform(scene, pChildNode, accTransf * pChildNode->mTransformation);
    }
}

void AssimpLoader::createBonesFromNode(const aiScene* scene, const aiNode *node)
{
    if(isNodeNeeded(node->mName.data))
    {
        Ogre::v1::OldBone* bone = mSkeleton->createBone(Ogre::String(node->mName.data), _msBoneCount);

        aiQuaternion rot;
        aiVector3D pos;
        aiVector3D scale;

        /*
        aiMatrix4x4 aiM = mNodeDerivedTransformByName.find(pNode->mName.data)->second;
        const aiNode* parentNode = NULL;
        {
            boneMapType::iterator it = boneMap.find(pNode->mName.data);
            if(it != boneMap.end())
            {
                parentNode = it->second.parent;
            }
        }
        if(parentNode)
        {
            aiMatrix4x4 aiMParent = mNodeDerivedTransformByName.find(parentNode->mName.data)->second;
            aiM = aiMParent.Inverse() * aiM;
        }
        */

        // above should be the same as
        aiMatrix4x4 aiM = node->mTransformation;
        aiM.Decompose(scale, rot, pos);

        /*
        // debug render
        Ogre::SceneNode* sceneNode = NULL;
        if(parentNode)
        {
            Ogre::SceneNode* parent = static_cast<Ogre::SceneNode*>(
                GOOF::NodeUtils::GetNodeMatch(getSceneManager()->getRootSceneNode(), parentNode->mName.data, false));
            assert(parent);
            sceneNode = parent->createChildSceneNode(pNode->mName.data);
        }
        else
        {
            sceneNode = getSceneManager()->getRootSceneNode()->createChildSceneNode(pNode->mName.data);
        }

        sceneNode->setScale(scale.x, scale.y, scale.z);
        sceneNode->setPosition(pos.x, pos.y, pos.z);
        sceneNode->setOrientation(rot.w, rot.x, rot.y, rot.z);

        sceneNode = sceneNode->createChildSceneNode();
        sceneNode->setScale(0.01, 0.01, 0.01);
        sceneNode->attachObject(getSceneManager()->createEntity("Box1m.mesh"));
        */

        if (!aiM.IsIdentity())
        {
            bone->setPosition(pos.x, pos.y, pos.z);
            bone->setOrientation(rot.w, rot.x, rot.y, rot.z);
        }

        if(!mQuietMode)
        {
            Ogre::LogManager::getSingleton().logMessage(Ogre::StringConverter::toString(_msBoneCount) + ") Creating bone '" + Ogre::String(node->mName.data) + "'");
        }
        _msBoneCount++;
    }
    // Traverse all child nodes of the current node instance
    for ( unsigned int childIdx=0; childIdx<node->mNumChildren; ++childIdx )
    {
        const aiNode *pChildNode = node->mChildren[ childIdx ];
        createBonesFromNode(scene, pChildNode);
    }
}

void AssimpLoader::createBoneHiearchy(const aiScene* scene, const aiNode *node)
{
    if(isNodeNeeded(node->mName.data))
    {
        Ogre::v1::OldBone* parent = 0;
        Ogre::v1::OldBone* child = 0;
        if(node->mParent)
        {
            if(mSkeleton->hasBone(node->mParent->mName.data))
            {
                parent = mSkeleton->getBone(node->mParent->mName.data);
            }
        }
        if(mSkeleton->hasBone(node->mName.data))
        {
            child = mSkeleton->getBone(node->mName.data);
        }
        if(parent && child)
        {
            parent->addChild(child);
        }
    }
    // Traverse all child nodes of the current node instance
    for ( unsigned int childIdx=0; childIdx<node->mNumChildren; childIdx++ )
    {
        const aiNode *pChildNode = node->mChildren[ childIdx ];
        createBoneHiearchy(scene, pChildNode);
    }
}

void AssimpLoader::flagNodeAsNeeded(const char* name)
{
    mapBonenode::iterator iter = mBonenodes.find(Ogre::String(name));
    if( iter != mBonenodes.end())
    {
        iter->second.isNeeded = true;
    }
}

bool AssimpLoader::isNodeNeeded(const char* name)
{
    mapBonenode::iterator iter = mBonenodes.find(Ogre::String(name));
    if( iter != mBonenodes.end())
    {
        return iter->second.isNeeded;
    }
    return false;
}

void AssimpLoader::grabBoneNamesFromNode(const aiScene* scene, const aiNode *node)
{
    static int _meshNum = 0;
    _meshNum++;
    if(node->mNumMeshes > 0)
    {
        for ( unsigned int idx=0; idx<node->mNumMeshes; ++idx )
        {
            aiMesh *pAIMesh = scene->mMeshes[node->mMeshes[ idx ] ];
            if(pAIMesh->HasBones())
            {
                for ( Ogre::uint32 i=0; i < pAIMesh->mNumBones; ++i )
                {
                    aiBone *pAIBone = pAIMesh->mBones[ i ];
                    if ( NULL != pAIBone )
                    {
                        mBonesByName[pAIBone->mName.data] = pAIBone;
                        if(!mQuietMode)
                        {
                            Ogre::LogManager::getSingleton().logMessage(
								Ogre::StringConverter::toString(i) + ") REAL BONE with name : " + Ogre::String(pAIBone->mName.data) );
                        }

                        // flag this node and all parents of this node as needed, until we reach the node holding the mesh, or the parent.
                        aiNode* node = scene->mRootNode->FindNode(pAIBone->mName.data);
                        while(node)
                        {
                            if(node->mName.data == node->mName.data)
                            {
                                flagNodeAsNeeded(node->mName.data);
                                break;
                            }
                            if(node->mName.data == node->mParent->mName.data)
                            {
                                flagNodeAsNeeded(node->mName.data);
                                break;
                            }

                            // Not a root node, flag this as needed and continue to the parent
                            flagNodeAsNeeded(node->mName.data);
                            node = node->mParent;
                        }

                        // Flag all children of this node as needed
                        node = scene->mRootNode->FindNode(pAIBone->mName.data);
                        markAllChildNodesAsNeeded(node);

                    } // if we have a valid bone
                } // loop over bones
            } // if this mesh has bones
        } // loop over meshes
    } // if this node has meshes

    // Traverse all child nodes of the current node instance
    for ( unsigned int childIdx=0; childIdx<node->mNumChildren; childIdx++ )
    {
        const aiNode *pChildNode = node->mChildren[ childIdx ];
        grabBoneNamesFromNode(scene, pChildNode);
    }
}

Ogre::String ReplaceSpaces(const Ogre::String& ss)
{
    Ogre::String res(ss);
    replace(res.begin(), res.end(), ' ', '_');

    return res;
}

Ogre::MaterialPtr AssimpLoader::createMaterialByScript(int index, const aiMaterial* mat)
{
    // Create a material in code as using script inheritance variable substitution and other goodies
    Ogre::MaterialManager* matMgr = Ogre::MaterialManager::getSingletonPtr();
    Ogre::String materialName = mBasename + "#" + Ogre::StringConverter::toString(index);
    if(matMgr->resourceExists(materialName))
    {
        Ogre::MaterialPtr matPtr = matMgr->getByName(materialName).staticCast<Ogre::Material>();
        if(matPtr->isLoaded())
        {
            return matPtr;
        }
    }

    Ogre::String code;
    aiColor4D c;
    if(aiGetMaterialColor(mat, AI_MATKEY_COLOR_AMBIENT,  &c) == aiReturn_SUCCESS)
        code += "\t\t\tambient " + toString(c) + "\n";
    if(aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &c) == aiReturn_SUCCESS)
        code += "\t\t\tdiffuse " + toString(c) + "\n";
    if(aiGetMaterialColor(mat, AI_MATKEY_COLOR_SPECULAR, &c) == aiReturn_SUCCESS)
        code += "\t\t\tspecular " + toString(c) + "\n";
    if(aiGetMaterialColor(mat, AI_MATKEY_COLOR_EMISSIVE, &c) == aiReturn_SUCCESS)
        code += "\t\t\temissive " + toString(c) + "\n";

    int shade = aiShadingMode_NoShading;
    if (AI_SUCCESS == mat->Get(AI_MATKEY_SHADING_MODEL, shade) && shade != aiShadingMode_NoShading) { 
        switch (shade) {
            case aiShadingMode_Phong:
			// Phong shading mode was added to opengl and directx years ago to be ready for gpus to support it (in fixed function pipeline), 
			// but no gpus ever did, so it has never done anything. From directx 10 onwards it was removed again.
            case aiShadingMode_Gouraud:
                code += "\t\t\tshading gouraud\n";
                break;
            case aiShadingMode_Flat:
                code += "\t\t\tshading flat\n";
                break;
            default:
                break;
        }
    }

    // Specifies the type of the texture to be retrieved ( e.g. diffuse, specular, height map ...)
    enum aiTextureType type = aiTextureType_DIFFUSE;
    // Index of the texture to be retrieved. The function fails if there is no texture of that type with this index.
    // GetTextureCount() can be used to determine the number of textures per texture type.
    // Receives the path to the texture. NULL is a valid value.
    aiString path;
    // The texture mapping. NULL is allowed as value.
    aiTextureMapping mapping = aiTextureMapping_UV;
    // Receives the UV index of the texture. NULL is a valid value.
    unsigned int uvindex = 0;
    // Receives the blend factor for the texture NULL is a valid value.
    float blend = 1.0f;
    // Receives the texture operation to be performed between this texture and the previous texture. NULL is allowed as value.
    aiTextureOp op = aiTextureOp_Multiply;
    // Receives the mapping modes to be used for the texture. The parameter may be NULL but if it is a valid pointer it
    // MUST point to an array of 3 aiTextureMapMode's (one for each axis: UVW order (=XYZ)).
    aiTextureMapMode mapmode[3] =  { aiTextureMapMode_Wrap, aiTextureMapMode_Wrap, aiTextureMapMode_Wrap };    // mapmode

    // For now assuming at most that only one diffuse texture exists
    if (mat->GetTexture(type, 0, &path, &mapping, &uvindex, &blend, &op, mapmode) == AI_SUCCESS)
    {
        Ogre::String texBasename, texExtention, texPath;
        Ogre::StringUtil::splitFullFilename(Ogre::String(path.data), texBasename, texExtention, texPath);
        Ogre::String texName = texBasename + "." + texExtention;
        int twoSided = 0;
        mat->Get(AI_MATKEY_TWOSIDED, twoSided);
        if(twoSided != 0)
        {
            code += "\t\t\t\tcull_hardware none\n";
        }
        //code += "\tset $diffuse_map " + texName + "\n";
        code += "\n\t\t\ttexture_unit\n\t\t\t{\n\t\t\t\ttexture " + texName + "\n";
        // no infomation on the alpha channel in the texture will have to load the texture and look at it
        code += "\t\t\t}\n";
    }

    code = "\ttechnique\n\t{\n\t\tpass\n\t\t{\n" + code + "\t\t}\n\t}\n";
    //code = "material " + materialName + " : base\n{\n" + code + "}\n\n";
    code = "material " + materialName + "\n{\n" + code + "}\n\n";
    mMaterialCode += code;

    // compile the material
    //code = "import * from base.material\n" + code;

/*    std::cout << "-------------------------------------------code" << std::endl;
    std::cout << code << std::endl;
    std::cout << "-------------------------------------------code" << std::endl;*/

    Ogre::DataStreamPtr stream(OGRE_NEW Ogre::MemoryDataStream(
									const_cast<void*>( static_cast<const void*>(code.c_str()) ), 
									code.length() * sizeof(char), 
									false) );
    Ogre::MaterialManager::getSingleton().parseScript(stream, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::MaterialPtr omat = Ogre::MaterialManager::getSingleton().getByName(materialName).staticCast<Ogre::Material>();
    //omat->compile(false);
    //omat->load();

    return omat;
}

Ogre::MaterialPtr AssimpLoader::createMaterial(int index, const aiMaterial* mat, const Ogre::String& dir)
{
    static int _dummyMatCount = 0;
    // extreme fallback texture -- 2x2 hot pink
    static Ogre::uint8 _sRGB[] = {128, 0, 255, 128, 0, 255, 128, 0, 255, 128, 0, 255};

    std::ostringstream matname;
    Ogre::MaterialManager* omatMgr = Ogre::MaterialManager::getSingletonPtr();
    enum aiTextureType type = aiTextureType_DIFFUSE;
    static aiString _path;
    aiTextureMapping mapping = aiTextureMapping_UV;       // the mapping (should be uv for now)
    unsigned int uvindex = 0;                             // the texture uv index channel
    float blend = 1.0f;                                   // blend
    aiTextureOp op = aiTextureOp_Multiply;                // op
    aiTextureMapMode mapmode[3] = { aiTextureMapMode_Wrap, aiTextureMapMode_Wrap, aiTextureMapMode_Wrap }; // mapmode
    std::ostringstream texname;

    aiString szPath;
    if(AI_SUCCESS == aiGetMaterialString(mat, AI_MATKEY_TEXTURE_DIFFUSE(0), &szPath))
    {
        if(!mQuietMode)
        {
            Ogre::LogManager::getSingleton().logMessage(
				"Using aiGetMaterialString : Found texture " + Ogre::String(szPath.data) + " for channel " + Ogre::StringConverter::toString(uvindex) );
        }
    }
    if(szPath.length < 1)
    {
        if(!mQuietMode)
        {
            Ogre::LogManager::getSingleton().logMessage("Didn't find any texture units...");
        }
        szPath = Ogre::String("dummyMat" + Ogre::StringConverter::toString(_dummyMatCount)).c_str();
        _dummyMatCount++;
    }

    Ogre::String basename;
    Ogre::String outPath;
    Ogre::StringUtil::splitFilename(Ogre::String(szPath.data), basename, outPath);
    if(!mQuietMode)
    {
        Ogre::LogManager::getSingleton().logMessage("Creating " + basename);
    }

    Ogre::ResourceManager::ResourceCreateOrRetrieveResult status = omatMgr->createOrRetrieve(ReplaceSpaces(basename), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
    Ogre::MaterialPtr omat = status.first.staticCast<Ogre::Material>();
    if (!status.second)
        return omat;

    // ambient
    aiColor4D clr(1.0f, 1.0f, 1.0f, 1.0);
    //Ambient is usually way too low! FIX ME!
    if (mat->GetTexture(type, 0, &_path) != AI_SUCCESS)
        aiGetMaterialColor(mat, AI_MATKEY_COLOR_AMBIENT,  &clr);
    omat->setAmbient(clr.r, clr.g, clr.b);
    // diffuse
    clr = aiColor4D(1.0f, 1.0f, 1.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &clr))
    {
        omat->setDiffuse(clr.r, clr.g, clr.b, clr.a);
    }
    // specular
    clr = aiColor4D(1.0f, 1.0f, 1.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mat, AI_MATKEY_COLOR_SPECULAR, &clr))
    {
        omat->setSpecular(clr.r, clr.g, clr.b, clr.a);
    }
    // emissive
    clr = aiColor4D(1.0f, 1.0f, 1.0f, 1.0f);
    if(AI_SUCCESS == aiGetMaterialColor(mat, AI_MATKEY_COLOR_EMISSIVE, &clr))
    {
        omat->setSelfIllumination(clr.r, clr.g, clr.b);
    }

    float fShininess;
    if(AI_SUCCESS == aiGetMaterialFloat(mat, AI_MATKEY_SHININESS, &fShininess))
    {
        omat->setShininess(Ogre::Real(fShininess));
    }

    int shade = aiShadingMode_NoShading;
    if (AI_SUCCESS == mat->Get(AI_MATKEY_SHADING_MODEL, shade) && shade != aiShadingMode_NoShading) { 
        switch (shade) {
        case aiShadingMode_Phong:
			// Phong shading mode was added to opengl and directx years ago to be ready for gpus to support it (in fixed function pipeline), 
			// but no gpus ever did, so it has never done anything. From directx 10 onwards it was removed again.
        case aiShadingMode_Gouraud:
            omat->setShadingMode(Ogre::SO_GOURAUD);
            break;
        case aiShadingMode_Flat:
            omat->setShadingMode(Ogre::SO_FLAT);
            break;
        default:
            break;
        }
    }

    if (mat->GetTexture(type, 0, &_path) == AI_SUCCESS)
    {
        if(!mQuietMode)
        {
            Ogre::LogManager::getSingleton().logMessage(
				"Found texture " + Ogre::String(_path.data) + " for channel " + Ogre::StringConverter::toString(uvindex) );
        }
        if(AI_SUCCESS == aiGetMaterialString(mat, AI_MATKEY_TEXTURE_DIFFUSE(0), &szPath))
        {
            if(!mQuietMode)
            {
                Ogre::LogManager::getSingleton().logMessage(
					"Using aiGetMaterialString : Found texture " + Ogre::String(szPath.data) + " for channel " + Ogre::StringConverter::toString(uvindex) );
            }
        }

        // attempt to load the image
        Ogre::Image image;
        // possibly if we fail to actually find it, pop up a box?
        Ogre::String pathname(dir + "\\" + _path.data);
        std::ifstream imgstream;
        imgstream.open(_path.data, std::ios::binary);
        if(!imgstream.is_open())
            imgstream.open(Ogre::String(mPath + Ogre::String("\\") + Ogre::String(_path.data)).c_str(), std::ios::binary);

        if (imgstream.is_open())
        {
            // Wrap as a stream
            Ogre::DataStreamPtr strm(OGRE_NEW Ogre::FileStreamDataStream(_path.data, &imgstream, false));
            if (!strm->size() || strm->size() == 0xffffffff)
            {
                // fall back to our very simple and very hardcoded hot-pink version
                Ogre::DataStreamPtr altStrm(OGRE_NEW Ogre::MemoryDataStream(_sRGB, sizeof(_sRGB)));
                image.loadRawData(altStrm, 2, 2, Ogre::PF_R8G8B8);
                if(!mQuietMode)
                {
                    Ogre::LogManager::getSingleton().logMessage("Could not load texture, falling back to hotpink");
                }
            }
			else
            {
                // extract extension from filename
                size_t pos = pathname.find_last_of('.');
                Ogre::String ext = pathname.substr(pos+1);
                image.load(strm, ext);
                imgstream.close();
            }
        }
		else
		{
            // fall back to our very simple and very hardcoded hot-pink version
            Ogre::DataStreamPtr altStrm(OGRE_NEW Ogre::MemoryDataStream(_sRGB, sizeof(_sRGB)));
            image.loadRawData(altStrm, 2, 2, Ogre::PF_R8G8B8);
            if(!mQuietMode)
            {
                Ogre::LogManager::getSingleton().logMessage("Could not load texture, falling back to hotpink - 2");
            }
        }

		// Ogre::TextureManager::getSingleton().loadImage(Ogre::String(szPath.data), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        //TODO: save this to materials/textures ?
        Ogre::TextureUnitState* texUnitState = omat->getTechnique(0)->getPass(0)->createTextureUnitState(basename);
    }

    omat->load(); // would need a rendersystem
    return omat;
}

}
}
