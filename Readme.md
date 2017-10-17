# Reference
# Assimp -> Ogre adaptor (fork from here)
https://github.com/kenkit/ogre-assimp/tree/Ogre_Assimp_Adaptor
# And original source
https://github.com/OGRECave/ogre-assimp

# Ogre forum discuss thread
http://forums.ogre3d.org/viewtopic.php?f=25&t=93647&p=539322#p539322

- Trying to work for OGRE v2.1 mesh.
- Avoid boost library.
- Currently only mesh/submesh, but have bugs on (index) buffers...

# Usage
```
#include "AssimpImporter"

    Demo::assimp::Importer assimpimport;
    Ogre::MeshPtr pmesh;
    bool ret = assimpimport.createMeshV2(pmesh, [meshname], [pathname], quality);
```

### Liscence:
This code use OgreAssimp Converter liscence.
