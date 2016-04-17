Terrain and world objects are PhysicsNodes with mass=0 and can have meshes as collision shape.

## TriMeshes ##
You can use any TriMesh (such as loaded models) as terrain or as world objects.
```
TriMesh myMesh=getMesh();
PhysicsNode terrainPhysicsNode=new PhysicsNode(myMesh, CollisionShape.ShapeTypes.MESH, 0);
```

## JME TerrainBlocks ##
TerrainBlocks are normal TriMeshes in jme so to have physics terrain, you create a PhysicsNode with a MeshCollisionShape and mass=0 for your TerrainBlock.

```
RawHeightMap heightMap = new RawHeightMap(ActionGame.class
                .getClassLoader().getResource("heights.raw"),
                129, RawHeightMap.FORMAT_16BITLE, false);

TerrainBlock terrainBlock = new TerrainBlock("Terrain", heightMap.getSize(), terrainScale,
                                           heightMap.getHeightMap(),
                                           new Vector3f(0, 0, 0));

MeshCollisionShape collisionShape=new MeshCollisionShape(terrainBlock);
PhysicsNode terrainPhysicsNode=new PhysicsNode(terrainBlock, collisionShape,0);

terrainPhysicsNode.setLocalTranslation(new Vector3f(-300,-10,-300));
terrainPhysicsNode.rotateUpTo(new Vector3f(0,1,0));
PhysicsSpace.getPhysicsSpace().add(terrainPhysicsNode);

```