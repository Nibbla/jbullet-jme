The bullet PhysicsCharacterNode is pretty basic, its aim is performance with lots of character nodes, not realism. For something more advanced, I'd suggest building up the character from multiple PhysicsNodes.

## Rotation ##
The character node itself is not supposed to turn. You animate or turn your added spatial/mesh. The CharacterNode is just for collision and location.

## JavaDoc ##
http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/com/jmex/jbullet/nodes/PhysicsCharacterNode.html