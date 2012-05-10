package com.jmex.jbullet.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import com.jme.bounding.BoundingBox;
import com.jme.bounding.BoundingCapsule;
import com.jme.bounding.BoundingSphere;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.shapes.BoxCollisionShape;
import com.jmex.jbullet.collision.shapes.CapsuleCollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.collision.shapes.CompoundCollisionShape;
import com.jmex.jbullet.collision.shapes.CylinderCollisionShape;
import com.jmex.jbullet.collision.shapes.GImpactCollisionShape;
import com.jmex.jbullet.collision.shapes.MeshCollisionShape;
import com.jmex.jbullet.collision.shapes.SphereCollisionShape;
import com.jmex.jbullet.nodes.PhysicsNode;

/**
 * Clase de utilidad para facilitar el trabajo con las físicas de JBullet.
 * 
 * @author David García Illanas (dagarill)
 * 
 */
public final class PhysicsUtil {

	/**
	 * Clase de utilidad no instanciable. Constructor privado.
	 */
	private PhysicsUtil() {
		super();
	}

	/**
	 * Contruye un <code>PhysicsNode</code> con el modelo indicado, usando el
	 * tipo de geometría de colisión especificado.
	 * 
	 * @param modelo
	 *            el modelo que se usará para representar el nodo de físicas
	 * @param collisionClass
	 *            el tipo de geometría de colisión que se usará para calcular
	 *            las colisiones del modelo
	 * @return dependiendo de la clase de colisión indicada, se deolverán los
	 *         siguientes tipos de objetos:
	 *         <ul>
	 *         <li><code>GImpactCollisionShape</code>,
	 *         <code>MeshCollisionShape</code>: Se devuelve un objeto de tipo
	 *         <code>Node</code> que contiene tantos objetos de tipo
	 *         <code>PhysicsNode</code> como submayas (TriMesh) que contenga el
	 *         modelo indicado. Es decir, se crea u n nodo físico por cada
	 *         submaya.</li>
	 *         <li><code>CompoundCollisionShape</code>: No permitido en este
	 *         método. Usar el método que admite 3 parámetros para configurar el
	 *         tipo de forma que compondrán la maya resultante.
	 *         <li>Resto de tipos de colisión: Retorna un objeto de tipo
	 *         <code>PhysicsNode</code>
	 *         </ul>
	 *         una instancia de <code>PhysicsNode</code> con el modelo asociado.
	 * @see PhysicsNode
	 */
	public static Node crearNodoFisicas(Spatial modelo, Class<? extends CollisionShape> collisionClass, Quaternion rotacion) {
		return crearNodoFisicas(modelo, collisionClass, null, rotacion);

	}

	/**
	 * Contruye un <code>PhysicsNode</code> con el modelo indicado, usando el
	 * tipo de geometría de colisión especificado.
	 * 
	 * @param modelo
	 *            el modelo que se usará para representar el nodo de físicas
	 * @param collisionClass
	 *            el tipo de geometría de colisión que se usará para calcular
	 *            las colisiones del modelo
	 * @param compoundChildShapeClass
	 *            el tipo de <code>CollisionShape</code> que se utilizará para
	 *            cada una se las submayas cuando se quiere crear un nodo con
	 *            una <code>CompoundCollisionShape</code>. Es obligatorio cuando
	 *            se indica como <code>collisionClass</code> una clase del tipo
	 *            <code>CompoundCollisionShape</code>. Para el resto de tipos de
	 *            formas de colisión, este parámetro se ignora.
	 * @return dependiendo de la clase de colisión indicada, se deolverán los
	 *         siguientes tipos de objetos:
	 *         <ul>
	 *         <li><code>GImpactCollisionShape</code>,
	 *         <code>MeshCollisionShape</code>: Se devuelve un objeto de tipo
	 *         <code>Node</code> que contiene tantos objetos de tipo
	 *         <code>PhysicsNode</code> como submayas (TriMesh) que contenga el
	 *         modelo indicado. Es decir, se crea u n nodo físico por cada
	 *         submaya.</li>
	 *         <li><code>CompoundCollisionShape</code>: Se devuelve un objeto de
	 *         tipo <code>PhysicsNode</code> que contiene una geometría de
	 *         colisión compuesta por las geometrías del modelo indicado. La
	 *         <code>CompoundCollisionShape</code> estará formadas por
	 *         geometrias de colisión del tipo especificado por el parámetro
	 *         <code>compoundChildShapeClass</code></li>.
	 *         <li>Resto de tipos de colisión: Retorna un objeto de tipo
	 *         <code>PhysicsNode</code>
	 *         </ul>
	 *         una instancia de <code>PhysicsNode</code> con el modelo asociado.
	 * @see PhysicsNode
	 */
	public static Node crearNodoFisicas(Spatial modelo, Class<? extends CollisionShape> collisionClass,
			Class<? extends CollisionShape> compoundChildShapeClass, Quaternion rotacion) {
		modelo.updateGeometricState(0, false);
		Node result = null;
		PhysicsSpace pSpace = PhysicsSpace.getPhysicsSpace();
		if (collisionClass.equals(CompoundCollisionShape.class)) {
				if (compoundChildShapeClass == null) {
					throw new IllegalArgumentException(
							"Es necesario indicar el tipo de collisionShape para los hijos de la CompoundCollisionShape.");
				}
				result = new PhysicsNode(modelo, createCompoundCollisionShape(modelo, compoundChildShapeClass), 0f);
				result.setName(modelo.getName());
				if (rotacion != null) {
					result.setLocalRotation(rotacion.clone());
				}
				pSpace.add(result);
		} else {
			result = new Node(modelo.getName());			
			// Hay que descomponer el modelo en subnodos
			//TODO pasar por parámetro la profundidad
			Collection<Spatial> subnodos = descomponerEnNodos(modelo, 2);
			PhysicsNode physNode;
			CollisionShape collisionShape;
			try {
				for (Spatial subnodo : subnodos) {
					collisionShape = createCompoundCollisionShape(subnodo, collisionClass);					
					physNode = new PhysicsNode(subnodo, collisionShape, 0f);
					physNode.setName(subnodo.getName());
					if (rotacion != null) {
						physNode.setLocalRotation(rotacion.clone());
					}
					result.attachChild(physNode);
					pSpace.add(physNode);

				}
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}
		return result;
	}

	/**
	 * Crea una collisionShape compuesta por tantas <code>CollisionShape</code>
	 * como submayas (TriMesh) tenga el modelo indicado. El método busca de
	 * forma recuriva todas las mayas.
	 * 
	 * @param modelo
	 *            el modelo para el que se quiere crear la
	 *            <code>CompoundCollisionShape</code>
	 * @param shapeClass
	 *            el tipo de <code>CollisionShape</code> que se utilizará para
	 *            cada una se las submayas
	 * @return la geometia compuesta de colisión para el modelo especificado por
	 *         parámetro
	 */
	public static CompoundCollisionShape createCompoundCollisionShape(Spatial modelo,
			Class<? extends CollisionShape> shapeClass) {
		CompoundCollisionShape shape = new CompoundCollisionShape();
		addCollisionShape(modelo, shape, shapeClass);
		return shape;
	}

	public static CompoundCollisionShape shiftCenterOfMass(Spatial modelo, CollisionShape shape, Vector3f translation) {
		/*
		 * Seguimos las instrucciones de la Wiki de JBullet-jme para construir
		 * un método que desplaze el centro de masa de un PhysicsNode la
		 * cantidad indicada; El nodo deberá utilizar como forma de colisión la
		 * que se devuelve en el método.
		 */
		CompoundCollisionShape resultShape = new CompoundCollisionShape();
		resultShape.addChildShape(shape, translation);
		modelo.setLocalTranslation(translation);
		return resultShape;
	}

	/**
	 * Añade a la collisionShape tantas <code>CollisionShape</code> como
	 * submayas (TriMesh) tenga el modelo indicado. El método busca de forma
	 * recuriva todas las mayas.
	 * 
	 * @param modelo
	 *            el modelo del que se crearán nuevas geometrías de colisión y
	 *            se añadirán a la <code>CompoundCollisionShape</code>
	 * @param shape
	 *            la geometia compuesta de colisión a la que se añadirán las
	 *            geometrías de colisión creadas
	 * @param shapeClass
	 *            el tipo de <code>CollisionShape</code> que se utilizará para
	 *            cada una se las submayas
	 */
	public static void addCollisionShape(Spatial modelo, CompoundCollisionShape shape,
			Class<? extends CollisionShape> shapeClass) {
		if (shapeClass.equals(CompoundCollisionShape.class)) {
			throw new IllegalArgumentException(
					"No se admite el tipo 'CompoundCollisionShape' como parámetro 'shapeClass'");
		}
		if (modelo instanceof TriMesh) {

			CollisionShape childShape = generateCollisionShape(modelo, shapeClass);
			if (childShape != null) {
				Vector3f location = modelo.getWorldTranslation();
				if (!(shapeClass.equals(MeshCollisionShape.class) || shapeClass.equals(GImpactCollisionShape.class))) {
					location =  modelo.getWorldBound().getCenter().add(location);
				}
				shape.addChildShape(childShape, location, modelo.getWorldRotation().toRotationMatrix());
			}

		} else if (modelo instanceof Node) {
			List<Spatial> hijos = ((Node) modelo).getChildren();
			if (hijos != null) {
				for (Spatial spatial : hijos) {
					// Llamada recursiva
					addCollisionShape(spatial, shape, shapeClass);
				}
			}
		}
	}

	public static CollisionShape generateCollisionShape(Spatial modelo, Class<? extends CollisionShape> shapeClass) {
		// Nueva instancia del tipo de forma de colisión indicado en
		// 'shapeClass'
		modelo.updateGeometricState(0, false);
		modelo.updateModelBound();
		CollisionShape childShape = null;
		if (shapeClass.equals(BoxCollisionShape.class)) {
			if (!(modelo.getWorldBound() instanceof BoundingBox)) {
				modelo.setModelBound(new BoundingBox());
				modelo.updateModelBound();
			}
			childShape = new BoxCollisionShape(((BoundingBox) modelo.getWorldBound()).getExtent(null));
		} else if (shapeClass.equals(CylinderCollisionShape.class)) {
			if (!(modelo.getWorldBound() instanceof BoundingBox)) {
				modelo.setModelBound(new BoundingBox());
				modelo.updateModelBound();
			}
			childShape = new CylinderCollisionShape(((BoundingBox) modelo.getWorldBound()).getExtent(null));
		} else if (shapeClass.equals(CapsuleCollisionShape.class)) {
			if (!(modelo.getWorldBound() instanceof BoundingCapsule)) {
				modelo.setModelBound(new BoundingCapsule());
				modelo.updateModelBound();
			}
			childShape = new CapsuleCollisionShape((BoundingCapsule) modelo.getWorldBound());
		} else if (shapeClass.equals(SphereCollisionShape.class)) {
			if (!(modelo.getWorldBound() instanceof BoundingSphere)) {
				modelo.setModelBound(new BoundingSphere());
				modelo.updateModelBound();
			}
			childShape = new SphereCollisionShape(((BoundingSphere) modelo.getWorldBound()).getRadius());
		} else if (shapeClass.equals(GImpactCollisionShape.class)) {
			childShape = new GImpactCollisionShape((TriMesh) modelo);
		} else if (shapeClass.equals(MeshCollisionShape.class)) {
			childShape = new MeshCollisionShape((TriMesh) modelo);
		}
		return childShape;
	}
	
	private static Collection<Spatial> descomponerEnNodos(Spatial spatial, int profundidad) {
		return descomponerEnNodosRecur(spatial, profundidad, 0);
	}

	private static Collection<Spatial> descomponerEnNodosRecur(Spatial spatial, int profundidad, int profAct) {
		Collection<Spatial> result = null;
		// se se ha llegado a la profundidad deseada o el spatial no es un nodo,
		// termina la recursividad
		if (profAct >= profundidad || !(spatial instanceof Node)) {
			if (!(spatial instanceof Node)) {
				result = Collections.singletonList(spatial);
			} else {
				result = ((Node) spatial).getChildren();
			}
		} else {
			List<Spatial> listaHijos = ((Node) spatial).getChildren();
			result = new ArrayList<Spatial>();
			for (Spatial hijo : listaHijos) {
				result.addAll(descomponerEnNodosRecur(hijo, profundidad, profAct + 1));
			}
		}
		return result;

	}
}
