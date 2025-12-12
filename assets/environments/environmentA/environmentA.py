# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 

import pychrono as chrono 
import builtins 

# some global settings: 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('environmentA-1')
body_1.SetPos(chrono.ChVector3d(-3.7,0,0))
body_1.SetRot(chrono.ChQuaterniond(1,0,0,0))
body_1.SetMass(2010)
body_1.SetInertiaXX(chrono.ChVector3d(1168.14720149254,3382.48964552239,4058.23684701493))
body_1.SetInertiaXY(chrono.ChVector3d(-174.915111940299,4.44557490469223e-13,-3.72506596464338e-14))
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(7.03917910447761,0.895522388059702,0.3),chrono.ChQuaterniond(1,0,0,0)))
body_1.SetFixed(True)

# Visualization shape 
# body_1_1_shape = chrono.ChObjFileShape() 
# body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
# body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

# Attach a visualization shape
# First load a .obj from disk into a ChTriangleMeshConnected:
mesh_for_visualization1 = chrono.ChTriangleMeshConnected()
mesh_for_visualization1.LoadWavefrontMesh(shapes_dir +'body_1_1.obj')
# Now the  triangle mesh is inserted in a ChVisualShapeTriangleMesh visualization asset, 
# and added to the body
visualization_shape1 = chrono.ChVisualShapeTriangleMesh()
visualization_shape1.SetMesh(mesh_for_visualization1)
visualization_shape1.SetColor(chrono.ChColor(0.96,0.96,0.86)) # beige
# visualization_shape1.SetTexture(chrono.GetChronoDataFile("textures/light_gray.png"))
body_1.AddVisualShape(visualization_shape1)

# Collision material 
mat_1 = chrono.ChContactMaterialNSC()

# Collision shapes 
collision_model = chrono.ChCollisionModel()
body_1.AddCollisionModel(collision_model)
body_1.GetCollisionModel().Clear()

# Triangle mesh collision shape 
body_1_1_collision_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(shapes_dir + 'body_1_1_collision.obj', False, True) 
mr = chrono.ChMatrix33d()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_1_1_collision_mesh.Transform(chrono.ChVector3d(0, 0, 0), mr) 
mesh_shape = chrono.ChCollisionShapeTriangleMesh(mat_1, body_1_1_collision_mesh, False, False, sphereswept_r)
frame = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(chrono.ChQuaterniond(1, 0, 0, 0)))
body_1.GetCollisionModel().AddShape(mesh_shape, frame)
body_1.EnableCollision(True)

exported_items.append(body_1)



