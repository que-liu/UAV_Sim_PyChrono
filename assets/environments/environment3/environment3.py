# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: 


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
body_1.SetName('environment1-1')
body_1.SetPos(chrono.ChVector3d(-2,0,0))
body_1.SetRot(chrono.ChQuaterniond(1,0,0,0))
body_1.SetMass(2068.75)
body_1.SetInertiaXX(chrono.ChVector3d(1686.35839155337,2587.17447916667,2733.23339155338))
body_1.SetInertiaXY(chrono.ChVector3d(5.13603929443284e-14,1.11109038636314e-13,-9.66819544831267e-14))
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(6.125,0.839274924471298,-6.79214992406629e-17),chrono.ChQuaterniond(1,0,0,0)))
body_1.SetFixed(True)

# # Visualization shape 
# body_1_1_shape = chrono.ChObjFileShape() 
# body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
# body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))
# # body_1_1_shape.SetTexture(chrono.GetChronoDataFile("textures/light_gray.png"))
# body_1_1_shape.SetColor(chrono.ChColor(1, 0, 0))

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



