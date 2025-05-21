import sys
import pychrono as chrono

def loadBodies(my_system):
  my_ground = my_system.SearchBody('ground')
  if my_ground:
    print('Ground found!')
  else:
    sys.exit('Error: cannot find ground from its name in the C::E system!')

  my_frame = my_system.SearchBody('drone_big_box-1')
  if my_frame:
    print('Frame found!')
  else:
    sys.exit('Error: cannot find drone frame from its name in the C::E system!')

  my_box = my_system.SearchBody('box_big_200x200x100-1')
  if my_box:
    print('Box found!')
  else:
    sys.exit('Error: cannot find box from its name in the C::E system!')

  props = []
  for i in range(1, 9):
    name = f'3_blade_prop-{i}'
    prop = my_system.SearchBody(name)
    if prop:
      print(f'{name} found!')
      props.append(prop)
    else:
      sys.exit(f'Error: cannot find {name} from its name in the C::E system!')

  return my_ground, my_frame, my_box, *props

def loadMarkers(my_system):
  markers = []
  for i in range(1, 9):
    name = f'Coordinate System{i}'
    marker = my_system.SearchMarker(name)
    if marker:
      print(f'Marker_{i} found!')
      markers.append(marker)
    else:
      sys.exit(f'Error: cannot find marker{i} from its name in the C::E system!')

  return tuple(markers)

def addMotors(my_system, my_frame, props, markers):
  motors = []
  for i in range(8):
    frame = markers[i].GetAbsFrame()
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(props[i], my_frame, frame)
    motor.SetSpindleConstraint(chrono.ChLinkMotorRotationSpeed.SpindleConstraint_CYLINDRICAL)
    # motor.SetMotorFunction(chrono.ChFunction_Const(5.0 * chrono.CH_C_2PI))
    my_system.Add(motor)
    motors.append(motor)
  return motors


