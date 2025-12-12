import pychrono as chrono
import pychrono.fea as fea


class ANCFCableRope:
  """
  Builds a sling rope using ANCF cable elements.
  
  - The rope is a FEA mesh with ChNodeFEAxyzD + ChElementCableANCF
  - Attachable to any two rigid bodies (UAV and payload)
  - Automatically creates nodes, elements, and constraints
  """

  def __init__(self,
    system: chrono.ChSystem,
    start_pos: chrono.ChVector3d,
    end_pos: chrono.ChVector3d,
    uav_body: chrono.ChBody,
    payload_body: chrono.ChBody,
    n_elements: int = 10,
    cable_diameter: float = 0.005,
    young_modulus: float = 0.01e9,
    density: float = 800,
    damping: float = 0.0
    ):

    self.system = system
    self.start_pos = start_pos
    self.end_pos = end_pos
    self.n_elements = n_elements

    # Store
    self.mesh = fea.ChMesh()
    system.Add(self.mesh)
    self.builder = fea.ChBuilderCableANCF()
    self.section = fea.ChBeamSectionCable()

    # Material
    self.young_modulus = young_modulus
    self.density = density
    self.cable_diameter = cable_diameter
    self.damping = damping

    self._build_mesh(uav_body, payload_body)
    self._addVisualization(self.mesh)

  # ----------------------------------------------------------------------

  def _build_mesh(self, uav_body: chrono.ChBody, payload_body: chrono.ChBody):
    """
    Build the ANCF rope: nodes + elements + section.
    """
    direction = self.end_pos - self.start_pos
    self.cable_length = direction.Length()
    direction.Normalize()

    # ANCF cross-section
    self.section.SetDensity(self.density)
    self.section.SetYoungModulus(self.young_modulus)
    self.section.SetDiameter(self.cable_diameter)
    self.section.SetRayleighDamping(self.damping)

    self.builder.BuildBeam(
      self.mesh,       # the mesh where to put the created nodes and elements
      self.section,    # the ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
      self.n_elements, # the number of ChElementBeamANCF_3333 to create
      self.start_pos,  # the 'A' point space (beginning of beam)
      self.end_pos     # the 'B' point space (end of beam)
    )

    self.attach_start(uav_body)
    self.attach_end(payload_body)

  # ----------------------------------------------------------------------

  def attach_start(self, body):
    """
    Attach first cable node to a rigid body using a point-frame constraint.
    """
    constraint_hinge = fea.ChLinkNodeFrame()
    constraint_hinge.Initialize(self.builder.GetLastBeamNodes().front(), body)
    self.system.Add(constraint_hinge)
    msphere = chrono.ChVisualShapeSphere(0.008)
    constraint_hinge.AddVisualShape(msphere)

  # ----------------------------------------------------------------------

  def attach_end(self, body):
    """
    Attach last cable node to a rigid body.
    """
    constraint_hinge = fea.ChLinkNodeFrame()
    constraint_hinge.Initialize(self.builder.GetLastBeamNodes().back(), body)
    self.system.Add(constraint_hinge)
    msphere = chrono.ChVisualShapeSphere(0.008)
    constraint_hinge.AddVisualShape(msphere)

    constraint_dir = fea.ChLinkNodeSlopeFrame()
    constraint_dir.Initialize(self.builder.GetLastBeamNodes().back(), body)
    constraint_dir.SetDirectionInAbsoluteCoords(chrono.ChVector3d(0, 1, 0))
    self.system.Add(constraint_dir)

  def _addVisualization(self, mesh: fea.ChMesh):
    visualizebeamA = chrono.ChVisualShapeFEA()
    visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    visualizebeamA.SetColormapRange(-0.4, 0.4)
    visualizebeamA.SetSmoothFaces(True)
    visualizebeamA.SetWireframe(False)
    mesh.AddVisualShapeFEA(visualizebeamA)

    visualizebeamB = chrono.ChVisualShapeFEA()
    visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS) # NODE_CSYS
    visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    visualizebeamB.SetSymbolsThickness(0.006)
    visualizebeamB.SetSymbolsScale(0.01)
    visualizebeamB.SetZbufferHide(False)
    mesh.AddVisualShapeFEA(visualizebeamB)