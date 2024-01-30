import math

import matplotlib.pyplot as plt
import mpld3
import numpy as np
from IPython.display import HTML, display
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    MultibodyPlant,
    Parser,
    Propeller,
    PropellerInfo,
    RigidTransform,
    RobotDiagramBuilder,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    WrapToSystem,
    namedview,
)
from pydrake.examples import (
    AcrobotGeometry,
    AcrobotInput,
    AcrobotPlant,
    AcrobotState,
    QuadrotorGeometry,
    QuadrotorPlant,
    StabilizingLQRController,
)
from pydrake.solvers import MathematicalProgram, Solve

from underactuated import ConfigureParser, running_as_notebook
from underactuated.meshcat_utils import MeshcatSliders
from underactuated.quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer
from underactuated.scenarios import AddFloatingRpyJoint

# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()

builder = DiagramBuilder()
acrobot = builder.AddSystem(AcrobotPlant())

# Setup visualization
scene_graph = builder.AddSystem(SceneGraph())
AcrobotGeometry.AddToBuilder(
    builder, acrobot.get_output_port(0), scene_graph
)
meshcat.Delete()
meshcat.Set2dRenderMode(xmin=-4, xmax=4, ymin=-4, ymax=4)
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

# Setup slider input
meshcat.AddSlider("u", min=-5, max=5, step=0.1, value=0.0)
torque_system = builder.AddSystem(MeshcatSliders(meshcat, ["u"]))
builder.Connect(torque_system.get_output_port(), acrobot.get_input_port())

diagram = builder.Build()

# Set up a simulator to run this diagram
simulator = Simulator(diagram)
context = simulator.get_mutable_context()

# Set the initial conditions (theta1, theta2, theta1dot, theta2dot)
context.SetContinuousState([1, 0, 0, 0])

simulator.AdvanceTo(0.1)

meshcat.DeleteAddedControls()
