# -*- coding: utf-8 -*-
"""A first controllable deformable-tongue scene for SOFA 25.12.

The anatomical meshes come from SOFA's Dentistry_Haptics tutorial.  The
original tutorial only renders the tongue.  Here the tetrahedral tongue is a
real FEM body: its posterior part is fixed and its anterior part follows a
smooth elastic motion target.
"""

import math
import os

import Sofa
import Sofa.Core
import Sofa.Simulation
from Sofa.constants import Key


PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))


# -----------------------------------------------------------------------------
# 1. Asset paths
# -----------------------------------------------------------------------------
def asset(*parts):
    return os.path.join(PROJECT_DIR, "assets", *parts).replace("\\", "/")


# -----------------------------------------------------------------------------
# 2. Motion controller
# -----------------------------------------------------------------------------
class TongueController(Sofa.Core.Controller):
    """控制自动轨迹、键盘输入和舌尖目标。"""

    # 用户可调整的运动参数
    TARGET_STEP = 0.08
    TARGET_LIMIT = 0.55
    CYCLE_SECONDS = 4.0
    LIFT_AMPLITUDE = 0.22
    FORWARD_SWING = 0.32
    FORWARD_BIAS = 0.18

    # 舌尖权重：最前端完全跟随，向舌中逐渐减弱为 0
    FULL_TIP_WEIGHT_Z = -2.70
    BLEND_END_Z = -1.55

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.listening.value = True  # 必须打开，否则收不到键盘事件

        self.root = kwargs["root"]
        self.dofs = kwargs["dofs"]
        self.tip_roi = kwargs["tip_roi"]
        self.target_dofs = kwargs["target_dofs"]
        self.visual_model = kwargs["visual_model"]
        self.status_label = kwargs["status_label"]

        self.tip_indices = []
        self.tip_weights = {}
        self.rest_positions = None
        self.rest_visual_positions = None
        self.current_command = [0.0, 0.0, 0.0]

        self.addData(
            name="automaticMotion", value=True, type="bool",
            group="Tongue movement",
            help="Checked: automatic loop. Unchecked: manual target.",
        )
        self.addData(
            name="manualLeftRight", value=0.0, type="double",
            group="Tongue movement", help="Negative left, positive right.",
        )
        self.addData(
            name="manualUpDown", value=0.0, type="double",
            group="Tongue movement", help="Negative down, positive up.",
        )
        self.addData(
            name="manualForwardBack", value=0.0, type="double",
            group="Tongue movement", help="Negative retract, positive protrude.",
        )
        self.addData(
            name="visibleSurfaceDisplacement", value=0.0, type="double",
            group="Diagnostics", help="Maximum visible-surface displacement.",
        )
        self.addData(
            name="automaticCyclePhase", value=0.0, type="double",
            group="Diagnostics", help="Automatic phase from 0 to 1.",
        )
        self.addData(
            name="keyboardEventCount", value=0, type="int",
            group="Diagnostics", help="Number of received keyboard events.",
        )

    def onSimulationInitDoneEvent(self, _event):
        """网格生成完成后，保存中性形状并计算渐变权重。"""
        self.tip_indices = [int(i) for i in self.tip_roi.indices.value]
        self.rest_positions = [
            [float(v) for v in point] for point in self.dofs.position.value
        ]

        # 隐藏目标与舌头初始形状完全相同
        with self.target_dofs.position.writeable() as target:
            for i, point in enumerate(self.rest_positions):
                target[i][0], target[i][1], target[i][2] = point

        blend_length = self.BLEND_END_Z - self.FULL_TIP_WEIGHT_Z
        for i in self.tip_indices:
            weight = (self.BLEND_END_Z - self.rest_positions[i][2]) / blend_length
            self.tip_weights[i] = max(0.0, min(1.0, weight))

        Sofa.Simulation.updateVisual(self.root)
        self.rest_visual_positions = [
            [float(v) for v in point] for point in self.visual_model.position.value
        ]
        self.root.animate.value = True
        print("[TongueControl] initialized; keyboard listening is ON")

    @staticmethod
    def _to_model_coordinates(command):
        """把直观命令 [右, 上, 前] 转成模型坐标 [X, Y, Z]。"""
        # 这个模型中，屏幕向上是 -Y，口腔外方向是 -Z。
        return [command[0], -command[1], -command[2]]

    def _automatic_command(self, simulation_time):
        """生成连续椭圆轨迹；返回直观方向 [右, 上, 前]。"""
        phase = (simulation_time % self.CYCLE_SECONDS) / self.CYCLE_SECONDS
        angle = 2.0 * math.pi * phase
        self.automaticCyclePhase.value = phase
        return [
            0.0,
            self.LIFT_AMPLITUDE * (1.0 - math.cos(angle)),
            self.FORWARD_SWING * math.sin(angle)
            + self.FORWARD_BIAS * (1.0 - math.cos(angle)),
        ]

    def _manual_command(self):
        return [
            float(self.manualLeftRight.value),
            float(self.manualUpDown.value),
            float(self.manualForwardBack.value),
        ]

    def _write_manual_command(self, command):
        self.manualLeftRight.value = command[0]
        self.manualUpDown.value = command[1]
        self.manualForwardBack.value = command[2]

    def _apply_command(self, command):
        """移动隐藏目标；弹簧再让有限元舌头平滑跟随。"""
        if self.rest_positions is None:
            return
        model_offset = self._to_model_coordinates(command)
        with self.target_dofs.position.writeable() as target:
            for i in self.tip_indices:
                weight = self.tip_weights[i]
                for axis in range(3):
                    target[i][axis] = (
                        self.rest_positions[i][axis]
                        + weight * model_offset[axis]
                    )
        self.current_command = [float(v) for v in command]

    def _visible_displacement(self):
        if self.rest_visual_positions is None:
            return 0.0
        maximum_squared = 0.0
        for current, rest in zip(
            self.visual_model.position.value, self.rest_visual_positions
        ):
            squared = sum((float(current[a]) - rest[a]) ** 2 for a in range(3))
            maximum_squared = max(maximum_squared, squared)
        return maximum_squared ** 0.5

    def _update_label(self, visible):
        command = self.current_command
        mode = "AUTO" if bool(self.automaticMotion.value) else "MANUAL"
        self.status_label.label.value = (
            f"TONGUE: {mode} | LR={command[0]:+.2f}  "
            f"UP={command[1]:+.2f}  FB={command[2]:+.2f}  "
            f"VISIBLE={visible:.2f}"
        )

    def onAnimateBeginEvent(self, _event):
        if bool(self.automaticMotion.value):
            command = self._automatic_command(float(self.root.time.value))
        else:
            command = self._manual_command()
        self._apply_command(command)

    def onAnimateEndEvent(self, _event):
        Sofa.Simulation.updateVisual(self.root)
        visible = self._visible_displacement()
        self.visibleSurfaceDisplacement.value = visible
        self._update_label(visible)

    def onKeypressedEvent(self, event):
        key = event["key"]
        self.keyboardEventCount.value = int(self.keyboardEventCount.value) + 1

        if key == Key.space:
            self.automaticMotion.value = not bool(self.automaticMotion.value)
            if not bool(self.automaticMotion.value):
                self._write_manual_command([0.0, 0.0, 0.0])
            return

        key = key.upper() if isinstance(key, str) else key
        if key not in ("W", "S", "A", "D", "E", "Q", "X", "0"):
            return

        self.automaticMotion.value = False
        command = self._manual_command()
        if key == "W":
            command[1] += self.TARGET_STEP
        elif key == "S":
            command[1] -= self.TARGET_STEP
        elif key == "A":
            command[0] -= self.TARGET_STEP
        elif key == "D":
            command[0] += self.TARGET_STEP
        elif key == "E":
            command[2] += self.TARGET_STEP
        elif key == "Q":
            command[2] -= self.TARGET_STEP
        else:  # X 或 0
            command = [0.0, 0.0, 0.0]

        command = [
            max(-self.TARGET_LIMIT, min(self.TARGET_LIMIT, value))
            for value in command
        ]
        self._write_manual_command(command)
        self._apply_command(command)
        print(f"[TongueControl] key={key}, command={command}")


# -----------------------------------------------------------------------------
# 3. Static mouth appearance
# -----------------------------------------------------------------------------
def add_static_anatomy(root):
    anatomy = root.addChild("StaticMouth")

    head_loader = anatomy.addObject(
        "MeshOBJLoader",
        name="headLoader",
        filename=asset("mesh", "head.obj"),
        handleSeams=True,
    )
    anatomy.addObject(
        "OglModel",
        name="HeadVisual",
        src="@headLoader",
        texturename=asset("textures", "head_baseTexBaked.png"),
    )

    teeth = anatomy.addChild("Teeth")
    teeth_loader = teeth.addObject(
        "MeshOBJLoader",
        name="teethLoader",
        filename=asset("mesh", "teeth.obj"),
        handleSeams=True,
    )
    teeth.addObject(
        "OglModel",
        name="TeethVisual",
        src="@teethLoader",
        texturename=asset("textures", "teeth_color.png"),
    )


# -----------------------------------------------------------------------------
# 4. Deformable FEM tongue
# -----------------------------------------------------------------------------
def add_deformable_tongue(root, status_label):
    tongue = root.addChild("DeformableTongue")
    tongue.addObject(
        "EulerImplicitSolver",
        name="timeIntegrator",
        rayleighMass=0.80,
        rayleighStiffness=0.30,
    )
    tongue.addObject(
        "ParallelCGLinearSolver",
        name="linearSolver",
        iterations=30,
        tolerance=1.0e-6,
        threshold=1.0e-10,
    )

    tongue.addObject(
        "MeshOBJLoader",
        name="volumeSurface",
        filename=asset("mesh", "tongue.obj"),
        triangulate=True,
    )
    tongue.addObject(
        "MeshTetraStuffing",
        name="tetraGenerator",
        snapPoints=True,
        splitTetras=True,
        size=0.30,
        alphaLong=0.20,
        alphaShort=0.40,
        inputPoints="@volumeSurface.position",
        inputTriangles="@volumeSurface.triangles",
        inputQuads="@volumeSurface.quads",
    )
    tongue.addObject(
        "TetrahedronSetTopologyContainer",
        name="topology",
        position="@tetraGenerator.outputPoints",
        tetrahedra="@tetraGenerator.outputTetras",
    )
    tongue.addObject("TetrahedronSetTopologyModifier", name="topologyModifier")
    tongue.addObject("TetrahedronSetGeometryAlgorithms", name="geometry")
    dofs = tongue.addObject(
        "MechanicalObject",
        name="dofs",
        template="Vec3",
        position="@tetraGenerator.outputPoints",
    )

    # The units of the tutorial mesh are close to centimetres.  These are
    # intentionally conservative starting parameters; they are easy to tune
    # later when a physical actuator and material have been chosen.
    tongue.addObject("UniformMass", name="mass", totalMass=0.07)
    tongue.addObject(
        "ParallelTetrahedronFEMForceField",
        name="fem",
        method="large",
        youngModulus=700.0,
        poissonRatio=0.42,
    )

    # The anatomical tongue root is the posterior end near Z=0.  It remains
    # attached to the floor/back of the mouth.  The anterior tongue tip is
    # the narrow end near Z=-3.2 and is the region we actively move.
    root_roi = tongue.addObject(
        "BoxROI",
        name="rootROI",
        box=[-1.1, -0.2, -0.65, 1.1, 1.5, 0.2],
        drawBoxes=False,
    )
    tongue.addObject(
        "FixedProjectiveConstraint",
        name="fixedTongueRoot",
        indices="@rootROI.indices",
    )

    tip_roi = tongue.addObject(
        "BoxROI",
        name="tipROI",
        # Use a longer anterior region.  The controller applies a Z-based
        # weight so the target motion fades continuously toward the middle.
        box=[-1.1, -0.2, -3.3, 1.1, 1.5, -1.55],
        drawBoxes=False,
    )
    target_node = root.addChild("TongueMotionTarget")
    target_dofs = target_node.addObject(
        "MechanicalObject",
        name="targetDofs",
        template="Vec3",
        position="@../DeformableTongue/tetraGenerator.outputPoints",
    )
    tongue.addObject(
        "RestShapeSpringsForceField",
        name="tipTargetSprings",
        points="@tipROI.indices",
        external_rest_shape="@../TongueMotionTarget/targetDofs",
        external_points="@tipROI.indices",
        stiffness=85.0,
        rayleighStiffness=0.08,
        drawSpring=False,
    )

    visual = tongue.addChild("Surface")
    visual.addObject(
        "MeshOBJLoader",
        name="surfaceLoader",
        filename=asset("mesh", "tongue.obj"),
        handleSeams=True,
    )
    tongue_visual = visual.addObject(
        "OglModel",
        name="TongueVisual",
        src="@surfaceLoader",
        texturename=asset("textures", "tongue_color.png"),
    )
    visual.addObject(
        "BarycentricMapping",
        name="surfaceMapping",
        input="@../dofs",
        output="@TongueVisual",
    )

    root.addObject(
        TongueController(
            name="TongueControl",
            root=root,
            dofs=dofs,
            tip_roi=tip_roi,
            status_label=status_label,
            visual_model=tongue_visual,
            target_dofs=target_dofs,
        )
    )
    return tongue


# -----------------------------------------------------------------------------
# 5. SOFA scene entry point
# -----------------------------------------------------------------------------
def createScene(root):
    root.dt = 0.03
    root.gravity = [0.0, 0.0, 0.0]

    root.addObject(
        "RequiredPlugin",
        pluginName=[
            "MultiThreading",
            "Sofa.Component.AnimationLoop",
            "Sofa.Component.Constraint.Projective",
            "Sofa.Component.Engine.Generate",
            "Sofa.Component.Engine.Select",
            "Sofa.Component.IO.Mesh",
            "Sofa.Component.LinearSolver.Iterative",
            "Sofa.Component.Mapping.Linear",
            "Sofa.Component.Mass",
            "Sofa.Component.ODESolver.Backward",
            "Sofa.Component.Setting",
            "Sofa.Component.SolidMechanics.FEM.Elastic",
            "Sofa.Component.SolidMechanics.Spring",
            "Sofa.Component.StateContainer",
            "Sofa.Component.Topology.Container.Dynamic",
            "Sofa.Component.Visual",
            "Sofa.GL.Component.Rendering2D",
            "Sofa.GL.Component.Rendering3D",
        ],
    )

    root.addObject("DefaultAnimationLoop")
    root.addObject("DefaultVisualManagerLoop")
    root.addObject(
        "VisualStyle",
        displayFlags=(
            "showVisualModels hideBehaviorModels hideCollisionModels "
            "hideForceFields hideInteractionForceFields hideWireframe"
        ),
    )
    root.addObject(
        "BackgroundSetting", color=[0.901960784, 0.925490196, 0.964705882, 1.0]
    )

    status_label = root.addObject(
        "OglLabel",
        name="TongueStatus",
        label="TONGUE: loading...",
        x=20,
        y=30,
        fontsize=16,
        color=[0.0, 0.9, 0.0, 1.0],
        updateLabelEveryNbSteps=1,
    )

    add_static_anatomy(root)
    add_deformable_tongue(root, status_label)
    root.animate.value = True

    print("[TongueControl] scene loaded")
    print("[TongueControl] animation starts automatically; use the toolbar to pause/reset")
    print("[TongueControl] SPACE: auto/manual; W/S: up/down; A/D: left/right")
    print("[TongueControl] E/Q: protrude/retract; X or 0: neutral target")
    return root
