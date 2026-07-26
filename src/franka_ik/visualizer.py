"""Viser scene: robot, target gizmo, traces, and GUI controls.

Uses viser.extras.ViserUrdf to render the FR3 URDF, plus:
  - transform_controls gizmo as the interactive IK target
  - reference trajectory line + live EE trace
  - GUI: mode + trajectory dropdowns, speed/gain sliders, error readout
"""

# TODO(M1): implement Scene class
#   import viser
#   from viser.extras import ViserUrdf
#
# class Scene:
#     def __init__(self, urdf_path): ...
#     def set_joint_config(self, q): ...
#     def target_pose(self): ...          # from gizmo (interactive mode)
#     def draw_reference(self, points): ...
#     def append_ee_trace(self, point): ...
#     def set_error_text(self, pos_mm, rot_deg): ...
