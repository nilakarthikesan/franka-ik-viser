"""Main entry point: 50 Hz loop tying trajectory -> IK -> Viser together.

Run:  python -m franka_ik.app
"""

# TODO(M4/M5):
#   model = FrankaModel(); solver = DLSSolver(model); scene = Scene(model.urdf_path)
#   q = Q_HOME.copy()
#   loop @ 50 Hz:
#     T_des = trajectory(t) if playback else scene.target_pose()
#     q = solver.solve_step(q, T_des)
#     scene.set_joint_config(q); update traces + error readout


def main() -> None:
    raise NotImplementedError


if __name__ == "__main__":
    main()
