      
# -- coding: UTF-8
"""
#!/usr/bin/python3 
"""
import argparse
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from piper_msgs.msg import PosCmd
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header
import signal
import sys
import os
import threading
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# import pinocchio for ik calculations
# from pinocchio import casadi as cpin#
# from pinocchio.robot_wrapper import RobotWrapper
# from pinocchio.visualize import MeshcatVisualizer

from openpi_client import image_tools, websocket_client_policy
# from openpi_client import image_tools, websocket_client_policy_2
# from openpi.policies.pika_ik import Arm_IK, eef_pos_to_qpos, batch_eef_pos_to_qpos


CAMERA_NAMES = ["cam_high", "cam_right_wrist", "cam_left_wrist"]

stream_buffer = None   # type: StreamActionBuffer

observation_window = None

# lang_embeddings = "fold the cloth"
# lang_embeddings = "Flatten and fold the short sleeve"

RIGHT_OFFSET = 0.003

DH_MODIFIED = np.array([
    [0.0,         0.0,       0.123,    0.0],
    [-np.pi/2.0,  0.0,       0.0,      -172.22/180.0*np.pi],
    [0.0,         0.28503,   0.0,      -102.78/180.0*np.pi],
    [np.pi/2.0,  -0.021984,  0.25075,  0.0],
    [-np.pi/2.0,  0.0,       0.0,      0.0],
    [np.pi/2.0,   0.0,       0.211,    0.0],
], dtype=float)

# Debug/plotting related global caches
published_actions_history = []  # list[np.ndarray(shape=(14,))]
observed_qpos_history = []      # list[np.ndarray(shape=(14,))]
publish_step_global = 0         # current publish step (for plotting x-axis alignment)
inferred_chunks = []            # list[dict(start_step:int, chunk:np.ndarray[chunk,14])]
inferred_chunks_lock = threading.Lock()
shutdown_event = threading.Event()


class SimpleKalmanFilter:
    def __init__(self, process_variance=1e-6, measurement_variance=1e-7, initial_value=None):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.error_estimate = 1.0

    def update(self, measurement):
        """
        Update the internal estimate given a new measurement.
        """
        if self.estimate is None:
            self.estimate = measurement.copy()
            return self.estimate
        # Compute Kalman gain
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)
        # Update estimate
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        # Update the error estimate
        self.error_estimate = (
            (1 - kalman_gain) * self.error_estimate
            + abs(self.estimate - measurement) * self.process_variance
        )
        return self.estimate


def create_kalman_filters(state_dim: int,
                          process_variance: float = 1e-6,
                          measurement_variance: float = 1e-7) -> list[SimpleKalmanFilter]:
    """
    Create a list of SimpleKalmanFilter instances, one per action dimension.

    This serves as a reusable interface for Kalman-based action smoothing.
    """
    return [
        SimpleKalmanFilter(process_variance=process_variance,
                           measurement_variance=measurement_variance)
        for _ in range(state_dim)
    ]


def apply_kalman_filter_to_action(raw_action: np.ndarray,
                                  kalman_filters: list[SimpleKalmanFilter] | None) -> np.ndarray:
    """
    Apply a bank of 1-D Kalman filters (one per action component) to a raw action.
    """
    if kalman_filters is None:
        return raw_action
    return np.array([kf.update(raw_action[i]) for i, kf in enumerate(kalman_filters)], dtype=float)

def joint_actions_clip(action: np.ndarray):
    pass


class Arm_IK:
    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        urdf_path = '/home/agilex/Manipulation_Demo/piper_ros/src/piper_description/urdf/piper_description.urdf'    
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path)

        self.mixed_jointsToLockIDs = ["joint7","joint8"]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        # q = quaternion_from_euler(0, -1.57, -1.57)
        q = quaternion_from_euler(0, 0, 0)
        self.reduced_robot.model.addFrame(
            pin.Frame('ee',
                      self.reduced_robot.model.getJointId('joint6'),
                      pin.SE3(
                          # pin.Quaternion(1, 0, 0, 0),
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([0.0, 0.0, 0.0]),
                      ),
                      pin.FrameType.OP_FRAME)
        )

        self.geom_model = pin.buildGeomFromUrdf(self.robot.model, urdf_path, pin.GeometryType.COLLISION)
        for i in range(4, 9):
            for j in range(0, 3):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
        self.geometry_data = pin.GeometryData(self.geom_model)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)

        # # Initialize the Meshcat visualizer  for visualization
        self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
        self.vis.initViewer(open=True)
        self.vis.loadViewerModel("pinocchio")
        self.vis.displayFrames(True, frame_ids=[113, 114], axis_length=0.15, axis_width=5)
        self.vis.display(pin.neutral(self.reduced_robot.model))

        # Enable the display of end effector target frames with short axis lengths and greater width.
        frame_viz_names = ['ee_target']
        FRAME_AXIS_POSITIONS = (
            np.array([[0, 0, 0], [1, 0, 0],
                      [0, 0, 0], [0, 1, 0],
                      [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        FRAME_AXIS_COLORS = (
            np.array([[1, 0, 0], [1, 0.6, 0],
                      [0, 1, 0], [0.6, 1, 0],
                      [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        axis_length = 0.1
        axis_width = 10
        for frame_viz_name in frame_viz_names:
            self.vis.viewer[frame_viz_name].set_object(
                mg.LineSegments(
                    mg.PointsGeometry(
                        position=axis_length * FRAME_AXIS_POSITIONS,
                        color=FRAME_AXIS_COLORS,
                    ),
                    mg.LineBasicMaterial(
                        linewidth=axis_width,
                        vertexColors=True,
                    ),
                )
            )

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # # Get the hand joint ID and define the error function
        self.gripper_id = self.reduced_robot.model.getFrameId("ee")
        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.gripper_id].inverse() * cpin.SE3(self.cTf)
                    ).vector,
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        # self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf = self.opti.parameter(4, 4)
        self.totalcost = casadi.sumsqr(self.error(self.var_q, self.param_tf))
        self.regularization = casadi.sumsqr(self.var_q)
        # self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last) # for smooth

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        # print("self.reduced_robot.model.lowerPositionLimit:", self.reduced_robot.model.lowerPositionLimit)
        # print("self.reduced_robot.model.upperPositionLimit:", self.reduced_robot.model.upperPositionLimit)
        self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization)
        # self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization + 0.1 * self.smooth_cost) # for smooth

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,
                'tol': 1e-4
            },
            'print_time': False
        }
        self.opti.solver("ipopt", opts)

    def ik_fun(self, target_pose, gripper=0, motorstate=None, motorV=None):
        gripper = np.array([gripper/2.0, -gripper/2.0])
        if motorstate is not None:
            self.init_data = motorstate
        self.opti.set_initial(self.var_q, self.init_data)

        self.vis.viewer['ee_target'].set_transform(target_pose)     # for visualization

        self.opti.set_value(self.param_tf, target_pose)
        # self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            # sol = self.opti.solve()
            sol = self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)

            if self.init_data is not None:
                max_diff = max(abs(self.history_data - sol_q))
                # print("max_diff:", max_diff)
                self.init_data = sol_q
                if max_diff > 30.0/180.0*3.1415:
                    # print("Excessive changes in joint angle:", max_diff)
                    self.init_data = np.zeros(self.reduced_robot.model.nq)
            else:
                self.init_data = sol_q
            self.history_data = sol_q

            self.vis.display(sol_q)  # for visualization

            if motorV is not None:
                v = motorV * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            tau_ff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v,
                              np.zeros(self.reduced_robot.model.nv))

            is_collision = self.check_self_collision(sol_q, gripper)

            return sol_q, tau_ff, not is_collision

        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")
            # sol_q = self.opti.debug.value(self.var_q)   # return original value
            return sol_q, '', False

    def check_self_collision(self, q, gripper=np.array([0, 0])):
        pin.forwardKinematics(self.robot.model, self.robot.data, np.concatenate([q, gripper], axis=0))
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
        # print("collision:", collision)
        return collision

    def get_ik_solution(self, x,y,z,roll,pitch,yaw):
        
        q = quaternion_from_euler(roll, pitch, yaw, axes='rzyx')
        target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([x, y, z]),
        )
        print(target)
        sol_q, tau_ff, get_result = self.ik_fun(target.homogeneous,0)
        # print("result:", sol_q)
        
        if get_result :
            piper_control.joint_control_piper(sol_q[0],sol_q[1],sol_q[2],sol_q[3],sol_q[4],sol_q[5],0)
        else :
            print("collision!!!")


def inference_fn_non_blocking_fast(args, config, policy, ros_operator):
    """
    Non-blocking inference thread (rate-limited only by `args.inference_rate`).

    - Each iteration, take the most recent observation and build a payload.
    - Call the remote policy and immediately push the returned action chunk into `stream_buffer`.
    - Regardless of success/failure, move on to the next iteration until ROS shuts down.
    """
    global stream_buffer
    # assert stream_buffer is not None, "[inference_fn_non_blocking_fast] stream_buffer not initialized"

    global observation_window

    # global lang_embeddings

    # global action_lock
    rate = rospy.Rate(getattr(args, "inference_rate", 4))
    while not rospy.is_shutdown():
        try:
            time1 = time.time()
            # 1) Get the latest observation (non-blocking at ROS level)
            update_observation_window(args, config, ros_operator)

            print("Get Observation Time", time.time() - time1, "s")
            time1 = time.time()

            latest_obs = observation_window[-1]
            imgs = [
                latest_obs["images"][config["camera_names"][0]],
                latest_obs["images"][config["camera_names"][1]],
                latest_obs["images"][config["camera_names"][2]],
            ]
            # BGR->RGB & pad/resize to model input resolution
            if not os.path.exists('test.jpg'):
                cv2.imwrite('test.jpg', imgs[0])
                cv2.imwrite('test1.jpg', imgs[1])
                cv2.imwrite('test2.jpg', imgs[2])
            # imgs = [cv2.cvtColor(im, cv2.COLOR_BGR2RGB)/255 for im in imgs]
            # imgs = [im / 255 for im in imgs]
            # imgs = image_tools.resize_with_pad(np.array(imgs), 224, 224)
            # print(imgs[0].max(), imgs[1].max(), imgs[2].max())
            print(imgs[0].shape)
            proprio = latest_obs["qpos"]

            # 2) Build payload (field names follow current inference interface)
            payload = {
                "state": proprio,
                "images": {
                    "top_head":  imgs[0].transpose(2, 0, 1),   # CHW
                    "hand_right": imgs[1].transpose(2, 0, 1),
                    "hand_left":  imgs[2].transpose(2, 0, 1),
                },
                # "prompt": lang_embeddings,
            }


            # 3) Run inference (blocking until a single chunk is returned).
            #    Expected return: {"actions": np.ndarray [chunk, state_dim]}
            # out = policy.infer(payload)
            # actions = out.get("actions", None)
            actions = policy.infer(payload)["actions"]
            print("Inference Time", time.time() - time1, "s")
            time1 = time.time()

            # 4) Push to concurrent buffer (for main control loop to consume / smooth)
            # with action_lock:
            if actions is not None and len(actions) > 0:
                # Entry point for temporal smoothing integration
                max_k = int(getattr(args, "latency_k", 0))
                min_m = int(getattr(args, "min_smooth_steps", 8))

                # TODO(YCC): apply eef action chunk integration here
                stream_buffer.integrate_new_chunk(actions, max_k=max_k, min_m=min_m)
                # Record this chunk only for debug plotting
                try:
                    step_now = max(len(published_actions_history), len(observed_qpos_history))
                    with inferred_chunks_lock:
                        inferred_chunks.append({
                            "start_step": int(step_now),
                            "chunk": np.asarray(actions, dtype=float).copy()
                        })
                except Exception:
                    pass
                # Keep the full chunk sequence inside `stream_buffer` for plotting.
                # No additional copy is made; plotting reads `stream_buffer.chunks` directly.
            elif actions is None:
                print("actions is None")
            elif len(actions) == 0:
                print("len(actions) == 0")

            # 5) Continue immediately to the next iteration (only rate-limited by ROS Rate)
            print("Append Buffer Time", time.time() - time1, "s")
            time1 = time.time()
            # Control thread loop rate
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

        except Exception as e:
            # Log and continue quickly to avoid killing the thread or pinning a CPU core at 100%
            raise
            rospy.logwarn(f"[inference_fn_non_blocking_fast] {e}")
            try:
                rate.sleep()
            except Exception:
                try:
                    time.sleep(0.001)
                except Exception:
                    pass
            continue


class StreamActionBuffer:
    """
    Maintain action chunks and a smoothed execution sequence for temporal smoothing.

    - New predicted chunks are integrated via `integrate_new_chunk`.
    - The main control loop pops smoothed actions via `pop_next_action`.
    """
    def __init__(self, max_chunks=10, decay_alpha=0.25, state_dim=14, smooth_method="temporal"):
        # kept only for backward compatibility with older interfaces
        self.chunks = deque()
        self.max_chunks = max_chunks
        self.lock = threading.Lock()
        # reserved exponential-decay parameter (not used in current implementation)
        self.decay_alpha = float(decay_alpha)
        self.state_dim = state_dim
        self.smooth_method = smooth_method
        # current active, smoothed sequence to execute
        self.cur_chunk = deque()
        # number of steps already published from the current sequence (for latency compensation)
        self.k = 0
        # last successfully popped action (used for padding when merging chunks)
        self.last_action = None

    def push_chunk(self, actions_chunk: np.ndarray):
        """Legacy interface (kept for compatibility, not used in the new design)."""
        with self.lock:
            if actions_chunk is None or len(actions_chunk) == 0:
                return
            dq = deque([a.copy() for a in actions_chunk], maxlen=None)
            self.chunks.append(dq)
            while len(self.chunks) > self.max_chunks:
                self.chunks.popleft()

    def integrate_new_chunk(self, actions_chunk: np.ndarray, max_k: int, min_m: int = 8):
        """
        Integrate a newly predicted chunk:

        1) Use current `k` and `max_k` to drop the leading part of the new chunk
           (latency compensation).
        2) If an old chunk exists in `cur_chunk`, perform linear temporal smoothing
           over the overlapping region:
           - First overlap element: 100% old, 0% new.
           - Last  overlap element: 0% old, 100% new.
           - Any extra tail from the new chunk is appended directly.
        3) Reset `k = 0` so that the merged sequence becomes the new execution plan.
        """
        with self.lock:
            if actions_chunk is None or len(actions_chunk) == 0:
                return
            max_k = max(0, int(max_k))
            min_m = max(1, int(min_m))
            drop_n = min(self.k, max_k)
            if drop_n >= len(actions_chunk):
                return
            new_chunk = [a.copy() for a in actions_chunk[drop_n:]]
            # Build the "old" sequence:
            # - if empty but `last_action` exists, extend it into `min_m`-length history;
            # - if non-empty but shorter than `min_m`, pad tail frames up to `min_m`;
            # - if empty and no `last_action`, directly adopt the new sequence.
            if len(self.cur_chunk) == 0 and self.last_action is not None:
                old_list = [np.asarray(self.last_action, dtype=float).copy() for _ in range(min_m)]
                self.last_action = None
            else:
                old_list = list(self.cur_chunk)
                if len(old_list) > 0 and len(old_list) < min_m:
                    tail = np.asarray(old_list[-1], dtype=float).copy()
                    old_list.extend([tail.copy() for _ in range(min_m - len(old_list))])
                elif len(old_list) == 0:
                    self.cur_chunk = deque(new_chunk, maxlen=None)
                    self.k = 0
                    return
            new_list = list(new_chunk)

            overlap_len = min(len(old_list), len(new_list))
            if overlap_len <= 0:
                self.cur_chunk = deque(new_list, maxlen=None)
                self.k = 0
                return

            if len(old_list) > len(new_list):
                old_list = old_list[:len(new_list)]
                overlap_len = len(new_list)

            if overlap_len == 1:
                w_old = np.array([1.0], dtype=float)
            else:
                w_old = np.linspace(1.0, 0.0, overlap_len, dtype=float)*0.99
            w_new = 1.0 - w_old

            smoothed = [
                (w_old[i] * np.asarray(old_list[i], dtype=float) +
                 w_new[i] * np.asarray(new_list[i], dtype=float))
                for i in range(overlap_len)
            ]
            combined = smoothed + new_list[overlap_len:]
            self.cur_chunk = deque([a.copy() for a in combined], maxlen=None)
            self.k = 0

    

    def pop_left_step_from_all(self):
        """Legacy helper kept for compatibility (no longer used)."""
        with self.lock:
            if len(self.cur_chunk) > 0:
                self.cur_chunk.popleft()

    def has_any(self):
        with self.lock:
            return len(self.cur_chunk) > 0

    def pop_next_action(self) -> np.ndarray | None:
        """
        Pop and return the next action to be executed, and increase the
        internal published-step counter `k` by one.
        """
        with self.lock:
            if len(self.cur_chunk) == 0:
                return None
            # If we are about to pop the last element, cache it as `last_action`
            if len(self.cur_chunk) == 1:
                self.last_action = np.asarray(self.cur_chunk[0], dtype=float).copy()
            act = np.asarray(self.cur_chunk.popleft(), dtype=float)
            self.k += 1
            return act

    def temporal_smooth_action_at_index(self, idx_from_left: int) -> np.ndarray | None:
        """Legacy interface kept for compatibility (not used in the new design)."""
        with self.lock:
            if len(self.cur_chunk) == 0:
                return None
            return np.asarray(self.cur_chunk[0], dtype=float)


    def delta_eef_smooth_action_at_index(self, idx_from_left: int) -> np.ndarray | None:
        """
        Placeholder interface for delta end-effector based smoothing.

        A concrete implementation can be added later without changing the
        rest of the control pipeline.
        """
        raise NotImplementedError("delta_eef_smooth_action_at_index is not implemented yet.")


# Start inference thread for temporal smoothing mode
def start_inference_thread(args, config, policy, ros_operator):
    inference_thread = threading.Thread(target=inference_fn_non_blocking_fast, args=(args, config, policy, ros_operator))
    inference_thread.daemon = True
    inference_thread.start()


def save_debug_plots_on_interrupt(output_dir: str = "./debug_plots"):
    """
    Save debug plots when interrupted (e.g. Ctrl+C).

    - Generate 14 figures, one per joint.
    - For each joint:
      - Plot all predicted chunks currently buffered (each as a short trajectory).
      - Plot actually published action trajectory (`published_actions_history`).
      - Plot observed joint position trajectory (`observed_qpos_history`).
    """
    try:
        import os
        os.makedirs(output_dir, exist_ok=True)

        # Prepare x-axis: step indices
        pub_len = len(published_actions_history)
        obs_len = len(observed_qpos_history)
        x_pub = np.arange(pub_len)
        x_obs = np.arange(obs_len)

        # Read independently recorded chunks for plotting
        with inferred_chunks_lock:
            chunks_copy = [
                {"start_step": item["start_step"], "chunk": np.asarray(item["chunk"], dtype=float)}
                for item in inferred_chunks
            ]

        # Plot for each joint
        for joint_idx in range(14):
            try:
                plt.figure(figsize=(12, 4))

                # Plot all predicted chunks for this joint
                if len(chunks_copy) > 0:
                    for item in chunks_copy:
                        arr = item["chunk"]
                        if arr.ndim != 2 or arr.shape[1] < 14:
                            continue
                        start_step = int(item["start_step"])
                        xs = np.arange(start_step, start_step + arr.shape[0])
                        ys = arr[:, joint_idx]
                        plt.plot(xs, ys, color='C0', alpha=0.35)

                # Plot published action trajectory
                if pub_len > 0:
                    pub_arr = np.asarray(published_actions_history, dtype=float)
                    if pub_arr.ndim == 2 and pub_arr.shape[1] >= 14:
                        ys_pub = pub_arr[:, joint_idx]
                        plt.plot(x_pub, ys_pub, color='C1', linewidth=2.0, label='published_action')

                # Plot observed joint position (qpos) trajectory
                if obs_len > 0:
                    obs_arr = np.asarray(observed_qpos_history, dtype=float)
                    if obs_arr.ndim == 2 and obs_arr.shape[1] >= 14:
                        ys_obs = obs_arr[:, joint_idx]
                        plt.plot(x_obs, ys_obs, color='C2', linewidth=2.0, label='observed_qpos')

                plt.title(f"Joint {joint_idx}")
                plt.xlabel("step")
                plt.ylabel("value")
                if plt.gca().has_data():
                    plt.legend(loc='best')
                plt.grid(True, alpha=0.25)
                out_path = os.path.join(output_dir, f"joint_{joint_idx:02d}.png")
                plt.tight_layout()
                plt.savefig(out_path)
                print(f"saved {out_path}")
            except Exception as e:
                print(f"plot joint {joint_idx} failed: {e}")
            finally:
                plt.close()

        print(f"Saved debug plots to {output_dir}")
    except Exception as e:
        print(f"Failed to save debug plots: {e}")


def _on_sigint(signum, frame):
    """
    SIGINT (Ctrl+C) handler.

    - Mark the global shutdown event so the main loop can exit gracefully.
    - Forward shutdown to ROS.
    """
    try:
        shutdown_event.set()
    except Exception:
        pass
    try:
        rospy.signal_shutdown("SIGINT")
    except Exception:
        pass

def rotation_matrix_to_rotvec(R):
    return 0.5 * np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]], dtype=float)

def full_fk_transform(q6):
    return fk_piper_modified(q6)

def full_fk_transform_batch(q6_batch):
    return fk_piper_modified_batch(q6_batch)

def full_jacobian_numeric(q6, delta=1e-6):
    """
    return 6x6 numeric Jacobian: [dp/dq; d(rotvec)/dq]
    rotvec using small-angle approx from rotation matrices.
    """
    q6 = np.array(q6, dtype=float).reshape(6)
    J = np.zeros((6, 6), dtype=float)
    T0 = full_fk_transform(q6)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]

    for i in range(6):
        q_pert = q6.copy()
        q_pert[i] += delta
        Ti = full_fk_transform(q_pert)
        pi = Ti[:3, 3]
        Ri = Ti[:3, :3]
        dp = (pi - p0) / delta
        # rotation difference: Ri * R0^T
        R_err = Ri @ R0.T
        rotvec_err = rotation_matrix_to_rotvec(R_err) / delta  # approximate derivative
        J[:3, i] = dp
        J[3:, i] = rotvec_err
    return J

def full_jacobian_numeric_batch(q6_batch, delta=1e-6):
    """
    Batch version: returns (N,6,6)
    """
    q6_batch = np.array(q6_batch, dtype=float)
    N = q6_batch.shape[0]
    J_batch = np.zeros((N, 6, 6), dtype=float)
    T0 = full_fk_transform_batch(q6_batch)
    p0 = T0[:, :3, 3]
    R0 = T0[:, :3, :3]
    for i in range(6):
        q_pert = q6_batch.copy()
        q_pert[:, i] += delta
        Ti = full_fk_transform_batch(q_pert)
        pi = Ti[:, :3, 3]
        Ri = Ti[:, :3, :3]
        dp = (pi - p0) / delta  # (N,3)
        # R_err_i = Ri @ R0.transpose (batch)
        R_err = np.einsum('nij,njk->nik', Ri, np.transpose(R0, (0,2,1)))
        # rotation vector approx
        rotvec_err = 0.5 * np.stack([R_err[:,2,1] - R_err[:,1,2],
                                     R_err[:,0,2] - R_err[:,2,0],
                                     R_err[:,1,0] - R_err[:,0,1]], axis=1) / delta
        J_batch[:, :3, i] = dp
        J_batch[:, 3:, i] = rotvec_err
    return J_batch

def dls_step_pose(q6, dp6, lambda_):
    """
    One-step Damped Least Squares for full 6-DOF pose error.
    dp6: (6,) = [dx,dy,dz, drotx, droty, drotz]  (rot as small-angle vector)
    returns dtheta (6,)
    """
    J = full_jacobian_numeric(q6)  # 6x6
    Jt = J.T
    JJt = J @ Jt
    JJt_reg = JJt + (lambda_ * lambda_) * np.eye(6, dtype=float)
    try:
        x = np.linalg.solve(JJt_reg, dp6)
    except np.linalg.LinAlgError:
        x = np.linalg.lstsq(JJt_reg, dp6, rcond=None)[0]
    dtheta = Jt @ x
    return dtheta

def apply_ik_step(q6, target_pose, frame='base', lambda_=0.01, max_step_limits=None):
    q6 = np.array(q6, dtype=float).reshape(6)
    target_pose = np.array(target_pose, dtype=float).reshape(7)
    # current FK
    T = full_fk_transform(q6)
    p_cur = T[:3, 3]
    R_cur = T[:3, :3]
    # desired
    x,y,z, r,pitch,yaw, gripper = target_pose
    # construct desired rotation matrix from rpy
    # NOTE: 使用 ZYX 顺序（yaw-pitch-roll）
    cr = np.cos(r); sr = np.sin(r)
    cp = np.cos(pitch); sp = np.sin(pitch)
    cy = np.cos(yaw); sy = np.sin(yaw)
    R_des = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,             cp*cr]
    ], dtype=float)
    p_des = np.array([x,y,z], dtype=float)
    # position error
    dp_pos = p_des - p_cur
    # orientation error: R_err = R_des * R_cur^T -> rotation vector approx
    R_err = R_des @ R_cur.T
    drot = rotation_matrix_to_rotvec(R_err)  # small-angle vector
    dp6 = np.concatenate([dp_pos, drot], axis=0)
    # clamp dp6 magnitude a bit? we'll let DLS manage but also limit joint step
    dtheta = dls_step_pose(q6, dp6, lambda_)
    if max_step_limits is not None:
        dtheta = np.clip(dtheta, -np.array(max_step_limits), np.array(max_step_limits))
    return q6 + dtheta

def apply_ik_batch(q6_batch, target_pose_batch, frame='base', lambda_=0.01, max_step_limits=None,
                   max_iters=5, tol=1e-4):
    q6_batch = np.array(q6_batch, dtype=float)
    target_pose_batch = np.array(target_pose_batch, dtype=float)
    N = q6_batch.shape[0]
    out = q6_batch.copy()
    pj_lim = np.array(max_step_limits, dtype=float) if max_step_limits is not None else None
    for it in range(max_iters):
        T = full_fk_transform_batch(out)  # (N,4,4)
        p_cur = T[:, :3, 3]
        R_cur = T[:, :3, :3]
        # desired
        x = target_pose_batch[:, 0]
        y = target_pose_batch[:, 1]
        z = target_pose_batch[:, 2]
        roll = target_pose_batch[:, 3]
        pitch = target_pose_batch[:, 4]
        yaw = target_pose_batch[:, 5]
        # build R_des batch
        cr = np.cos(roll); sr = np.sin(roll)
        cp = np.cos(pitch); sp = np.sin(pitch)
        cy = np.cos(yaw); sy = np.sin(yaw)
        R_des = np.zeros((N,3,3), dtype=float)
        R_des[:,0,0] = cy*cp
        R_des[:,0,1] = cy*sp*sr - sy*cr
        R_des[:,0,2] = cy*sp*cr + sy*sr
        R_des[:,1,0] = sy*cp
        R_des[:,1,1] = sy*sp*sr + cy*cr
        R_des[:,1,2] = sy*sp*cr - cy*sr
        R_des[:,2,0] = -sp
        R_des[:,2,1] = cp*sr
        R_des[:,2,2] = cp*cr
        p_des = np.stack([x,y,z], axis=1)
        dp_pos = p_des - p_cur  # (N,3)
        R_err = np.einsum('nij,njk->nik', R_des, np.transpose(R_cur, (0,2,1)))
        drot = 0.5 * np.stack([R_err[:,2,1] - R_err[:,1,2],
                               R_err[:,0,2] - R_err[:,2,0],
                               R_err[:,1,0] - R_err[:,0,1]], axis=1)  # (N,3)
        dp6_batch = np.concatenate([dp_pos, drot], axis=1)  # (N,6)
        J_batch = full_jacobian_numeric_batch(out)  # (N,6,6)
        lam2 = float(lambda_) * float(lambda_)
        dtheta_out = np.zeros_like(out)
        for i in range(N):
            Ji = J_batch[i]
            Jt = Ji.T
            JJt = Ji @ Jt
            JJt[0,0] += lam2; JJt[1,1] += lam2; JJt[2,2] += lam2
            JJt[3,3] += lam2; JJt[4,4] += lam2; JJt[5,5] += lam2
            try:
                x = np.linalg.solve(JJt, dp6_batch[i])
            except np.linalg.LinAlgError:
                x = np.linalg.lstsq(JJt, dp6_batch[i], rcond=None)[0]
            dtheta = Jt @ x
            if pj_lim is not None:
                dtheta = np.clip(dtheta, -pj_lim, pj_lim)
            dtheta_out[i] = dtheta
        out += dtheta_out
        # check convergence (max position+orientation residual)
        max_err = np.max(np.linalg.norm(dp6_batch, axis=1))
        if max_err < tol:
            break
    return out


# --- EEF micro-correction helpers (FK/Jacobian/DLS) ---
# Piper modified DH parameters: [alpha, a, d, theta_offset]
# Sourced from C++ PiperForwardKinematics (MODIFIED)

def _modified_dh_transform(alpha, a, d, theta):
    ca = np.cos(alpha); sa = np.sin(alpha)
    ct = np.cos(theta); st = np.sin(theta)
    T = np.array([
        [ct,        -st,         0.0,  a],
        [st*ca,     ct*ca,      -sa,  -sa*d],
        [st*sa,     ct*sa,       ca,   ca*d],
        [0.0,        0.0,        0.0,  1.0],
    ], dtype=float)
    return T

def fk_piper_modified(q6):
    """Compute FK 4x4 for 6-DOF Piper using modified DH and joint vector (len 6)."""
    T = np.eye(4, dtype=float)
    for i in range(6):
        alpha, a, d, theta_offset = DH_MODIFIED[i]
        T = T @ _modified_dh_transform(alpha, a, d, q6[i] + theta_offset)
    return T

def _modified_dh_transform_batch(alpha, a, d, theta_vec):
    ct = np.cos(theta_vec); st = np.sin(theta_vec)
    ca = np.cos(alpha); sa = np.sin(alpha)
    N = theta_vec.shape[0]
    T = np.zeros((N, 4, 4), dtype=float)
    T[:, 0, 0] = ct
    T[:, 0, 1] = -st
    T[:, 0, 2] = 0.0
    T[:, 0, 3] = a
    T[:, 1, 0] = st * ca
    T[:, 1, 1] = ct * ca
    T[:, 1, 2] = -sa
    T[:, 1, 3] = -sa * d
    T[:, 2, 0] = st * sa
    T[:, 2, 1] = ct * sa
    T[:, 2, 2] = ca
    T[:, 2, 3] = ca * d
    T[:, 3, 0] = 0.0
    T[:, 3, 1] = 0.0
    T[:, 3, 2] = 0.0
    T[:, 3, 3] = 1.0
    return T

def _matmul_batch(A, B):
    # A: (N,4,4), B: (N,4,4) -> (N,4,4)
    return np.einsum('nij,njk->nik', A, B)

def fk_piper_modified_batch(q6_batch):
    # q6_batch: (N,6)
    N = q6_batch.shape[0]
    T = np.broadcast_to(np.eye(4, dtype=float), (N, 4, 4)).copy()
    for i in range(6):
        alpha, a, d, theta_offset = DH_MODIFIED[i]
        theta_vec = q6_batch[:, i] + theta_offset
        Ti = _modified_dh_transform_batch(alpha, a, d, theta_vec)
        T = _matmul_batch(T, Ti)
    return T

def position_jacobian_numeric_batch(q6_batch, delta=1e-6):
    # Returns J_batch: (N,3,6)
    N = q6_batch.shape[0]
    J = np.zeros((N, 3, 6), dtype=float)
    T0 = fk_piper_modified_batch(q6_batch)
    p0 = T0[:, :3, 3]
    for i in range(6):
        q_pert = q6_batch.copy()
        q_pert[:, i] += delta
        Ti = fk_piper_modified_batch(q_pert)
        pi = Ti[:, :3, 3]
        J[:, :, i] = (pi - p0) / delta
    return J

def position_jacobian_numeric(q6, delta=1e-6):
    # Returns J: (3,6) for single configuration
    q6 = np.array(q6, dtype=float).reshape(6)
    J = np.zeros((3, 6), dtype=float)
    T0 = fk_piper_modified(q6)
    p0 = T0[:3, 3]
    for i in range(6):
        q_pert = q6.copy()
        q_pert[i] += delta
        Ti = fk_piper_modified(q_pert)
        pi = Ti[:3, 3]
        J[:, i] = (pi - p0) / delta
    return J

def _clamp_norm(v, max_norm):
    n = float(np.linalg.norm(v))
    if max_norm <= 0.0 or n <= 1e-12 or n <= max_norm:
        return v
    return v * (max_norm / n)

def dls_step_pos(q6, dp_base, lambda_):
    """One-step DLS for position-only task: J is 3x6, dp_base is 3x1."""
    J = position_jacobian_numeric(q6)
    Jt = J.T
    JJt = J @ Jt
    JJt_reg = JJt + (lambda_ * lambda_) * np.eye(3, dtype=float)
    try:
        x = np.linalg.solve(JJt_reg, dp_base)
    except np.linalg.LinAlgError:
        x = np.linalg.lstsq(JJt_reg, dp_base, rcond=None)[0]
    dtheta = Jt @ x
    return dtheta

def apply_micro_correction(q6, dp_vec, frame, lambda_, step_limit_m, per_joint_step_limits):
    """
    Apply EE micro-correction dp_vec (3,) in given frame ("tool"|"base")
    using one DLS step. Returns new q6.
    """
    if dp_vec is None:
        return q6
    dp_vec = np.array(dp_vec, dtype=float).reshape(3)
    if np.allclose(dp_vec, 0.0):
        return q6
    T = fk_piper_modified(q6)
    R = T[:3, :3]
    if str(frame).lower() == "tool":
        dp_base = R @ dp_vec
    else:
        dp_base = dp_vec
    dp_base = _clamp_norm(dp_base, float(step_limit_m))
    dtheta = dls_step_pos(q6, dp_base, float(lambda_))
    if per_joint_step_limits is not None:
        lim = np.array(per_joint_step_limits, dtype=float).reshape(6)
        dtheta = np.clip(dtheta, -lim, lim)
    return (np.array(q6, dtype=float) + dtheta)

def apply_micro_correction_batch(q6_batch, dp_vec, frame, lambda_, step_limit_m, per_joint_step_limits):
    # q6_batch: (N,6)
    if dp_vec is None:
        return q6_batch
    dp_vec = np.array(dp_vec, dtype=float).reshape(3)
    if np.allclose(dp_vec, 0.0):
        return q6_batch
    N = q6_batch.shape[0]
    T = fk_piper_modified_batch(q6_batch)
    R = T[:, :3, :3]
    if str(frame).lower() == "tool":
        dp_base = np.einsum('nij,j->ni', R, dp_vec)
    else:
        dp_base = np.broadcast_to(dp_vec, (N, 3)).copy()
    # clamp per sample
    norms = np.linalg.norm(dp_base, axis=1)
    mask = norms > step_limit_m
    if np.any(mask):
        scales = (step_limit_m / (norms[mask] + 1e-12))
        dp_base[mask] *= scales[:, None]
    # Jacobian batch
    J = position_jacobian_numeric_batch(q6_batch)
    dtheta_out = np.zeros_like(q6_batch)
    lam2 = float(lambda_) * float(lambda_)
    pj_lim = np.array(per_joint_step_limits, dtype=float).reshape(6) if per_joint_step_limits is not None else None
    # Solve per sample 3x3
    for i in range(N):
        Ji = J[i]
        Jt = Ji.T
        JJt = Ji @ Jt
        JJt[0, 0] += lam2; JJt[1, 1] += lam2; JJt[2, 2] += lam2
        try:
            x = np.linalg.solve(JJt, dp_base[i])
        except np.linalg.LinAlgError:
            x = np.linalg.lstsq(JJt, dp_base[i], rcond=None)[0]
        dtheta = Jt @ x
        if pj_lim is not None:
            dtheta = np.clip(dtheta, -pj_lim, pj_lim)
        dtheta_out[i] = dtheta
    return q6_batch + dtheta_out


def set_seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)


def interpolate_action(args, prev_action, cur_action):
    steps = np.concatenate((np.array(args.arm_steps_length), np.array(args.arm_steps_length)), axis=0)
    diff = np.abs(cur_action - prev_action)
    step = np.ceil(diff / steps).astype(int)
    step = np.max(step)
    if step <= 1:
        return cur_action[np.newaxis, :]
    new_actions = np.linspace(prev_action, cur_action, step + 1)
    return new_actions[1:]

def minimum_jerk_interpolation(args, prev_action, cur_action):
    num_steps = args.jerk_num_steps
    t_normalized = np.linspace(0, 1, num_steps + 1)[1:]
    trajectory = []
    for tau in t_normalized:
        factor = 10 * (tau ** 3) - 15 * (tau ** 4) + 6 * (tau ** 5)
        trajectory.append(prev_action + factor * (cur_action - prev_action))
    return np.array(trajectory)


def get_config(args):
    config = {
        "episode_len": args.max_publish_step,
        "state_dim": 14,
        "chunk_size": args.chunk_size,
        "camera_names": CAMERA_NAMES,
    }
    return config


# Get the observation from the ROS topic
def get_ros_observation(args, ros_operator):
    rate = rospy.Rate(args.publish_rate)
    print_flag = True
    time3 = time.time()

    while True and not rospy.is_shutdown():
        result = ros_operator.get_frame()
        if time.time() - time3 > 0.01:
            print("Get Frame Time is too long", time.time() - time3, "s")
        if not result:
            if print_flag:
                print("syn fail when get_ros_observation")
                print_flag = False
            rate.sleep()
            continue
        print_flag = True
        (
            img_front,
            img_left,
            img_right,
            img_front_depth,
            img_left_depth,
            img_right_depth,
            puppet_arm_left,
            puppet_arm_right,
            robot_base,
        ) = result
        return (img_front, img_left, img_right, puppet_arm_left, puppet_arm_right)


# Update the observation window buffer
def update_observation_window(args, config, ros_operator):
    # JPEG transformation to align with training preprocessing
    def jpeg_mapping(img):
        img = cv2.imencode(".jpg", img)[1].tobytes()
        img = cv2.imdecode(np.frombuffer(img, np.uint8), cv2.IMREAD_COLOR)
        return img

    global observation_window
    if observation_window is None:
        observation_window = deque(maxlen=2)

        # Append the first dummy image
        observation_window.append(
            {
                "qpos": None,
                "images": {
                    config["camera_names"][0]: None,
                    config["camera_names"][1]: None,
                    config["camera_names"][2]: None,
                },
            }
        )

    img_front, img_left, img_right, puppet_arm_left, puppet_arm_right = get_ros_observation(args, ros_operator)
    img_front = jpeg_mapping(img_front)
    img_left = jpeg_mapping(img_left)
    img_right = jpeg_mapping(img_right)

    qpos = np.concatenate(
        (np.array(puppet_arm_left.position), np.array(puppet_arm_right.position)),
        axis=0,
    )

    observation_window.append(
        {
            "qpos": qpos,
            "images": {
                config["camera_names"][0]: img_front,
                config["camera_names"][1]: img_right,
                config["camera_names"][2]: img_left,
            },
        }
    )
    try:
        observed_qpos_history.append(np.asarray(qpos, dtype=float).copy())
    except Exception:
        pass


def inference_fn(args, config, policy):
    global observation_window
    # global lang_embeddings

    while True and not rospy.is_shutdown():
        # fetch images in sequence [front, right, left]
        image_arrs = [
            observation_window[-1]["images"][config["camera_names"][0]],
            observation_window[-1]["images"][config["camera_names"][1]],
            observation_window[-1]["images"][config["camera_names"][2]],
        ]
        # convert BGR to RGB
        image_arrs = [cv2.cvtColor(img, cv2.COLOR_BGR2RGB) for img in image_arrs]

        # image_arrs = image_tools.resize_with_pad(np.array(image_arrs), 224, 224)

        # get last qpos in shape [14, ]
        proprio = observation_window[-1]["qpos"]

        payload = {
            "state": proprio,
            "images": {
                "top_head": image_arrs[0].transpose(2, 0, 1),
                "hand_right": image_arrs[1].transpose(2, 0, 1),
                "hand_left": image_arrs[2].transpose(2, 0, 1),
            },
            # "prompt": lang_embeddings,
        }

        time1 = time.time()

        # actions shaped as [64, 14] in format [left, right]
        actions = policy.infer(payload)["actions"]


        print(f"Model inference time: {(time.time() - time1)*1000:.3f} ms")

        return actions


# Main loop for the manipulation task
def model_inference(args, config, ros_operator):
    # global lang_embeddings

    global stream_buffer

    # Load WebSocket client policy
    policy = websocket_client_policy.WebsocketClientPolicy(
        args.host,
        args.port,
    )
    # policy = websocket_client_policy_2.WebsocketClientPolicy(
    #     args.host,
    #     args.port,
    #     use_wss=True,
    # )
    print(f"Server metadata: {policy.get_server_metadata()}")

    max_publish_step = config["episode_len"]
    chunk_size = config["chunk_size"]

    # Initialize position of the puppet arm
    left0 = [0, 0.32, -0.36, 0, 0.24, 0, 0.1]
    right0 = [0, 0.32, -0.36, 0, 0.24, 0, 0.1]
    ros_operator.puppet_arm_publish_continuous(left0, right0)
    input("Press enter to continue")
    ros_operator.puppet_arm_publish_continuous(left0, right0)
    # 启动阶段：阻塞地推理一帧用于热身（丢弃结果）
    try:
        update_observation_window(args, config, ros_operator)
        latest_obs = observation_window[-1]
        image_arrs = [
            latest_obs["images"][config["camera_names"][0]],
            latest_obs["images"][config["camera_names"][1]],
            latest_obs["images"][config["camera_names"][2]],
        ]
        image_arrs = [cv2.cvtColor(img, cv2.COLOR_BGR2RGB) for img in image_arrs]
        image_arrs = image_tools.resize_with_pad(np.array(image_arrs), 224, 224)
        proprio = latest_obs["qpos"]
        payload = {
            "state": proprio,
            "images": {
                "top_head": image_arrs[0].transpose(2, 0, 1),
                "hand_right": image_arrs[1].transpose(2, 0, 1),
                "hand_left": image_arrs[2].transpose(2, 0, 1),
            },
            # "prompt": lang_embeddings,
        }
        try:
            _ = policy.infer(payload)
        except Exception as e:
            rospy.logwarn(f"[startup_warmup_infer] {e}")
    except Exception as e:
        rospy.logwarn(f"[startup_warmup_prep] {e}")
    # Initialize the previous action to be the initial robot state
    pre_action = np.zeros(config["state_dim"])
    action = None
    # Persistent accumulated joint correction offsets (first 6 joints)
    corr_left_q6 = np.zeros(6, dtype=float)
    corr_right_q6 = np.zeros(6, dtype=float)

    # Optional Kalman filters for action smoothing
    kalman_filters = create_kalman_filters(
        config["state_dim"]
    ) if getattr(args, "use_kalman_filter", False) else None

    # Inference loop
    with torch.inference_mode():
        while True and not rospy.is_shutdown():
            # The current time step
            t = 0
            rate = rospy.Rate(args.publish_rate)

            action_buffer = np.zeros([chunk_size, config["state_dim"]])

            while t < max_publish_step and not rospy.is_shutdown() and not shutdown_event.is_set():


                if not args.use_temporal_smoothing and not args.use_delta_eef_smoothing:
                    # Update observation window
                    update_observation_window(args, config, ros_operator)
                   
                    # When coming to the end of the action chunk
                    if t % chunk_size == 0:
                        # Start inference
                        action_buffer = inference_fn(args, config, policy).copy()
                        # Build corrected actions for the whole chunk (apply deltas on raw actions)
                        corrected_action_buffer = action_buffer.copy()

                        if args.use_ik_fine_tuning:
                            max_n = min(chunk_size, corrected_action_buffer.shape[0]) if hasattr(corrected_action_buffer, "shape") else chunk_size
                            if getattr(args, "eef_corr_all_action_chunk", True):
                                # Batch correction per raw action
                                ql_batch = corrected_action_buffer[:max_n, :6]
                                qr_batch = corrected_action_buffer[:max_n, 7:13]
                                ql_new_batch = apply_micro_correction_batch(
                                    ql_batch,
                                    args.eef_corr_left,
                                    args.eef_corr_left_frame,
                                    args.eef_corr_lambda,
                                    args.eef_corr_step_limit_m,
                                    args.eef_corr_joint_step_limits,
                                )
                                qr_new_batch = apply_micro_correction_batch(
                                    qr_batch,
                                    args.eef_corr_right,
                                    args.eef_corr_right_frame,
                                    args.eef_corr_lambda,
                                    args.eef_corr_step_limit_m,
                                    args.eef_corr_joint_step_limits,
                                )
                                corrected_action_buffer[:max_n, :6] = ql_new_batch
                                corrected_action_buffer[:max_n, 7:13] = qr_new_batch
                            else:
                                # Single delta computed from the first raw action, apply to all (batch)
                                ba0 = action_buffer[0]
                                ql_ref0 = np.array(ba0[:6], dtype=float)
                                qr_ref0 = np.array(ba0[7:13], dtype=float)
                                ql_new0 = apply_micro_correction(
                                    ql_ref0,
                                    args.eef_corr_left,
                                    args.eef_corr_left_frame,
                                    args.eef_corr_lambda,
                                    args.eef_corr_step_limit_m,
                                    args.eef_corr_joint_step_limits,
                                )
                                qr_new0 = apply_micro_correction(
                                    qr_ref0,
                                    args.eef_corr_right,
                                    args.eef_corr_right_frame,
                                    args.eef_corr_lambda,
                                    args.eef_corr_step_limit_m,
                                    args.eef_corr_joint_step_limits,
                                )
                                delta_left_single = (ql_new0 - ql_ref0)
                                delta_right_single = (qr_new0 - qr_ref0)
                                corrected_action_buffer[:max_n, :6] = action_buffer[:max_n, :6] + delta_left_single
                                corrected_action_buffer[:max_n, 7:13] = action_buffer[:max_n, 7:13] + delta_right_single

                    # Use corrected raw action for this step
                    raw_action = corrected_action_buffer[t % chunk_size]

                    # Optional Kalman filter based smoothing
                    if kalman_filters is not None:
                        raw_action = apply_kalman_filter_to_action(raw_action, kalman_filters)

                    # Interpolate the original action sequence (now already corrected at raw level)
                    if args.use_actions_interpolation:
                        if getattr(args, "interpolate_method", "linear") == "linear":
                            interp_actions = interpolate_action(args, pre_action, raw_action)
                        elif getattr(args, "interpolate_method", "linear") == "minimum_jerk":
                            interp_actions = minimum_jerk_interpolation(args, pre_action, raw_action)
                        else:
                            raise NotImplementedError(
                                f"Unknown interpolate_method: {args.interpolate_method}"
                            )
                    else:
                        interp_actions = raw_action[np.newaxis, :]
                    # Execute the interpolated actions one by one (no extra delta here)
                    for act in interp_actions:
                        if args.use_temporal_smoothing:
                            with ros_operator.lock:
                                if ros_operator.communication_flag:
                                    rate.sleep()
                                    continue
                                ros_operator.communication_flag = True
                        else:
                            if args.ctrl_type == "joint":
                                left_action = act[:7].copy()
                                right_action = act[7:14].copy()
                                left_action[6] = max(0.0, left_action[6]-RIGHT_OFFSET)
                                right_action[6] = max(0.0, right_action[6]-RIGHT_OFFSET)
                                ros_operator.puppet_arm_publish(left_action, right_action)
                                try:
                                    published_actions_history.append(np.concatenate([left_action, right_action], axis=0).astype(float))
                                except Exception:
                                    pass
                            elif args.ctrl_type == "eef":
                                left_action = act[:7]
                                right_action = act[7:14]

                                ros_operator.endpose_publish(left_action, right_action)
                            elif args.ctrl_type =="ik":
                                left_eef = act[:7]   # x,y,z,roll,pitch,yaw,gripper
                                right_eef = act[7:14]
                                # get current puppet arm qpos from latest observed_qpos (observation_window)
                                current_q = observation_window[-1]["qpos"]  # shape (14,)
                                ql_cur = current_q[:6]
                                qr_cur = current_q[7:13]

                                # form batch
                                q_batch = np.stack([ql_cur, qr_cur], axis=0)  # (2,6)
                                target_batch = np.stack([left_eef, right_eef], axis=0)  # (2,7)

                                q_solved = apply_ik_batch(
                                    q_batch,
                                    target_batch,
                                    frame=getattr(args, "eef_corr_left_frame", "base"),
                                    lambda_=getattr(args, "eef_corr_lambda", 0.01),
                                    max_step_limits=getattr(args, "eef_corr_joint_step_limits", [0.1]*6),
                                    max_iters=6,
                                    tol=1e-4
                                )  # (2,6)

                                left_q_new = list(q_solved[0]) + [left_eef[6]]   # append gripper as last element
                                right_q_new = list(q_solved[1]) + [right_eef[6]]
                                # apply RIGHT_OFFSET to right gripper joint if needed (follow original)
                                left_q_new[6] = max(0.0, left_q_new[6] - RIGHT_OFFSET)
                                right_q_new[6] = max(0.0, right_q_new[6] - RIGHT_OFFSET)
                                # publish as joint
                                ros_operator.puppet_arm_publish(left_q_new, right_q_new)
                        rate.sleep()
                    t += 1

                    print("Published Step", t)
                    try:
                        publish_step_global = len(published_actions_history)
                    except Exception:
                        pass
                    try:
                        published_actions_history.append(np.concatenate([left_action, right_action], axis=0).astype(float))
                    except Exception:
                        pass
                    # Track previous action as corrected raw action for next interpolation
                    pre_action = raw_action.copy()

             
                if shutdown_event.is_set():
                    break
                if args.use_temporal_smoothing:

                    if stream_buffer is None:
                        stream_buffer = StreamActionBuffer(
                            max_chunks=args.buffer_max_chunks,
                            decay_alpha=args.exp_decay_alpha,
                            state_dim=config["state_dim"],
                            smooth_method="temporal",
                        )
                        start_inference_thread(args, config, policy, ros_operator)

                    # pop next action from stream buffer
                    act = stream_buffer.pop_next_action()

                    if act is not None:
                        if args.ctrl_type == "joint":
                            left_action = act[:7].copy()
                            right_action = act[7:14].copy()
                            left_action[6] = max(0.0, left_action[6]-RIGHT_OFFSET)
                            right_action[6] = max(0.0, right_action[6]-RIGHT_OFFSET)
                            ros_operator.puppet_arm_publish(left_action, right_action)
                        elif args.ctrl_type == "eef":
                            left_action = act[:7]
                            right_action = act[7:14]
                            left_action[6] = max(0.0, left_action[6]-RIGHT_OFFSET)
                            right_action[6] = max(0.0, right_action[6]-RIGHT_OFFSET)
                            ros_operator.endpose_publish(left_action, right_action)
                        try:
                            published_actions_history.append(np.concatenate([left_action, right_action], axis=0).astype(float))
                        except Exception:
                            pass

                    else:

                        # camera rgb not received in time
                        # print("Stream buffer has been popped out, act is None")
                        time.sleep(0.001)
                        continue

                    print("Published Step", t)
                    try:
                        publish_step_global = len(published_actions_history)
                    except Exception:
                        pass

                    rate.sleep()
                    t += 1

                if shutdown_event.is_set():
                    break


# ROS operator class
class RosOperator:
    def __init__(self, args):
        # Helpers for temporal-smoothing communication mode
        self.communication_thread = None
        self.communication_flag = False
        self.lock = threading.Lock()
        self.robot_base_deque = None
        self.puppet_arm_right_deque = None
        self.puppet_arm_left_deque = None
        self.img_front_deque = None
        self.img_right_deque = None
        self.img_left_deque = None
        self.img_front_depth_deque = None
        self.img_right_depth_deque = None
        self.img_left_depth_deque = None
        self.bridge = None
        self.puppet_arm_left_publisher = None
        self.puppet_arm_right_publisher = None
        self.endpose_left_publisher = None
        self.endpose_right_publisher = None
        self.robot_base_publisher = None
        self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_lock = None
        self.args = args
        self.init()
        self.init_ros()

    def init(self):
        self.bridge = CvBridge()
        self.img_left_deque = deque()
        self.img_right_deque = deque()
        self.img_front_deque = deque()
        self.img_left_depth_deque = deque()
        self.img_right_depth_deque = deque()
        self.img_front_depth_deque = deque()
        self.puppet_arm_left_deque = deque()
        self.puppet_arm_right_deque = deque()
        self.robot_base_deque = deque()
        self.puppet_arm_publish_lock = threading.Lock()
        self.puppet_arm_publish_lock.acquire()

    def puppet_arm_publish(self, left, right):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()  # Set timestep
        joint_state_msg.name = [
            "joint0",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]  # joint names
        joint_state_msg.position = left
        self.puppet_arm_left_publisher.publish(joint_state_msg)
        joint_state_msg.position = right
        self.puppet_arm_right_publisher.publish(joint_state_msg)

    def endpose_publish(self, left, right):
        endpose_msg = PosCmd()
        endpose_msg.x, endpose_msg.y, endpose_msg.z = left[:3]
        endpose_msg.roll, endpose_msg.pitch, endpose_msg.yaw = left[3:6]
        endpose_msg.gripper = left[6]
        self.endpose_left_publisher.publish(endpose_msg)

        endpose_msg.x, endpose_msg.y, endpose_msg.z = right[:3]
        endpose_msg.roll, endpose_msg.pitch, endpose_msg.yaw = right[3:6]
        endpose_msg.gripper = right[6]
        self.endpose_right_publisher.publish(endpose_msg)

    def robot_base_publish(self, vel):
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vel[1]
        self.robot_base_publisher.publish(vel_msg)

    def puppet_arm_publish_continuous(self, left, right):
        rate = rospy.Rate(self.args.publish_rate)
        left_arm = None
        right_arm = None
        while True and not rospy.is_shutdown():
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break
        left_symbol = [1 if left[i] - left_arm[i] > 0 else -1 for i in range(len(left))]
        right_symbol = [1 if right[i] - right_arm[i] > 0 else -1 for i in range(len(right))]
        flag = True
        step = 0
        while flag and not rospy.is_shutdown():
            if self.puppet_arm_publish_lock.acquire(False):
                return
            left_diff = [abs(left[i] - left_arm[i]) for i in range(len(left))]
            right_diff = [abs(right[i] - right_arm[i]) for i in range(len(right))]
            flag = False
            for i in range(len(left)):
                if left_diff[i] < self.args.arm_steps_length[i]:
                    left_arm[i] = left[i]
                else:
                    left_arm[i] += left_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            for i in range(len(right)):
                if right_diff[i] < self.args.arm_steps_length[i]:
                    right_arm[i] = right[i]
                else:
                    right_arm[i] += right_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()  # Set the timestep
            joint_state_msg.name = [
                "joint0",
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ]  # joint names
            joint_state_msg.position = left_arm
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = right_arm
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            step += 1
            print("puppet_arm_publish_continuous:", step)
            rate.sleep()

    def puppet_arm_publish_linear(self, left, right):
        num_step = 100
        rate = rospy.Rate(200)

        left_arm = None
        right_arm = None

        while True and not rospy.is_shutdown():
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break

        traj_left_list = np.linspace(left_arm, left, num_step)
        traj_right_list = np.linspace(right_arm, right, num_step)

        for i in range(len(traj_left_list)):
            traj_left = traj_left_list[i]
            traj_right = traj_right_list[i]
            traj_left[-1] = left[-1]
            traj_right[-1] = right[-1]
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()  # 设置时间戳
            joint_state_msg.name = [
                "joint0",
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ]  # 设置关节名称
            joint_state_msg.position = traj_left
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = traj_right
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            rate.sleep()

    def puppet_arm_publish_continuous_thread(self, left, right):
        if self.puppet_arm_publish_thread is not None:
            self.puppet_arm_publish_lock.release()
            self.puppet_arm_publish_thread.join()
            self.puppet_arm_publish_lock.acquire(False)
            self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_thread = threading.Thread(target=self.puppet_arm_publish_continuous, args=(left, right))
        self.puppet_arm_publish_thread.start()

    def get_frame(self):
        if (
            len(self.img_left_deque) == 0
            or len(self.img_right_deque) == 0
            or len(self.img_front_deque) == 0
            or (
                self.args.use_depth_image
                and (
                    len(self.img_left_depth_deque) == 0
                    or len(self.img_right_depth_deque) == 0
                    or len(self.img_front_depth_deque) == 0
                )
            )
        ):
            return False
        if self.args.use_depth_image:
            frame_time = min(
                [
                    self.img_left_deque[-1].header.stamp.to_sec(),
                    self.img_right_deque[-1].header.stamp.to_sec(),
                    self.img_front_deque[-1].header.stamp.to_sec(),
                    self.img_left_depth_deque[-1].header.stamp.to_sec(),
                    self.img_right_depth_deque[-1].header.stamp.to_sec(),
                    self.img_front_depth_deque[-1].header.stamp.to_sec(),
                ]
            )
        else:
            frame_time = min(
                [
                    self.img_left_deque[-1].header.stamp.to_sec(),
                    self.img_right_deque[-1].header.stamp.to_sec(),
                    self.img_front_deque[-1].header.stamp.to_sec(),
                ]
            )

        if len(self.img_left_deque) == 0 or self.img_left_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.img_right_deque) == 0 or self.img_right_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.img_front_deque) == 0 or self.img_front_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.puppet_arm_left_deque) == 0 or self.puppet_arm_left_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if len(self.puppet_arm_right_deque) == 0 or self.puppet_arm_right_deque[-1].header.stamp.to_sec() < frame_time:
            return False
        if self.args.use_depth_image and (
            len(self.img_left_depth_deque) == 0 or self.img_left_depth_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False
        if self.args.use_depth_image and (
            len(self.img_right_depth_deque) == 0 or self.img_right_depth_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False
        if self.args.use_depth_image and (
            len(self.img_front_depth_deque) == 0 or self.img_front_depth_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False
        if self.args.use_robot_base and (
            len(self.robot_base_deque) == 0 or self.robot_base_deque[-1].header.stamp.to_sec() < frame_time
        ):
            return False

        while self.img_left_deque[0].header.stamp.to_sec() < frame_time:
            self.img_left_deque.popleft()
        img_left = self.bridge.imgmsg_to_cv2(self.img_left_deque.popleft(), "passthrough")

        while self.img_right_deque[0].header.stamp.to_sec() < frame_time:
            self.img_right_deque.popleft()
        img_right = self.bridge.imgmsg_to_cv2(self.img_right_deque.popleft(), "passthrough")

        while self.img_front_deque[0].header.stamp.to_sec() < frame_time:
            self.img_front_deque.popleft()
        img_front = self.bridge.imgmsg_to_cv2(self.img_front_deque.popleft(), "passthrough")

        while self.puppet_arm_left_deque[0].header.stamp.to_sec() < frame_time:
            self.puppet_arm_left_deque.popleft()
        puppet_arm_left = self.puppet_arm_left_deque.popleft()

        while self.puppet_arm_right_deque[0].header.stamp.to_sec() < frame_time:
            self.puppet_arm_right_deque.popleft()
        puppet_arm_right = self.puppet_arm_right_deque.popleft()

        img_left_depth = None
        if self.args.use_depth_image:
            while self.img_left_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_left_depth_deque.popleft()
            img_left_depth = self.bridge.imgmsg_to_cv2(self.img_left_depth_deque.popleft(), "passthrough")

        img_right_depth = None
        if self.args.use_depth_image:
            while self.img_right_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_right_depth_deque.popleft()
            img_right_depth = self.bridge.imgmsg_to_cv2(self.img_right_depth_deque.popleft(), "passthrough")

        img_front_depth = None
        if self.args.use_depth_image:
            while self.img_front_depth_deque[0].header.stamp.to_sec() < frame_time:
                self.img_front_depth_deque.popleft()
            img_front_depth = self.bridge.imgmsg_to_cv2(self.img_front_depth_deque.popleft(), "passthrough")

        robot_base = None
        if self.args.use_robot_base:
            while self.robot_base_deque[0].header.stamp.to_sec() < frame_time:
                self.robot_base_deque.popleft()
            robot_base = self.robot_base_deque.popleft()

        return (
            img_front,
            img_left,
            img_right,
            img_front_depth,
            img_left_depth,
            img_right_depth,
            puppet_arm_left,
            puppet_arm_right,
            robot_base,
        )

    def img_left_callback(self, msg):
        if len(self.img_left_deque) >= 2000:
            self.img_left_deque.popleft()
        self.img_left_deque.append(msg)

    def img_right_callback(self, msg):
        if len(self.img_right_deque) >= 2000:
            self.img_right_deque.popleft()
        self.img_right_deque.append(msg)

    def img_front_callback(self, msg):
        if len(self.img_front_deque) >= 2000:
            self.img_front_deque.popleft()
        self.img_front_deque.append(msg)

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= 2000:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= 2000:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= 2000:
            self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)

    def puppet_arm_left_callback(self, msg):
        if len(self.puppet_arm_left_deque) >= 2000:
            self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)

    def puppet_arm_right_callback(self, msg):
        if len(self.puppet_arm_right_deque) >= 2000:
            self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)

    def robot_base_callback(self, msg):
        if len(self.robot_base_deque) >= 2000:
            self.robot_base_deque.popleft()
        self.robot_base_deque.append(msg)

    def init_ros(self):
        rospy.init_node("joint_state_publisher", anonymous=True)
        rospy.Subscriber(
            self.args.img_left_topic,
            Image,
            self.img_left_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.img_right_topic,
            Image,
            self.img_right_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.img_front_topic,
            Image,
            self.img_front_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        if self.args.use_depth_image:
            rospy.Subscriber(
                self.args.img_left_depth_topic,
                Image,
                self.img_left_depth_callback,
                queue_size=1000,
                tcp_nodelay=True,
            )
            rospy.Subscriber(
                self.args.img_right_depth_topic,
                Image,
                self.img_right_depth_callback,
                queue_size=1000,
                tcp_nodelay=True,
            )
            rospy.Subscriber(
                self.args.img_front_depth_topic,
                Image,
                self.img_front_depth_callback,
                queue_size=1000,
                tcp_nodelay=True,
            )
        rospy.Subscriber(
            self.args.puppet_arm_left_topic,
            JointState,
            self.puppet_arm_left_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.puppet_arm_right_topic,
            JointState,
            self.puppet_arm_right_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.args.robot_base_topic,
            Odometry,
            self.robot_base_callback,
            queue_size=1000,
            tcp_nodelay=True,
        )
        self.puppet_arm_left_publisher = rospy.Publisher(self.args.puppet_arm_left_cmd_topic, JointState, queue_size=10)
        self.puppet_arm_right_publisher = rospy.Publisher(
            self.args.puppet_arm_right_cmd_topic, JointState, queue_size=10
        )
        self.endpose_left_publisher = rospy.Publisher(self.args.endpose_left_cmd_topic, PosCmd, queue_size=10)
        self.endpose_right_publisher = rospy.Publisher(self.args.endpose_right_cmd_topic, PosCmd, queue_size=10)
        self.robot_base_publisher = rospy.Publisher(self.args.robot_base_cmd_topic, Twist, queue_size=10)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--max_publish_step",
        action="store",
        type=int,
        help="Maximum number of action publishing steps",
        default=10000,
        required=False,
    )
    parser.add_argument(
        "--seed",
        action="store",
        type=int,
        help="Random seed",
        default=None,
        required=False,
    )
    parser.add_argument(
        "--img_front_topic",
        action="store",
        type=str,
        help="img_front_topic",
        default="/camera_f/color/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_left_topic",
        action="store",
        type=str,
        help="img_left_topic",
        default="/camera_l/color/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_right_topic",
        action="store",
        type=str,
        help="img_right_topic",
        default="/camera_r/color/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_front_depth_topic",
        action="store",
        type=str,
        help="img_front_depth_topic",
        default="/camera_f/depth/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_left_depth_topic",
        action="store",
        type=str,
        help="img_left_depth_topic",
        default="/camera_l/depth/image_raw",
        required=False,
    )
    parser.add_argument(
        "--img_right_depth_topic",
        action="store",
        type=str,
        help="img_right_depth_topic",
        default="/camera_r/depth/image_raw",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_left_cmd_topic",
        action="store",
        type=str,
        help="puppet_arm_left_cmd_topic",
        default="/master/joint_left",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_right_cmd_topic",
        action="store",
        type=str,
        help="puppet_arm_right_cmd_topic",
        default="/master/joint_right",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_left_topic",
        action="store",
        type=str,
        help="puppet_arm_left_topic",
        default="/puppet/joint_left",
        required=False,
    )
    parser.add_argument(
        "--puppet_arm_right_topic",
        action="store",
        type=str,
        help="puppet_arm_right_topic",
        default="/puppet/joint_right",
        required=False,
    )
    parser.add_argument(
        "--endpose_left_cmd_topic",
        action="store",
        type=str,
        help="endpose_left_cmd_topic",
        default="/pos_cmd_left",
        required=False,
    )
    parser.add_argument(
        "--endpose_right_cmd_topic",
        action="store",
        type=str,
        help="endpose_right_cmd_topic",
        default="/pos_cmd_right",
        required=False,
    )
    parser.add_argument(
        "--robot_base_topic",
        action="store",
        type=str,
        help="robot_base_topic",
        default="/odom_raw",
        required=False,
    )
    parser.add_argument(
        "--robot_base_cmd_topic",
        action="store",
        type=str,
        help="robot_base_topic",
        default="/cmd_vel",
        required=False,
    )
    parser.add_argument(
        "--use_robot_base",
        action="store_true",
        help="Whether to use the robot base to move around",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--publish_rate",
        action="store",
        type=int,
        help="The rate at which to publish the actions",
        default=30,
        required=False,
    )
    parser.add_argument(
        "--chunk_size",
        action="store",
        type=int,
        help="Action chunk size",
        default=50,
        required=False,
    )
    parser.add_argument(
        "--arm_steps_length",
        action="store",
        type=float,
        help="The maximum change allowed for each joint per timestep",
        default=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.2],
        required=False,
    )
    parser.add_argument(
        "--use_actions_interpolation",
        action="store_true",
        help="Whether to interpolate the actions if the difference is too large",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--use_kalman_filter",
        action="store_true",
        help="Whether to apply a per-dimension Kalman filter to smooth actions",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--interpolate_method",
        type=str,
        choices=["linear", "minimum_jerk"],
        help="Interpolation method for action smoothing (if enabled)",
        default="linear",
    )
    parser.add_argument(
        "--jerk_num_steps",
        action="store",
        type=int,
        help="Number of interpolation steps for minimum-jerk interpolation",
        default=10,
        required=False,
    )
    parser.add_argument(
        "--use_depth_image",
        action="store_true",
        help="Whether to use depth images",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--host",
        action="store",
        type=str,
        help="Websocket server host",
        default="localhost",
        required=False,
    )
    parser.add_argument(
        "--port",
        action="store",
        type=int,
        help="Websocket server port",
        default=8000,
        required=False,
    )

    parser.add_argument(
        "--ctrl_type",
        type=str,
        choices=["joint", "eef", "ik"],
        help="Control type for the robot arm",
        default="joint",
    )

    parser.add_argument(
        "--use_ik_fine_tuning",
        action="store_true",
        help="Whether to use IK fine-tuning",
        default=False,
        required=False,
    )

    parser.add_argument(
        "--eef_corr_left",
        nargs=3,
        type=float,
        metavar=("DX","DY","DZ"),
        help="Left arm EE micro-correction (meters) in selected frame",
        default=[0.0, 0.0, 0.0],
        required=False,
    )
    parser.add_argument(
        "--eef_corr_right",
        nargs=3,
        type=float,
        metavar=("DX","DY","DZ"),
        help="Right arm EE micro-correction (meters) in selected frame",
        default=[0.0, 0.0, -0.1],
        required=False,
    )
    parser.add_argument(
        "--eef_corr_left_frame",
        type=str,
        choices=["tool", "base"],
        help="Interpret left correction in this frame",
        default="base",
        required=False,
    )
    parser.add_argument(
        "--eef_corr_right_frame",
        type=str,
        choices=["tool", "base"],
        help="Interpret right correction in this frame",
        default="base",
        required=False,
    )
    parser.add_argument(
        "--eef_corr_lambda",
        type=float,
        help="Damping for DLS (position-only IK step)",
        default=0.001,
        required=False,
    )
    parser.add_argument(
        "--eef_corr_step_limit_m",
        type=float,
        help="Max EE correction magnitude per cycle (meters)",
        default=0.5,
        required=False,
    )
    parser.add_argument(
        "--eef_corr_joint_step_limits",
        nargs=6,
        type=float,
        metavar=("dq1","dq2","dq3","dq4","dq5","dq6"),
        help="Per-joint max step (rad) applied to DLS increment",
        default=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        required=False,
    )
    parser.add_argument(
        "--eef_corr_all_action_chunk",
        action="store_true",
        help="Apply EE correction per action in a chunk before interpolation (default ON)",
        default=True,
        required=False,
    )

    parser.add_argument(
        "--use_temporal_smoothing",
        action="store_true",
        help="Enable non-blocking communication and control execution",
        default=False,
        required=False,
    )
    parser.add_argument(
        "--latency_k",
        type=int,
        help="Max Latency in steps",
        default=8,
        required=False,
    )
    parser.add_argument(
        "--inference_rate",
        type=float,
        help="Inference loop rate (Hz)",
        default=3.0,
        required=False,
    )
    parser.add_argument(
        "--min_smooth_steps",
        type=int,
        help="Minimum smoothing steps m",
        default=8,
        required=False,
    )
    parser.add_argument(
        "--buffer_max_chunks",
        type=int,
        help="Maximum number of chunks in the buffer",
        default=10,
        required=False,
    )
    parser.add_argument(
        "--exp_decay_alpha",
        type=float,
        help="Exponential decay alpha",
        default=0.25,
        required=False,
    )
    
    parser.add_argument(
        "--use_delta_eef_smoothing",
        action="store_true",
        help="Whether to use delta eef smoothing",
        default=False,
        required=False,
    )

    parser.add_argument(
        "--eef_frame",
        type=str,
        choices=["base", "tool"],
        default="base",
        help="Frame for interpreting model eef outputs",
    )

    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    ros_operator = RosOperator(args)
    if args.seed is not None:
        set_seed(args.seed)
    config = get_config(args)
    # 注册 SIGINT 处理器，确保 Ctrl+C 时一定保存图
    signal.signal(signal.SIGINT, _on_sigint)
    try:
        model_inference(args, config, ros_operator)
    except KeyboardInterrupt:
        pass
    finally:
        save_debug_plots_on_interrupt()


if __name__ == "__main__":
    main()
