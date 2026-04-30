"""Model-agnostic inference loop for the YAM bimanual robot.

Provides:

- ``ModelBridge`` — abstract interface for model-specific logic
- ``RaidenInferenceLoop`` — camera/motor infrastructure + control loop

The split: **raiden** owns hardware (cameras, motors, 14D joint commands).
The **model repo** owns everything model-specific (action space, preprocessing,
inference, action chunking).

Usage from a model repo::

    from raiden.inference import ModelBridge, RaidenInferenceLoop

    class MyBridge(ModelBridge):
        def load(self, ckpt_path, **kwargs): ...
        def predict(self, obs):              ...  # returns (14,)

    if __name__ == "__main__":
        bridge = MyBridge()
        loop = RaidenInferenceLoop(bridge, ckpt_path="...", action_hz=30)
        loop.run()

Or via CLI::

    rd infer --bridge my_repo.deployment.bridge:MyBridge \\
             --ckpt_path /path/to/checkpoint.pt

Action layout
-------------
``predict()`` must return a ``(14,)`` float32 array using raiden's bimanual
convention::

    [0:6]   right arm joint angles (rad)
    [6]     right gripper (0=open, 1=closed)
    [7:13]  left arm joint angles (rad)
    [13]    left gripper (0=open, 1=closed)

Thread layout (inherited from RaidenPolicyServer)::

    camera-<name>       : grabs frames at ~30 Hz (per camera)
    proprio             : reads joint state at ~100 Hz
    main thread         : inference loop (gated by bridge.predict() speed)
"""

import importlib
import time
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
from chiral.types import Observation

from raiden.server import RaidenPolicyServer

DOF = 7  # joints per arm (6 revolute + 1 gripper)


# ---------------------------------------------------------------------------
# Abstract bridge interface
# ---------------------------------------------------------------------------


class ModelBridge(ABC):
    """Abstract interface between a policy model and the YAM robot.

    Implement this in your model's repo to handle all model-specific logic:

    - Model loading (checkpoint format, framework imports)
    - Observation preprocessing (image resize, pointmaps, state assembly,
      normalization — whatever your model's action space requires)
    - Inference (forward pass)
    - Action postprocessing (denormalization, chunking, format conversion)

    The ``predict()`` method receives a raw ``Observation`` from the robot
    and must return a ``(14,)`` float32 array of motor commands::

        [0:6]   right arm joint angles (rad)
        [6]     right gripper (0=open, 1=closed)
        [7:13]  left arm joint angles (rad)
        [13]    left gripper (0=open, 1=closed)

    Action chunking, if used, should be managed internally — ``predict()``
    is called once per control step and should return exactly one action.
    """

    @abstractmethod
    def load(self, ckpt_path: str, **kwargs) -> None:
        """Load model from checkpoint.

        Args:
            ckpt_path: Path to model checkpoint file.
            **kwargs: Model-specific arguments (e.g. ``host``, ``port``,
                ``action_horizon``, ``prompt``).
        """

    @abstractmethod
    def predict(self, obs: Observation) -> np.ndarray:
        """Given a raw observation, return motor commands.

        Args:
            obs: Raw observation containing:

                - ``obs.cameras``: list of ``CameraInfo``
                  (each has ``.name``, ``.image``, ``.depth``,
                  ``.intrinsics``, ``.extrinsics``)
                - ``obs.proprios``: dict of ``(N,)`` float32 arrays
                  (``follower_r_joint_pos``, ``follower_l_joint_pos``, etc.)
                - ``obs.timestamp``: float

        Returns:
            ``(14,)`` float32 motor commands:
            ``[right_joints(6), right_grip(1), left_joints(6), left_grip(1)]``
        """

    def reset(self) -> None:
        """Called when the robot is homed. Override to reset internal state
        (e.g. clear action chunk buffer)."""


# ---------------------------------------------------------------------------
# Inference loop
# ---------------------------------------------------------------------------


class RaidenInferenceLoop(RaidenPolicyServer):
    """Model-agnostic inference loop for the YAM robot.

    Reuses ``RaidenPolicyServer``'s camera capture, depth computation,
    proprioception reading, and motor control.  Delegates all model-specific
    logic to a :class:`ModelBridge` instance.

    Args:
        bridge: A ``ModelBridge`` implementation from the model repo.
        ckpt_path: Passed to ``bridge.load()``.
        action_hz: Control loop frequency in Hz (default 30, should match
            training data frame rate).
        bridge_kwargs: Extra keyword arguments forwarded to ``bridge.load()``.
        **kwargs: Forwarded to ``RaidenPolicyServer`` (camera_config_file,
            calibration_file, stereo_method, etc.).
    """

    def __init__(
        self,
        bridge: ModelBridge,
        ckpt_path: str,
        action_hz: float = 30.0,
        bridge_kwargs: Optional[dict] = None,
        **kwargs,
    ):
        self._bridge = bridge
        self._action_hz = action_hz

        # Load model FIRST — this can take seconds (downloading weights,
        # building the network) and doesn't need hardware.  Loading after
        # robot init causes CAN timeouts on idle motor chains.
        print(f"\nLoading model from {ckpt_path}...")
        self._bridge.load(ckpt_path, **(bridge_kwargs or {}))
        print("Model loaded.\n")

        # Now initialize cameras, proprio threads, and robots (inherited).
        super().__init__(**kwargs)

    def _safety_check_rl(self, action_14d: np.ndarray) -> None:
        """Trip the e-stop if any joint delta exceeds ``_max_joint_delta``.

        Uses raiden's bimanual inference layout: ``[r(7), l(7)]``.  This is
        independent of ``RaidenPolicyServer._check_joint_delta`` (which uses
        the ``[l, r]`` layout served over the WebSocket protocol).
        """
        pairs = []
        if self._robot.follower_r:
            q_r = self._read_proprio("follower_r_joint_pos")
            if q_r is not None:
                pairs.append(("right", q_r, action_14d[:DOF]))
        if self._robot.follower_l:
            q_l = self._read_proprio("follower_l_joint_pos")
            if q_l is not None:
                pairs.append(("left", q_l, action_14d[DOF : DOF * 2]))

        for arm, current, commanded in pairs:
            # Only check the 6 arm joints — gripper uses a wider linear range.
            delta = np.abs(commanded[:6] - current[:6])
            max_delta = float(delta.max())
            if max_delta > self._max_joint_delta:
                joint_idx = int(delta.argmax())
                print(
                    f"\n[SAFETY] Dangerously large joint delta on {arm} arm — "
                    f"joint {joint_idx}: {max_delta:.4f} rad "
                    f"(limit={self._max_joint_delta:.4f} rad). "
                    "Triggering emergency stop."
                )
                self._estop_active.set()
                self._robot.emergency_stop()

    def run(self) -> None:
        """Run the closed-loop inference loop.  Blocks until Ctrl+C."""
        dt = 1.0 / self._action_hz
        step = 0

        # Home the robot before inference so the first commanded action does
        # not cause a large jump (RaidenPolicyServer.__init__ does not home).
        print("Homing robot...")
        self._robot.move_to_home_positions(simultaneous=True)

        self._bridge.reset()

        print(f"Starting inference at {self._action_hz} Hz")
        print("Press Ctrl+C to stop.\n")

        try:
            while True:
                t_step = time.perf_counter()

                # Build observation from live sensors.
                obs = self._make_obs()

                # Ask bridge for next motor command (raiden layout: [r, l]).
                action = self._bridge.predict(obs)

                # Convert EE pose to joint command if needed.  Note: the
                # parent's ``_ee_pose_to_joint_cmd`` returns ``[l, r]``; for
                # a model that emits raiden-layout EE poses, callers should
                # set ``action_type="joint"`` and let the bridge produce the
                # ``[r, l]`` joint command directly.
                if self._action_type == "ee_pose":
                    init_cmd = self._last_joint_cmd
                    action_14d = self._ee_pose_to_joint_cmd(action, init_cmd)
                else:
                    action_14d = action

                # Safety check — abort if any joint jumps too far ([r, l]).
                self._safety_check_rl(action_14d)

                self._last_joint_cmd = action_14d

                # Command motors (raiden inference layout: right first, then left).
                if self._robot.follower_r:
                    self._robot.follower_r.command_joint_pos(action_14d[:DOF])
                if self._robot.follower_l:
                    self._robot.follower_l.command_joint_pos(action_14d[DOF : DOF * 2])

                step += 1

                elapsed = time.perf_counter() - t_step
                hz = 1.0 / max(elapsed, 1e-6)
                r_act = np.array2string(action_14d[:DOF], precision=3, suppress_small=True)
                l_act = np.array2string(
                    action_14d[DOF : DOF * 2], precision=3, suppress_small=True
                )
                print(f"step={step:5d}  hz={hz:.1f}  r={r_act}  l={l_act}")

                # Sleep to maintain target Hz.
                elapsed = time.perf_counter() - t_step
                remaining = dt - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            print("\n\nStopping inference...")
        finally:
            self._bridge.reset()
            if step > 0:
                print("Returning to home...")
                self._robot.move_to_home_positions(simultaneous=True)
            self._running = False
            time.sleep(0.1)
            self.close()
            print("Done.")


# ---------------------------------------------------------------------------
# Dynamic bridge loading (used by ``rd infer``)
# ---------------------------------------------------------------------------


def load_bridge(bridge_spec: str, **kwargs) -> ModelBridge:
    """Import a ModelBridge class from a ``module.path:ClassName`` string.

    Extra *kwargs* are forwarded to the bridge constructor.

    Examples::

        load_bridge("deployment.openpi_bridge:OpenPiBridge")
        load_bridge("deployment.openpi_bridge:OpenPiBridge", action_horizon=10)
    """
    if ":" not in bridge_spec:
        raise ValueError(
            f"Bridge spec must be 'module.path:ClassName', got '{bridge_spec}'"
        )
    module_path, class_name = bridge_spec.rsplit(":", 1)
    try:
        module = importlib.import_module(module_path)
    except ModuleNotFoundError as exc:
        raise ImportError(
            f"Cannot import bridge module '{module_path}'. "
            f"Make sure it is installed or on PYTHONPATH."
        ) from exc
    cls = getattr(module, class_name, None)
    if cls is None:
        raise AttributeError(
            f"Module '{module_path}' has no attribute '{class_name}'"
        )
    bridge = cls(**kwargs)
    if not isinstance(bridge, ModelBridge):
        raise TypeError(f"'{bridge_spec}' is not a ModelBridge subclass")
    return bridge
