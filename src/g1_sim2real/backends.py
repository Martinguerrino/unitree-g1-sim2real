"""Asset adapters. Imported only in the selected simulator's own process.

The MVP samples initialized, paused scenes. It does not run control or physics
benchmarks and never invokes the upstream task demos.
"""
from contextlib import contextmanager
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET
from .config import joint_indices


@contextmanager
def mujoco_state(root, config):
    import mujoco
    repository = root / config['backend']['repository']
    backend = config['backend']
    with tempfile.TemporaryDirectory(prefix='g1-mjcf-') as directory:
        generated = Path(directory)
        model_xml = ET.parse(repository / backend['model'])
        compiler = model_xml.getroot().find('compiler')
        if compiler is None:
            raise ValueError('External MJCF has no compiler element')
        compiler.set('meshdir', str((repository / backend['meshes']).resolve()))
        model_xml.write(generated / 'g1_29dof.xml')
        scene = ET.parse(repository / backend['scene'])
        scene.write(generated / 'scene.xml')
        model = mujoco.MjModel.from_xml_path(str(generated / 'scene.xml'))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
        indices = joint_indices(names, config['joints'])
        positions = [model.jnt_qposadr[i] for i in indices]
        velocities = [model.jnt_dofadr[i] for i in indices]
        yield lambda: (data.qpos[positions].copy(), data.qvel[velocities].copy())


@contextmanager
def isaac_state(root, config):
    # SimulationApp must precede omni/pxr/core imports.
    from isaacsim import SimulationApp
    backend = config['backend']
    app = SimulationApp({'headless': backend['headless'], 'width': 640, 'height': 480})
    try:
        import omni.usd
        import omni.timeline
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import is_stage_loading
        from pxr import PhysxSchema, Usd

        path = (root / backend['repository'] / backend['scene']).resolve()
        if omni.usd.get_context().open_stage(str(path)) is False:
            raise RuntimeError(f'Could not open {path}')
        deadline = time.monotonic() + config['telemetry']['startup_timeout_s']
        app.update()
        app.update()
        while is_stage_loading():
            if time.monotonic() >= deadline:
                raise TimeoutError('Isaac USD payload loading timed out')
            app.update()
        stage = omni.usd.get_context().get_stage()
        # All changes stay in the session layer, never in the external USD.
        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        prim = stage.GetPrimAtPath(backend['robot_prim'])
        if not prim.IsValid():
            raise ValueError(f"Robot prim missing: {backend['robot_prim']}")
        for child in Usd.PrimRange(prim):
            if child.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI(child).CreateDisableGravityAttr(True)
        robot = SingleArticulation(prim_path=backend['robot_prim'], name='g1_dds_probe', reset_xform_properties=False)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        app.update()
        timeline.pause()
        app.update()
        robot.initialize()
        if not robot.handles_initialized:
            raise RuntimeError('Isaac articulation handles were not initialized')
        indices = joint_indices(robot.dof_names, config['joints'])

        def read():
            app.update()
            return (robot.get_joint_positions()[indices], robot.get_joint_velocities()[indices])

        yield read
    finally:
        app.close()
