"""Unitree HG LowState transport, shared by both simulator processes."""
import math
import queue
import time
from .contracts import RobotState


def initialize(config):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(config['domain'], config['interface'])


def decode(message, joints):
    q = tuple(float(m.q) for m in message.motor_state[:len(joints)])
    dq = tuple(float(m.dq) for m in message.motor_state[:len(joints)])
    if len(q) != len(joints) or not all(math.isfinite(x) for x in q + dq):
        raise ValueError('Incomplete or non-finite joint state')
    return RobotState(tuple(joints), q, dq, int(message.tick), time.monotonic())


class DDSRobotInterface:
    """Fresh joint telemetry only. IMU, torques, hands and commands are unsupported."""
    def __init__(self, config, joints):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self.joints = joints
        self.messages = queue.Queue(maxsize=1)
        self.last_tick = None
        self.subscriber = ChannelSubscriber(config['state_topic'], LowState_)
        self.subscriber.Init(self._receive, 1)

    def _receive(self, message):
        try:
            state = decode(message, self.joints)
        except ValueError as error:
            state = error
        try:
            self.messages.get_nowait()
        except queue.Empty:
            pass
        try:
            self.messages.put_nowait(state)
        except queue.Full:
            pass

    def read_state(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('No fresh DDS joint state received')
            try:
                state = self.messages.get(timeout=remaining)
            except queue.Empty as error:
                raise TimeoutError('No DDS state: check backend logs, SDK2 and domain/interface') from error
            if isinstance(state, Exception):
                raise state
            if state.tick != self.last_tick:
                self.last_tick = state.tick
                return state

    def close(self):
        self.subscriber.Close()


class StatePublisher:
    def __init__(self, config):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.utils.crc import CRC
        self.message = unitree_hg_msg_dds__LowState_()
        self.crc = CRC()
        self.publisher = ChannelPublisher(config['state_topic'], LowState_)
        self.publisher.Init()

    def write(self, q, dq, tick):
        if len(q) != 29 or len(dq) != 29:
            raise ValueError('Expected 29 measured body joints')
        if not all(math.isfinite(float(x)) for x in list(q) + list(dq)):
            raise ValueError('Non-finite backend state')
        for index, (position, velocity) in enumerate(zip(q, dq)):
            self.message.motor_state[index].q = float(position)
            self.message.motor_state[index].dq = float(velocity)
        self.message.tick = tick % (2**32)
        self.message.crc = self.crc.Crc(self.message)
        self.publisher.Write(self.message)

    def close(self):
        self.publisher.Close()
