import airsim
from message_types.msg_state import MsgState
from tools.rotations import rotation_to_quaternion

class QuadViewer:
    def __init__(self):
        self._client = airsim.VehicleClient()
        self._client.confirmConnection()

    def update(self, state):
        # 'state' is of type MsgState

        poseObj = self._client.simGetVehiclePose()
        poseObj.position.x_val = state.pos.item(0)
        poseObj.position.y_val = state.pos.item(1)
        poseObj.position.z_val = state.pos.item(2)
        # get the quaternion of the state
        quat = rotation_to_quaternion(state.rot).reshape((4))
        poseObj.orientation.w_val = quat.item(0)
        poseObj.orientation.x_val = quat.item(1)
        poseObj.orientation.y_val = quat.item(2)
        poseObj.orientation.z_val = quat.item(3)

        self._client.simSetVehiclePose(pose=poseObj, ignore_collision=True)