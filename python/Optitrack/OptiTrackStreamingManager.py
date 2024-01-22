from Optitrack import NatNetClient
import numpy as np
import json as js

class OptiTrackStreamingManager:
    # ---------- Settings: Optitrack streaming address ---------- #
    serverAddress	= '133.68.108.109'
    localAddress	= '133.68.108.109'

    # ---------- Variables ---------- #
    position = {}	# dict { 'RigidBodyN': [x, y, z] }.  Unit = [m]
    rotation = {}	# dict { 'RigidBodyN': [x, y, z, w]}. 
    
    def __init__(self):
        rigidbody_f = open("rigidbody_setting.json","r")
        self.rigidbody_js = js.load(rigidbody_f)

        for rigidbody in self.rigidbody_js["RigidBodyConfig"]:
            self.position[rigidbody] = np.zeros(3)
            self.rotation[rigidbody] = np.zeros(4)

    # This is a callback function that gets connected to the NatNet client and called once per mocap frame.
    def receive_new_frame(self, data_dict):
        order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                    "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
        dump_args = False
        if dump_args == True:
            out_string = "    "
            for key in data_dict:
                out_string += key + "="
                if key in data_dict :
                    out_string += data_dict[key] + " "
                out_string+="/"
            print(out_string)

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receive_rigid_body_frame( self, new_id, position, rotation ):
        """
        Receives the position and rotation of the active RigidBody.
        Position: [x, y, z], Unit = [m]
        Rotation: [x, y, z, w]

        Parameters
        ----------
        new_id: int
            RigidBody id
        position: array
            Position
        rotation: array
            Rotation
        """
        if 'RigidBody'+str(new_id) in self.position:
            self.position['RigidBody'+str(new_id)] = np.array(position)
            self.rotation['RigidBody'+str(new_id)] = np.array(rotation)

    def stream_run(self):
        streamingClient = NatNetClient.NatNetClient(serverIP=self.serverAddress, localIP=self.localAddress)
        
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streamingClient.new_frame_listener = self.receive_new_frame
        streamingClient.rigid_body_listener = self.receive_rigid_body_frame
        streamingClient.run()