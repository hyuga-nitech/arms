from threading import local
import NatNetClient
from FileIO.FileIO import FileIO
import numpy as np

class OptiTrackStreamingManager:
    # ---------- Settings: Optitrack streaming address ---------- #
    serverAddress	= '192.168.1.1'
    localAddress	= '192.168.1.2'

    # ---------- Variables ---------- #
    position = {}	# dict { 'RigidBodyN': [x, y, z] }.  Unit = [m]
    rotation = {}	# dict { 'RigidBodyN': [x, y, z, w]}. 
    
    def __init__(self,defaultRigidBodyNum: int = 2): #RigidBody1 is for xArm , RigidBody2 is for mikataArm. if need some additional RigidBody , use after RigidBody3.
        for i in range(defaultRigidBodyNum):
            self.position['RigidBody'+str(i+1)] = np.zeros(3)
            self.rotation['RigidBody'+str(i+1)] = np.zeros(4)
        
        fileIO = FileIO()
        settings = fileIO.Read('settings.csv',',')
        serverIP = [addr for addr in settings if 'motiveServer' in addr[0]][0][1]
        localIP =[addr for addr in settings if 'motiveLocal' in addr [0]][0][1]

        self.serverAddress = serverIP
        self.localAddress = localIP

    def print(self):
        print(self.serverAddress)
        print(self.localAddress)

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
        if 'participant'+str(new_id) in self.position:
            self.position['participant'+str(new_id)] = np.array(position)
            self.rotation['participant'+str(new_id)] = np.array(rotation)
        
        if new_id == 3:
            self.position['endEffector'] = np.array(position)
            self.rotation['endEffector'] = np.array(rotation)


    def stream_run(self):
        streamingClient = NatNetClient.NatNetClient(serverIP=self.serverAddress, localIP=self.localAddress)
        
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streamingClient.new_frame_listener = self.receive_new_frame
        streamingClient.rigid_body_listener = self.receive_rigid_body_frame
        streamingClient.run()

a=OptiTrackStreamingManager()
a.print()