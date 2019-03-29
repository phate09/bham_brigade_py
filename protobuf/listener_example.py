import zmq
import protobuf.heatmap_pb2
import numpy as np

from protobuf import heatmap_pb2

port="5556"

class HeatmapListener:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5556")
        filter="" #filters out a specific type of message
        self.socket.setsockopt_string(zmq.SUBSCRIBE, filter)

    def start(self):
        while(True):
            recv_string = self.socket.recv_string()#receive the string from the channel
            heatmap:heatmap_pb2.Heatmap = heatmap_pb2.Heatmap() #creates the heatmap object
            heatmap.ParseFromString(recv_string) #fills it with the data
            self.onMessageReceived(heatmap)

    def onMessageReceived(self,heatmap:heatmap_pb2.Heatmap):
        pass #fills it with the method to consume the heatmap
