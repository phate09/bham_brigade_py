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
        # filter = filter.decode('ascii')
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')

    def start(self):
        while(True):
            message = self.socket.recv()#receive the string from the channel
            heatmap: heatmap_pb2.Heatmap = heatmap_pb2.Heatmap()  # creates the heatmap object
            heatmap.ParseFromString(message)
             #fills it with the data
            self.onMessageReceived(heatmap)
    # def deserialize(self,object):
    #     heatmap: heatmap_pb2.Heatmap = heatmap_pb2.Heatmap()  # creates the heatmap object
    #     heatmap.ParseFromString(object)
    def onMessageReceived(self,heatmap:heatmap_pb2.Heatmap):
        print(heatmap)
        pass #fills it with the method to consume the heatmap

if __name__ == '__main__':
    listener= HeatmapListener()
    listener.start()
