import zmq
import protobuf.heatmap_pb2
import numpy as np

port = "5556"


class CommunicationChannel:

    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5556")

    def send(self, simulation_time, heatmap: np.ndarray):
        heatmap_message: protobuf.heatmap_pb2.Heatmap = protobuf.heatmap_pb2.Heatmap()
        heatmap_message.time = simulation_time
        size = heatmap.shape
        size = list(size)
        heatmap_message.size[:] = size
        heatmap_message.map[:] = list(heatmap.flatten())
        self.socket.send(heatmap_message.SerializeToString())
