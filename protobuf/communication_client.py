import zmq
import protobuf.heatmap_pb2
import numpy as np

port = "5556"


class CommunicationChannel:

    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5556")

    def send(self, simulation_time, heatmap: np.ndarray, max_lat, max_long, min_lat, min_long):
        heatmap_message: protobuf.heatmap_pb2.Heatmap = protobuf.heatmap_pb2.Heatmap()
        heatmap_message.time = simulation_time
        heatmap_message.max_lat = max_lat
        heatmap_message.max_long = max_long
        heatmap_message.min_lat = min_lat
        heatmap_message.min_long = min_long
        size = heatmap.shape
        size = list(size)
        heatmap_message.size[:] = size
        heatmap_message.map[:] = list(heatmap.flatten())
        self.socket.send(heatmap_message.SerializeToString())
