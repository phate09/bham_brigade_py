from afrl.cmasi.SessionStatus import SessionStatus, SimulationStatusType
from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from afrl.cmasi.Circle import Circle
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.Waypoint import Waypoint
from afrl.cmasi.VehicleActionCommand import VehicleActionCommand
from afrl.cmasi.LoiterAction import LoiterAction
from afrl.cmasi.LoiterType import LoiterType
from afrl.cmasi.LoiterDirection import LoiterDirection
from afrl.cmasi.CommandStatusType import CommandStatusType
from afrl.cmasi.AltitudeType import AltitudeType
from afrl.cmasi.searchai.HazardZoneDetection import HazardZoneDetection
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.Location3D import Location3D
import xml.etree.ElementTree
import matplotlib.pyplot as plt
from xml.dom import minidom
from visdom import Visdom
import argparse
import numpy as np
import protobuf.communication_client
from scipy.spatial import ConvexHull

CONTOUR = "contour"

VIZ_SCATTER = "viz_scatter"

HEATMAP = "heatmap"


class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))


class SampleHazardDetector(IDataReceived):

    def __init__(self, tcpClient):
        DEFAULT_PORT = 8097
        DEFAULT_HOSTNAME = "http://localhost"
        parser = argparse.ArgumentParser(description='Demo arguments')
        parser.add_argument('-port', metavar='port', type=int, default=DEFAULT_PORT,
                            help='port the visdom server is running on.')
        parser.add_argument('-server', metavar='server', type=str,
                            default=DEFAULT_HOSTNAME,
                            help='Server address of the target to run the demo on.')
        FLAGS = parser.parse_args()
        self.viz = Visdom(port=FLAGS.port, server=FLAGS.server)

        assert self.viz.check_connection(timeout_seconds=3), 'No connection could be formed quickly, remember to run \'visdom\' in the terminal'

        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__estimatedHazardZone = Polygon()
        self.scenario = minidom.parse('/home/edoardo/Development/amase-firehack/example scenarios/HazardZoneDetectionScenario.xml')
        simulation_view_node = self.scenario.getElementsByTagName('SimulationView')
        self.latitude = float(simulation_view_node[0].attributes['Latitude'].value)
        self.longitude = float(simulation_view_node[0].attributes['Longitude'].value)
        self.long_extent = float(simulation_view_node[0].attributes['LongExtent'].value)
        # self.centre_lat = self.latitude / 2
        # self.centre_long = self.longitude / 2
        self.max_lat = self.latitude + self.long_extent
        self.min_lat = self.latitude - self.long_extent
        self.max_long = self.longitude + self.long_extent
        self.min_long = self.longitude - self.long_extent
        self.coordinates = []
        self.heatmap = np.zeros((100, 100))
        self.communication_channel = protobuf.communication_client.CommunicationChannel()
        # self.contour = self.viz.contour(X=self.heatmap, opts=dict(title='Contour plot'))
        # self.viz_heatmap = self.viz.heatmap(X=self.heatmap, win=HEATMAP, opts=dict(title='Heatmap plot'))
        # self.viz_scatter = self.viz.scatter(X=np.array([]), Y=np.array([]))

    def dataReceived(self, lmcpObject):
        # try:
        #     if hasattr(lmcpObject, 'Time'):
        #         self.current_time = int(getattr(lmcpObject, 'Time'))
        # except Exception as ex:
        #     print(ex)

        if isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            # Get location where zone first detected
            detectedLocation = hazardDetected.get_DetectedLocation()
            lat, long = self.normalise_coordinates(detectedLocation)

            try:
                self.heatmap[lat][long] = 1.0
            except Exception as ex:
                print(ex)
            # self.viz.contour(X=self.heatmap, win=self.contour, opts=dict(title='Contour plot'))
            self.viz.heatmap(X=self.heatmap, win=HEATMAP, opts=dict(title='Heatmap plot'))
            self.viz.contour(X=self.heatmap, win=CONTOUR, opts=dict(title='Contour plot'))
            self.viz.scatter(X=np.array([[detectedLocation.get_Longitude(), detectedLocation.get_Latitude()]]), Y=np.array([1]), win=VIZ_SCATTER, update='append')
        if isinstance(lmcpObject, SessionStatus):
            session_status: SessionStatus = lmcpObject
            self.current_time = session_status.get_ScenarioTime()
            state: SimulationStatusType.SimulationStatusType = session_status.get_State()
            if state is SimulationStatusType.SimulationStatusType.Reset or state is SimulationStatusType.SimulationStatusType.Stopped:
                self.viz.close(win=HEATMAP)
                self.viz.close(win=VIZ_SCATTER)
                self.viz.close(win=CONTOUR)
                self.heatmap = np.zeros((100, 100))
            delta_time = session_status.ScenarioTime - self.current_time
            self.current_time = session_status.ScenarioTime
            # self.heatmap = self.update_heatmap(delta_time)
            self.communication_channel.send(self.current_time, self.heatmap)
            self.convex_hull()

    def normalise_coordinates(self, detectedLocation):
        lat = int((detectedLocation.get_Latitude() - self.min_lat) / (self.max_lat - self.min_lat) * self.heatmap.shape[0])
        long = int((detectedLocation.get_Longitude() - self.min_long) / (self.max_long - self.min_long) * self.heatmap.shape[1])
        return lat, long

    def denormalise_coordinates(self, lat, long):
        norm_lat = (lat * (self.max_lat - self.min_lat) / self.heatmap.shape[0]) + self.min_lat
        norm_long = (long * (self.max_long - self.min_long) / self.heatmap.shape[1]) + self.min_long
        return norm_lat, norm_long

    def update_heatmap(self, deltaTime):
        """
        Updates the heatmap and returns a new heatmap
        """

    def convex_hull(self):
        """
        Generates the convex hull from the point cloud
        :return:
        """
        coords = []
        for row in range(self.heatmap.shape[0]):
            for col in range(self.heatmap.shape[1]):
                if self.heatmap[row][col] > 0.95:
                    coords.append((row, col))
        if len(coords) < 3:
            return
        try:
            poly = ConvexHull(coords)
            self.__estimatedHazardZone.get_BoundaryPoints().clear()
            for index in poly.vertices:
                point = Location3D()
                lat, long = self.denormalise_coordinates(poly.points[index][0], poly.points[index][1])
                point.set_Latitude(lat)
                point.set_Longitude(long)
                # point.set_Latitude(index.)
                self.__estimatedHazardZone.get_BoundaryPoints().append(point)
            self.sendEstimateReport()
        except Exception as ex:
            print(ex)

    def sendLoiterCommand(self, vehicleId, location):
        # Setting up the mission to send to the UAV
        vehicleActionCommand = VehicleActionCommand()
        vehicleActionCommand.set_VehicleID(vehicleId)
        vehicleActionCommand.set_Status(CommandStatusType.Pending)
        vehicleActionCommand.set_CommandID(1)

        # Setting up the loiter action
        loiterAction = LoiterAction()
        loiterAction.set_LoiterType(LoiterType.Circular)
        loiterAction.set_Radius(250)
        loiterAction.set_Axis(0)
        loiterAction.set_Length(0)
        loiterAction.set_Direction(LoiterDirection.Clockwise)
        loiterAction.set_Duration(100000)
        loiterAction.set_Airspeed(15)

        # Creating a 3D location object for the stare point
        loiterAction.set_Location(location)

        # Adding the loiter action to the vehicle action list
        vehicleActionCommand.get_VehicleActionList().append(loiterAction)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(vehicleActionCommand)

    def sendEstimateReport(self):
        # Setting up the mission to send to the UAV
        hazardZoneEstimateReport = HazardZoneEstimateReport()
        hazardZoneEstimateReport.set_EstimatedZoneShape(self.__estimatedHazardZone)
        hazardZoneEstimateReport.set_UniqueTrackingID(1)
        hazardZoneEstimateReport.set_EstimatedGrowthRate(0)
        hazardZoneEstimateReport.set_PerceivedZoneType(HazardType.Fire)
        hazardZoneEstimateReport.set_EstimatedZoneDirection(0)
        hazardZoneEstimateReport.set_EstimatedZoneSpeed(0)

        # Sending the Vehicle Action Command message to AMASE to be interpreted
        self.__client.sendLMCPObject(hazardZoneEstimateReport)


#################
## Main
#################

if __name__ == '__main__':
    myHost = 'localhost'
    myPort = 5555
    amaseClient = AmaseTCPClient(myHost, myPort)
    # amaseClient.addReceiveCallback(PrintLMCPObject())
    detector = SampleHazardDetector(amaseClient)
    # detector.convex_hull()
    amaseClient.addReceiveCallback(detector)

    try:
        # make a threaded client, listen until a keyboard interrupt (ctrl-c)
        # start client thread
        amaseClient.start()

        while True:
            # wait for keyboard interrupt
            pass
    except KeyboardInterrupt as ki:
        print("Stopping amase tcp client")
    except Exception as ex:
        print(ex)
    amaseClient.stop()
