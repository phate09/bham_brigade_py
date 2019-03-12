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
        self.scenario = minidom.parse('/home/edoardo/Development/amase-firehack/example scenarios/Challenge_0_Scenario_UK.xml')
        simulationViewNode = self.scenario.getElementsByTagName('SimulationView')
        self.latitude = float(simulationViewNode[0].attributes['Latitude'].value)
        self.longitude = float(simulationViewNode[0].attributes['Longitude'].value)
        self.long_extent = float(simulationViewNode[0].attributes['LongExtent'].value)
        # self.centre_lat = self.latitude / 2
        # self.centre_long = self.longitude / 2
        self.max_lat = self.latitude + self.long_extent
        self.min_lat = self.latitude - self.long_extent
        self.max_long = self.longitude + self.long_extent
        self.min_long = self.longitude - self.long_extent
        self.coordinates = []
        self.heatmap = np.zeros((100, 100))
        # self.contour = self.viz.contour(X=self.heatmap, opts=dict(title='Contour plot'))
        self.viz_heatmap = self.viz.heatmap(X=self.heatmap, opts=dict(title='Heatmap plot'))
        # self.viz_scatter = self.viz.scatter(X=np.array([]), Y=np.array([]))

    def scale(self, X, x_min, x_max):
        nom = (X - X.min(axis=0)) * (x_max - x_min)
        denom = X.max(axis=0) - X.min(axis=0)
        denom[denom == 0] = 1
        return x_min + nom / denom

    def dataReceived(self, lmcpObject):
        if isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            # Get location where zone first detected
            detectedLocation = hazardDetected.get_DetectedLocation()
            lat = int((detectedLocation.get_Latitude() - self.min_lat) / (self.max_lat - self.min_lat) * 100)
            long = int((detectedLocation.get_Longitude() - self.min_long) / (self.max_long - self.min_long) * 100)

            try:
                # self.heatmap = np.zeros((100, 100))
                # for pair in np_coordinates:
                #     self.heatmap[pair[0]][pair[1]] = 1.0
                self.heatmap[lat][long] = 1.0
            except Exception as ex:
                print(ex)
            # self.viz.contour(X=self.heatmap, win=self.contour, opts=dict(title='Contour plot'))
            self.viz.heatmap(X=self.heatmap, win=self.viz_heatmap, opts=dict(title='Heatmap plot'))
            self.viz.scatter(X=np.array([[detectedLocation.get_Longitude(), detectedLocation.get_Latitude()]]), Y=np.array([1]), win="viz_scatter", update='append')

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
    amaseClient.addReceiveCallback(SampleHazardDetector(amaseClient))

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
