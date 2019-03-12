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
import numpy as np
from xml.dom import minidom


class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))


class SampleHazardDetector(IDataReceived):

    def __init__(self, tcpClient):
        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__estimatedHazardZone = Polygon()
        self.scenario = minidom.parse('/home/edoardo/Development/amase-firehack-2019-02-21-UK/amase-firehack/example scenarios/HazardZoneDetectionScenario.xml')
        simulationViewNode = self.scenario.getElementsByTagName('SimulationView')
        self.latitude = float(simulationViewNode[0].attributes['Latitude'].value)
        self.longitude = float(simulationViewNode[0].attributes['Longitude'].value)
        self.centre_lat = self.latitude / 2
        self.centre_long = self.longitude / 2
        self.max_lat = self.centre_lat + 500
        self.min_lat = self.centre_lat - 500
        self.max_long = self.centre_long + 500
        self.min_long = self.centre_long - 500
        self.heatmap = np.ones((1000, 1000))
        self.graph = plt.imshow(self.heatmap, cmap='hot', interpolation='nearest')
        plt.show(block=False)
        plt.pause(0.1)

    def dataReceived(self, lmcpObject):
        if isinstance(lmcpObject, HazardZoneDetection):
            hazardDetected = lmcpObject
            # Get location where zone first detected
            detectedLocation = hazardDetected.get_DetectedLocation()
            lat = int((detectedLocation.get_Latitude() - self.min_lat) / (self.max_lat - self.min_lat) * 1000)
            long = int((detectedLocation.get_Longitude() - self.min_long) / (self.max_long - self.min_long) * 1000)
            # norm_lat=
            heatmap2 = self.heatmap.copy()
            heatmap2[lat][long] = 0.0
            self.graph.set_data(heatmap2)
            plt.pause(0.1)
            self.heatmap=heatmap2
            # plt.show(block=False)
            # # Get entity that detected the zone
            # detectingEntity = hazardDetected.get_DetectingEnitiyID()
            # # Check if the UAV has already been sent the loiter command and proceed if it hasn't
            # if not detectingEntity in self.__uavsLoiter:
            #     # Send the loiter command
            #     self.sendLoiterCommand(detectingEntity, detectedLocation)
            #
            #     # Note: Polygon points must be in clockwise or counter-clockwise order to get a shape without intersections
            #     self.__estimatedHazardZone.get_BoundaryPoints().append(detectedLocation)
            #
            #     # Send out the estimation report to draw the polygon
            #     self.sendEstimateReport();
            #
            #     self.__uavsLoiter[detectingEntity] = True
            #     print('UAV' + str(detectingEntity) + ' detected hazard at ' + str(detectedLocation.get_Latitude()) + ',' + str(detectedLocation.get_Longitude()) + '. Sending loiter command.');

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
