from BeliefModel import BeliefModel
from afrl.cmasi.AirVehicleState import AirVehicleState
from afrl.cmasi.KeyValuePair import KeyValuePair
from afrl.cmasi.SessionStatus import SessionStatus, SimulationStatusType
from afrl.cmasi.searchai.HazardZone import HazardZone
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
from xml.dom import minidom
from visdom import Visdom
import argparse
import numpy as np
import protobuf.communication_client
from scipy.spatial import ConvexHull
from scipy import ndimage

import calculate_polygons

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
        self.fake_point = True  # wether import the boundaries of the fire from the xml

        assert self.viz.check_connection(timeout_seconds=3), 'No connection could be formed quickly, remember to run \'visdom\' in the terminal'

        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__estimatedHazardZone = Polygon()
        self.filename = None
        self.heatmap = np.zeros((100, 100))  # the places where the fire was detected
        self.last_detected = np.zeros((100, 100))  # the time at which the fire was last detected (or not) in that cell
        self.smooth = np.zeros((100, 100))
        self.drones_status = {}
        self.communication_channel = protobuf.communication_client.CommunicationChannel()
        self.belief_model = BeliefModel()

    def load_scenario(self, filename):
        self.scenario = minidom.parse(filename)
        simulation_view_node = self.scenario.getElementsByTagName('SimulationView')
        self.latitude = float(simulation_view_node[0].attributes['Latitude'].value)
        self.longitude = float(simulation_view_node[0].attributes['Longitude'].value)
        self.long_extent = float(simulation_view_node[0].attributes['LongExtent'].value)
        self.max_lat = self.latitude + self.long_extent
        self.min_lat = self.latitude - self.long_extent
        self.max_long = self.longitude + self.long_extent
        self.min_long = self.longitude - self.long_extent
        if self.fake_point:
            self.fake_points(filename)

    def fake_points(self, filename):
        self.scenario = minidom.parse(filename)
        hazardZone_node = self.scenario.getElementsByTagName('HazardZone')
        boundary_points = hazardZone_node[0].getElementsByTagName('Location3D')
        for point_string in boundary_points:
            latitude = float(point_string.getElementsByTagName('Latitude')[0].childNodes[0].nodeValue)
            longitude = float(point_string.getElementsByTagName('Longitude')[0].childNodes[0].nodeValue)
            location = Location3D()
            location.set_Latitude(latitude)
            location.set_Longitude(longitude)
            lat, long = self.normalise_coordinates(location)
            try:
                self.heatmap[lat][long] = 1.0
                self.last_detected[lat][long] = self.current_time  # the last registered time

            except Exception as ex:
                print(ex)
        self.apply_smoothing()
        self.update_visdom()

    def dataReceived(self, lmcpObject):
        try:
            if isinstance(lmcpObject, SessionStatus):
                session_status: SessionStatus = lmcpObject
                self.current_time = session_status.get_ScenarioTime()  # save the last registered time to use in other parts of the code
                state: SimulationStatusType.SimulationStatusType = session_status.get_State()
                if state is SimulationStatusType.SimulationStatusType.Reset:
                    self.viz.close(win=HEATMAP)
                    self.viz.close(win=VIZ_SCATTER)
                    self.viz.close(win=CONTOUR)
                    self.viz.close(win="Trajectory")
                    self.heatmap = np.zeros((100, 100))
                    self.last_detected = np.zeros((100, 100))  # the time at which the fire was last detected (or not) in that cell
                    self.smooth = np.zeros((100, 100))
                    self.drones_status = {}
                    self.belief_model = BeliefModel()
                    if len(session_status.get_Parameters()) > 0:
                        param: KeyValuePair
                        for param in session_status.get_Parameters():
                            if param.Key == b'source':
                                self.filename = param.Value.decode("utf-8")
                    if self.filename is not None:
                        self.load_scenario(self.filename)
                if self.filename is None:  # only move on when the scenario is ready
                    return
                delta_time = session_status.ScenarioTime - self.current_time
                self.current_time = session_status.ScenarioTime
                # self.heatmap = self.update_heatmap(delta_time)
                self.communication_channel.send(self.current_time, self.heatmap, self.max_lat, self.max_long, self.min_lat, self.min_long)
                self.compute_and_send_estimate()
            if isinstance(lmcpObject, AirVehicleState):
                vehicleState: AirVehicleState = lmcpObject
                id = vehicleState.ID
                heading = vehicleState.Heading
                location: Location3D = vehicleState.get_Location()
                self.drones_status[id] = (heading, location)
                try:
                    locations = []
                    y = []
                    markers = []
                    for key in self.drones_status:
                        location: Location3D
                        heading: int
                        heading, location = self.drones_status[key]
                        locations.append([location.get_Longitude(), location.get_Latitude()])
                        y.append([1])
                        heading = (360.0 - heading) % 360.0  # counterclockwise to clockwise
                        markers.append((3, 0, heading))
                    self.viz.scatter(X=np.array(locations), Y=np.array(y), win=VIZ_SCATTER, opts=dict(
                        xtickmin=self.min_long,
                        xtickmax=self.max_long,
                        ytickmin=self.min_lat,
                        ytickmax=self.max_lat,
                        marker=markers,
                        markersize=10,
                        linestyle='None'
                    ))
                except BaseException as err:
                    print('Skipped matplotlib example')
                    print('Error message: ', err)

                pass
            if isinstance(lmcpObject, HazardZoneDetection):
                hazardDetected: HazardZoneDetection = lmcpObject
                # Get location where zone first detected
                detectedLocation = hazardDetected.get_DetectedLocation()
                lat, long = self.normalise_coordinates(detectedLocation)
                detecting_id = hazardDetected.DetectingEnitiyID
                try:
                    self.heatmap[lat][long] = 1.0
                    self.last_detected[lat][long] = self.current_time  # the last registered time
                    self.apply_smoothing()

                except Exception as ex:
                    print(ex)
                # self.viz.contour(X=self.heatmap, win=self.contour, opts=dict(title='Contour plot'))
                self.update_visdom()
            if isinstance(lmcpObject, HazardZone):
                '''here it fake the points for debugging purporses taking them from the hazardzone message'''
                hazardZone: HazardZone = lmcpObject
                polygon: Polygon = hazardZone.get_Boundary()
                points = polygon.get_BoundaryPoints()
                for detectedLocation in points:
                    lat, long = self.normalise_coordinates(detectedLocation)
                    try:
                        self.heatmap[lat][long] = 1.0
                        self.last_detected[lat][long] = self.current_time  # the last registered time
                        self.apply_smoothing()

                    except Exception as ex:
                        print(ex)

        except Exception as ex:
            print(ex)

    def update_visdom(self):
        self.viz.heatmap(X=self.heatmap, win=HEATMAP, opts=dict(title='Heatmap plot'))
        self.viz.contour(X=self.smooth, win=CONTOUR, opts=dict(title='Contour plot'))

    def apply_smoothing(self):
        self.smooth = ndimage.gaussian_filter(self.heatmap, 10)
        pass

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



    '''gets the points in the heatmap where there is fire'''

    def compute_coords(self):
        coords = []
        for row in range(self.heatmap.shape[0]):
            for col in range(self.heatmap.shape[1]):
                if self.heatmap[row][col] > 0.95:  # This could be a 1 check but we are pre-empting expanding this for decay.
                    coords.append((row, col))

        return coords

    def set_coord_as_hazard_zone(self, norm_poly):
        self.__estimatedHazardZone.get_BoundaryPoints().clear()
        for point in norm_poly.points:
            denormalised_point = Location3D()
            lat, long = self.denormalise_coordinates(point[0], point[1])
            denormalised_point.set_Latitude(lat)
            denormalised_point.set_Longitude(long)
            print(denormalised_point)
            # point.set_Latitude(index.)
            self.__estimatedHazardZone.get_BoundaryPoints().append(denormalised_point)


    def compute_and_send_estimate(self):
        coords = self.compute_coords()

        # Simple triangle
        if len(coords) < 3:
            return

        try:
            # Different options to create polygon.
            norm_polys = calculate_polygons.calculate_polygons(coords)
            #norm_poly = ConvexHull(coords)
            # For now just get first polygon.
            for index,poly in enumerate(norm_polys):
            # norm_poly = norm_polys[0]
                self.belief_model.polygons.append(poly)

                self.set_coord_as_hazard_zone(poly)
                self.sendEstimateReport(index)
        except Exception as ex:
            raise ex
            #print(ex)

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

    def sendEstimateReport(self,id=1):
        # Setting up the mission to send to the UAV
        hazardZoneEstimateReport = HazardZoneEstimateReport()
        hazardZoneEstimateReport.set_EstimatedZoneShape(self.__estimatedHazardZone)
        hazardZoneEstimateReport.set_UniqueTrackingID(id)
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
