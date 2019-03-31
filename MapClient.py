import argparse
import math
from xml.dom import minidom

import numpy as np
from geopy.distance import VincentyDistance
from scipy import ndimage
from visdom import Visdom

import calculate_polygons
import protobuf.communication_client
from afrl.cmasi.AirVehicleState import AirVehicleState
from afrl.cmasi.KeyValuePair import KeyValuePair
from afrl.cmasi.Location3D import Location3D
from afrl.cmasi.Polygon import Polygon
from afrl.cmasi.SessionStatus import SessionStatus, SimulationStatusType
from afrl.cmasi.searchai.HazardType import HazardType
from afrl.cmasi.searchai.HazardZoneDetection import HazardZoneDetection
from afrl.cmasi.searchai.HazardZoneEstimateReport import HazardZoneEstimateReport
from amase.TCPClient import AmaseTCPClient
from amase.TCPClient import IDataReceived
import geopy
import argparse

CONTOUR = "contour"

HEATMAP = "heatmap"
POLYGON_REFRESH_RATE = 1000000000000
SIZE_LAT = 120  # size of the heatmap
SIZE_LONG = 120  # size of the heatmap
DECAY_REFRESH = 10000  # how often to refresh visdom due to decay
MIN_ELAPSED_TIME = 10000  # milliseconds before considering removing the same point if seen again but with no fire
USE_DECAY = False  # decays the heatmap gradually
DECAY_MINUTES = 15
FAKE_INITIAL_POINTS = False  # for debug purposes, fills the heatmap with the points from the scenario


class PrintLMCPObject(IDataReceived):
    def dataReceived(self, lmcpObject):
        print(lmcpObject.toXMLStr(""))


class SampleHazardDetector(IDataReceived):

    def __init__(self, tcpClient):
        DEFAULT_PORT = 8097
        DEFAULT_HOSTNAME = "http://localhost"
        # parser = argparse.ArgumentParser(description='Demo arguments')
        # parser.add_argument('-port', metavar='port', type=int, default=DEFAULT_PORT,
        #                     help='port the visdom server is running on.')
        # parser.add_argument('-server', metavar='server', type=str,
        #                     default=DEFAULT_HOSTNAME,
        #                     help='Server address of the target to run the demo on.')
        # FLAGS = parser.parse_args()
        self.viz = Visdom(port=DEFAULT_PORT, server=DEFAULT_HOSTNAME)
        self.last_refresh = 0
        self.new_points_detected = False

        assert self.viz.check_connection(timeout_seconds=3), 'No connection could be formed quickly, remember to run \'visdom\' in the terminal'

        self.__client = tcpClient
        self.__uavsLoiter = {}
        self.__estimatedHazardZone = Polygon()
        self.communication_channel = protobuf.communication_client.CommunicationChannel()
        self.reset_model()

    def dataReceived(self, lmcpObject):
        try:
            if isinstance(lmcpObject, SessionStatus):
                print(f'time: {self.current_time} - session status'.ljust(100), end='\r', flush=True)
                session_status: SessionStatus = lmcpObject
                delta_time = session_status.ScenarioTime - self.current_time
                self.current_time = session_status.get_ScenarioTime()  # save the last registered time to use in other parts of the code
                state: SimulationStatusType.SimulationStatusType = session_status.get_State()
                if state is SimulationStatusType.SimulationStatusType.Reset:
                    self.reset_model()
                    if len(session_status.get_Parameters()) > 0:
                        param: KeyValuePair
                        for param in session_status.get_Parameters():
                            if param.Key == b'source':
                                self.filename = param.Value.decode("utf-8")
                    if self.filename is not None:
                        self.load_scenario(self.filename)
                if self.filename is None:  # only move on when the scenario is ready
                    return

                self.communication_channel.send(self.current_time, self.heatmap, self.max_lat, self.max_long, self.min_lat, self.min_long)
                self.compute_and_send_estimate_hazardZone()
                if USE_DECAY:
                    self.decay_heatmap(delta_time)
            if isinstance(lmcpObject, AirVehicleState):
                print(f'time: {self.current_time} - vehicle update'.ljust(100), end='\r', flush=True)
                vehicleState: AirVehicleState = lmcpObject
                id = vehicleState.ID
                heading = vehicleState.Heading
                location: Location3D = vehicleState.get_Location()
                origin = geopy.Point(location.get_Latitude(), location.get_Longitude())
                destination = geopy.distance.distance(kilometers=0.5).destination(origin, heading)
                targetLocation = Location3D()  # the estimate of where the drone is looking
                targetLocation.set_Latitude(destination.latitude)
                targetLocation.set_Longitude(destination.longitude)
                new_point = False  # whether there has been a change
                lat, long = self.normalise_coordinates(targetLocation)
                if not self.is_legal(lat, long):
                    return
                try:
                    if self.heatmap[lat][long] != 0.0:  # delete the point from the heatmap
                        delta_time = self.current_time - self.last_detected[lat][long]
                        if delta_time > MIN_ELAPSED_TIME:
                            self.heatmap[lat][long] = 0.0
                            self.last_detected[lat][long] = self.current_time  # the last registered time
                            self.apply_smoothing()
                            self.new_points_detected = True
                            new_point = True

                except Exception as ex:
                    print(ex)
                if new_point:
                    self.update_visdom()
                pass
            if isinstance(lmcpObject, HazardZoneDetection):
                print(f'time: {self.current_time} - hazardzone detection'.ljust(100), end='\r', flush=True)
                hazardDetected: HazardZoneDetection = lmcpObject
                # Get location where zone first detected
                new_point = False
                detectedLocation = hazardDetected.get_DetectedLocation()
                lat, long = self.normalise_coordinates(detectedLocation)
                if not self.is_legal(lat, long):
                    return
                detecting_id = hazardDetected.DetectingEnitiyID
                try:
                    if self.heatmap[lat][long] != 1.0:
                        self.heatmap[lat][long] = 1.0
                        self.last_detected[lat][long] = self.current_time  # the last registered time
                        self.apply_smoothing()
                        self.new_points_detected = True
                        new_point = True

                except Exception as ex:
                    print(ex)
                if new_point:
                    self.update_visdom()

        except Exception as ex:
            print(ex)

    def reset_model(self):
        self.viz.close(win=HEATMAP)
        self.viz.close(win=CONTOUR)
        self.viz.close(win="Trajectory")
        self.current_time = 0
        self.last_decay_refresh = 0
        self.scenario = None
        self.filename = None  # scenario not ready
        self.heatmap = np.zeros((SIZE_LAT, SIZE_LONG))
        self.last_detected = np.zeros((SIZE_LAT, SIZE_LONG))  # the time at which the fire was last detected (or not) in that cell
        self.smooth = np.zeros((SIZE_LAT, SIZE_LONG))
        self.drones_status = {}
        self.force_recompute_times = []

    def update_visdom(self):
        self.viz.heatmap(X=self.heatmap, win=HEATMAP, opts=dict(title='Heatmap plot', xmin=0, xmax=1.0))
        self.viz.contour(X=self.smooth, win=CONTOUR, opts=dict(title='Contour plot'))

    def decay_heatmap(self, delta_time):

        temp_decay = 1 / (DECAY_MINUTES * 60 * 1000)  # decays to 0 in 15 minutes
        self.heatmap = self.heatmap - (temp_decay * delta_time)
        self.heatmap = np.clip(self.heatmap, 0.0, None)  # clip the value to positive values
        if (self.current_time - self.last_decay_refresh) > DECAY_REFRESH:
            self.last_decay_refresh = self.current_time
            self.update_visdom()

    def is_legal(self, lat, long):
        return lat < SIZE_LAT and long < SIZE_LONG

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
                if self.heatmap[row][col] > 0.01:  # used for decay, only cells with more than a small probability are allowed
                    coords.append((row, col))

        return coords

    def set_coord_as_hazard_zone(self, norm_poly):
        self.__estimatedHazardZone.get_BoundaryPoints().clear()
        for point in norm_poly.points:
            denormalised_point = Location3D()
            lat, long = self.denormalise_coordinates(point[0], point[1])
            denormalised_point.set_Latitude(lat)
            denormalised_point.set_Longitude(long)
            self.__estimatedHazardZone.get_BoundaryPoints().append(denormalised_point)

    def load_scenario(self, filename):
        print("loading scenario")
        self.scenario = minidom.parse(filename)
        simulation_view_node = self.scenario.getElementsByTagName('SimulationView')
        self.latitude = float(simulation_view_node[0].attributes['Latitude'].value)
        self.longitude = float(simulation_view_node[0].attributes['Longitude'].value)
        self.long_extent = float(simulation_view_node[0].attributes['LongExtent'].value)
        self.max_lat = self.latitude + self.long_extent
        self.min_lat = self.latitude - self.long_extent
        self.max_long = self.longitude + self.long_extent
        self.min_long = self.longitude - self.long_extent
        self.last_refresh = 0
        self.new_points_detected = False
        keep_in_zone = self.scenario.getElementsByTagName('KeepInZone')
        ############### KEEP IN ZONE ############################
        if len(keep_in_zone) > 0:  # if there is a keep in zone then constraint to that
            self.latitude = float(keep_in_zone[0].getElementsByTagName('Latitude')[0].childNodes[0].nodeValue)
            self.longitude = float(keep_in_zone[0].getElementsByTagName('Longitude')[0].childNodes[0].nodeValue)
            width = float(keep_in_zone[0].getElementsByTagName('Width')[0].childNodes[0].nodeValue)
            height = float(keep_in_zone[0].getElementsByTagName('Height')[0].childNodes[0].nodeValue)
            centerPoint = Location3D()
            centerPoint.set_Latitude(self.latitude)
            centerPoint.set_Longitude(self.longitude)
            low_loc: Location3D = self.newLocation(centerPoint, height / -2, width / -2)
            high_loc: Location3D = self.newLocation(centerPoint, height / 2, width / 2)
            self.max_lat = max(low_loc.get_Latitude(), high_loc.get_Latitude())
            self.max_long = max(low_loc.get_Longitude(), high_loc.get_Longitude())
            self.min_lat = min(low_loc.get_Latitude(), high_loc.get_Latitude())
            self.min_long = min(low_loc.get_Longitude(), high_loc.get_Longitude())
        ################# SCORING TIMES ######################
        scoring_times = self.scenario.getElementsByTagName("Hack")
        self.force_recompute_times = []
        for scoring_time in scoring_times:
            time = float(scoring_time.attributes['Time'].value)
            self.force_recompute_times.append(time)

        ################FAKE POINTS
        if FAKE_INITIAL_POINTS:
            self.fake_points()
        print('scenario loaded')

    def newLocation(self, loc: Location3D, dx, dy):
        R_EARTHKM = 6372.8
        latitude = loc.get_Latitude()
        longitude = loc.get_Longitude()
        new_latitude = latitude + (dy / (R_EARTHKM * 1000)) * (180 / math.pi)

        new_longitude = longitude + (dx / (R_EARTHKM * 1000)) * (180 / math.pi) / math.cos(latitude * math.pi / 180)

        new_location = Location3D()
        new_location.set_Latitude(new_latitude)
        new_location.set_Longitude(new_longitude)
        return new_location

    def fake_points(self):
        hazardZone_nodes = self.scenario.getElementsByTagName('HazardZone')
        for hazardZone_node in hazardZone_nodes:
            boundary_points = hazardZone_node.getElementsByTagName('Location3D')
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
        self.compute_and_send_estimate_hazardZone(True)

    def compute_and_send_estimate_hazardZone(self, force=False):
        delta_time = self.current_time - self.last_refresh
        if (len(self.force_recompute_times) > 0 and self.current_time > (self.force_recompute_times[0] - 10) * 1000):  # 10 seconds before each scoring
            self.force_recompute_times.pop(0)  # remove first time
            force = True  # force recomputing
        if (force or (delta_time > POLYGON_REFRESH_RATE and self.new_points_detected)):
            self.last_refresh = self.current_time
            self.new_points_detected = False
            coords = self.compute_coords()

            # Simple triangle
            if len(coords) < 3:
                return

            try:
                # Different options to create polygon.
                norm_polys = calculate_polygons.calculate_polygons(coords)
                # norm_poly = ConvexHull(coords)
                # For now just get first polygon.
                for index, poly in enumerate(norm_polys):
                    # norm_poly = norm_polys[0]
                    # self.belief_model.polygons.append(poly)

                    self.set_coord_as_hazard_zone(poly)
                    self.sendEstimateReport(index)
            except Exception as ex:
                raise ex
                # print(ex)

    def sendEstimateReport(self, id=1):
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
    parser = argparse.ArgumentParser(description='Keep track of a map of fires in an area')
    parser.add_argument('--fake', help='fakes the initial detection points. For debug purposes', action='store_true')
    parser.add_argument('--decay', help='applies a decay to the heatmap so that old fires gradually get forgotten', action='store_true')
    parser.add_argument('--decay_rate', type=float,default=15, help='minutes needed to erase a fire')
    args = parser.parse_args()
    USE_DECAY = args.decay
    FAKE_INITIAL_POINTS = args.fake
    DECAY_MINUTES = args.decay_rate
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
