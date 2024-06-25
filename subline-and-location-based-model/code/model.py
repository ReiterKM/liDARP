from util import *
from travelRequests import TravelRequests

class Model:

    def __init__(self, requests: TravelRequests, num_stations: int, num_busses: int, Q_max: int, timeframe:float, alpha:int, beta:int,
              speed:float = 1, t_turn:float = 0.5) -> None:
        self.requests = requests
        self.num_stations = num_stations
        self.num_busses = num_busses
        self.Q_max = Q_max

        if alpha < 0:
            raise ValueError("Value $\alpha$ must be greater than 0, currently $\alpha = {0}$".format(alpha))
        self.alpha = alpha
        if beta < 0:
            raise ValueError("Value $\beta$ must be greater than 0, currently $\beta = {0}$".format(alpha))
        self.beta = beta

        self.max_travel_minutes = timeframe

        self.speed = speed
        self.t_turn = t_turn

        self._calculateConstants()
        self.model = self._initModel()
    
    def optimize(self, verbose = True, params: dict = None):
        if verbose:
            self.model.setParam('OutputFlag', 1)
        else:
            self.model.setParam('OutputFlag', 0)

        if params:
            for key, value in params.items():
                self.model.setParam(key, value)

        self.model.optimize()  

    def _initModel(self):
        pass

    def _calculateConstants(self):
        pass

    def _calculateVariables(self):
        pass

    def createModel(self):
        pass

    def _setObjective(self):
        pass

    def postprocessing(self):
        pass

    def calculateStatistics(self):
        self.avg_waiting_time = 0
        self.avg_ride_time = 0
        self.avg_transportation_time = 0
        self.avg_detour_factor = 0
        self.mean_occupancy = 0
        self.share_empty_mileage = 0
        self.system_efficiency = 0
        self.pooling_factor = 0

    def _printStatistics(self):
        print("Average waiting time: {0}".format(self.avg_waiting_time))
        print("Average ride time (of solution): {0}".format(self.avg_ride_time))
        print("Average transportation time: {0}".format(self.avg_transportation_time))

        print("Average detour factor: {0}".format(self.avg_detour_factor))
        print("Mean occupancy: {0:g}".format(self.mean_occupancy))
        print("Share empty mileage: {0:g}".format(self.share_empty_mileage))
        print("System efficiency: {0:g}".format(self.system_efficiency))
        print("Pooling factor: {0:g}".format(self.pooling_factor))

    def detailed_file(self, path, instance:str):
        output_path = os.path.join(path, "instance" + "_details.txt")

        try:
            f = open(output_path, "w")
        except FileNotFoundError as e:
            print("Error when opening detail file.")
            raise e
        else:
            f.write(instance + "\n")
            f.write("total time & model time & status & MIP Gap\n")
            f.write("{0} & {1} & {2} & {3}\n".format(self.model.Runtime, self.Buildtime, self.model.Status, self.model.MIPGap))
            f.write("obj value & total routing costs & rejected requests & amount of vehicles & direct pax km\n")
            f.write("{0} & {1} & {2} & {3} & {4}\n".format(self.model.getObjective().getValue(), self.total_distance, self.num_pax_rejected, self.z.sum().getValue(), self.pax_km))
            f.write("avg_waiting_time & avg_ride_time & avg_transportation_time & avg_detour & mean_occupancy & share_empty_mileage & efficiency & pooling_factor\n")
            f.write("{0} & {1} & {2} & {3} & {4} & {5} & {6} & {7}\n".format(self.avg_waiting_time, self.avg_ride_time, self.avg_transportation_time, self.avg_detour_factor, 
                                                                            self.mean_occupancy, self.share_empty_mileage, self.system_efficiency, self.pooling_factor))
            
            f.close()
    
        print("Detail file saved succesfully.")