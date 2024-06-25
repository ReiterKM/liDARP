from util import *
from travelRequests import TravelRequests
from darpNode import DarpNode
from darpEdge import DarpEdge

class DarpGraph:
    def __init__(self, speed: float, t_turn: float, max_time: float, boarding_time:float, alpha:float, beta:float, travel_requests: TravelRequests) -> None:
        self.speed = speed
        self.t_turn = t_turn
        self.boarding_time = boarding_time
        self.time_to_depots = 0

        self.travel_requests = travel_requests.requests
        self.num_requests = travel_requests.num_requests
        self.num_bus_stations = travel_requests.num_stations
        self.max_time = max_time

        self.time_windows = travel_requests.generate_time_windows(travel_speed = speed, boarding_time = boarding_time,
                                                                  max_time = max_time, alpha = alpha, beta = beta)
        self.distances = travel_requests.distances
        self.request_direct_distances = travel_requests.direct_distances

        self.alpha = alpha
        self.beta = beta

    def generate_graph(self):
        self.generate_node_and_index_sets()
        self.generate_new_node_groups()
        self._assign_time_to_stations()
        self.generate_depots()

        self.request_nodes = self.P + self.D + self.P_bar + self.D_bar

        self.nodes = self.request_nodes + [self.start_depot, self.end_depot]

        self._set_names()
        self.nodes_by_label = {i.name : i for i in self.nodes}

        self.generate_edges()
        self.eliminate_infeasible_edges()
        self._generate_edge_distances()

    def generate_node_and_index_sets(self):
        self.P_idx = range(self.num_requests)
        self.D_idx = range(self.num_requests, 2*self.num_requests)
        self.P_bar_idx = range(2*self.num_requests, 3*self.num_requests)
        self.D_bar_idx = range(3*self.num_requests, 4*self.num_requests)
        self.start_depot_idx = 4*self.num_requests
        self.end_depot_idx = 4*self.num_requests + 1
        self.num_nodes = 4*self.num_requests + 2

    def generate_new_node_groups(self):
        self.P = []
        self.D = []
        self.P_bar = []
        self.D_bar = []
        self.F = []
        self.R = []

        for idx, request in enumerate(self.travel_requests):
            # (extended) physical stations
            origin_station, destination_station = request[:2]
            extended_origin_station = 2*self.num_bus_stations - origin_station + 1
            extended_destination_station = 2*self.num_bus_stations - destination_station + 1

            if origin_station < destination_station:
                origin = DarpNode(darp_station = origin_station, bus_station = origin_station, type = "o", corresponding_request=idx, direction = "F")
                destination = DarpNode(darp_station = destination_station, bus_station=destination_station, type = "d", corresponding_request=idx, direction = "F")
                origin_bar = DarpNode(darp_station=extended_origin_station, bus_station=origin_station, type="o_bar", corresponding_request=idx, direction = "R")
                destination_bar = DarpNode(darp_station=extended_destination_station, bus_station=destination_station, type="d_bar", corresponding_request=idx, direction = "R")

                self.F += [origin, destination]
                self.R += [origin_bar, destination_bar]

            elif origin_station > destination_station:
                origin = DarpNode(darp_station=extended_origin_station, bus_station=origin_station, type = "o", corresponding_request=idx, direction = "R")
                destination = DarpNode(darp_station=extended_destination_station, bus_station=destination_station, type = "d", corresponding_request=idx, direction = "R")
                origin_bar = DarpNode(darp_station=origin_station, bus_station=origin_station, type = "o_bar", corresponding_request=idx, direction = "F")
                destination_bar = DarpNode(darp_station=destination_station, bus_station=destination_station, type = "d_bar", corresponding_request=idx, direction = "F")

                self.F += [origin_bar, destination_bar]
                self.R += [origin, destination]

            elif origin_station == destination_station:
                # add to F per default, but notify user
                warnings.warn("Request {0} has same origin and destination at {1}. Added to F per default".format(idx, origin_station))
                origin = DarpNode(darp_station=origin_station, bus_station = origin_station, type = "o", corresponding_request=idx, direction = "F")
                destination = DarpNode(darp_station=destination_station, bus_station=destination_station, type = "d", corresponding_request=idx, direction = "F")
                origin_bar = DarpNode(darp_station=extended_origin_station, bus_station=origin_station, type="o_bar", corresponding_request=idx, direction = "R")
                destination_bar = DarpNode(darp_station=extended_destination_station, bus_station=destination_station, type="d_bar", corresponding_request=idx, direction = "R")

                self.F += [origin, destination]
                self.R += [origin_bar, destination_bar]
                
            self.P.append(origin)
            self.D.append(destination)
            self.P_bar.append(origin_bar)
            self.D_bar.append(destination_bar)

    def _set_names(self):
        for idx, node in enumerate(self.nodes):
            node.name = idx                         

    def generate_depots(self):
        self.start_depot = DarpNode(darp_station=None, bus_station=None, type = "start_depot", corresponding_request=None, direction=None)
        self.start_depot.e = 0
        self.start_depot.l = max([node.l for node in self.P]) - self.time_to_depots

        self.end_depot = DarpNode(darp_station=None, bus_station=None, type = "end_depot", corresponding_request=None, direction=None)
        self.end_depot.e = 0
        self.end_depot.l = max([node.l for node in self.D_bar]) + self.time_to_depots

    def _assign_time_to_stations(self):
        for idx in range(self.num_requests):
            request_origin_time_window, request_destination_time_window, service_promise_made = self.time_windows[idx]

            self.P[idx].e = request_origin_time_window[0]
            self.P[idx].l = request_origin_time_window[1]
            if service_promise_made[0]:
                self.P[idx].has_departure_service_promise = True

            self.D[idx].e = request_destination_time_window[0]
            self.D[idx].l = request_destination_time_window[1]
            if service_promise_made[1]:
                self.D[idx].has_arrival_service_promise = True

            self.P_bar[idx].e = max(0, request_origin_time_window[0] - self.t_turn)
            self.P_bar[idx].l = request_origin_time_window[1] - self.t_turn

            self.D_bar[idx].e = request_destination_time_window[0] + self.boarding_time + self.t_turn
            self.D_bar[idx].l = min(self.max_time, request_destination_time_window[1] + self.boarding_time + self.t_turn)

    def _add_edge(self, u, v) -> None:
        edge = DarpEdge(u,v)
        if edge.label not in self.edge_labels:
            self.edge_labels.add(edge.label)
            self.edges.append(edge)
            self.edges_by_label[edge.label] = edge

    def _remove_edge(self, u, v):
        for edge in self.edges:
            if edge == (u,v):
                self.edges.remove(edge)
                self.edge_labels.remove(edge.label)

    def generate_edges(self):
        self.edges = []
        self.edge_labels = set()
        self.edges_by_label = {}

        for request_idx in range(self.num_requests):
            if self._is_time_infeasible(self.nodes_by_label[request_idx], self.nodes_by_label[self.D_idx[request_idx]]):
                raise ValueError("The edge connecting origin and destination of request {0} is time infeasible.".format(request_idx))
            self._add_edge(self.nodes_by_label[request_idx], self.nodes_by_label[self.D_idx[request_idx]])

        self._add_preceeding_edges(self.R)
        self._add_preceeding_edges(self.F)

        self._add_edges_at_same_station(self.R)
        self._add_edges_at_same_station(self.F)

        for v in self.P + self.P_bar:
            self._add_edge(self.start_depot, v)

        for w in self.D + self.D_bar:
            self._add_edge(w, self.end_depot)

        self._add_edge(self.start_depot, self.end_depot)

        for (o_bar, o) in zip(self.P_bar, self.P):
            self._add_edge(o_bar, o)

        for (d, d_bar) in zip (self.D, self.D_bar):
            self._add_edge(d, d_bar)

        for (v,w) in itertools.combinations(self.P,2):
            if v == w:
                if v.e < w.e:
                    self._add_edge(v, w)
                elif v.e > w.e:
                    self._add_edge(w, v)

        for (v,w) in itertools.combinations(self.D, 2):
            if v == w:
                if v.l < w.l:
                    self._add_edge(v, w)
                elif v.l > w.l:
                    self._add_edge(w, v)

    def _add_preceeding_edges(self, vertices):
        for (v,w) in itertools.permutations(vertices, 2):
            if (v.type != "o_bar") and (w.type != "d_bar") and (v < w) and (not self._is_time_infeasible(v,w)) and (v.request != w.request):
                self._add_edge(v, w)

    def _add_edges_at_same_station(self, vertices):
        for (v,w) in itertools.permutations(vertices, 2):
            if (v.type != "o_bar") and (w.type != "d_bar") and not (v.type == "o" and w.type == "d") and (v == w) and (not self._is_time_infeasible(v,w)) and (v.request != w.request):
                self._add_edge(v,w)

    def eliminate_infeasible_edges(self):
        for (i,j) in self.edges:
            if self._is_time_infeasible(i,j):
                self._remove_edge(i,j)

        for o in self.P:
            # arcs (i,j) and (j, i+n) removed if infeasible
            all_outgoing_edges = [(i,j) for (i,j) in self.edges if i.name == o]
            for (i,j) in all_outgoing_edges:
                if (self.travel_time(i,j) + self.boarding_time + self.travel_time(j, self.M+i) > self.max_time):
                    self._remove_edge(i,j)
                    self._remove_edge(j, self.M + i)
            
            # arcs (i, n+j) removed if j -> i -> n+j -> n+i infeasible
            all_edges_to_dropoffs = [(i,j) for (i,j) in self.edges if (i.name == o) and (j.type == "d")]
            for (i,j_dropoff) in all_edges_to_dropoffs:
                j = self.nodes_by_label[j_dropoff.request]
                i_dropoff = self.nodes_by_label[i.name + self.M]
                path = (j, i, j_dropoff, i_dropoff)
                if self._path_is_infeasible(path): 
                    self._remove_edge(i, j_dropoff)

            # arcs (i,j) removed if (i -> j -> n+i -> n+j) and (i -> j -> n+j -> n+i) ininfeasible
            all_edges_to_pickups = [(i,j) for (i,j) in self.edges if (i.name == o) and (j.type == "o")]
            for (i,j) in all_edges_to_pickups:
                i_dropoff = self.nodes_by_label[i.name + self.M]
                j_dropoff = self.nodes_by_label[j.name + self.M]
                path_1 = (i,j,i_dropoff,j_dropoff)
                path_2 = (i,j,j_dropoff,i_dropoff)
                if self._path_is_infeasible(path_1) and self._path_is_infeasible(path_2):
                    self._remove_edge(i,j)

            # arcs (n+i, j) removed if i -> i+n -> j -> j+n infeasible
            all_edges_from_dropoffs = [(i,j) for (i,j) in self.edges if (j.name == o) and (i.type == "d")]
            for (i_dropoff,j) in all_edges_from_dropoffs:
                i = self.nodes_by_label[i_dropoff.request]
                j_dropoff = self.nodes_by_label[j.name + self.M]
                path = (i, i_dropoff, j, j_dropoff)
                if self._path_is_infeasible(path):
                    self._remove_edge(i_dropoff, j)
        
        for d in self.D:
            # arcs (n+i, n+j) removed if (i->j->n+i->n+j) and (j->i->n+i->n+j) infeasible
            all_edges_to_dropoffs = [(i,j) for (i,j) in self.edges if (i.name == d) and (j.type == "d")]
            for (i_dropoff, j_dropoff) in all_edges_to_dropoffs:
                i = self.nodes_by_label[i_dropoff.request]
                j = self.nodes_by_label[j_dropoff.request]
                path_1 = (i,j,i_dropoff,j_dropoff)
                path_2 = (j,i,i_dropoff,j_dropoff)
                if self._path_is_infeasible(path_1) and self._path_is_infeasible(path_2):
                    self._remove_edge(i_dropoff, j_dropoff)

    def _path_is_infeasible(self, path):
        T = [path[0].e]
        for idx in range(1, len(path) - 1):
            start, end = path[idx, idx+1]
            T_start = max(end.e, T[-1] + self.boarding_time + self.travel_time(start, end))
            if T_start > end.l: return True
        return False
            
    def _is_time_infeasible(self, i, j):
        if i.type in ["o", "d"]:
            return (i.e + self.boarding_time + self.travel_time(i,j)) > j.l
        else:
            return (i.e + self.travel_time(i,j)) > j.l

    def travel_time(self, v: DarpNode, w: DarpNode):
        if (v == self.start_depot) or (w == self.end_depot):
            return 0
        if v.direction != w.direction:
            return self.t_turn
        else:
            return self.distances[v.bus_station][w.bus_station] * self.speed
        
    def travel_distance(self, v: DarpNode, w: DarpNode): 
        if v.is_depot() or w.is_depot() or (v.direction != w.direction):
            return 0
        else:
            return self.distances[v.bus_station][w.bus_station]
        
    def _generate_edge_distances(self):
        for e in self.edges:
            e.distance = self.travel_distance(e._from, e._to)
    
    def get_directional_nodesets(self, as_labels = False):
        if as_labels:
            return [n.name for n in self.F], [n.name for n in self.R]
        else:
            return self.F, self.R
    
    def get_edge_distances(self):
        return {e.label: e.distance for e in self.edges}
    
    def get_edges(self):
        return [e.label for e in self.edges]