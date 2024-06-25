from util import *
from darpModel import DARPModel
from sublineModel import SublineModel
from travelRequests import TravelRequests

station_file = None
distance_matrix_file = None

# parse arguments from command line
parser = argparse.ArgumentParser("main.py")
parser.add_argument("requests", help="Path to request files.", type=str)
parser.add_argument("instance_mode", help="1: discrete line, 2: station locations, 3: distance matrix.", type=int, default=1,choices=[1,2,3])
parser.add_argument("--station_locations", help="Path to location / distance file name, in .txt format.", type=str, nargs="?", required=False)
args = parser.parse_args()

# validation
if not os.path.exists(args.requests):
    raise ValueError("Path to request files does not exist.")
if (args.instance_mode == 2) or (args.instance_mode == 3):
    if args.station_locations is None:
        raise ValueError("Expected a location file when selecting instance_mode == {}.".format(args.instance_mode))
    if not os.path.exists(args.station_locations):
        raise ValueError("Path to location file does not exist.")
    if (args.instance_mode == 2):
        station_file = args.station_locations
    else:
        distance_matrix_file = args.station_locations
    

input_folder = args.requests

# settings
TESTING = True # toggle to change verbose setting
RUN_LOCATION_MODEL = True
RUN_SUBLINE_MODEL = True

# TODO: simplify output to command line
# TODO: decide on symmetry breaking in Subline-Based Model

# model parameters and input
obj_weights = [10,1] # w_1: accepted passengers, w_2: saved distance 

speed = 1
time_to_turn = 0.5
consider_shortcuts = True

time_limit_in_minutes = 60
time_limit = time_limit_in_minutes * 60

output_path = "output/"

for request_name in os.listdir(args.requests):
    if (request_name == args.station_locations):
        continue

    request_file = os.path.join(input_folder, request_name)

    print(SEPERATOR)
    print("Request file:", request_name)
    print(SEPERATOR)

    # parse requests
    parsed_requests = TravelRequests()
    number_of_busses, max_time_in_minutes, bus_capacity, number_of_stations, service_time, alpha, beta = parsed_requests.read_file(
        instance_file=request_file, consider_shortcuts=consider_shortcuts, station_location_file=station_file, distance_matrix_file=distance_matrix_file)

    # Location-Based model
    if RUN_LOCATION_MODEL:
        print("Running the Location-Based model.")
        print(SEPERATOR)
        DARP = DARPModel(requests=parsed_requests, num_stations=number_of_stations, num_busses=number_of_busses, timeframe=max_time_in_minutes,
                            boarding_time=service_time, Q_max=bus_capacity, speed=speed, t_turn=time_to_turn, alpha=alpha, beta=beta)
        DARP.createModel(obj_weights=obj_weights)
        DARP.optimize(verbose=TESTING, params={"TimeLimit": time_limit})
        DARP.postprocessing(verbose=TESTING)
        if not TESTING:
            DARP.detailed_file(path=output_path, instance=request_name)
            print("Saved detail file.")
        print(SEPERATOR)

    # Subline-Based model
    if RUN_SUBLINE_MODEL:
        print("Running the Subline-Based model.")
        print(SEPERATOR)

        subline = SublineModel(requests=parsed_requests, num_stations=number_of_stations, num_busses=number_of_busses,
                                boarding_time = service_time, timeframe=max_time_in_minutes, Q_max=bus_capacity, 
                                speed=speed, t_turn=time_to_turn, alpha=alpha, beta=beta)
        subline.createModel(obj_weights=obj_weights)
        subline.optimize(verbose=TESTING, params={"TimeLimit": time_limit})
        subline.postprocessing(verbose=TESTING)
        if not TESTING:
            subline.detailed_file(path=output_path, instance=request_name)
            print("Saved detail file.")

        print(SEPERATOR)