# The line-based Dial-a-Ride problem

This is the implementation of the Subline-Based, Location-Based, and Event-Based formulations for the static line-based Dial-a-Ride problem. We include the benchmark instances used to compare the models.

## Description
This repository is split into three parts: the Subline- and Location-Based models, which are implemented in Python 3 with Gurobi 10.0, and the Event-Based model, which is implemented in C++ 17 with CPLEX 22.1, and the benchmark instances.

### Location- and Subline-Based models 

Both the Subline-Based and the Location-Based models are built upon the model class (in model.py). 
The Subline-Based model is available in sublineModel.py. The Location-Based models is built using an underlying graph, constructed in darpGraph using darpEdge and darpNode. The model is available at darpModel.py.
The util.py file provides inputs and general utility functions and the requests.py file provides a way of reading and handling requests.

Run the models using the main.py file. Here, you can select between a debugging & productive mode as well as disabling either model.

### Event-Based model
This repository is a fork of the [Event-Based MILP for the DARP](https://git.uni-wuppertal.de/dgaul/event-based-milp-for-darp) by Daniela Gaul. The code has been adapted for the static liDARP, ensuring directionality constraints are respected. Note that it is required to set *dynamic = false*, as the code for the Rolling Horizon has not yet been adapted to the liDARP structure. 

Compile the project and run it using the darp_milp_3.cpp or darp_milp_6.cpp file, depending on the desired vehicle capacity.

## Installation
This project was written on Windows. Make sure you have all required packages installed on your local system, including the required licenses for Gurobi and CPLEX. For other operating systems, you may have to adjust system paths.

## Usage
All models are run from the command line and accept three parameters: the path to the directory containing request files, an integer [1,2,3] to determine the instance mode, and an optional path to the location file (required for instance modes 2 and 3). 

- Instance mode 1: consider the line with equal distances = 1. Shortcuts are possible and can be toggled separately.
- Instance mode 2: consider the line with given (x,y) locations of bus stations. Distances are calculated using the Euclidean metric. Shortcuts are possible and can be toggled separately.
- Instance mode 3: consider the line with a full distance matrix. Shortcuts need to be included in the file.

## Instance Format
The models accept instance files as .txt files in one of the following formats.

**Request File**

This file details the requests and basic parameters of the instance.

The first row constains:

    num_vehicles num_requests num_stations  max_time vehicle_capacity alpha beta
        |K|           |R|          |H|         T            Q         alpha beta

The remaining rows denote, for all requests, the following:

    idx origin_station destination_station service_time demand earliest_start_time latest_arrival_time

The values for earliest_start_time and latest_arrival_time are either floats or NaN. The index starts at 1.

If only the request file is input, all distances between stations are assumed to be 1.

**Station File**

This file details the location of all bus stations. The rows are of the format

````
station_id x_coordinate y_coordinate
````

If input, the distances between stations will be calculated using the Euclidean metric.

**Distance File**

This file inputs a fixed distance matrix between pairs of stations, such that there are m rows and m columns, formatted as

````
d_{1,1} d_{1,2} ... d_{1,m}
d_{2,1} d_{2,2} ... d_{2,m}
...
d_{m,1} d_{m,2}... d_{m,m}
````

If input, the distances between stations will be taken from this matrix. It needs to be a complete matrix with zeros on the diagonal.

## Support
E-Mail kendra.reiter@uni-wuerzburg.de

## Authors and acknowledgment
The author of the code is Kendra Reiter (kendra.reiter@uni-wuerzburg.de), partially based on code for the [Rolling-Horizon Event-Based Graph](https://git.uni-wuppertal.de/dgaul/rolling-horizon-algorithm-for-dynamic-darp) by Daniela Gaul (gaul@math.uni-wuppertal.de). The code was developed at the University of Würzburg, Germany with support from Marie Schmidt and Michael Stiglmayr.


## License

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />

This work is licensed under [CC BY-NC-SA 4.0 ](https://creativecommons.org/licenses/by-nc-sa/4.0/).
