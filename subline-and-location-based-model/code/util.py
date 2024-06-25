# handle all imports
import os
import sys
import warnings
import pickle
import datetime
import math
import argparse
import pandas as pd
import numpy as np
import itertools
import gurobipy as gp
from gurobipy import GRB
from darpNode import DarpNode
from darpEdge import DarpEdge
from typing import Union, List, Optional

REQUEST_TYPE = list[int, int, float, float]
REQUEST_LIST_TYPE = List[Union[int, int, Optional[int], Optional[int]]]

ORIGIN_IDX = 0
DESTINATION_IDX = 1
EARLIEST_START_TIME_IDX = 2
LATEST_ARRIVAL_TIME_IDX = 3
TIME_WINDOW_START_IDX = 4
TIME_WINDOW_END_IDX = 5

EPSILON = 10**(-5)

SEPERATOR = "----------------------------------------"


def isvalid(number):
    if (number is None) or np.isnan(number):
        return False
    else:
        return True