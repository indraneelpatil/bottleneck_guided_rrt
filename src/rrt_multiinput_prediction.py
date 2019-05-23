from nolearn.lasagne import BatchIterator
import numpy as np
import tensorflow as tf
import matplotlib
matplotlib.use('TkAgg', warn = False)
from matplotlib import pyplot
import os
import time

from pandas import DataFrame
from pandas.io.parsers import read_csv
from sklearn.utils import shuffle
from sklearn.cross_validation import train_test_split
import utils.data_helper as helper
import utils.visualization as viewer

os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

