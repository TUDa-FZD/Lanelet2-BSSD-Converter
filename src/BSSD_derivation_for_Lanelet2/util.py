import math
import logging

import numpy as np


class MsgCounterHandler(logging.Handler):
    """ A logging handler that counts messages the logger receives per level.  """
    def __init__(self, *args, **kwargs):
        super(MsgCounterHandler, self).__init__(*args, **kwargs)
        self.levelcount = {'DEBUG': 0,
                           'INFO': 0,
                           'WARNING': 0,
                           'CRITICAL': 0,
                           'ERROR': 0}

    def emit(self, record):
        self.levelcount[record.levelname] += 1


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def linestring_to_vector(ls):
    """ Creates a numpy vector object by using the first and last point of a linestring.  """
    v = [ls[-1].x - ls[0].x, ls[-1].y - ls[0].y]

    return np.array(v)


def angle_between(v1, v2):
    """ Returns the angle in degree between vectors 'v1' and 'v2'::

            >> angle_between((1, 0, 0), (0, 1, 0))
            90
            >> angle_between((1, 0, 0), (1, 0, 0))
            0
            >> angle_between((1, 0, 0), (-1, 0, 0))
            180
    """

    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))*360/(2*math.pi)


def angle_between_lanelets(lanelet_1, lanelet_2):
    """
    Returns the angle in degree between two lanelets. Therefore, their centerlines are being used. Their order doesn't
    influence the result, since always the minimal possible, positive angle is being calculated.

    Parameters:
        lanelet_1 (lanelet):The first lanelet.
        lanelet_2 (lanelet):The second lanelet.

    Returns:
        angle (float):Angle between lanelets in degree.
    """
    v1 = linestring_to_vector(lanelet_1.centerline)
    v2 = linestring_to_vector(lanelet_2.centerline)
    return angle_between(v1, v2)


def angle_between_linestrings(linestring_1, linestring_2):
    """
    Returns the angle in degree between two linestrings. Therefore, first and last points are being used. Their order
    doesn't influence the result, since always the minimal possible, positive angle is being calculated.

    Parameters:
        linestring_1 (linestring):The first linestring.
        linestring_2 (linestring):The second linestring.

    Returns:
        angle (float):Angle between linestrings in degree.
    """
    v1 = linestring_to_vector(linestring_1)
    v2 = linestring_to_vector(linestring_2)
    return angle_between(v1, v2)


def join_dictionaries(dict_a, dict_b):
    '''
    Joins two dictionaries. Intended for dictionaries with partially mutual keys. This way the values of
    the two dictionaries for the same key are being combined in a list. This function is used for the segment search.

    Parameters:
        dict_a (defaultdict):First dictionary.
        dict_b (defaultdict):Second dictionary.

    Returns:
        dict (defaultdict):Combined defaultdict.
    '''
    for d in (dict_a, dict_b):
        for key, value in d.items():
            dict_a[key].update(value)

    return dict_a


def setup_logger(file):
    """
    Sets up the logger. Requires the filepath of the Lanelet2/BSSD output map to store the log-file at the same location.

    Parameters:
        file (path):Path where the output map is written.

    Returns:
        logger (logging):logger object that contains the different handlers required in the framework.
        log_file (path):Automatically created path the log file ('{map_name}+_BSSD.log').
    """
    # Creating file path for the log file based on the output filename of the map
    # log_file = 'Output/' + file[4:-4] + '_BSSD_derivation.log'
    log_file = file[:-4] + '_BSSD_derivation.log'
    # setting up the basicconfig for the logging module to save log messages to file
    logging.basicConfig(filename=log_file,
                        level=logging.DEBUG,
                        filemode='w',
                        format='[%(asctime)s] %(levelname)s %(message)s')
    logger = logging.getLogger('framework')

    # add the handler that counts messages per level
    msg_counter = MsgCounterHandler()
    msg_counter.setLevel(logging.DEBUG)
    logger.addHandler(msg_counter)

    # add the streamhandler that streams messages of INFO and higher to the terminal
    stream = logging.StreamHandler()
    stream.setLevel(logging.INFO)
    streamformat = logging.Formatter("%(levelname)s:%(message)s")
    stream.setFormatter(streamformat)
    logger.addHandler(stream)

    return logger, log_file


def edit_log_file(log_file):
    """
    Edit the final log file to place the statistics section at the top so that it can be seen first.

    Parameters:
        log_file (path):Path where log-file is located.
    """
    # open log_file
    with open(log_file, "r") as file:
        # read the text file per line
        contents = file.readlines()
        # run a for loop that searches for the first line that contains the word 'Statistics'. Run loop in reversed
        # order, because the statistics are at the end of the file.
        for nr, line in enumerate(reversed(contents)):
            if 'Statistics' in line:
                # if found, run for-loop that goes from line with Statistics to the end of the document.
                # reverse order of those lines and insert them one by one at the top of the document
                for new_line in reversed(contents[-nr - 1:]):
                    contents.insert(0, new_line)
                break

    # write the edited line order into the log-file
    with open(log_file, "w") as file:
        file.writelines(contents[:-nr-2])


def make_positive(layer):
    """
    For every element of a layer, check if their ID is negative. If yes, assign a positive ID.

    Parameters:
        layer (layer):Layer of a Lanelet2 map.
    """
    for elem in layer:
        if elem.id < 0:
            elem.id = layer.uniqueId()
