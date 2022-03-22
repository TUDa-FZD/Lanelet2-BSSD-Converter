import numpy as np
import math
import logging


class MsgCounterHandler(logging.Handler):

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
    v = [ls[-1].x - ls[0].x, ls[-1].y - ls[0].y]

    return np.array(v)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """

    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))*360/(2*math.pi)


def angle_between_lanelets(ll1, ll2):
    v1 = linestring_to_vector(ll1.centerline)
    v2 = linestring_to_vector(ll2.centerline)
    return angle_between(v1, v2)


def angle_between_linestrings(ls1, ls2):
    v1 = linestring_to_vector(ls1)
    v2 = linestring_to_vector(ls2)
    return angle_between(v1, v2)


def setup_logger(file):
    log_file = 'Output/' + file[4:-4] + '_BSSD_derivation.log'
    logging.basicConfig(filename=log_file,
                        level=logging.DEBUG,
                        filemode='w',
                        format='[%(asctime)s] %(levelname)s %(message)s')
    logger = logging.getLogger('framework')

    msg_counter = MsgCounterHandler()
    msg_counter.setLevel(logging.DEBUG)
    logger.addHandler(msg_counter)

    stream = logging.StreamHandler()
    stream.setLevel(logging.INFO)
    streamformat = logging.Formatter("%(levelname)s:%(message)s")
    stream.setFormatter(streamformat)
    logger.addHandler(stream)

    return logger, log_file


def edit_log_file(log_file):
    with open(log_file, "r") as file:
        contents = file.readlines()
        for nr, line in enumerate(reversed(contents)):
            if 'Statistics' in line:
                for new_line in reversed(contents[-nr - 1:]):
                    contents.insert(0, new_line)
                break

    with open(log_file, "w") as file:
        file.writelines(contents[:-nr-2])
