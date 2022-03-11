from lanelet2.geometry import distance as dist
import math
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
from constants import LONG_BDR_TAGS, LONG_BDR_DICT
import logging
logger = logging.getLogger('framework.geometry_derivation')


def find_line_insufficient(ls_list, point_matching, point_free):
    # Find the points for a new longitudinal boundary in case there is an existing stop line
    # that doesn't contain BOTH of the given endpoints of the lanelets lateral boundaries.
    # Instead, the linestring contains only one of the points and has a loose end. Goal of this
    # function is to find the list of points between the matching point and loose end and add the
    # open end on the other lateral boundary. This list of points can be used to create a new
    # linestring which represents the longitudinal boundary of the behavior space.
    # Todo: Find better function name
    for line in ls_list:
        if 'type' in line.attributes and line.attributes['type'] in LONG_BDR_TAGS:
            # ls shows to the right
            pt_list = [el for el in line]
            if dist(line[-1], point_free) < dist(line[-1], point_matching):
                pts_for_ls = pt_list[pt_list.index(point_matching):]
                pts_for_ls.append(point_free)
                logger.debug(f'Found partially fitting line with ID {line.id}')
                return [line.id, pts_for_ls]

            # ls shows to the left
            elif dist(line[0], point_free) < dist(line[0], point_matching):
                pts_for_ls = pt_list[:pt_list.index(point_matching) + 1]
                pts_for_ls.insert(0, point_free)
                logger.debug(f'Found partially fitting line with ID {line.id}')
                return [line.id, pts_for_ls]

    return [None, None]


def make_orth_bounding_box(pt_left, pt_right):
    # vector v from left point to right point
    # v = [pt_right.x - pt_left.x, pt_right.y - pt_left.y]
    # create orthogonal vector from vector v
    v_orth = [pt_right.y - pt_left.y, -(pt_right.x - pt_left.x)]
    # calculate length of vector v/v_orth
    length_v = math.sqrt(v_orth[0] * v_orth[0] + v_orth[1] * v_orth[1])
    # normalize vector v_orth
    v_orth = [el / length_v for el in v_orth]

    # add orthogonal vector to coordinates of left point and substract from coordinates of the right point
    # from new coordinates: find min and max x and y values and initalize new points
    min_pt = BasicPoint2d(min(pt_left.x + v_orth[0], pt_right.x - v_orth[0]),
                          min(pt_left.y + v_orth[1], pt_right.y - v_orth[1]))
    max_pt = BasicPoint2d(max(pt_left.x + v_orth[0], pt_right.x - v_orth[0]),
                          max(pt_left.y + v_orth[1], pt_right.y - v_orth[1]))

    # from previously created points initalize BoundingBox2d and return it
    return BoundingBox2d(min_pt, max_pt)


def find_flush_bdr(pt_left, pt_right, list_mutual):
    lines_local = {'exact': [None],
                   'protruding': [None, None]}

    for line in list_mutual:

        if points_are_endpoints(line, pt_left, pt_right):
            lines_local['exact'] = [line.id]
            logger.debug(f'Found exactly fitting line with ID {line.id}')
        elif line.attributes['type'] in LONG_BDR_TAGS:
            pt_list = [pt for pt in line]
            idx_l = pt_list.index(pt_left)
            idx_r = pt_list.index(pt_right)
            min_idx = min(idx_l, idx_r)
            max_idx = max(idx_l, idx_r)
            lines_local['protruding'] = [line.id, pt_list[min_idx:max_idx + 1]]
            logger.debug(f'Found protrudingly fitting line with ID {line.id}')
        else:
            pass
            # print(mutual_ls, pt_left, pt_right)

    return lines_local


def points_are_endpoints(line, pt_left, pt_right):
    if ((line[0].id == pt_left.id and line[-1].id == pt_right.id) or
            (line[-1].id == pt_left.id and line[0].id == pt_right.id)):
        return True
    else:
        return False
