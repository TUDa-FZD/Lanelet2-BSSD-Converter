import logging
import math

from lanelet2.geometry import distance as dist
from lanelet2.core import Point2d, Point3d, BoundingBox2d, BasicPoint2d

from .constants import LONG_BDR_TAGS

logger = logging.getLogger('framework.geometry_derivation')


def find_line_insufficient(ls_list, point_matching, point_free):
    """
    Find the points for a new longitudinal boundary in case there is an existing linestring that doesn't contain
    BOTH of the given endpoints of the lanelets lateral boundaries. Instead, the linestring contains only one of the
    points and has a loose end. Goal of this function is to find the list of points between the matching point and
    loose end and close the gap to the other lateral boundary. This list of points can be used to create a new
    linestring which represents the longitudinal boundary of the behavior space. This function is used once for
    each side of each lanelet.

    Parameters:
        ls_list (list | set):List of linestrings that contain either the startpoint of the left or right lateral boundary
        point_matching (Point2d | Point3d):A point that is part of the linestring in ls_list and of a lateral bdr.
        point_free (Point2d | Point3d):The startpoint of the other lateral boundary that is not part of a ls in ls_list

    Returns:
        lines (list):Pair of ID of found reference linestring and a list of points for the new linestring that is
                           actually used for the longitudinal boundary.
    """

    # Todo: Find better function name
    # Check all linestring of this list whether their linestring type is in the list of potential long bdr types
    for line in ls_list:
        if 'type' in line.attributes and line.attributes['type'] in LONG_BDR_TAGS:

            # extract all points of the current linestring and store them in a list, this enables easier access
            pt_list = [el for el in line]

            # Next, it needs to be determined which end of the linestring lies between the lateral boundaries of the
            # lanelet. Furthermore, the condition that this point is closer to the free point than to the matching
            # point needs to be met. This avoids that linestrings that barely reach into the lanelet are used for
            # determining longitudinal boundaries.
            # 1. case: The last point lies in the lanelet
            if dist(line[-1], point_free) < dist(line[-1], point_matching):
                # linestring can be used for deriving points for new linestring that will serve as long boundary

                # select the points of the linestring from the point that coincides with the lateral boundary until
                # the endpoint that lies inbetween the lateral boundaries of the lanelet
                pts_for_ls = pt_list[pt_list.index(point_matching):]
                pts_for_ls.append(point_free)
                logger.debug(f'Found partially fitting line with ID {line.id}')

                # return a list with the id of the reference linestring and the points for creating the new linestring
                return [line.id, pts_for_ls]

            # 2. case: The first point lies in the lanelet
            elif dist(line[0], point_free) < dist(line[0], point_matching):
                # linestring can be used for deriving points for new linestring that will serve as long boundary

                # select the points of the linestring from the point that coincides with the lateral boundary until
                # the endpoint that lies inbetween the lateral boundaries of the lanelet
                pts_for_ls = pt_list[:pt_list.index(point_matching) + 1]
                pts_for_ls.insert(0, point_free)
                logger.debug(f'Found partially fitting line with ID {line.id}')

                # return a list with the id of the reference linestring and the points for creating the new linestring
                return [line.id, pts_for_ls]

    # In case the conditions haven't been met, return a list with two empty entries
    return [None, None]


def make_orthogonal_bounding_box(pt_left, pt_right):
    """
    Creates a bounding box based on the two start-/endpoints of a lanelet to search for linestrings that are lying
    within that area. The box is created by moving the two points orthogonally to the vector between them.

    Parameters:
        pt_left (Point2d or Point3d): Startpoint of the left lateral boundary of a lanelet
        pt_right (Point2d or Point3d): Startpoint of the right lateral boundary of a lanelet

    Returns:
        bounding_box (BoundingBox2d):Bounding box that can be used to search linestrings and points.
    """

    # create orthogonal vector based on the vector between the two points
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
    """
    This function checks linestrings that contain the startpoint of the left and right lateral boundary linestrings
    of a lanelet for usability as longitudinal boundary. Two cases are possible: The linestring fits exact or
    it covers more than one lane(let). In the first case, the linestring can be used immediately. In the second
    case, the points that geometrically describe the longitudinal boundary need to be extracted from the linestring.

    Parameters:
        pt_left (Point2d or Point3d): Startpoint of the left lateral boundary of a lanelet
        pt_right (Point2d or Point3d): Startpoint of the right lateral boundary of a lanelet
        list_mutual (list | set): List of linestrings that contain pt_left and pt_right

    Returns:
        lines_local (dictionary):Pair of ID of found reference linestring and a list of points for the new linestring
        that is ctually used for the longitudinal boundary. Saves these information for each case to a different key.
    """

    # initialize empty dictionary where potential linestrings and points can be stored in
    lines_local = {'exact': [None],
                   'protruding': [None, None]}

    # loop throught the linestrings of the list
    for line in list_mutual:

        # if left and right point are the endpoints of the current linestring, this linestring fits exactly as the
        # longitudinal boundary
        if points_are_endpoints(line, pt_left, pt_right):
            lines_local['exact'] = [line.id]
            logger.debug(f'Found exactly fitting line with ID {line.id}')

        # If points are not the endpoints, the linestring exceeds the width of the lanelet
        # Check, if this linestring type is among the types that are considered to be potential longitudinal boundaries
        elif line.attributes['type'] in LONG_BDR_TAGS:

            # extract the points of the linestring that are covering the width of the lanelet

            # first, create list of points from current linestring
            pt_list = [pt for pt in line]

            # find indexes of left and right point
            idx_l = pt_list.index(pt_left)
            idx_r = pt_list.index(pt_right)
            min_idx = min(idx_l, idx_r)
            max_idx = max(idx_l, idx_r)

            # extract the points and save them in lines_local
            lines_local['protruding'] = [line.id, pt_list[min_idx:max_idx + 1]]
            logger.debug(f'Found protrudingly fitting line with ID {line.id}')

    # return the dictionary that contains the found points/linestrings. If no linestring met the conditions,
    # the dictionary will contain empty lists
    return lines_local


def points_are_endpoints(line, pt_left, pt_right):
    """
    Checks if two points are the endpoints of a linestring. Therefore, both combinations are considered.

    Parameters:
        line (Linestring2d or Linestring3d): Linestring to be checked for
        pt_left (Point2d or Point3d): first point
        pt_right (list): second point

    Returns:
        points_are_endpoints (bool):True if condition is met
    """

    if ((line[0].id == pt_left.id and line[-1].id == pt_right.id) or
            (line[-1].id == pt_left.id and line[0].id == pt_right.id)):
        return True
    else:
        return False
