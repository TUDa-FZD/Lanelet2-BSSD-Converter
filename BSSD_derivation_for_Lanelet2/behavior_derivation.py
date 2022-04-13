import logging

import lanelet2.geometry as geo
from bssd.core._types import CrossingType as ct
from lanelet2.core import AttributeMap

from .preprocessing import is_ll_relevant

logger = logging.getLogger('framework.behavior_derivation')

LINE = {'solid': ct.PROHIBITED,
        'solid_solid': ct.PROHIBITED,
        'dashed': ct.ALLOWED,
        'dashed_solid': {'left': ct.PROHIBITED, 'right': ct.ALLOWED},
        'solid_dashed': {'left': ct.ALLOWED, 'right': ct.PROHIBITED}
        }

lane_mark = {'curbstone': {'low': ct.PROHIBITED,
                           'high': ct.NOT_POSSIBLE
                           },
             'line_thick': LINE,
             'line_thin': LINE,
             'virtual': ct.PROHIBITED,
             'unmarked': ct.ALLOWED,
             'road_border': ct.NOT_POSSIBLE,
             'guard_rail': ct.NOT_POSSIBLE,
             'fence': ct.NOT_POSSIBLE,
             'wall': ct.NOT_POSSIBLE,
             'keepout': ct.PROHIBITED,
             'zig-zag': ct.ALLOWED,
             'BSSD': {'boundary': ct.ALLOWED},
             }


def derive_crossing_type_for_lat_boundary(att: AttributeMap, side: str):
    """
    This function checks for type, subtype and, if necessary, the side of a linestring to determine the CrossingType
    for this linestring. For this purpose a combination of dictionaries is used which assigns CrossingTypes to
    specific values for linestring types and subtypes. Also, the side of the linestring in a lanelet is relevant
    for solid_dashed/dashed_solid marked lines. If at a given point in the function not correct crossing type
    can be derived, a warning is written to the logger.

    Parameters:
        att (AttributeMap):AttributeMap of a linestring of a lateral boundary.
        side (str):'left' or 'right', needed for dashed_solid/solid_dashed linestrings.

    Returns:
        crossing_type (str):CrossingType as a string.
    """

    # Get the linestring type out of the AttributeMap
    ls_type = get_item(att, 'type')
    ls_subtype = None

    # Get the crossing type that is stored for this linestring type
    crossing_type = lane_mark.get(ls_type)

    # If the retrieved object is a dictionary, the next check is done using the subtype
    if isinstance(crossing_type, dict):

        # get the subtype from the AttributeMap
        ls_subtype = get_item(att, 'subtype')
        # Get the crossing type that is stored for this linestring subtype
        crossing_type = crossing_type.get(ls_subtype)

        # If the retrieved object is also a dictionary, the next check is done using the side
        # This is only necessary for solid_dashed/dashed_solid linestrings
        if isinstance(crossing_type, dict):
            # Use the submitted side to lookup the crossing type in this third dictionary
            crossing_type = crossing_type.get(side)
            logger.debug(f'For {ls_type}: {ls_subtype} and side {side} CrossingType {crossing_type.value} has been derived.')
        elif crossing_type:
            logger.debug(f'For {ls_type}: {ls_subtype} CrossingType {crossing_type.value} has been derived.')
    elif crossing_type:
        logger.debug(f'For {ls_type} CrossingType {crossing_type.value} has been derived.')

    # In case a derivation using the dictionary was not possible, write a warning message to the logger
    if not crossing_type:
        logger.warning(f'For type: {ls_type} and subtype: {ls_subtype} CrossingType couln\'t be derived.')

    # return the derived crossing type (may be None)
    return crossing_type


def is_zebra_and_intersecting(ll, ref_ll):
    """
    Returns boolean variable after checking whether two lanelets are having intersection
    centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

    Parameters:
        ll (lanelet):The lanelet that is being checked.
        ref_ll (lanelet):The lanelet on which the behavior spaced is based.

    Returns:
        Bool (bool):True if conditions are met, otherwise False.
    """

    if geo.intersectCenterlines2d(ll, ref_ll) and not is_ll_relevant(ll.attributes) and \
            ll.leftBound.attributes['type'] == ll.rightBound.attributes['type'] == 'zebra_marking':
        return True
    else:
        return False


def get_item(d, t):
    """
    Retrieves value using get function, but checks first whether the requested item exists. Returns None if not.
    """
    if t in d:
        return d[t]
    else:
        return None
