import logging

import lanelet2.geometry as geo
from lanelet2.core import AttributeMap

from .preprocessing import is_lanelet_relevant
from .constants import LANE_MARK

logger = logging.getLogger('framework.behavior_derivation')


def derive_crossing_type_for_lat_boundary(linestring_attributes: AttributeMap, side: str):
    """
    This function checks for type, subtype and, if necessary, the side of a linestring to determine the CrossingType
    for this linestring. For this purpose a combination of dictionaries is used which assigns CrossingTypes to
    specific values for linestring types and subtypes. Also, the side of the linestring in a lanelet is relevant
    for solid_dashed/dashed_solid marked lines. If at a given point in the function not correct crossing type
    can be derived, a warning is written to the logger.

    Parameters:
        linestring_attributes (AttributeMap):AttributeMap of a linestring of a lateral boundary.
        side (str):'left' or 'right', needed for dashed_solid/solid_dashed linestrings.

    Returns:
        crossing_type (str):CrossingType as a string.
    """

    # Get the linestring type out of the AttributeMap
    linestring_type = get_item(linestring_attributes, 'type')
    linestring_subtype = None

    # Get the crossing type that is stored for this linestring type
    crossing_type = LANE_MARK.get(linestring_type)

    # If the retrieved object is a dictionary, the next check is done using the subtype
    if isinstance(crossing_type, dict):

        # get the subtype from the AttributeMap
        linestring_subtype = get_item(linestring_attributes, 'subtype')
        # Get the crossing type that is stored for this linestring subtype
        crossing_type = crossing_type.get(linestring_subtype)

        # If the retrieved object is also a dictionary, the next check is done using the side
        # This is only necessary for solid_dashed/dashed_solid linestrings
        if isinstance(crossing_type, dict):
            # Use the submitted side to lookup the crossing type in this third dictionary
            crossing_type = crossing_type.get(side)
            logger.debug(f'For {linestring_type}: {linestring_subtype} and '
                         f'side {side} CrossingType {crossing_type.value} has been derived.')
        elif crossing_type:
            logger.debug(f'For {linestring_type}: {linestring_subtype} '
                         f'CrossingType {crossing_type.value} has been derived.')
    elif crossing_type:
        logger.debug(f'For {linestring_type} CrossingType {crossing_type.value} has been derived.')

    # In case a derivation using the dictionary was not possible, write a warning message to the logger
    if not crossing_type:
        logger.warning(f'For type: {linestring_type} and subtype: {linestring_subtype} '
                       f'CrossingType couln\'t be derived.')

    # return the derived crossing type (may be None)
    return crossing_type


def is_zebra_and_intersecting(lanelet, ref_lanelet):
    """
    Returns boolean variable after checking whether two lanelets are having intersecting
    centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

    Parameters:
        lanelet (lanelet):The lanelet that is being checked.
        ref_lanelet (lanelet):The lanelet on which the behavior spaced is based.

    Returns:
        Bool (bool):True if conditions are met, otherwise False.
    """

    if geo.intersectCenterlines2d(lanelet, ref_lanelet) and not is_lanelet_relevant(lanelet.attributes) and \
            lanelet.leftBound.attributes['type'] == lanelet.rightBound.attributes['type'] == 'zebra_marking':
        return True
    else:
        return False


def get_item(dictionary, key):
    """
    Retrieves value using get function, but checks first whether the requested item exists. Returns None if not.
    Dictionaries actually have built-in function that does the exact same, but this function is used instead for
    AttributeMaps from the Lanelet2-framework.
    """
    if key in dictionary:
        return dictionary[key]
    else:
        return None
