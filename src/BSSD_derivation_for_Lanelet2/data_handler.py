import logging
from collections import defaultdict
from typing import Dict, Any

from lanelet2.geometry import distance as dist
from lanelet2.core import LineString3d, getId, SpeedLimit
from lanelet2 import traffic_rules
import lanelet2.geometry as geo
from bssd.core import _types as tp

from .preprocessing import is_lanelet_relevant
from . import BSSD_elements
from .geometry_derivation import make_orthogonal_bounding_box, find_flush_bdr, find_line_insufficient
from .behavior_derivation import derive_crossing_type_for_lat_boundary, is_zebra_and_intersecting
from . import util
from .constants import LONG_BDR_TAGS, LONG_BDR_DICT

logger = logging.getLogger('framework.data_handler')


class DataHandler:
    """
    A class that contains the map data of Lanelet2 objects as well as BSSD objects. Main functionality to process
    Lanelet2 map data and derive the BSSD extension. Therefore, it contains multiple methods that use that data to
    loop through the map, derive behavior and more.

    Attributes
    ----------
        map_lanelet : LaneletMap
            Layered lanelet2 map that contains all lanelet2 objects of a loaded map.
        map_bssd : BssdMap
            Layered bssd map that contains all bssd objects.
        relevant_lanelets : list
            List of lanelets of a Lanelet2 map that are considered relevant (see preprocessing for more info)
        graph : RoutingGraph
            Graph for the lanelet map that is adjusted to contain all the lanelets of a map.
        traffic_rules : traffic_rules
            traffic rules object from lanelet2 for participant = vehicle

    Methods
    -------
        __init__(map_lanelet):
            Initiates class instance by getting lanelet map object. Creates empty bssd map object.
            Creates RoutingGraph and also calls function to find relevant lanelets.
        recursive_loop(lanelet_id, direction=None, linestring=None):
            Recursively loops through lanelets in a map. Function is called from framework.py
        identify_longitudinal_boundary(point_left, point_right, use_previous, previous):
            For end-/startpoints of lateral boundaries of lanelet this function searches for potential
            linestrings that can be used to determine the linestring of the correspondent behavior space.
            Returns a found or newly created linetring.
        find_inside_lines(behavior_space, lanelet):
            Searches for linestrings that are not sharing any points with the points of a laneletes lateral boundaries.
        derive_behavior(behavior_space, lanelet):
            Main function for behavior derivation. Calls multiple subfunctions to derive behavioral attributes.
        derive_behavior_boundary_lateral(behavior_a, behavior_b, side):
            Derives CrossingType through comparing linestring types with a predefined dictionary.
        derive_boundary_long_behavior(behavior, lanelet):
            Searches for conflicting zebra lanelets to derive the no_stagnant_traffic attribute.
        derive_segment_speed_limit(lanelet):
            Starting from one lanelet this function auto detects all lanelets of the same segment and
            cross assigns speed limits for behaviors against reference direction.
        find_adjacent(current_lanelet, level, previous_lanelet=None):
            Recursively searches for every lanelet of the same segment but for the same reference
            driving direction. Returns dictionary with lanelets assigned to a respective level on the roadway.
        assign_speed_limit_along(segment):
            Temporarily assigns lanelet speed limit to lanelet attributes for every lanelet in a segment.
        assign_speed_limit_against(segment, other_ll=None):
            Temporarily assigns lanelet speed limit for behavior against reference direction distinguishing between
            structurally divided driving directions.
        find_one_sided_neighbors(lanelet, linestring, orientation):
            Searches for neighbors of a lanelet either through direct neighborhood or next to a keepout area.
        neighbor_next_to_area(linestring):
            Searches for keepout areas and returns every lanelet surrounding this area.
        filter_for_segment_membership(sourrounding_lanelets, ref_lanelet, ref_linestring, orientation):
            For given lanelets that surround area this function filters them to find lanelets of the same segment.
        are_linestrings_orthogonal(linestring_1, linestring_2, points_to_link_linestrings):
            Compares two linestrings and a newly created linestring inbetween them in all ways for orthogonality.
        derive_conflicts(behavior_space):
            Derives conflicts of a given lanelet. For zebra lanelets set external reservation and reservation links.
        find_neighbor_areas(linestring, subtype=None):
            Finds direct neighbors of an area to set the reservation links at a zebra crossing.
    """

    def __init__(self, map_lanelet, relevant_lanelets, routing_graph):
        self.map_lanelet = map_lanelet
        self.map_bssd = BSSD_elements.BssdMap()
        self.relevant_lanelets = relevant_lanelets
        self.traffic_rules = traffic_rules.create(traffic_rules.Locations.Germany,
                                                  traffic_rules.Participants.Vehicle)
        self.graph = routing_graph

    # -----------------------------------------------
    # -------------------- loop ---------------------
    # -----------------------------------------------
    def recursive_loop(self, lanelet_id, direction=None, linestring=None):
        """
        Starting at any given lanelet of a map, this function loop through all lanelets that can be reached via
        successor/predecessor connections. To achieve this, this function calls itself for every successor and
        predecessor of a lanelet. For the next lanelets, it will repeat this process. Removing a processed lanelet
        from a list of relevant lanelets assures that no lanelet will be touched twice. As soon as the end of every
        possible path is reached, the loop ends. To make sure every other lanelet of a map is considered as well
        a while-loop in framework.py makes sure that the loop is started again for the remaining lanelets.

        During the processing of a lanelet, this function calls other functions that create BSSD elements and link
        the behavior space to the lanelet. Furthermore, longitudinal boundaries are identified an derivations
        of behavioral demand are being performed.

        Parameters:
            lanelet_id (int):The id of the lanelet that is being processed.
            direction (str):The direction from which the previous lanelet called the function for this lanelet.
            linestring (LineString3d | LineString3d):Longitudinal boundary of previous lanelet (if exists).
        """

        # Retrieve lanelet object from lanelet map via ID
        lanelet = self.map_lanelet.laneletLayer[lanelet_id]
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        self.relevant_lanelets.remove(lanelet_id)

        logger.debug(f'----------------------------------------------------------------------------------------------')
        logger.debug(f'Derivation for Lanelet {lanelet_id}')

        # Determine longitudinal boundaries of both sides of the lanelet
        # Based on the assumption that lanelets and behavior space are covering the same part of the roadway
        logger.debug(f'Derivation of longitudinal boundary for along behavior')
        linestring_along_boundary_long, ref_linestring_along_boundary_long = \
            self.identify_longitudinal_boundary(lanelet.leftBound[0], lanelet.rightBound[0],
                                                'along' == direction, linestring)
        logger.debug(f'Derivation of longitudinal boundary for against behavior')
        linestring_against_boundary_long, ref_linestring_against_boundary_long = \
            self.identify_longitudinal_boundary(lanelet.leftBound[-1], lanelet.rightBound[-1],
                                                'against' == direction, linestring)

        # create behavior space object and alanelet bssd objects that are necessary for that
        # Arguments are the lanelet and the longitudinal boundaries so that they can be assigned immediately
        new_behavior_space = \
            self.map_bssd.create_placeholder(lanelet, linestring_along_boundary_long, linestring_against_boundary_long)

        # If a reference linestring has been found from which the longitudinal have been derived,
        # save their IDs in the longitudinal boundary objects.
        if ref_linestring_along_boundary_long:
            new_behavior_space.alongBehavior.longBound.ref_line = ref_linestring_along_boundary_long
        if ref_linestring_against_boundary_long:
            new_behavior_space.againstBehavior.longBound.ref_line = ref_linestring_against_boundary_long
        logger.debug(f'Created Behavior Space {new_behavior_space.id}')

        # Call function for behavior derivation for the behavior space that was created for the current lanelet
        self.derive_behavior(new_behavior_space, lanelet)

        # Call function in itself for the succeeding and preceding lanelet(s) and hand over information
        # about already derived boundaries. Check if the following lanelet is relevant to avoid deriving
        # behavior spaces for irrelevant lanelets.
        for successor in self.graph.following(lanelet):
            if successor.id in self.relevant_lanelets:
                self.recursive_loop(successor.id, 'along', linestring_against_boundary_long)
        for predecessor in self.graph.previous(lanelet):
            if predecessor.id in self.relevant_lanelets:
                self.recursive_loop(predecessor.id, 'against', linestring_along_boundary_long)

    # -----------------------------------------------
    # ----------- longitudinal boundary -------------
    # -----------------------------------------------
    def identify_longitudinal_boundary(self, point_left, point_right, use_previous, id_previous_linestring):
        """
        Determine the geometrical representation of the longitudinal boundary for one side of a lanelet which
        corresponds with longitudinal boundary of one of the behaviors. For this, multiple cases are considered and
        checked.

        Parameters:
            point_left (Point2d | Point3d):First/last point of the left lateral boundary of a lanelet.
            point_right (Point2d | Point3d):First/last point of the right lateral boundary of a lanelet.
            use_previous (bool):True, if the linestring identified for the previous lanelet can be used.
            id_previous_linestring (lanelet):Linestring of the previous lanelet/behavior space.

        Returns:
            linestring (LineString2d | LineString3d):Linestring representing determined geometry of longitudinal bdry.
            ref_line (bool):ID of the orig. linestring the new linestring has been derived from (None if not existent).
        """

        # Initalize empty variables for the linestring and reference line
        linestring = None
        ref_line = None

        # Check different possible cases
        # First case is that the linestring information are given from the previous lanelet. In that case, the same
        # linestring will be used to represent the longitudinal boundary of the current behavior space (at one side)
        if use_previous:
            # Use previously created linestring
            linestring = id_previous_linestring
            ref_line = linestring.id
            logger.debug(f'Using linestring from successor/predecessor (ID: {linestring.id})')

        # If the start-/endpoints of this side of a lanelet are identical, no longitudinal boundary exists
        elif point_left.id == point_right.id:
            # No longitudinal boundary exists
            logger.debug(f'Longitudinal boundary doesn\'t exist')
            pass

        # Otherwise, check for existing lineStrings (e.g. stop_line). In each case, if a linestring matches
        # the conditions, the points that are necessary for creating a new linestring will be extracted
        else:
            # Setup a dictionary to store linestrings for each possible case.
            lines = LONG_BDR_DICT

            # Find every usage of the left and right point
            linestring_list_point_left = set(self.map_lanelet.lineStringLayer.findUsages(point_left))
            linestring_list_point_right = set(self.map_lanelet.lineStringLayer.findUsages(point_right))

            # Determine the linestrings that contain the left and the right point
            mutual_linestring = set.intersection(linestring_list_point_left, linestring_list_point_right)

            # Remove the mutual linestrings from the lists of each point to make sure that in the functions no
            # wrong derivations will be made.
            linestring_list_point_left = linestring_list_point_left - mutual_linestring
            linestring_list_point_right = linestring_list_point_right - mutual_linestring

            # FIRST CASE: linestring contains both points
            # This gives two options: The linestring is fitting exactly OR is overarching.
            lines.update(find_flush_bdr(point_left, point_right, mutual_linestring))

            # SECOND CASE: linestrings that contain only one point
            # The linestring is therefore covering the lanelet insufficiently
            lines['insufficient_half_left'] = \
                find_line_insufficient(linestring_list_point_left, point_left, point_right)
            lines['insufficient_half_right'] = \
                find_line_insufficient(linestring_list_point_right, point_right, point_left)

            # THIRD CASE: linestrings that do not contain one of the points
            # linestrings will be searched using a BoundingBox
            lines['free'] = self.find_free_lines(point_left, point_right)

            # In case multiple linestrings have been found, write a warning
            if len([v for k, v in lines.items() if v[0]]) > 1:
                logger.warning(f'Multiple possible long. boundaries found for points {point_left} and {point_right}')

            # Check if a linestring has been found
            # First condition is that an exact linestring has been found. For this case, this linestring will
            # be used directly as the longitudinal boundary
            if lines['exact'][0]:
                linestring = self.map_lanelet.lineStringLayer[lines['exact'][0]]
                ref_line = linestring.id
                logger.debug(f'Using existing line with ID {linestring.id} as long. boundary')

            # In every other case, the creation of a new line is necessary
            else:
                # If a linestring has been found get the points and ref line from the lines-dictionary
                if any(value[0] for key, value in lines.items()):

                    # Find a case in the dictionary that has been found
                    matching_case = [key for key, value in lines.items() if value[0]]
                    # Retrieve the list of points that will be used to create the new linestring object
                    points_for_new_linestring = lines[matching_case[0]][1]
                    # Extract the linestring id of the linestring that will be used as a reference
                    ref_line = lines[matching_case[0]][0]
                    logger.debug(f'Using existing line with ID {lines[matching_case[0]][0]}'
                                 f'partially for long. boundary')
                # If no linestring has been found, save the start-/endpoints of of the lateral boundaries of the lanelet
                else:
                    points_for_new_linestring = [point_left, point_right]
                    logger.debug(f'No existing line has been found, using endpoints for new linestring.')

                # For the identified points, create a new linestring and add it to the lanelet map
                # First, get the mutable point object from the lanelet map, because also ConstPoints
                # are used in linestrings
                points_for_new_linestring = \
                    [self.map_lanelet.pointLayer[pt.id] for pt in points_for_new_linestring]
                linestring = LineString3d(getId(), points_for_new_linestring, {'type': 'BSSD', 'subtype': 'boundary'})
                logger.debug(f'Created new linestring as longitudinal boundary with ID {linestring.id}')
                self.map_lanelet.add(linestring)

        return linestring, ref_line

    def find_free_lines(self, point_left, point_right):
        """
        Searches for linestrings that are relevant for the longitudinal boundary of a behavior space but don't contain
        one of the two start-/endpoints of a lanelet (which is referenced by the behavior space)

        Parameters:
            point_left (Point2d | Point3d):The start-/endpoint of the linestring of the left lateral boundary
            point_right (Point2d | Point3d):The start-/endpoint of the linestring of the right lateral boundary

        Returns:
            result (list):list with two items: 1 ref ls id and 2 points for the creation of a new linestring
        """

        # Create a bounding box that is created so that it finds linestrings that don't exceed the lanelet borders
        search_box = make_orthogonal_bounding_box(point_left, point_right)

        # Use bounding box to search for linestrings in the area of a potential long boundary
        # do not consider any linestring that contain either the left or the right start-/endpoint of the lat boundaries
        # With this method every linestring that at least overlaps a bit with the bounding box will be found. Because
        # of that, another condition is checked later within the for-loop-
        nearby_linestrings = [linestring for linestring in self.map_lanelet.lineStringLayer.search(search_box)
                              if point_left not in linestring or point_right not in linestring]

        # For-loop through the lines that were found to check further conditions
        for linestring in nearby_linestrings:
            # Distinguish inside and outside of lanelet
            # This is achieved by checking whether the two endpoints of the linestring ly within the bounding box
            # First, the type of the linestring is checked, if it generally could be considered for a long boundary
            if 'type' in linestring.attributes and linestring.attributes['type'] in LONG_BDR_TAGS and \
                    all(x in self.map_lanelet.pointLayer.search(search_box) for x in [linestring[0], linestring[-1]]):

                # If conditions are met, the linestring will be used to derive the actual longitudinal boundary
                # Store the points of the linestring in a list
                points_for_new_linestring = [point for point in linestring]

                # Check the orientation of the linestring to append point_left and point_right at the right place
                if dist(linestring[0], point_left) < dist(linestring[0], point_right):
                    points_for_new_linestring.insert(0, point_left)
                    points_for_new_linestring.append(point_right)
                else:
                    points_for_new_linestring.insert(0, point_right)
                    points_for_new_linestring.append(point_left)
                logger.debug(f'Found inside line with ID {linestring.id}')
                return [linestring.id, points_for_new_linestring]

        # If nothing was found, return a list with two empty items
        return [None, None]

    # -----------------------------------------------
    # ------------ behavior derivation --------------
    # -----------------------------------------------
    def derive_behavior(self, behavior_space, lanelet):
        """
        This is the main function for actual derivations of behavioral demands. It integrates calls for other functions
        that are deriving specific behavior attributes and properties. Therefore, it is possible to extend the behavior
        derivations by adding more subfunctions in the future. The derivation is started after creating a behavior space
        placeholder element for a lanelet in "recursive_loop".

        Parameters:
            behavior_space (BehaviorSpace):Behavior space object that is supposed to be filled within this function.
            lanelet (Lanelet):The lanelet on which the behavior spaced is mapped.
        """

        # 1. Derive behavioral demand of the lateral boundaries
        logger.debug(f'_______ Deriving behavioral demand of lateral boundaries _______')
        logger.debug(f'Deriving behavioral demand of lateral boundary of alongBehavior (left,'
                     f' ID:{behavior_space.alongBehavior.leftBound.id}) and '
                     f'againstBehavior (right, ID:{behavior_space.againstBehavior.rightBound.id})')
        if behavior_space.alongBehavior.leftBound.lineString.inverted():
            side_along = 'right'
        else:
            side_along = 'left'
        self.derive_behavior_boundary_lateral(behavior_space.alongBehavior, behavior_space.againstBehavior, side_along)
        logger.debug(f'Deriving behavioral demand of lateral boundary of againstBehavior (left,'
                     f' ID:{behavior_space.againstBehavior.leftBound.id}) and '
                     f'alongBehavior (right, ID:{behavior_space.alongBehavior.rightBound.id})')
        if behavior_space.againstBehavior.leftBound.lineString.inverted():
            side_against = 'left'
        else:
            side_against = 'right'
        self.derive_behavior_boundary_lateral(behavior_space.againstBehavior, behavior_space.alongBehavior, side_against)

        # 2. Derive behavioral demand of the longitudinal boundaries
        logger.debug(f'_______ Deriving behavioral demand of longitudinal boundaries _______')
        if behavior_space.alongBehavior.longBound:
            logger.debug(f'Deriving behavioral demand of longitudinal boundary of'
                         f' alongBehavior (ID:{behavior_space.alongBehavior.longBound.id})')
            self.derive_boundary_long_behavior(behavior_space.alongBehavior, lanelet)
        if behavior_space.againstBehavior.longBound:
            logger.debug(f'Deriving behavioral demand of longitudinal boundary of'
                         f' againstBehavior (ID:{behavior_space.againstBehavior.longBound.id})')
            self.derive_boundary_long_behavior(behavior_space.againstBehavior, lanelet)

        # 3. Derive speed limits for along and against reference direction of a behavior space
        # To do so, check first whether the speed limits have already been derived for this segment.
        # If not, the segmentwise derivation will be started.
        logger.debug(f'_______ Deriving speed limits _______')
        if 'along_speed_limit' not in lanelet.attributes and 'against_speed_limit' not in lanelet.attributes:
            logger.debug(f'Derive speed limit for the segment the current lanelet belongs to.')
            self.derive_segment_speed_limit(lanelet)
        else:
            logger.debug(f'Segmentwise speed limit derivation has already been done for this lanelet.')

        # If speed limit already has been derived for the current lanelet, get the values that are temporarily 
        # stored in the lanelet attributes to save them in the respective behaviors. If existing, add a reference
        # to the speed indicator
        speed_limit = lanelet.attributes['along_speed_limit']
        behavior_space.alongBehavior.attributes.speed_max = speed_limit
        logger.debug(f'For behavior along (ID: {behavior_space.alongBehavior.id}) '
                     f'speed limit {speed_limit} extracted from lanelet')
        if 'along_speed_limit_link' in lanelet.attributes:
            speed_ind_id = int(lanelet.attributes['along_speed_limit_link'])
            logger.debug(f'Referencing regulatory element {speed_ind_id} as speed indicator for alongBehavior')
            behavior_space.alongBehavior.attributes.add_speed_indicator(speed_ind_id)

        speed_limit = lanelet.attributes['against_speed_limit']
        behavior_space.againstBehavior.attributes.speed_max = speed_limit
        logger.debug(
            f'For behavior against (ID: {behavior_space.againstBehavior.id})'
            f'speed limit {speed_limit} extracted from lanelet')
        if 'against_speed_limit_link' in lanelet.attributes:
            speed_ind_id = int(lanelet.attributes['against_speed_limit_link'])
            logger.debug(f'Referencing regulatory element {speed_ind_id} as speed indicator for againstBehavior')
            behavior_space.againstBehavior.attributes.add_speed_indicator(speed_ind_id)

        # 4. Derive external reservation at zebra crossings
        logger.debug(f'_______ Deriving Reservation _______')
        self.derive_conflicts(behavior_space)

    # -------------------------------------------------------------------
    # ------------ behavior derivation of lateral boundary --------------
    # -------------------------------------------------------------------
    def derive_behavior_boundary_lateral(self, behavior_a, behavior_b, side):
        """
        Derive the behavioral demands for one lateral boundary of a behavior space (currently CrossingType and
        parking_only). Since the two behavior elements of a behavior space (along and against reference direction) share
        the same linestring on opposite lateral boundaries, the derivation is performed once and the identified
        CrossingType is assigned to both lateral boundary objects. For this, both behavior objects are given as
        arguments and the derivation takes place for the left lateral boundary of the first given behavior object.

        Parameters:
            behavior_a (Behavior):Behavior of which the left lateral boundary is used for derivations.
            behavior_b (Behavior):Behavior of which the right lateral boundary is used for derivations.
            side (str):'left' or 'right', referring to behavior_a.
        """

        # First for the attributes of the linestring, derive the CrossingType. If no CrossingType could be
        # derived None will be returned.
        logger.debug(f'Deriving CrossingType for linestring {behavior_a.leftBound.lineString.id}.')
        crossing_type = derive_crossing_type_for_lat_boundary(behavior_a.leftBound.lineString.attributes, side)
        if crossing_type:  # assign value two both lateral boundary elements for this side of the behavior space
            behavior_a.leftBound.attributes.crossing = behavior_b.rightBound.attributes.crossing = crossing_type

        # Second, derive whether a parking area is lying next to the boundary. If that is the case, the property
        # parking_only will be set for both lateral boundary objects and change the CrossingType to 'conditional'
        areas = self.find_neighbor_areas(behavior_a.leftBound.lineString, subtype='parking')
        if areas:
            parking_only = True
            behavior_a.leftBound.attributes.crossing = behavior_b.rightBound.attributes.crossing = 'conditional'
            logger.debug(f'Found parking area with ID {list(areas)[0].id} next to lateral boundaries'
                         f' {behavior_a.leftBound.id} and {behavior_b.rightBound.id}. Setting parking_only=yes')
        else:
            # Assumption that the checked conditions are the only way to determine the 'parking_only' property
            # May needs to be adjusted in future updates
            parking_only = False

        # Set the parking_only property in the behavior attributes
        behavior_a.leftBound.attributes.parking_only = \
            behavior_b.rightBound.attributes.parking_only = parking_only

    # ------------------------------------------------------------------------
    # ------------ behavior derivation of longitudinal boundary --------------
    # ------------------------------------------------------------------------
    def derive_boundary_long_behavior(self, behavior, motor_lanelet):
        """
        Derive behavioral demands for the longitudinal boundary (currently no_stagnant_traffic at zebra crossings).
        To do so, the type of the reference linestring is checked. Afterwards, the lanelets this linestring
        is used in are checked for intersecting centerlines with the lanelet that this derivation has been started for.

        Parameters:
            behavior (Behavior):Behavior element the demand is supposed to be derived.
            motor_lanelet (Lanelet):Lanelet for motorized vehicles on which the behavior spaced is mapped.
        """

        # Create a traffic rules object for pedestrians
        traffic_rules_pedestrian = traffic_rules.create(traffic_rules.Locations.Germany,
                                                        traffic_rules.Participants.Pedestrian)

        # Check, if the longitudinal boundary of the given behavior object is referencing to a linestring
        # The is based on the assumption that a zebra crossing linestring was found for the derivation of the
        # longitudinal boundary
        if behavior.longBound.ref_line:

            # Use the ID of the ref line to get the corresponding linestring object
            linestring_long_boundary = self.map_lanelet.lineStringLayer[behavior.longBound.ref_line]

            # Check whether the ref line is of type zebra_marking
            if linestring_long_boundary.attributes['type'] in ['zebra_marking']:

                # If condition is met, search for all lanelets that conflict with the lanelet that this behavior is
                # derived for. Check these lanelets for conflicting centerline and passability for pedestrians.
                zebra_lanelet = next((pedestrian_lanelet for pedestrian_lanelet in self.graph.conflicting(motor_lanelet)
                                      if geo.intersectCenterlines2d(pedestrian_lanelet, motor_lanelet)
                                      and traffic_rules_pedestrian.canPass(pedestrian_lanelet)), None)

                # If a lanelet has been found that meets these conditions, the conclusion is made, that the current
                # lanelet overlaps with a zebra crossing lanelet. Thus, the property no_stagnant_traffic will be set.
                if zebra_lanelet:
                    behavior.longBound.attributes.no_stagnant_traffic = True
                    logger.debug(f'For linestring {linestring_long_boundary.id} attribute no_stagnant_traffic=yes '
                                 f'has been derived due to overlapping lanelet {zebra_lanelet.id}.')
                    return
        logger.debug(f'No behavioral demand for longitudinal boundary detected.')

    # ------------------------------------------------------------------------------------
    # ------------ derivation of speed limits for all lanelets of a segment --------------
    # ------------------------------------------------------------------------------------
    def derive_segment_speed_limit(self, start_lanelet):
        """
        Starting from one lanelet, this function derives the speed limit along and against reference direction for every
        lanelet of the segment. To achieve this, the first step is to find every lanelet of the same reference
        direction. Second, for the outer left lanelet neighbors for the opposing driving direction of the roadway are
        being searched. If no neighbor is found, structural separation is assumed. If a neighbor is found, a search for
        all lanelets of that driving direction will be started. Depending on the type of linestring inbetween the two
        driving directions, structural separation is assumed or not.

        For every lanelet of a segment the speed limit along reference direction is derived using the traffic rules
        module of Lanelet2. If regulatory elements for speed limits are referenced within the lanelet, their IDs will be
        stored as well. If structural separation is found, the speed limit along reference direction for a given is used
        as the speed limit against reference direction for the same lanelet. If no structural separation is identified,
        the speed limits against reference direction are used from lanelets of the opposing driving direction of the
        roadway.

        The information about speed limit values and potential regulatory elements are stored in the attributes of
        lanelets, because at the moment of the derivation there doesn't exist a behavior space element for each lanelet
        of the map. As soon as the recursive loop processes a lanelet with already determined speed limit information,
        those will be assigned to the behavior elements of the behavior space.

        Parameters:
            start_lanelet (Lanelet):Start segment identification and derivation of the speed limit.
        """

        logger.debug(f'-.-.-.-.-.-.-.-.-.-.-.-')

        # First, identify lanelets of the same driving direction within the same segment.
        # The search is based on direct neighbors and neighbors next to keepout areas.
        logger.debug(f'Searching for lanelets of the same segment and driving direction for lanelet {start_lanelet.id}')
        lanelets_of_same_direction = self.find_adjacent(start_lanelet, 0)
        logger.debug(f'Found lanelets '
                     f'{[[key, [ll.id for ll in value]] for key, value in lanelets_of_same_direction.items()]}')
        first_opposing_lanelets = None

        # Derive and assign information about the speed limit for the
        # identified lanelets and store them in their attributes
        self.assign_speed_limit_along(lanelets_of_same_direction)

        # Search for lanelets of the opposing driving direction for the outerleft lanelet of the identified lanelets
        # Using a for loop, because there may be multiple lanelets assigned to the same level
        logger.debug(f'Searching for lanelets of the same segment but in the opposing driving direction.')
        for lanelet in lanelets_of_same_direction[max(lanelets_of_same_direction.keys())]:
            # Search for a direct neighbor of the opposing direction through direct neighborhood or next to a keepout
            first_opposing_lanelets = self.find_one_sided_neighbors(lanelet, lanelet.leftBound.invert(), 'against')

            # If a lanelet is found, check whether structural separation is dividing the roadway
            if first_opposing_lanelets:
                
                # Identify every lanelet of the opposing direction of the segment
                lanelets_of_opposing_direction = self.find_adjacent(next(iter(first_opposing_lanelets)), 0)
                logger.debug(f'Found lanelets'
                             f'{[[key, [lanelet.id for lanelet in value]] for key, value in lanelets_of_opposing_direction.items()]} '
                             f'for opposing driving direction.')
                # derive speed limit for lanelets_of_opposing_direction along their reference direction
                self.assign_speed_limit_along(lanelets_of_opposing_direction)

                # distinguish passability between driving directions
                if not derive_crossing_type_for_lat_boundary(lanelet.leftBound.attributes, 'left') == 'not_possible':
                    # no structural separation
                    logger.debug(f'Driving directions for this segment are not structurally separated.'
                                 f' Cross assign speed limits to lanelets.')
                    outer_left_opposing_lanelet =\
                        list(lanelets_of_opposing_direction[max(lanelets_of_opposing_direction.keys())])[0]
                    self.assign_speed_limit_against(lanelets_of_opposing_direction, lanelet)
                    self.assign_speed_limit_against(lanelets_of_same_direction, outer_left_opposing_lanelet)
                else:
                    # driving directions are structurally separated
                    logger.debug(f'Driving directions for this segment are structurally separated.'
                                 f' Use along behavior speed limit for against behavior.')
                    self.assign_speed_limit_against(lanelets_of_opposing_direction)
                    self.assign_speed_limit_against(lanelets_of_same_direction)

                break

        # if no lanelet of the opposing direction is found, it is assumed that the driving directions
        # of the roadway are structurally separated
        if not first_opposing_lanelets:
            logger.debug(f'Driving directions for this segment are structurally separated.'
                         f' Use along behavior speed limit for against behavior.')
            self.assign_speed_limit_against(lanelets_of_same_direction)

        logger.debug(f'-.-.-.-.-.-.-.-.-.-.-.-')

    def find_adjacent(self, current_lanelet, level, previous_lanelet=None):
        """
        Find all lanelets of the same segment that have the same reference direction. Search direct neighbors as well
        as neighbors next to a keepout area. To find these, recursively call this function for the neighbors of a
        lanelet. Pass up information to earlier calls of the function and through this, build up a dictionary with
        all the lanelets and there respective level in the street.

        Parameters:
            current_lanelet (Point2d | Point3d):The lanelet that is being checked.
            level (Point2d | Point3d):The number for the current lanelet for the position on the roadway.
            previous_lanelet (Lanelet): The previously considered lanelet to avoid infinite recursive loops.

        Returns:
            lanelets_for_direction (dict):Contains lanelets assigned to their lateral level in the roadway.
        """

        # Find all lanelets that are lying right next to each other
        # condition is that they share there lateral boundary or that they are lying next to a keepout area
        lefts = self.find_one_sided_neighbors(current_lanelet, current_lanelet.leftBound, 'along')
        rights = self.find_one_sided_neighbors(current_lanelet, current_lanelet.rightBound, 'along')

        # Remove the previously considered lanelet to avoid recursive loops
        lefts.discard(previous_lanelet)
        rights.discard(previous_lanelet)

        # Create a defaultdict to store the lanelets, because this allows to insert items for keys that don't exist yet
        lanelets_for_direction = defaultdict(set)
        # Making use of that, add the current lanelet for the current level as key
        lanelets_for_direction[level].add(current_lanelet)

        # For both sides call this function recursivly again. Consequentially, the later function calls will report the
        # information back to the earlier function calls so that eventually the first function call includes every
        # lanelet that met the conditions for being part of the segment. The dictionaries are therefore concatenated
        for lanelet in lefts:
            sub_set = self.find_adjacent(lanelet, level + 1, current_lanelet)
            util.join_dictionaries(lanelets_for_direction, sub_set)

        for lanelet in rights:
            sub_set = self.find_adjacent(lanelet, level - 1, current_lanelet)
            util.join_dictionaries(lanelets_for_direction, sub_set)

        return lanelets_for_direction

    def assign_speed_limit_along(self, lanelets_of_same_direction):
        """
        For a given dictionary of lanelets of the same reference direction of a segment, this function stores the speed
        limit value in the attributes of the lanelet. In case a regulatory element is indicating this limit, the
        function will determine its ID and also store it in the lanelets attributes.

        Parameters:
            lanelets_of_same_direction (dict):Contains lanelets assigned to their lateral level in the roadway.
        """

        # Loop through every level of the dictionary
        for level in lanelets_of_same_direction.keys():
            # Loop through every lanelet of the level
            for lanelet in lanelets_of_same_direction[level]:
                # Use the traffic rules function of lanelet to
                # derive the speed limit and save it to the lanelets attributes
                speed_limit = str(round(self.traffic_rules.speedLimit(lanelet).speedLimit))
                lanelet.attributes['along_speed_limit'] = speed_limit
                logger.debug(f'Saving speed limit {speed_limit} for along behavior in lanelet {lanelet.id}')

                # Extract every regulatory element that is referenced in this lanelt of type SpeedLimit
                speed_limit_objects =\
                    [regelem for regelem in lanelet.regulatoryElements if isinstance(regelem, SpeedLimit)]
                # If a speed limit item was found, save the ID in the lanelets attributes
                if speed_limit_objects:
                    logger.debug(f'Found regulatory element {speed_limit_objects[0].id} that indicates the speed limit')
                    lanelet.attributes['along_speed_limit_link'] = str(speed_limit_objects[0].id)

    @staticmethod
    def assign_speed_limit_against(lanelets_of_same_direction, opposing_lanelet=None):
        """
        Similarly to the function assign_speed_limit_along, this function instead assigns the speed limits against
        reference direction for the lanelets of a given segment (=only from one driving direction of the roadway). Two
        options for the assignment exist:
        1. No other lanelet is given in the function arguments. This means, based on the imposed conditions,
        the driving directions of the roadway are assumed as structurally separated. Therefore, within this function,
        every lanelet gets the speed limit from along their reference direction saved as their speed limit against
        reference direction.
        2. Another lanelet is given in the function arguments. Checks in other function already indicated that there
        is no structural separation. Thus, the speed limit information of this given lanelet are used as the speed
        limit information of every lanelet of the given driving direction of the segment.

        Parameters:
            lanelets_of_same_direction (dict):Contains lanelets assigned to their lateral level in the roadway.
            opposing_lanelet (Lanelet):Optionally given lanelet that is from the opposing driving direction to the
                                       lanelets in 'lanelets_of_same_direction'.
        """

        # Loop through every level of the dictionary
        for level in lanelets_of_same_direction.keys():
        # Loop through every lanelet of the level
            for ll in lanelets_of_same_direction[level]:

                # Depending on whether an opposing lanelet is given, either assign the speed limit information of this
                # opposing lanelet as the speed limit information against reference direction or use the lanelets own
                # information for along reference direction
                if opposing_lanelet:
                    ll.attributes['against_speed_limit'] = opposing_lanelet.attributes['along_speed_limit']
                    if 'along_speed_limit_link' in ll.attributes:
                        ll.attributes['against_speed_limit_link'] = opposing_lanelet.attributes['along_speed_limit_link']
                else:
                    ll.attributes['against_speed_limit'] = ll.attributes['along_speed_limit']
                    if 'along_speed_limit_link' in ll.attributes:
                        ll.attributes['against_speed_limit_link'] = ll.attributes['along_speed_limit_link']

    def find_one_sided_neighbors(self, start_lanelet, linestring_start_lanelet, orientation):
        """
        Search for neighbors on one side of a lanelet. Neighbors are considered as such if they either share the same
        linestring with a given lanelet or ly both next to a keepout area and belong - based on further conditions -
        to the same segment. Hence, lanelets for both options are being searched.

        Parameters:
            start_lanelet (Lanelet):The lanelet for which neighbors are being searched.
            linestring_start_lanelet (LineString2d | LineString3d):linestring of one side of start_lanelet.
            orientation (str):'along' or 'against'

        Returns:
            neighbor_lanelets (set):Set of all lanelets that are considered as neighbors.
        """

        # 1. Use the lanelet layer to find every usage of the linestring in lanelets that are relevant
        lanelet_layer = self.map_lanelet.laneletLayer
        neighbor_lanelets = {lanelet for lanelet in lanelet_layer.findUsages(linestring_start_lanelet)
                             if is_lanelet_relevant(lanelet.attributes)}
        # Discard the lanelet the search has been started from, since it is not its own neighbor
        neighbor_lanelets.discard(start_lanelet)
        # Remove lanelets from the set that are having an overlap with the starting lanelet
        neighbor_lanelets = {lanelet for lanelet in neighbor_lanelets if not geo.overlaps2d(lanelet, start_lanelet)}

        # 2. Additionally, search for lanelets that are lying next to keepout areas
        surrounding_lls = self.neighbor_next_to_area(linestring_start_lanelet)
        # Filter the surrounding lanelets of the keepout area for membership of the segment and add the filtered set
        # to the neighbor_lanelets-set
        surrounding_lls =\
            self.filter_for_segment_membership(surrounding_lls, start_lanelet, linestring_start_lanelet, orientation)
        neighbor_lanelets.update(surrounding_lls)

        return neighbor_lanelets

    def neighbor_next_to_area(self, linestring):
        """
        Based on one linestring (which may be the lateral boundary of a lanelet), search for usages in areas and if
        an area is found, for lanelets that surround this area. Since lanelets that surround that area are using an
        linestring that is also used to form that area, both the linestring and the lanelet are being saved in
        a dictionary. The linestring will be used in another method to determine whether the lanelet belongs to the same
        segment as the lanelet this search for neighbors has been started from. For that it is also important to save
        the linestring in its inverted form if that is the way it is used in the lanelet.

        Parameters:
            linestring (LineString2d | LineString3d):Linestring for which areas and after that lanelets surrounding that
                                             area are searched.

        Returns:
            surrounding_lanelets (dict):True if conditions are met, otherwise False.
        """

        lanelet_layer = self.map_lanelet.laneletLayer

        # Search for keepout areas that use the given linestring
        neighbor_areas = self.find_neighbor_areas(linestring, 'keepout')
        # Write a warning message if more than one area is found
        if len(neighbor_areas) > 1:
            logger.warning(f'For linestring {linestring.id}: Multiple adjacent areas have been found.'
                           f' No distinct derivation of driving directions possible')

        # If areas are found, use the first area (if more are found an warning will be send to the log anyway) to find
        # surrounding lanelets of this area
        surrounding_lanelets: Dict[Any, Any] = dict()
        if neighbor_areas:
            area = neighbor_areas.pop()
            # Get set of all linestrings in area
            linestrings_of_area_boundary = {area_boundary for area_boundary in area.outerBound}
            # Remove linestring of current lanelet from this list
            linestrings_of_area_boundary.discard(linestring)
            linestrings_of_area_boundary.discard(linestring.invert())
            # Search for all usages of those linestrings in other lanelets and collect them in a dictionary
            # Use the lanelet as a key (every lanelet should only appear once as a neighbor of an area) and store
            # the linestring as the value. Save the linestring in the way that it is used (normal or inverted)
            for area_boundary in linestrings_of_area_boundary:
                for lanelet in lanelet_layer.findUsages(area_boundary):
                    # Filter list of lanelets for ones that are relevant
                    if is_lanelet_relevant(lanelet.attributes):
                        surrounding_lanelets[lanelet] = area_boundary
                for lanelet in lanelet_layer.findUsages(area_boundary.invert()):
                    # Filter list of lanelets for ones that are relevant
                    if is_lanelet_relevant(lanelet.attributes):
                        surrounding_lanelets[lanelet] = area_boundary.invert()

            # If more than one lanelet has been found, write a warning to log
            if len(surrounding_lanelets) > 1:
                logger.warning(f'For area {area.id}: Multiple adjacent lanelets have been found.'
                               f' No distinct derivation of driving directions possible')

        return surrounding_lanelets

    def filter_for_segment_membership(self, surrounding_lanelets, ref_lanelet, ref_linestring, orientation):
        """
        This function is used to check, if lanelets that surround an area belong to the same segment as another
        lanelet that is a neighbor of this area. Therefore, geometrical criteria are used. It is possible to check for
        lanelets that either have the same or opposing reference directions.

        Parameters:
            surrounding_lanelets (dict):Lanelets that surround the keepout area including their linestrings.
            ref_lanelet (Lanelet):The lanelet for whose segment other lanelets are searched.
            ref_linestring (LineString2d | LineString3d):part of the boundary of the keepout area.
            orientation (str):'along' or 'against'

        Returns:
            belonging_to_segment (list):Contains every lanelet that met the condition of belonging to the segment.
        """

        belonging_to_segment = []
        for lanelet, linestring in surrounding_lanelets.items():
            angle = util.angle_between_lanelets(lanelet, ref_lanelet)
            if (orientation == 'along' and angle < 45) \
                    and (linestring[0] == ref_linestring[0] or linestring[-1] == ref_linestring[-1]
                         or self.are_linestrings_orthogonal(linestring, ref_linestring,
                                                            [linestring[0], ref_linestring[0]])):
                belonging_to_segment.append(lanelet)
            elif orientation == 'against' and 135 < angle < 225 \
                    and (linestring[0] == ref_linestring[-1] or linestring[-1] == ref_linestring[0]
                         or self.are_linestrings_orthogonal(linestring, ref_linestring,
                                                            [linestring[-1], ref_linestring[0]])):
                belonging_to_segment.append(lanelet)
            elif 45 < angle < 135:
                logger.warning(f'Lanelet {lanelet.id} and {ref_lanelet.id} border the same area and '
                               f'cannot be assigned to the same segment. Reason: Angle too large')

        if len(surrounding_lanelets) > 1:
            logger.warning(f'Multiple Lanelets bordering the same area.'
                           f' Lanelet IDs: {[lanelet.id for lanelet in surrounding_lanelets.keys()]}.'
                           f' Lanelet(s) {belonging_to_segment} has been selected due to given criteria.')

        return belonging_to_segment

    def are_linestrings_orthogonal(self, linestring_1, linestring_2, points_to_link_linestrings):
        """
        Criterium that is used for determining whether two lanelets that are surrounding an keepout area, are belonging
        to the same segment. Therefore, the two linestring that represent the border between the respective lanelets and
        the area are given and the angle between each of them and a connecting line between them is calculated. If the
        linestring are both having an angle to this line of 80° to 100°, the criterion is satisfied.

        Parameters:
            linestring_1 (Linestring2d | Linestring3d):First linestring.
            linestring_2 (Linestring2d | Linestring3d):Second linestring.
            points_to_link_linestrings (list): List containing two points that connect the two linestrings.

        Returns:
            linestrings_orthogonal (bool):True if conditions are met, otherwise False.
        """

        # Retrieve mutable point objects from the point layer
        points_to_link_linestrings = [self.map_lanelet.pointLayer[pt.id] for pt in points_to_link_linestrings]
        # Create a Linestring object with the two points
        connect_lateral_boundaries = LineString3d(getId(), points_to_link_linestrings)

        # Calculate the angles between each linestring and the connecting linestring
        angle_1 = util.angle_between_linestrings(linestring_1, connect_lateral_boundaries)
        angle_2 = util.angle_between_linestrings(linestring_2, connect_lateral_boundaries)

        # return a bool through checking whether the two angles are within the specified range of 80° to 100°
        return 80 < angle_1 < 100 and 80 < angle_2 < 100

    # ---------------------------------------------------------------------------------
    # ------------ behavior derivation of reservation at zebra crossings --------------
    # ---------------------------------------------------------------------------------
    def derive_conflicts(self, behavior_space):
        """
        For a given behavior space, use the referenced lanelet to derive conflicting lanelets in the Lanelet2 map.
        Derive behavioral demands based on these identified conflicts. Currently, only zebra crossings are identified
        and external reservation is therefore determined. Furthermore, reservation links are set for lanelets and areas
        where pedestrians may come from.

        Parameters:
            behavior_space (BehaviorSpace):Behavior space object for which derivation is performed.
        """

        # find all conflicting lanelets in RoutingGraph for lanelet of this behavior space
        for lanelet in self.graph.conflicting(behavior_space.ref_lanelet):
            # filter this list for lanelets whose centerline are intersecting with the behavior spaces lanelet
            if is_zebra_and_intersecting(lanelet, behavior_space.ref_lanelet):

                # If an intersecting zebra crossing is found, set the external reservation for both behaviors of this
                # behavior space and set the reservation to pedestrian
                logger.debug(f'Conflicting zebra crossing with lanelet ID {lanelet.id} has been found. Setting'
                             f' reservation for behavior space {behavior_space} for both behaviors to externally')
                behavior_space.alongBehavior.reservation[0].attributes.reservation = tp.ReservationType.EXTERNALLY
                behavior_space.againstBehavior.reservation[0].attributes.reservation = tp.ReservationType.EXTERNALLY
                behavior_space.alongBehavior.reservation[0].attributes.pedestrian = True
                behavior_space.againstBehavior.reservation[0].attributes.pedestrian = True

                # Identify lanelets and areas that need to be referenced as reservation links in the
                # given behavior space. To do so, identify every conflict with the zebra crossing lanelet.
                logger.debug(f'Searching for lanelets and areas that need to be referenced via reservation links.')
                for link_lanelet in self.graph.conflicting(lanelet):
                    # Check lanelet for being relevant and for intersecting centerlines
                    if is_lanelet_relevant(link_lanelet.attributes)\
                            and geo.intersectCenterlines2d(link_lanelet, lanelet):

                        # Avoid setting a reservation link to the lanelet that the behavior space is referencing
                        # For every other lanelet that met the previous conditions, set an reservation link in both
                        # behaviors.
                        if not link_lanelet == behavior_space.ref_lanelet:
                            logger.debug(f'Found lanelet {link_lanelet.id}, '
                                         f'which conflicts with crosswalk lanelet {lanelet.id}')
                            behavior_space.alongBehavior.reservation[0].attributes.add_link(link_lanelet.id)
                            behavior_space.againstBehavior.reservation[0].attributes.add_link(link_lanelet.id)

                        # For every lanelet that conflicts with the zebra crossing lanelet, search for neighbor areas
                        # of type 'walkway'. If one is found, also set a reservation link for these.
                        nbr_areas = self.find_neighbor_areas(link_lanelet.leftBound, 'walkway') | \
                                    self.find_neighbor_areas(link_lanelet.rightBound, 'walkway')
                        for area in nbr_areas:
                            logger.debug(f'Found walkway area {area.id}, '
                                         f'which lies next to crosswalk lanelet {lanelet.id}')
                            behavior_space.alongBehavior.reservation[0].attributes.add_link(area.id)
                            behavior_space.againstBehavior.reservation[0].attributes.add_link(area.id)

                # As a third option for reservation links, check if a lanelet is used to model the walkway space next to
                # the roadway. In this case, the zebra crossing lanelet has successors and/or predecessors
                for link_lanelet in (self.graph.previous(lanelet) + self.graph.following(lanelet)):
                    logger.debug(
                        f'Found walkway lanelet {link_lanelet.id}, '
                        f'which lies before/after to crosswalk lanelet {lanelet.id}')
                    behavior_space.alongBehavior.reservation[0].attributes.add_link(link_lanelet.id)
                    behavior_space.againstBehavior.reservation[0].attributes.add_link(link_lanelet.id)

                # If a zebra lanelet was found and the above listed steps have been performed, break out of the loop
                break

        # If nothing was found, write a message to log with this information
        logger.debug(f'No zebra crossing found.')

    def find_neighbor_areas(self, linestring, subtype=None):
        """
        Searches in the areaLayer of the lanelet map for usages of the given linestring. A subtype can be specified to
        only find areas of this subtype. E.g. 'parking' or 'keepout'.

        Parameters:
            linestring (Linestring2d | Linestring3d):Linestring that is used to search for neighboring areas.
            subtype (str):Optional specification of a subtype that the function searches for.

        Returns:
            neighbor_areas (set):Set of area elements that were found.
        """

        # Search for areas in which the linestring is used as part of the boundary. To make sure every area will be
        # found, include a search for the inverted linestring.
        neighbor_areas = set.union(set(self.map_lanelet.areaLayer.findUsages(linestring)),
                                   set(self.map_lanelet.areaLayer.findUsages(linestring.invert())))

        # If a subtype-string was given, filter the set and only keep the areas of the specified subtype
        if subtype:
            neighbor_areas = {area for area in neighbor_areas if area.attributes['subtype'] == subtype}

        return neighbor_areas
