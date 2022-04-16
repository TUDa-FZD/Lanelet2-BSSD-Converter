import logging
from collections import defaultdict

import lanelet2
from lanelet2.geometry import distance as dist
from lanelet2.core import LineString3d, getId, SpeedLimit
import lanelet2.geometry as geo
from bssd.core import _types as tp

from .preprocessing import is_ll_relevant
from . import BSSD_elements
from .geometry_derivation import make_orthogonal_bounding_box, find_flush_bdr, find_line_insufficient
from .behavior_derivation import derive_crossing_type_for_lat_boundary, is_zebra_and_intersecting
from . import util
from .constants import LONG_BDR_TAGS, LONG_BDR_DICT

logger = logging.getLogger('framework.data_handler')


class DataHandler:
    """
    A class that contains the map data of Lanelet2 objects as well as BSSD objects.
    Based on that it contains multiple methods that use that data to
    loop through the map, derive behavior and more.

    Attributes
    ----------
        map_lanelet : LaneletMap
            Layered lanelet2 map that contains all lanelet2 objects of a loaded map.
        map_bssd : BssdMap
            Layered bssd map that contains all bssd objects
        relevant_lanelets : list
            age of the person
        graph : RoutingGraph
            Graph for the lanelet map that is adjusted to contain all the lanelets instead of
            being focused on one traffic participant
        traffic_rules : traffic_rules
            traffic rules object from lanelet2 for participant = vehicle

    Methods
    -------
        __init__(map_lanelet):
            Initiates class instance by getting lanelet map object. Creates empty bssd map object.
            Creates RoutingGraph and also calls function to find relevant lanelets.
        recursive_loop(ll_id, direction=None, ls=None):
            Recursively loops through lanelets in a map. Needs to be called from a seperate place
            to make sure every lanelet is being considered.
        get_long_bdr(pt_left, pt_right, use_previous, previous):
            For end-/startpoints of lateral boundaries of lanelet this function searches for potential
            linestrings that can be used to determine the linestring of the correspondent behavior space.
            Returns a found or newly created linetring.
        find_inside_lines(bs, lanelet):
            Searches for linestrings that are not sharing any points with the points of a laneletes lateral boundaries.
        derive_behavior(bs, lanelet):
            Main function for behavior derivation. Calls multiple subfunctions (see following) to derive
            multiple behavioral attributes.
        derive_behavior_bdr_lat(behavior_a, behavior_b, side):
            Derives CrossingType through comparing linestring types with a predefined dictionary.
        derive_behavior_bdr_long(behavior, ll):
            Searches for conflicting zebra lanelets to derive the no_stagnant_traffic attribute.
        derive_segment_speed_limit(lanelet):
            Starting from one lanelet this function auto detects all lanelets of the same segment and
            cross assigns speed limits for behaviors against reference direction.
        find_adjacent(current_ll, level, prev_ll=None):
            Recursively searches for every lanelet of the same segment but for the same reference
            driving direction. Returns dictionary with lanelets assigned to a respective level on the roadway.
        assign_sl_along(segment):
            Temporarily assigns lanelet speed limit to lanelet attributes for every lanelet in a segment.
        assign_sl_against(segment, other_ll=None):
            Temporarily assigns lanelet speed limit for behavior against reference direction distinguishing between
            structurally divided driving directions.
        find_one_sided_neighbors(lanelet, ls, orientation):
            Searches for neighbors of a lanelet either through direct neighborhood or next to a keepout area.
        neighbor_next_to_area(ls):
            For a given linestring searches for keepout areas and returns a list with every lanelet
            surrounding this area besides of the lanelet from which the search has been started.
        filter_for_direction(sourrounding_lanelets, ref_lanelet, ref_ls, orientation):
            Used to filter the list of lanelets that surround an area. The goal is to find lanelets that belong to
            the same segment of a given lanelet.
        are_linestrings_orthogonal(ls1, ls2, pts):
            Compares two linestrings and a newly created linestring inbetween them in all ways for orthogonality.
        derive_conflicts(bs):
            Derives conflicts of a given lanelet. If zebra lanelets are found the behavior space gets the attribute
            of external reservation. Furthermore, reservation links are being set.
        find_neighbor_areas(ls, subtype=None):
            Finds direct neighbors of an area to set the reservation links at a zebra crossing.
    """

    def __init__(self, map_lanelet, relevant_lanelets, routing_graph):
        self.map_lanelet = map_lanelet
        self.map_bssd = BSSD_elements.BssdMap()
        self.relevant_lanelets = relevant_lanelets
        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                           lanelet2.traffic_rules.Participants.Vehicle)
        self.graph = routing_graph

    # -----------------------------------------------
    # -------------------- loop ---------------------
    # -----------------------------------------------
    def recursive_loop(self, ll_id, direction=None, ls=None):
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
            ll_id (int):The id of the lanelet that is being processed.
            direction (str):The direction from which the previous lanelet called the function for this lanelet.
            ls (LineString3d | LineString3d):Linestring of longitudinal boundary of previous lanelet (if exists).
        """

        # Retrieve lanelet object from lanelet map via ID
        ll = self.map_lanelet.laneletLayer[ll_id]
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        self.relevant_lanelets.remove(ll_id)

        logger.debug(f'----------------------------------------------------------------------------------------------')
        logger.debug(f'Derivation for Lanelet {ll_id}')

        # Determine longitudinal boundaries of both sides of the lanelet
        # Based on the assumption that lanelets and behavior space are covering the same part of the roadway
        logger.debug(f'Derivation of longitudinal boundary for along behavior')
        alg_ls, alg_ref = self.get_long_bdr(ll.leftBound[0], ll.rightBound[0], 'along' == direction, ls)
        logger.debug(f'Derivation of longitudinal boundary for against behavior')
        agst_ls, agst_ref = self.get_long_bdr(ll.leftBound[-1], ll.rightBound[-1], 'against' == direction, ls)

        # create behavior space object and all bssd objects that are necessary for that
        # Arguments are the lanelet and the longitudinal boundaries so that they can be assigned immediately
        new_bs = self.map_bssd.create_placeholder(ll, alg_ls, agst_ls)

        # If a reference linestring has been found from which the longitudinal have been derived,
        # save their IDs in the longitudinal boundary objects.
        if alg_ref:
            new_bs.alongBehavior.longBound.ref_line = alg_ref
        if agst_ref:
            new_bs.againstBehavior.longBound.ref_line = agst_ref
        logger.debug(f'Created Behavior Space {new_bs.id}')

        # Call function for behavior derivation for the behavior space that was created for the current lanelet
        self.derive_behavior(new_bs, ll)

        # Call function in itself for the succeeding and preceding lanelet(s) and hand over information
        # about already derived boundaries. Check if the following lanelet is relevant to avoid deriving
        # behavior spaces for irrelevant lanelets.
        for successor in self.graph.following(ll):
            if successor.id in self.relevant_lanelets:
                self.recursive_loop(successor.id, 'along', agst_ls)
        for predecessor in self.graph.previous(ll):
            if predecessor.id in self.relevant_lanelets:
                self.recursive_loop(predecessor.id, 'against', alg_ls)

    # -----------------------------------------------
    # ----------- longitudinal boundary -------------
    # -----------------------------------------------
    def get_long_bdr(self, pt_left, pt_right, use_previous, previous):
        """
        Determine the geometrical representation of the longitudinal boundary for one side of a lanelet which
        corresponds with longitudinal boundary of one of the behaviors. For this, multiple cases are considered and
        checked.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.
            use_previous (bool):True, if the linestring of the previous linestring matches with this site of the new ll.
            previous (lanelet):Linestring of the previous lanelet/bs. Only used in combination with use_previous

        Returns:
            ls (LineString3d | LineString3d):Linestring3d object of the determined geometry of the longitudinal bdr.
            ref_line (bool):ID of the orig. linestring the new linestring has been derived from (None if not existent).
        """

        # Initalize empty variables for the linestring and reference line
        ls = None
        ref_line = None

        # Check different possible cases
        # First case is that the linestring information are given from the previous lanelet. In that case, the same
        # linestring will be used to represent the longitudinal boundary of the current behavior space (at one side)
        if use_previous:
            # Use previously created linestring
            ls = previous
            ref_line = ls.id
            logger.debug(f'Using linestring from successor/predecessor (ID: {ls.id})')

        # If the start-/endpoints of this side of a lanelet are identical, no longitudinal boundary exists
        elif pt_left.id == pt_right.id:
            # No longitudinal boundary exists
            logger.debug(f'Longitudinal boundary doesn\'t exist')
            pass

        # Otherwise, check for existing lineStrings (e.g. stop_line). In each case, if a linestring matches
        # the conditions, the points that are necessary for creating a new linestring will be extracted
        else:
            # Setup a dictionary to store linestrings for each possible case.
            lines = LONG_BDR_DICT

            # Find every usage of the left and right point
            ls_list_pt_left = set(self.map_lanelet.lineStringLayer.findUsages(pt_left))
            ls_list_pt_right = set(self.map_lanelet.lineStringLayer.findUsages(pt_right))

            # Determine the linestrings that contain the left and the right point
            mutual_ls = set.intersection(ls_list_pt_left, ls_list_pt_right)

            # Remove the mutual linestrings from the lists of each point to make sure that in the functions no
            # wrong derivations will be made.
            ls_list_pt_left = ls_list_pt_left - mutual_ls
            ls_list_pt_right = ls_list_pt_right - mutual_ls

            # FIRST CASE: linestring contains both points
            # This gives two options: The linestring is fitting exactly OR is overarching.
            lines.update(find_flush_bdr(pt_left, pt_right, mutual_ls))

            # SECOND CASE: linestrings that contain only one point
            # insufficient
            lines['insufficient_half_left'] = find_line_insufficient(ls_list_pt_left, pt_left, pt_right)
            lines['insufficient_half_right'] = find_line_insufficient(ls_list_pt_right, pt_right, pt_left)

            # THIRD CASE: linestrings that do not contain one of the points
            # linestrings will be searched using a BoundingBox
            lines['free'] = self.find_free_lines(pt_left, pt_right)

            # In case multiple linestrings have been found, write a warning
            if len([v for k, v in lines.items() if v[0]]) > 1:
                logger.warning(f'Multiple possible long. boundaries found for points {pt_left} and {pt_right}')

            # Check if a linestring has been found
            # First condition is that an exact linestring has been found. For this case, this linestring will
            # be used directly as the longitudinal boundary
            if lines['exact'][0]:
                ls = self.map_lanelet.lineStringLayer[lines['exact'][0]]
                ref_line = ls.id
                logger.debug(f'Using existing line with ID {ls.id} as long. boundary')

            # In every other case, the creation of a new line is necessary
            else:
                # If a linestring has been found get the points and ref line from the lines-dictionary
                if any(v[0] for k, v in lines.items()):
                    matching_case = [k for k, v in lines.items() if v[0]]
                    pt_pairs = lines[matching_case[0]][1]
                    ref_line = lines[matching_case[0]][0]
                    logger.debug(f'Using existing line with ID {lines[matching_case[0]][0]}'
                                 f'partially for long. boundary')
                # If no linestring has been found, save the start-/endpoints of of the lateral boundaries of the lanelet
                else:
                    pt_pairs = [pt_left, pt_right]
                    logger.debug(f'No existing line has been found, using endpoints for new linestring.')

                # For the identified points, create a new linestring and add it to the lanelet map
                # First, get the mutable point object from the lanelet map, because also ConstPoints
                # are used in linestrings
                pt_pairs = [self.map_lanelet.pointLayer[pt.id] for pt in pt_pairs]
                ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'boundary'})
                logger.debug(f'Created new linestring as longitudinal boundary with ID {ls.id}')
                self.map_lanelet.add(ls)

        return ls, ref_line

    def find_free_lines(self, pt_left, pt_right):
        """
        Searches for linestrings that are relevant for the longitudinal boundary of a behavior space but don't contain
        one of the two start-/endpoints of a lanelet (which is referenced by the behavior space)

        Parameters:
            pt_left (Point2d | Point3d):The start-/endpoint of the linestring of the left lateral boundary
            pt_right (Point2d | Point3d):The start-/endpoint of the linestring of the right lateral boundary

        Returns:
            result (list):list with two items: 1 ref ls id and 2 points for the creation of a new linestring
        """

        # Create a bounding box that is created so that it finds linestrings that don't exceed the lanelet borders
        search_box = make_orthogonal_bounding_box(pt_left, pt_right)

        # Use bounding box to search for linestrings in the area of a potential long boundary
        # do not consider any linestring that contain either the left or the right start-/endpoint of the lat boundaries
        # With this method every linestring that at least overlaps a bit with the bounding box will be found. Because
        # of that, another condition is checked later within the for-loop-
        near_ls = [ls for ls in self.map_lanelet.lineStringLayer.search(search_box)
                   if pt_left not in ls or pt_right not in ls]

        # For-loop through the lines that were found to check further conditions
        for line in near_ls:
            # Distinguish line inside and outside of lanelet
            # This is achieved by checking whether the two endpoints of the linestring ly within the bounding box
            # First, the type of the linestring is checked, if it generally could be considered for a long boundary
            if 'type' in line.attributes and line.attributes['type'] in LONG_BDR_TAGS and \
                    all(x in self.map_lanelet.pointLayer.search(search_box) for x in [line[0], line[-1]]):

                # If conditions are met, the linestring will be used to derive the actual longitudinal boundary
                # Store the points of the linestring in a list
                ls_pts = [el for el in line]

                # Check the orientation of the linestring to append pt_left and pt_right at the right place
                if dist(line[0], pt_left) < dist(line[0], pt_right):
                    ls_pts.insert(0, pt_left)
                    ls_pts.append(pt_right)
                else:
                    ls_pts.insert(0, pt_right)
                    ls_pts.append(pt_left)
                logger.debug(f'Found inside line with ID {line.id}')
                return [line.id, ls_pts]

        # If nothing was found, return a list with two empty items
        return [None, None]

    # -----------------------------------------------
    # ------------ behavior derivation --------------
    # -----------------------------------------------
    def derive_behavior(self, bs, lanelet):
        """
        This is the main function for actual derivations of behavioral demands. It integrates calls for other functions that
        are deriving specific behavior attributes and properties. Therefore, it is possible to extend the behavior derivations
        by adding more subfunctions in the future. The derivation is started after creating a behavior space placeholder
        element for a lanelet in "recursive_loop". 

        Parameters:
            bs (BehaviorSpace):The behavior space instance that is supposed to be filled within this function.
            lanelet (Lanelet):The lanelet on which the behavior spaced is mapped.
        """

        # 1. Derive behavioral demand of the lateral boundaries
        logger.debug(f'_______ Deriving behavioral demand of lateral boundaries _______')
        logger.debug(f'Deriving behavioral demand of lateral boundary of alongBehavior (left,'
                     f' ID:{bs.alongBehavior.leftBound.id}) and '
                     f'againstBehavior (right, ID:{bs.againstBehavior.rightBound.id})')
        self.derive_behavior_bdr_lat(bs.alongBehavior, bs.againstBehavior, 'left')
        logger.debug(f'Deriving behavioral demand of lateral boundary of againstBehavior (left,'
                     f' ID:{bs.againstBehavior.leftBound.id}) and '
                     f'alongBehavior (right, ID:{bs.alongBehavior.rightBound.id})')
        self.derive_behavior_bdr_lat(bs.againstBehavior, bs.alongBehavior, 'right')

        # 2. Derive behavioral demand of the longitudinal boundaries
        logger.debug(f'_______ Deriving behavioral demand of longitudinal boundaries _______')
        if bs.alongBehavior.longBound:
            logger.debug(f'Deriving behavioral demand of longitudinal boundary of'
                         f' alongBehavior (ID:{bs.alongBehavior.longBound.id})')
            self.derive_behavior_bdr_long(bs.alongBehavior, lanelet)
        if bs.againstBehavior.longBound:
            logger.debug(f'Deriving behavioral demand of longitudinal boundary of'
                         f' againstBehavior (ID:{bs.againstBehavior.longBound.id})')
            self.derive_behavior_bdr_long(bs.againstBehavior, lanelet)

        # 3. Derive speed limits for along and against reference direction of a behavior space
        # To do so, check first whether the speed limits have already been derived for this segment. If not, the segmentwise
        # derivation will be started.
        logger.debug(f'_______ Deriving speed limits _______')
        if 'own_speed_limit' not in lanelet.attributes and 'other_speed_limit' not in lanelet.attributes:
            logger.debug(f'Derive speed limit for the segment the current lanelet belongs to.')
            self.derive_segment_speed_limit(lanelet)
        else:
            logger.debug(f'Segmentwise speed limit derivation has already been done for this lanelet.')

        # If speed limit already has been derived for the current lanelet, get the values that are temporarily 
        # stored in the lanelet attributes to save them in the respective behaviors. If existing, add a reference
        # to the speed indicator
        speed_limit = lanelet.attributes['own_speed_limit']
        bs.alongBehavior.attributes.speed_max = speed_limit
        logger.debug(f'For behavior along (ID: {bs.alongBehavior.id}) speed limit {speed_limit} extracted from lanelet')
        if 'own_speed_limit_link' in lanelet.attributes:
            speed_ind_id = int(lanelet.attributes['own_speed_limit_link'])
            logger.debug(f'Referencing regulatory element {speed_ind_id} as speed indicator for alongBehavior')
            bs.alongBehavior.attributes.add_speed_indicator(speed_ind_id)

        speed_limit = lanelet.attributes['other_speed_limit']
        bs.againstBehavior.attributes.speed_max = speed_limit
        logger.debug(
            f'For behavior against (ID: {bs.againstBehavior.id}) speed limit {speed_limit} extracted from lanelet')
        if 'other_speed_limit_link' in lanelet.attributes:
            speed_ind_id = int(lanelet.attributes['other_speed_limit_link'])
            logger.debug(f'Referencing regulatory element {speed_ind_id} as speed indicator for againstBehavior')
            bs.againstBehavior.attributes.add_speed_indicator(speed_ind_id)

        # 4. Derive external reservation at zebra crossings
        logger.debug(f'_______ Deriving Reservation _______')
        self.derive_conflicts(bs)

    # -------------------------------------------------------------------
    # ------------ behavior derivation of lateral boundary --------------
    # -------------------------------------------------------------------
    def derive_behavior_bdr_lat(self, behavior_a, behavior_b, side):
        """
        Derive the behavioral demands for the lateral boundary of a behavior space (currently CorssingType and parking_only). 
        Since the underlying linestring for both behavior elements (along and against reference direction) is the same, the 
        derivation is performed once and the identified CrossingType is assigned to both lateral boundary objects. Therefore, 
        both behavior objects are given as arguments and the "side"-argument is given to specify for which side of the first
        given behavior object the CrossingType needs to be determined. The second behavior object is reversed compared to the
        first one and therefore using the counterside. [ÜBERARBEITEN]

        Parameters:
            behavior_a (Behavior):The lanelet that is being checked.
            behavior_b (Behavior):The lanelet on which the behavior spaced is based.
            side (str):'left' or 'right', referring to behavior_a.
        """

        # First for the attributes of the linestring, derive the CrossingType. If no CrossingType could be
        # derived None will be returned.
        logger.debug(f'Deriving CrossingType for linestring {behavior_a.leftBound.lineString.id}.')
        ct = derive_crossing_type_for_lat_boundary(behavior_a.leftBound.lineString.attributes, side)
        if ct:  # assign value two both lateral boundary elements for this side of the behavior space
            behavior_a.leftBound.attributes.crossing = behavior_b.rightBound.attributes.crossing = ct

        # Second, derive whether a parking area is lying next to the boundary. If that is the case, the property
        # parking_only will be set for both lateral boundary objects and change the CrossingType to 'conditional'
        area_list = self.map_lanelet.areaLayer.findUsages(behavior_a.leftBound.lineString) + \
                    self.map_lanelet.areaLayer.findUsages(behavior_a.leftBound.lineString.invert())
        parking_only = False
        if any(item.attributes['subtype'] == 'parking' for item in area_list):
            logger.debug(f'Found parking area with ID {area_list[0].id} next to lateral boundaries'
                         f' {behavior_a.leftBound.id} and {behavior_b.rightBound.id}. Setting parking_only=yes')
            parking_only = True
            behavior_a.leftBound.attributes.crossing = behavior_b.rightBound.attributes.crossing = 'conditional'

        behavior_a.leftBound.attributes.parking_only = \
            behavior_b.rightBound.attributes.parking_only = parking_only

    # ------------------------------------------------------------------------
    # ------------ behavior derivation of longitudinal boundary --------------
    # ------------------------------------------------------------------------
    def derive_behavior_bdr_long(self, behavior, ll):
        """
        Derive behavioral demands for the longitudinal boundary (currently no_stagnant_traffic at zebra crossings).

        Parameters:
            behavior (Behavior):Behavior element the demand is supposed to be derived.
            ll (Lanelet):Lanelet on which the behavior spaced is mapped.
        """

        # Create a traffic rules object for pedestrians
        types = ['zebra_marking']
        tr_pedestrian = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Pedestrian)

        # Check, if the longitudinal boundary of the given behavior object is referencing to a linestring
        # The is based on the assumption that a zebra crossing linestring was found for the derivation of the
        # longitudinal boundary
        if behavior.longBound.ref_line:
            ls = self.map_lanelet.lineStringLayer[behavior.longBound.ref_line]
            if ls.attributes['type'] in types:
                zebra_lanelet = next((lanelet for lanelet in self.graph.conflicting(ll)
                                      if geo.intersectCenterlines2d(lanelet, ll)
                                      and tr_pedestrian.canPass(lanelet)), None)

                if zebra_lanelet:
                    behavior.longBound.attributes.no_stagnant_traffic = True
                    logger.debug(f'For linestring {ls.id} attribute no_stagnant_traffic=yes has been derived '
                                 f'due to overlapping lanelet {zebra_lanelet.id}.')
                    return
        logger.debug(f'No behavioral demand for longitudinal boundary detected.')

    # ------------------------------------------------------------------------------------
    # ------------ derivation of speed limits for all lanelets of a segment --------------
    # ------------------------------------------------------------------------------------
    def derive_segment_speed_limit(self, lanelet):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        logger.debug(f'-.-.-.-.-.-.-.-.-.-.-.-')

        logger.debug(f'Searching for lanelets of the same segment and driving direction for lanelet {lanelet.id}')
        own_direction = self.find_adjacent(lanelet, 0)
        logger.debug(f'Found lanelets {[[key, [ll.id for ll in value]] for key, value in own_direction.items()]}')
        first_opposing_lls = None
        self.assign_sl_along(own_direction)

        logger.debug(f'Searching for lanelets of the same segment but in the other driving direction.')
        # get speed limit of outer left lanelet
        for ll in own_direction[max(own_direction.keys())]:
            # find neighboring lanelet in other direction
            first_opposing_lls = self.find_one_sided_neighbors(ll, ll.leftBound.invert(), 'other')

            if first_opposing_lls:

                other_direction = self.find_adjacent(next(iter(first_opposing_lls)), 0)
                logger.debug(f'Found lanelets'
                             f'{[[key, [ll.id for ll in value]] for key, value in other_direction.items()]} '
                             f'for opposing driving direction.')
                # derive speed limit for other_direction
                self.assign_sl_along(other_direction)

                # distinguish passability between driving directions
                if not derive_crossing_type_for_lat_boundary(ll.leftBound.attributes, 'left') == 'not_possible':
                    # no structural separation
                    logger.debug(f'Driving directions for this segment are not structurally separated.'
                                 f' Cross assign speed limits to lanelets.')
                    ll_other_left = list(other_direction[max(other_direction.keys())])[0]
                    self.assign_sl_against(other_direction, ll)
                    self.assign_sl_against(own_direction, ll_other_left)
                else:
                    # driving directions are structurally separated
                    logger.debug(f'Driving directions for this segment are structurally separated.'
                                 f' Use along behavior speed limit for against behavior.')
                    self.assign_sl_against(other_direction)
                    self.assign_sl_against(own_direction)

                break

        # if no lanelet into the opposing direction is found,
        # it is assumed that the driving directions are structurally separated
        if not first_opposing_lls:
            # driving directions are structurally separated
            logger.debug(f'Driving directions for this segment are structurally separated.'
                         f' Use along behavior speed limit for against behavior.')
            self.assign_sl_against(own_direction)

        logger.debug(f'-.-.-.-.-.-.-.-.-.-.-.-')

    def find_adjacent(self, current_ll, level, prev_ll=None):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        # Find all lanelets that are lying right next to each other
        # condition is that they share there lateral boundary

        lefts = self.find_one_sided_neighbors(current_ll, current_ll.leftBound, 'same')
        rights = self.find_one_sided_neighbors(current_ll, current_ll.rightBound, 'same')
        lefts.discard(prev_ll)
        rights.discard(prev_ll)

        lanelets_for_direction = defaultdict(set)
        lanelets_for_direction[level].add(current_ll)

        for lanelet in lefts:
            sub_set = self.find_adjacent(lanelet, level + 1, current_ll)
            util.join_dictionaries(lanelets_for_direction, sub_set)

        for lanelet in rights:
            sub_set = self.find_adjacent(lanelet, level - 1, current_ll)
            util.join_dictionaries(lanelets_for_direction, sub_set)

        return lanelets_for_direction

    def assign_sl_along(self, segment):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        for level in segment.keys():
            for ll in segment[level]:
                speed_limit = str(round(self.traffic_rules.speedLimit(ll).speedLimit))
                ll.attributes['own_speed_limit'] = speed_limit
                logger.debug(f'Saving speed limit {speed_limit} for along behavior in lanelet {ll.id}')

                speed_limit_objects = [regelem for regelem in ll.regulatoryElements if isinstance(regelem, SpeedLimit)]
                if speed_limit_objects:
                    logger.debug(f'Found regulatory element {speed_limit_objects[0].id} that indicates the speed limit')
                    ll.attributes['own_speed_limit_link'] = str(speed_limit_objects[0].id)

    def assign_sl_against(self, segment, other_ll=None):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        for level in segment.keys():
            for ll in segment[level]:
                if other_ll:
                    ll.attributes['other_speed_limit'] = other_ll.attributes['own_speed_limit']
                    if 'own_speed_limit_link' in ll.attributes:
                        ll.attributes['other_speed_limit_link'] = other_ll.attributes['own_speed_limit_link']
                else:
                    ll.attributes['other_speed_limit'] = ll.attributes['own_speed_limit']
                    if 'own_speed_limit_link' in ll.attributes:
                        ll.attributes['other_speed_limit_link'] = ll.attributes['own_speed_limit_link']

    def find_one_sided_neighbors(self, lanelet, ls, orientation):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        ll_layer = self.map_lanelet.laneletLayer
        nbrs = {ll for ll in ll_layer.findUsages(ls) if is_ll_relevant(ll.attributes)}
        nbrs.discard(lanelet)
        nbrs = {ll for ll in nbrs if not geo.overlaps2d(ll, lanelet)}
        surrounding_lls_left = self.neighbor_next_to_area(ls)
        nbrs.update(self.filter_for_direction(surrounding_lls_left, lanelet, ls, orientation))

        return nbrs

    def neighbor_next_to_area(self, ls):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        ll_layer = self.map_lanelet.laneletLayer

        neighbor_areas = self.find_neighbor_areas(ls, 'keepout')
        if len(neighbor_areas) > 1:
            logger.warning(f'For linestring {ls.id}: Multiple adjacent areas have been found.'
                           f' No distinct derivation of driving directions possible')
        if neighbor_areas:
            area = neighbor_areas.pop()
            surrounding_lanelets = dict()
            # Get list of all linestrings in area
            ls_of_area = {area_boundary for area_boundary in area.outerBound}
            # Remove linestring of current lanelet from this list
            ls_of_area.discard(ls)
            ls_of_area.discard(ls.invert())
            # Search for all usages of those linestrings in other lanelets and collect them in a list
            for area_boundary in ls_of_area:
                for ll in ll_layer.findUsages(area_boundary):
                    # Filter list of lanelets for ones that are relevant
                    if is_ll_relevant(ll.attributes):
                        surrounding_lanelets[ll] = area_boundary
                for ll in ll_layer.findUsages(area_boundary.invert()):
                    # Filter list of lanelets for ones that are relevant
                    if is_ll_relevant(ll.attributes):
                        surrounding_lanelets[ll] = area_boundary.invert()

            # If more than one lanelets have been found, write a warning to log
            if len(surrounding_lanelets) > 1:
                logger.warning(f'For area {area.id}: Multiple adjacent lanelets have been found.'
                               f' No distinct derivation of driving directions possible')
            return surrounding_lanelets
        else:
            return dict()

    def filter_for_direction(self, sourrounding_lanelets, ref_lanelet, ref_ls, orientation):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        list_for_direction = []
        for ll, ls in sourrounding_lanelets.items():
            angle = util.angle_between_lanelets(ll, ref_lanelet)
            if (orientation == 'same' and angle < 45) \
                    and (ls[0] == ref_ls[0] or ls[-1] == ref_ls[-1]
                         or self.are_linestrings_orthogonal(ls, ref_ls, [ls[0], ref_ls[0]])):
                list_for_direction.append(ll)
            # Todo: Decide for one way
            # if orientation == 'same' and angle < 45:
            #     if ls[0] == ref_ls[0] or ls[-1] == ref_ls[-1]:
            #         list_for_direction.append(ll)
            #     elif self.are_linestrings_orthogonal(ls, ref_ls, [ls[0], ref_ls[0]]):
            #         list_for_direction.append(ll)
            elif orientation == 'other' and 135 < angle < 225:
                if ls[0] == ref_ls[-1] or ls[-1] == ref_ls[0]:
                    list_for_direction.append(ll)
                elif self.are_linestrings_orthogonal(ls, ref_ls, [ls[-1], ref_ls[0]]):
                    list_for_direction.append(ll)
            elif 45 < angle < 135:
                logger.warning(f'Lanelet {ll.id} and {ref_lanelet.id} border the same area and '
                               f'cannot be assigned to the same segment. Reason: Angle too large')

        if len(sourrounding_lanelets) > 1:
            logger.warning(f'Multiple Lanelets bordering the same area.'
                           f' Lanelet IDs: {[ll.id for ll in sourrounding_lanelets.keys()]}.'
                           f' Lanelet(s) {list_for_direction} has been selected due to given criteria.')

        return list_for_direction

    def are_linestrings_orthogonal(self, ls1, ls2, pts):
        """
        Criterium that is used for determining whether two lanelets that are surrounding an keepout area, are belonging
        to the same segment. Therefore, the two linestring that represent the border between the respective lanelets and
        the area are given and the angle between each of them and a connecting line between them is calculated. If the
        linestring are both having an angle to this line of 80° to 100°, the criterium is satisfied. 

        Parameters:
            ls1 (Linestring2d | Linestring3d):First linestring.
            ls2 (Linestring2d | Linestring3d):Second linestring.
            pts (list): List containing two points that connect the two linestrings.

        Returns:
            linestrings_orthogonal (bool):True if conditions are met, otherwise False.
        """

        # Retrieve mutable point objects from the point layer
        pts_corrected = [self.map_lanelet.pointLayer[pt.id] for pt in pts]
        # Create a Linestring object with the two points
        ls_connect = LineString3d(getId(), pts_corrected)

        # Calculate the angles between each linestring and the connecting linestring
        angle_1 = util.angle_between_linestrings(ls1, ls_connect)
        angle_2 = util.angle_between_linestrings(ls2, ls_connect)

        # return a bool through checking whether the two angles are within the specified range of 80° to 100°
        return 80 < angle_1 < 100 and 80 < angle_2 < 100

    # ---------------------------------------------------------------------------------
    # ------------ behavior derivation of reservation at zebra crossings --------------
    # ---------------------------------------------------------------------------------
    def derive_conflicts(self, bs):
        """
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            pt_left (Point2d | Point3d):The lanelet that is being checked.
            pt_right (Point2d | Point3d):The lanelet on which the behavior spaced is based.

        Returns:
            lines (dict):True if conditions are met, otherwise False.
        """

        # find all conflicting lanelets in RoutingGraph for lanelet of this behavior space
        for ll in self.graph.conflicting(bs.ref_lanelet):
            # filter this list for lanelets whose centerline are intersecting with the behavior spaces lanelet
            if is_zebra_and_intersecting(ll, bs.ref_lanelet):

                logger.debug(f'Conflicting zebra crossing with lanelet ID {ll.id} has been found. '
                             f'Setting reservation for behavior space {bs} for both directions to externally')
                bs.alongBehavior.reservation[0].attributes.reservation = tp.ReservationType.EXTERNALLY
                bs.againstBehavior.reservation[0].attributes.reservation = tp.ReservationType.EXTERNALLY
                bs.alongBehavior.reservation[0].attributes.pedestrian = True
                bs.againstBehavior.reservation[0].attributes.pedestrian = True

                logger.debug(f'Searching for lanelets and areas that need to be referenced via reservation links.')
                for link_ll in self.graph.conflicting(ll):
                    if is_ll_relevant(link_ll.attributes) and geo.intersectCenterlines2d(link_ll, ll):

                        if not link_ll == bs.ref_lanelet:
                            logger.debug(f'Found lanelet {link_ll.id}, which conflicts with crosswalk lanelet {ll.id}')
                            bs.alongBehavior.reservation[0].attributes.add_link(link_ll.id)
                            bs.againstBehavior.reservation[0].attributes.add_link(link_ll.id)

                        nbr_areas = self.find_neighbor_areas(link_ll.leftBound, 'walkway') | self.find_neighbor_areas(
                            link_ll.rightBound, 'walkway')
                        for area in nbr_areas:
                            logger.debug(f'Found walkway area {area.id}, which lies next to crosswalk lanelet {ll.id}')
                            bs.alongBehavior.reservation[0].attributes.add_link(area.id)
                            bs.againstBehavior.reservation[0].attributes.add_link(area.id)

                for link_ll in self.graph.previous(ll):
                    logger.debug(
                        f'Found walkway lanelet {link_ll.id}, which lies before/after to crosswalk lanelet {ll.id}')
                    bs.alongBehavior.reservation[0].attributes.add_link(link_ll.id)
                    bs.againstBehavior.reservation[0].attributes.add_link(link_ll.id)
                break
        logger.debug(f'No zebra crossing found.')

    def find_neighbor_areas(self, ls, subtype=None):
        """
        Searches in the areaLayer of the laneletMap for usages of the given linestring. A subtype can be specified to
        only find areas of this subtype. E.g. 'parking' or 'keepout'

        Parameters:
            ls (Linestring2d | Linestring3d):Linestring that is used to search for neighboring areas.
            subtype (str):Optional specification of a subtype that the function searches for.

        Returns:
            neighbor_areas (set):Set of area elements that were found.
        """

        # Get the areaLayer object
        a_layer = self.map_lanelet.areaLayer
        # Search for areas in which the linestring is used as part of the boundary. To make sure every area will be found,
        # include a search for the inverted linestring.
        neighbor_areas = set.union(set(a_layer.findUsages(ls)),
                                   set(a_layer.findUsages(ls.invert())))

        # If a subtype-string was given, filter the set and only keep the areas of the specified subtype
        if subtype:
            neighbor_areas = {area for area in neighbor_areas if area.attributes['subtype'] == subtype}

        return neighbor_areas
