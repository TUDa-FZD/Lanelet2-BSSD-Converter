import logging
from collections import defaultdict

import lanelet2
from lanelet2.geometry import distance as dist
from lanelet2.core import LineString3d, getId, SpeedLimit
import lanelet2.geometry as geo
from bssd.core import _types as tp

from preprocessing import is_ll_relevant
import BSSD_elements
from geometry_derivation import make_orthogonal_bounding_box, find_flush_bdr, find_line_insufficient
from behavior_derivation import derive_crossing_type_for_lat_boundary
import util
from constants import LONG_BDR_TAGS, LONG_BDR_DICT


logger = logging.getLogger('framework.data_handler')


def is_zebra_and_intersecting(ll, ref_ll):
    '''
    Returns boolean variable after checking whether two lanelets are having intersection
    centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

    Parameters:
        ll (lanelet):The lanelet that is being checked.
        ref_ll (lanelet):The lanelet on which the behavior spaced is based.

    Returns:
        Bool (bool):True if conditions are met, otherwise False.
    '''
    if geo.intersectCenterlines2d(ll, ref_ll) and not is_ll_relevant(ll.attributes) and \
            ll.leftBound.attributes['type'] == ll.rightBound.attributes['type'] == 'zebra_marking':
        return True
    else:
        return False


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
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            (bool):True if conditions are met, otherwise False.
        '''
        ll = self.map_lanelet.laneletLayer[ll_id]
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        self.relevant_lanelets.remove(ll_id)

        logger.debug(f'-----------------------------------------------')
        logger.debug(f'Derivation for Lanelet {ll_id}')

        # Perform derivation of behavior space from current lanelet
        logger.debug(f'Derivation of longitudinal boundary for along behavior')
        alg_ls, alg_ref = self.get_long_bdr(ll.leftBound[0], ll.rightBound[0], 'fwd' == direction, ls)
        logger.debug(f'Derivation of longitudinal boundary for against behavior')
        agst_ls, agst_ref = self.get_long_bdr(ll.leftBound[-1], ll.rightBound[-1], 'bwd' == direction, ls)
        # derive abs behavior
        new_bs = self.map_bssd.create_placeholder(ll, alg_ls, agst_ls)
        if alg_ref:
            new_bs.alongBehavior.longBound.ref_line = alg_ref
        if agst_ref:
            new_bs.againstBehavior.longBound.ref_line = agst_ref
        logger.debug(f'Created Behavior Space {new_bs.id}')
        self.derive_behavior(new_bs, ll)

        # Call function in itself for the succeeding and preceding lanelet(s) and hand over information
        # about already derived boundaries
        for successor in self.graph.following(ll):
            if successor.id in self.relevant_lanelets:
                self.recursive_loop(successor.id, 'fwd', agst_ls)
        for predecessor in self.graph.previous(ll):
            if predecessor.id in self.relevant_lanelets:
                self.recursive_loop(predecessor.id, 'bwd', alg_ls)

    # -----------------------------------------------
    # ----------- longitudinal boundary -------------
    # -----------------------------------------------
    def get_long_bdr(self, pt_left, pt_right, use_previous, previous):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        ls = None
        ref_line = None

        if use_previous:
            # Use previously created linestring
            ls = previous
            ref_line = ls.id
            logger.debug(f'Using linestring from successor/predecessor')
        elif pt_left.id == pt_right.id:
            # No longitudinal boundary exists
            logger.debug(f'Longitudinal boundary doesn\'t exist')
            pass
        else:
            # Check for existing lineStrings (e.g. stop_line)
            lines = LONG_BDR_DICT

            lsList_pt_left = set(self.map_lanelet.lineStringLayer.findUsages(pt_left))
            lsList_pt_right = set(self.map_lanelet.lineStringLayer.findUsages(pt_right))

            mutual_ls = set.intersection(lsList_pt_left, lsList_pt_right)
            lsList_pt_left = lsList_pt_left - mutual_ls
            lsList_pt_right = lsList_pt_right - mutual_ls

            # exact OR overarching
            lines.update(find_flush_bdr(pt_left, pt_right, mutual_ls))
            # insufficient
            lines['insufficient_half_left'] = find_line_insufficient(lsList_pt_left, pt_left, pt_right)
            lines['insufficient_half_right'] = find_line_insufficient(lsList_pt_right, pt_right, pt_left)
            # Both sides are not matching
            lines.update(self.find_inside_lines(pt_left, pt_right))

            # In case multiple linestrings have been found, write an error
            if len([v for k, v in lines.items() if v[0]]) > 1:
                logger.warning(f'Multiple possible long. boundaries found for points {pt_left} and {pt_right}')

            if lines['exact'][0]:
                ls = self.map_lanelet.lineStringLayer[lines['exact'][0]]
                ref_line = ls.id
                logger.debug(f'Using existing line with ID {ls.id} as long. boundary')
            # Create new line, if necessary
            else:
                if any(v[0] for k, v in lines.items()):
                    matching_case = [k for k, v in lines.items() if v[0]]
                    pt_pairs = lines[matching_case[0]][1]
                    ref_line = lines[matching_case[0]][0]
                    logger.debug(
                        f'Using existing line with ID {lines[matching_case[0]][0]} partially for long. boundary')
                else:
                    pt_pairs = [pt_left, pt_right]
                    logger.debug(f'No existing line has been found, using endpoints for new linestring.')

                pt_pairs = [self.map_lanelet.pointLayer[pt.id] for pt in pt_pairs]
                ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'boundary'})
                logger.debug(f'Created new linestring as longitudinal boundary with ID {ls.id}')
                self.map_lanelet.add(ls)

        return ls, ref_line

    def find_inside_lines(self, pt_left, pt_right):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        #### perhaps use geometry.inside
        searchBox = make_orthogonal_bounding_box(pt_left, pt_right)

        near_ls = [ls for ls in self.map_lanelet.lineStringLayer.search(searchBox) if
                   not pt_left in ls or not pt_right in ls]

        for line in near_ls:
            # Distinguish line inside and outside of lanelet
            if 'type' in line.attributes and line.attributes['type'] in LONG_BDR_TAGS and \
                    all(x in self.map_lanelet.pointLayer.search(searchBox) for x in [line[0], line[-1]]):
                ls_pts = [el for el in line]
                if dist(line[0], pt_left) < dist(line[0], pt_right):
                    ls_pts.insert(0, pt_left)
                    ls_pts.append(pt_right)
                else:
                    ls_pts.insert(0, pt_right)
                    ls_pts.append(pt_left)
                logger.debug(f'Found inside line with ID {line.id}')
                return {'insufficient_full': [line.id, ls_pts]}

        return {'insufficient_full': [None, None]}

    # -----------------------------------------------
    # ------------ behavior derivation --------------
    # -----------------------------------------------
    def derive_behavior(self, bs, lanelet):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        logger.debug(f'Deriving behavioral demand of lateral boundary of alongBehavior (left,'
                     f' ID:{bs.alongBehavior.leftBound.id}) and '
                     f'againstBehavior (right, ID:ID:{bs.againstBehavior.rightBound.id})')
        self.derive_behavior_bdr_lat(bs.alongBehavior, bs.againstBehavior, 'left')
        logger.debug(f'Deriving behavioral demand of lateral boundary of alongBehavior (left,'
                     f' ID:{bs.againstBehavior.leftBound.id}) and '
                     f'againstBehavior (right, ID:ID:{bs.alongBehavior.rightBound.id})')
        self.derive_behavior_bdr_lat(bs.againstBehavior, bs.alongBehavior, 'right')

        if bs.alongBehavior.longBound:
            logger.debug(f'Deriving behavioral demand of longitudinal boundary of'
                         f' alongBehavior (ID:{bs.alongBehavior.longBound.id})')
            self.derive_behavior_bdr_long(bs.alongBehavior, lanelet)
        if bs.againstBehavior.longBound:
            logger.debug(f'Deriving behavioral demand of longitudinal boundary of'
                         f' againstBehavior (ID:{bs.againstBehavior.longBound.id})')
            self.derive_behavior_bdr_long(bs.againstBehavior, lanelet)

        if 'own_speed_limit' not in lanelet.attributes and 'other_speed_limit' not in lanelet.attributes:
            self.derive_segment_speed_limit(lanelet)
        bs.alongBehavior.attributes.speed_max = lanelet.attributes['own_speed_limit']
        if 'own_speed_limit_link' in lanelet.attributes:
            bs.alongBehavior.attributes.add_speed_indicator(int(lanelet.attributes['own_speed_limit_link']))
        bs.againstBehavior.attributes.speed_max = lanelet.attributes['other_speed_limit']
        if 'other_speed_limit_link' in lanelet.attributes:
            bs.againstBehavior.attributes.add_speed_indicator(int(lanelet.attributes['other_speed_limit_link']))

        self.derive_conflicts(bs)

    # -------------------------------------------------------------------
    # ------------ behavior derivation of lateral boundary --------------
    # -------------------------------------------------------------------
    def derive_behavior_bdr_lat(self, behavior_a, behavior_b, side):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''

        logger.debug(f'Deriving CrossingType for linestring {behavior_a.leftBound.lineString.id}.')
        cr = derive_crossing_type_for_lat_boundary(behavior_a.leftBound.lineString.attributes, side)
        # Todo: adjust to new possiblities in core
        if cr:
            behavior_a.leftBound.attributes.crossing = behavior_b.rightBound.attributes.crossing = cr

        area_list = self.map_lanelet.areaLayer.findUsages(behavior_a.leftBound.lineString) + \
                    self.map_lanelet.areaLayer.findUsages(behavior_a.leftBound.lineString.invert())
        parking_only = False
        if any(item.attributes['subtype'] == 'parking' for item in area_list):
            logger.debug(f'Found parking area with ID {area_list[0].id} next to lateral boundaries'
                         f' {behavior_a.leftBound.id} and {behavior_b.rightBound.id}. Setting parking_only=yes')
            parking_only = True

        behavior_a.leftBound.attributes.parking_only = \
            behavior_b.rightBound.attributes.parking_only = parking_only

    # ------------------------------------------------------------------------
    # ------------ behavior derivation of longitudinal boundary --------------
    # ------------------------------------------------------------------------
    def derive_behavior_bdr_long(self, behavior, ll):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        types = ['zebra_marking']
        tr_pedestrian = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Pedestrian)
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
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        own_direction = self.find_adjacent(lanelet, 0)
        first_opposing_lls = None
        self.assign_sl_along(own_direction)

        # get speed limit of outer left lanelet
        for ll in own_direction[max(own_direction.keys())]:
            # find neighboring lanelet in other direction
            first_opposing_lls = self.find_one_sided_neighbors(ll, ll.leftBound.invert(), 'other')

            if first_opposing_lls:

                other_direction = self.find_adjacent(first_opposing_lls.pop(), 0)
                # derive speed limit for other_direction
                self.assign_sl_along(other_direction)

                # distinguish passability between driving directions
                if not derive_crossing_type_for_lat_boundary(ll.leftBound.attributes, 'left') == 'not_possible':
                    # no structural separation
                    ll_other_left = list(other_direction[max(other_direction.keys())])[0]
                    self.assign_sl_against(other_direction, ll)
                    self.assign_sl_against(own_direction, ll_other_left)
                else:
                    # driving directions are structurally separated
                    self.assign_sl_against(other_direction)
                    self.assign_sl_against(own_direction)

        # if no lanelet into the opposing direction is found,
        # it is assumed that the driving directions are structurally separated
        if not first_opposing_lls:
            # driving directions are structurally separated
            self.assign_sl_against(own_direction)

    def find_adjacent(self, current_ll, level, prev_ll=None):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''

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
            join_dictionaries(lanelets_for_direction, sub_set)

        for lanelet in rights:
            sub_set = self.find_adjacent(lanelet, level - 1, current_ll)
            join_dictionaries(lanelets_for_direction, sub_set)

        return lanelets_for_direction

    def assign_sl_along(self, segment):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        for level in segment.keys():
            for ll in segment[level]:
                ll.attributes['own_speed_limit'] = str(round(self.traffic_rules.speedLimit(ll).speedLimit))
                speed_limit_objects = [regelem for regelem in ll.regulatoryElements if isinstance(regelem, SpeedLimit)]
                if speed_limit_objects:
                    ll.attributes['own_speed_limit_link'] = str(speed_limit_objects[0].id)

    def assign_sl_against(self, segment, other_ll=None):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
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
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        ll_layer = self.map_lanelet.laneletLayer
        nbrs = {ll for ll in ll_layer.findUsages(ls) if is_ll_relevant(ll.attributes)}
        nbrs.discard(lanelet)
        nbrs = {ll for ll in nbrs if not geo.overlaps2d(ll, lanelet)}
        surrounding_lls_left = self.neighbor_next_to_area(ls)
        nbrs.update(self.filter_for_direction(surrounding_lls_left, lanelet, ls, orientation))

        return nbrs

    def neighbor_next_to_area(self, ls):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
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
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            sourrounding_lanelets (lanelet):The lanelet that is being checked.
            ref_lanelet (lanelet):The lanelet on which the behavior spaced is based.
            ref_ls (lanelet):The lanelet on which the behavior spaced is based.
            orientation (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.

        '''
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
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        pts_corrected = [self.map_lanelet.pointLayer[pt.id] for pt in pts]
        ls_connect = LineString3d(getId(), pts_corrected)
        angle_1 = util.angle_between_linestrings(ls1, ls_connect)
        angle_2 = util.angle_between_linestrings(ls2, ls_connect)

        return 80 < angle_1 < 100 and 80 < angle_2 < 100

    # ---------------------------------------------------------------------------------
    # ------------ behavior derivation of reservation at zebra crossings --------------
    # ---------------------------------------------------------------------------------
    def derive_conflicts(self, bs):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        # find all conflicting lanelets in RoutingGraph for lanelet of this behavior space
        for ll in self.graph.conflicting(bs.ref_lanelet):
            # filter this list for lanelets whose centerline are intersecting with the behavior spaces lanelet
            if is_zebra_and_intersecting(ll, bs.ref_lanelet):

                bs.alongBehavior.reservation[0].attributes.reservation = tp.ReservationType.EXTERNALLY
                bs.againstBehavior.reservation[0].attributes.reservation = tp.ReservationType.EXTERNALLY
                bs.alongBehavior.reservation[0].attributes.pedestrian = True
                bs.againstBehavior.reservation[0].attributes.pedestrian = True

                for link_ll in self.graph.conflicting(ll):
                    if is_ll_relevant(link_ll.attributes) and geo.intersectCenterlines2d(link_ll, ll):

                        if not link_ll == bs.ref_lanelet:
                            bs.alongBehavior.reservation[0].attributes.add_link(link_ll.id)
                            bs.againstBehavior.reservation[0].attributes.add_link(link_ll.id)

                        nbr_areas = self.find_neighbor_areas(link_ll.leftBound, 'walkway') | self.find_neighbor_areas(
                            link_ll.rightBound, 'walkway')
                        for area in nbr_areas:
                            bs.alongBehavior.reservation[0].attributes.add_link(area.id)
                            bs.againstBehavior.reservation[0].attributes.add_link(area.id)

                for link_ll in self.graph.previous(ll):
                    bs.alongBehavior.reservation[0].attributes.add_link(link_ll.id)
                    bs.againstBehavior.reservation[0].attributes.add_link(link_ll.id)
                break

    def find_neighbor_areas(self, ls, subtype=None):
        '''
        Returns boolean variable after checking whether two lanelets are having intersection
        centerlines. Furthermore, another criteria is that one of the lanelets is a zebra crossing.

        Parameters:
            ll (lanelet):The lanelet that is being checked.
            ref_ll (lanelet):The lanelet on which the behavior spaced is based.

        Returns:
            ls (bool):True if conditions are met, otherwise False.
        '''
        a_layer = self.map_lanelet.areaLayer
        neighbor_areas = set.union(set(a_layer.findUsages(ls)),
                                   set(a_layer.findUsages(ls.invert())))
        if subtype:
            neighbor_areas = {area for area in neighbor_areas if area.attributes['subtype'] == subtype}

        return neighbor_areas
