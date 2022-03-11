import BSSD_elements
from preprocessing import ll_relevant, protected_b_lane
import logging
import lanelet2
from lanelet2.geometry import distance as dist
import math
import util
from lanelet2.core import LineString3d, getId
from constants import LONG_BDR_TAGS, LONG_BDR_DICT
from geometry_derivation import make_orth_bounding_box, find_flush_bdr, find_line_insufficient
from behavior_derivation import distinguish_lat_boundary
import constants
import lanelet2.geometry as geo
from bssd.core import _types as tp
from collections import defaultdict
logger = logging.getLogger('framework.data_handler')


class data_handler():

    def __init__(self, map_lanelet):
        self.map_lanelet = map_lanelet
        self.map_bssd = BSSD_elements.bssdClass()
        self.relevant_lanelets = []
        self.graph = None
        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                           lanelet2.traffic_rules.Participants.Vehicle)

    def get_RoutingGraph_all(self):
        edited = {}

        for ll in self.map_lanelet.laneletLayer:
            if not self.traffic_rules.canPass(ll):
                if 'participant:vehicle' in ll.attributes:
                    edited[ll] = ll.attributes['participant:vehicle']
                else:
                    edited[ll] = None
                ll.attributes['participant:vehicle'] = 'yes'

        self.graph = lanelet2.routing.RoutingGraph(self.map_lanelet, self.traffic_rules)

        for ll, value in edited.items():
            if value:
                ll.attributes['participant:vehicle'] = value
            else:
                del ll.attributes['participant:vehicle']

    def find_relevant_lanelets(self):
        self.relevant_lanelets = [item.id for item in self.map_lanelet.laneletLayer if ll_relevant(item.attributes)]
        self.get_relevant_bicycle_lls()

    def ll_recursion(self, ll_id, direction=None, ls=None):

        ll = self.map_lanelet.laneletLayer[ll_id]
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        self.relevant_lanelets.remove(ll_id)

        logger.debug(f'-----------------------------------------------')
        logger.debug(f'Derivation for Lanelet {ll_id}')

        # Perform derivation of behavior space from current lanelet
        logger.debug(f'Derivation of longitudinal boundary for along behavior')
        fwd_ls = self.get_long_bdr(ll.leftBound[0], ll.rightBound[0], 'fwd' == direction, ls)
        logger.debug(f'Derivation of longitudinal boundary for against behavior')
        bwd_ls = self.get_long_bdr(ll.leftBound[-1], ll.rightBound[-1], 'bwd' == direction, ls)
        # derive abs behavior
        new_bs = self.map_bssd.create_placeholder(ll, fwd_ls, bwd_ls)
        logger.debug(f'Created Behavior Space {new_bs.id}')
        self.derive_behavior(new_bs, ll)

        # Call function in itself for the succeeding and preceding lanelet(s) and hand over information
        # about already derived boundaries
        for successor in self.graph.following(ll):
            if successor.id in self.relevant_lanelets:
                self.ll_recursion(successor.id, 'fwd', bwd_ls)
        for predecessor in self.graph.previous(ll):
            if predecessor.id in self.relevant_lanelets:
                self.ll_recursion(predecessor.id, 'bwd', fwd_ls)

    def find_usages_and_remove_self(self, ll, side):

        nbrs = []
        if side == 'r':
            nbrs = self.map_lanelet.laneletLayer.findUsages(ll.rightBound)
        elif side == 'l':
            nbrs = self.map_lanelet.laneletLayer.findUsages(ll.leftBound)
        nbrs.remove(ll)

        return nbrs

    def get_relevant_bicycle_lls(self):

        list_bicycle = [item for item in self.map_lanelet.laneletLayer if item.attributes['subtype'] == 'bicycle_lane']

        for ll in list_bicycle:
            nbrs_left = self.find_usages_and_remove_self(ll, 'l')
            nbrs_right = self.find_usages_and_remove_self(ll, 'r')

            if len(nbrs_left) > 1 or len(nbrs_right) > 1:
                logger.warning(f"For bicycle_lane with ID {ll.id}: Multiple neighbors have been found.\
                left: {nbrs_left}, right {nbrs_right}")

            # if (nbrs_left and ls_of_pbl(ll.leftBound.attributes) and ll_relevant(nbrs_left[0].attributes)) or \
            #         (nbrs_right and ls_of_pbl(ll.rightBound.attributes) and ll_relevant(nbrs_right[0].attributes)):
            if protected_b_lane(nbrs_left, ll.leftBound.attributes) \
                    or protected_b_lane(nbrs_right, ll.rightBound.attributes):
                logger.debug(f' Lanelet {ll.id} identified as protected bicycle lane')
                ll.attributes['subtype_alt'] = 'bicycle_lane_protected'
                ll.attributes['participant:vehicle'] = 'yes'
                ll.attributes['participant:bicycle'] = 'yes'
                self.relevant_lanelets.append(ll.id)
                list_bicycle.remove(ll)

    def get_long_bdr(self, pt_left, pt_right, use_previous, previous_id):
        ls = None

        if use_previous:
            # Use previously created linestring
            ls = previous_id
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

            ### EXACT OR overarching ###
            lines.update(find_flush_bdr(pt_left, pt_right, mutual_ls))

            ### insuFFICIENT
            lines['insufficient_half_left'] = find_line_insufficient(lsList_pt_left, pt_left, pt_right)
            lines['insufficient_half_right'] = find_line_insufficient(lsList_pt_right, pt_right, pt_left)

            ### Both sides are not matching
            lines.update(self.find_inside_lines(pt_left, pt_right))

            # In case multiple linestrings have been found, write an error
            if len([v for k, v in lines.items() if v[0]]) > 1:
                logger.warning(f'Multiple possible long. boundaries found for points {pt_left} and {pt_right}')

            if lines['exact'][0]:
                ls = self.map_lanelet.lineStringLayer[lines['exact'][0]]
                logger.debug(f'Using existing line with ID {ls.id} as long. boundary')
            # Create new line, if necessary
            else:
                if any(v[0] for k, v in lines.items()):
                    matching_case = [k for k, v in lines.items() if v[0]]
                    pt_pairs = lines[matching_case[0]][1]
                    logger.debug(f'Using existing line with ID {lines[matching_case[0]][0]} partially for long. boundary')
                else:
                    pt_pairs = [pt_left, pt_right]
                    logger.debug(f'No existing line has been found, using endpoints for new linestring.')

                pt_pairs = [self.map_lanelet.pointLayer[pt.id] for pt in pt_pairs]
                ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'boundary'})
                logger.debug(f'Created new linestring as longitudinal boundary with ID {ls.id}')
                self.map_lanelet.add(ls)

        return ls

    def find_inside_lines(self, pt_left, pt_right):

        #### perhaps use geometry.inside
        searchBox = make_orth_bounding_box(pt_left, pt_right)

        near_ls = [ls for ls in self.map_lanelet.lineStringLayer.search(searchBox) if not pt_left in ls or not pt_right in ls]

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

    def derive_behavior(self, bs, lanelet):

        logger.debug(f'Deriving CrossingType and parking_only for alongBehavior left/againstBehavior right')
        self.derive_behavior_bdr_lat(bs.alongBehavior, bs.againstBehavior, 'left')
        logger.debug(f'Deriving CrossingType and parking_only for againstBehavior left/alongBehavior right')
        self.derive_behavior_bdr_lat(bs.againstBehavior, bs.alongBehavior, 'right')

        self.derive_behavior_bdr_long(bs.alongBehavior, lanelet)
        self.derive_behavior_bdr_long(bs.againstBehavior, lanelet)

        if 'own_speed_limit' not in lanelet.attributes and 'other_speed_limit' not in lanelet.attributes:
            own, other = self.derive_segment_speed_limit(lanelet)
        bs.alongBehavior.speed_max = lanelet.attributes['own_speed_limit']
        bs.againstBehavior.speed_max = lanelet.attributes['other_speed_limit']

        self.derive_conflicts(bs)

    def derive_behavior_bdr_lat(self, behavior_a, behavior_b, side):

        cr = distinguish_lat_boundary(behavior_a.leftBound.lineString.attributes, side)
        # Todo: adjust to new possiblities in core
        if cr:
            behavior_a.leftBound.crossing = behavior_b.rightBound.crossing = cr

        area_list = self.map_lanelet.areaLayer.findUsages(behavior_a.leftBound.lineString) + \
                    self.map_lanelet.areaLayer.findUsages(behavior_a.leftBound.lineString.invert())
        parking_only = False
        if any(item.attributes['subtype'] == 'parking' for item in area_list):
            logger.debug(f'Found parking area with ID {area_list[0].id} next to lateral boundaries'
                         f' {behavior_a.leftBound.id} and {behavior_b.rightBound.id}. Setting parking_only=yes')
            parking_only = True

        behavior_a.leftBound.parking_only = \
            behavior_b.rightBound.parking_only = parking_only

    def derive_behavior_bdr_long(self, behavior, ll):

        ll_layer = self.map_lanelet.laneletLayer
        types = ['pedestrian_marking', 'zebra_marking']
        subtypes = ['crosswalk']
        if behavior.longBound and behavior.longBound.ref_line:
            ls = behavior.longBound.ref_line
            ls = self.map_lanelet.lineStringLayer[ls]
            if ls.attributes['type'] in types:
                set_from_ls = set(ll_layer.findUsages(ls) + ll_layer.findUsages(ls.invert()))
                set_from_conflict = set(self.graph.conflicting(ll))
                set_common = set_from_ls ^ set_from_conflict

                if any(lanelet.attributes['subtype'] in subtypes for lanelet in set_common):
                    behavior.longBound.tags['no_stagnant_traffic'] = 'yes'

    def derive_segment_speed_limit(self, lanelet):

        own_direction = self.find_adjacent(lanelet, 0)
        self.assign_sl_along(own_direction)

        # get speed limit of outer left lanelet
        for ll in own_direction[max(own_direction.keys())]:
            other_direction = self.find_adjacent(ll, 0)

            if other_direction:
                # derive speed limit for other_direction
                self.assign_sl_along(other_direction)

                # distinguish passability between driving directions
                if not distinguish_lat_boundary(ll.leftBound.attributes, 'left') == 'not_possible':
                    # no structural separation
                    ll_other_left = other_direction[max(other_direction.keys())].pop()
                    self.assign_sl_against(other_direction, ll)
                    self.assign_sl_against(own_direction, ll_other_left)
                else:
                    # driving directions are structurally separated
                    self.assign_sl_against(other_direction)
                    self.assign_sl_against(own_direction)

        # if no lanelet into the opposing direction is found,
        # it is assumed that the driving directions are structurally separated
        if not other_direction:
            # driving directions are structurally separated
            self.assign_sl_against(other_direction)
            self.assign_sl_against(own_direction)
        return own_direction, other_direction

    def assign_sl_along(self, segment):
        for level in segment.keys():
            for ll in segment[level]:
                ll.attributes['own_speed_limit'] = str(round(self.traffic_rules.speedLimit(ll).speedLimit))

    def assign_sl_against(self, segment, other_ll=None):
        for level in segment.keys():
            for ll in segment[level]:
                if other_ll:
                    ll.attributes['other_speed_limit'] = other_ll.attributes['own_speed_limit']
                else:
                    ll.attributes['other_speed_limit'] = ll.attributes['own_speed_limit']

    def neighbor_next_to_area(self, ls):
        a_layer = self.map_lanelet.areaLayer
        ll_layer = self.map_lanelet.laneletLayer
        neighbor_areas = set.union(set(a_layer.findUsages(ls)),
                                   set(a_layer.findUsages(ls.invert())))
        neighbor_areas = {area for area in neighbor_areas if area.attributes['subtype'] == 'keepout'}

        if len(neighbor_areas) > 1:
            logger.warning(f'For lanelet {ll.id}: Multiple adjacent areas have been found.'
                           f' No distinct derivation of driving directions possible')
        if neighbor_areas:
            area = neighbor_areas.pop()
            surrounding_lanelets = []
            # Get list of all linestrings in area
            ls_of_area = {ls for ls in area.outerBound}
            # Remove linestring of current lanelet from this list
            ls_of_area.discard(ls)
            ls_of_area.discard(ls.invert())
            # Search for all usages of those linestrings in other lanelets and collect them in a list
            for ls in ls_of_area:
                surrounding_lanelets = surrounding_lanelets + ll_layer.findUsages(ls) + ll_layer.findUsages(ls.invert())
            # Filter list of lanelets for ones that are relevant
            surrounding_lanelets = [ll for ll in surrounding_lanelets if ll_relevant(ll.attributes)]

            # If more than one lanelets have been found, write a warning to log
            if len(surrounding_lanelets) > 1:
                logger.warning(f'For area {area.id}: Multiple adjacent lanelets have been found.'
                               f' No distinct derivation of driving directions possible')
            return surrounding_lanelets
        else:
            return []

    def find_adjacent(self, current_ll, level, prev_ll=None):
        # Find all lanelets that are lying right next to each other
        # condition is that they share there lateral boundary

        lefts = self.find_one_sided_neighbors(current_ll, current_ll.leftBound)
        rights = self.find_one_sided_neighbors(current_ll, current_ll.rightBound)
        lefts.discard(prev_ll)
        rights.discard(prev_ll)

        lanelets_for_direction = defaultdict(set)
        lanelets_for_direction[level].add(current_ll)

        for lanelet in lefts:
            sub_set = self.find_adjacent(lanelet, level + 1, current_ll)
            self.join_dictionaries(lanelets_for_direction, sub_set)

        for lanelet in rights:
            sub_set = self.find_adjacent(lanelet, level - 1, current_ll)
            self.join_dictionaries(lanelets_for_direction, sub_set)

        return lanelets_for_direction

    def join_dictionaries(self, dict_a, dict_b):
        for d in (dict_a, dict_b):
            for key, value in d.items():
                dict_a[key].update(value)

        return dict_a

    def find_one_sided_neighbors(self, lanelet, ls):
        ll_layer = self.map_lanelet.laneletLayer
        nbrs = {ll for ll in ll_layer.findUsages(ls) if ll_relevant(ll.attributes)}
        nbrs.discard(lanelet)
        nbrs = {ll for ll in nbrs if not geo.overlaps2d(ll, lanelet)}
        surrounding_lls_left = self.neighbor_next_to_area(ls)
        nbrs.update(self.filter_for_direction(surrounding_lls_left, lanelet, 'same'))

        return nbrs

    def filter_for_direction(self, list_of_lanelets, ref_lanelet, orientation):
        list_for_direction = []
        for ll in list_of_lanelets:
            angle = util.angle_between(ll, ref_lanelet)
            if orientation == 'same' and angle < 15:
                list_for_direction.append(ll)
            elif orientation == 'other' and 165 < angle < 195:
                list_for_direction.append(ll)
            elif 15 < angle < 165:
                logger.warning(f'Lanelet {ll.id} and Linestring {ref_lanelet.id} border the area and '
                               f'cannot be assigned to the same segment. Reason: Angle too large')

        if len(list_of_lanelets) > 1:
            logger.warning(f'Multiple Lanelets for same direction as a neighbor to a area have been found.'
                           f' Lanelet IDs: {[ll.id for ll in list_of_lanelets]}')

        return list_of_lanelets

    def derive_conflicts(self, bs):

        conflicts = self.graph.conflicting(bs.ref_lanelet)
        conflicts = [ll for ll in conflicts if geo.intersectCenterlines2d(ll, bs.ref_lanelet)]
        for ll in conflicts:
            if not ll_relevant(ll.attributes): # Todo: bring conditions together
                if ll.leftBound.attributes['type'] == ll.rightBound.attributes['type'] == 'zebra_marking':
                    bs.alongBehavior.reservation_sub[0].reservation = tp.ReservationType.EXTERNALLY
                    bs.againstBehavior.reservation_sub[0].reservation = tp.ReservationType.EXTERNALLY
                    bs.alongBehavior.reservation_sub[0].pedestrian = True
                    bs.againstBehavior.reservation_sub[0].pedestrian = True
                    bs.alongBehavior.reservation_sub[0].add_link(ll.id)
                    bs.againstBehavior.reservation_sub[0].add_link(ll.id)
                    break
