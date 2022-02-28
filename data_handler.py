import BSSD_elements
from preprocessing import ll_relevant, protected_b_lane
import logging
import lanelet2
from lanelet2.geometry import distance as dist
import math
from lanelet2.core import LineString3d, getId
from constants import LONG_BDR_TAGS, LONG_BDR_DICT
from geometry_derivation import make_orth_bounding_box, find_flush_bdr, find_line_insufficient
from behavior_derivation import distinguish_lat_boundary
from BSSD_elements import create_placeholder
logger = logging.getLogger(__name__)


class data_handler():

    def __init__(self, map_lanelet):
        self.map_lanelet = map_lanelet
        self.map_bssd = BSSD_elements.bssdClass()
        self.relevant_lanelets = []
        self.graph = None

    def get_RoutingGraph_all(self):
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        edited = {}

        for ll in self.map_lanelet.laneletLayer:
            if not traffic_rules.canPass(ll):
                if 'participant:vehicle' in ll.attributes:
                    edited[ll] = ll.attributes['participant:vehicle']
                else:
                    edited[ll] = None
                ll.attributes['participant:vehicle'] = 'yes'

        self.graph = lanelet2.routing.RoutingGraph(self.map_lanelet, traffic_rules)

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
        # atm only long. boundary
        fwd_ls, long_ref_f = self.get_long_bdr(ll.leftBound[0], ll.rightBound[0], 'fwd' == direction, ls)
        bwd_ls, long_ref_b = self.get_long_bdr(ll.leftBound[-1], ll.rightBound[-1], 'bwd' == direction, ls)
        # derive abs behavior
        new_bs = create_placeholder(ll, fwd_ls, bwd_ls)
        logger.debug(f'Created Behavior Space {new_bs.id}')
        # new_bs.alongBehavior.assign_long_ref(long_ref_f, 'along')
        # new_bs.againstBehavior.assign_long_ref(long_ref_b, 'against')
        new_bs = self.derive_behavior(new_bs, ll)
        # new_bs = derive_speed_limit(new_bs, ll, self.map_lanelet, graph)
        self.map_bssd.add(new_bs)

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
        ref_line = None

        logger.debug(f'Derivation of longitudinal boundary')

        if use_previous:
            # Use previously created linestring
            ls = previous_id
            logger.debug(f'Using linestring from successor/predecessor')
        elif pt_left.id == pt_right.id:
            # No longitudinal boundary exists
            # print(ll.id)
            logger.debug(f'Longitudinal boundary doesn\'t exist')
            pass
        else:
            # Check for existing lineStrings (e.g. stop_line)
            lines = LONG_BDR_DICT

            # condition: linestrings shouldn't be part of lanelets as lat. boundary
            # lsList_pt_left = set(find_usage_special(llLayer, lsLayer, point=pt_left))
            # lsList_pt_right = set(find_usage_special(llLayer, lsLayer, point=pt_right))
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
                ref_line = lines['exact'][0]
            # Create new line, if necessary
            else:
                if any(v[0] for k, v in lines.items()):
                    matching_case = [k for k, v in lines.items() if v[0]]
                    pt_pairs = lines[matching_case[0]][1]
                    ref_line = lines[matching_case[0]][0]
                else:
                    pt_pairs = [pt_left, pt_right]

                pt_pairs = [self.map_lanelet.pointLayer[pt.id] for pt in pt_pairs]
                ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'boundary'})
                logger.debug(f'Created new linestring as longitudinal boundary with ID {ls.id}')
                self.map_lanelet.add(ls)

        return ls, ref_line

    def find_inside_lines(self, pt_left, pt_right):
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
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)

        bs.alongBehavior.speed_max = str(round(traffic_rules.speedLimit(lanelet).speedLimit))

        bs = self.derive_behavior_bdr_lat(bs, 'left')
        bs = self.derive_behavior_bdr_lat(bs, 'right')

        self.derive_behavior_bdr_long(bs.alongBehavior, lanelet)
        self.derive_behavior_bdr_long(bs.againstBehavior, lanelet)

        return bs

    def derive_behavior_bdr_lat(self, behavior_space, side):
        behavior_space.alongBehavior.leftBound.tags['crossing'] = \
            behavior_space.againstBehavior.rightBound.tags['crossing'] = \
            distinguish_lat_boundary(behavior_space.alongBehavior.leftBound.lineString.attributes, side)

        area_list = self.map_lanelet.areaLayer.findUsages(behavior_space.alongBehavior.leftBound.lineString)
        parking_only = 'False'
        if any(item.attributes['subtype'] == 'parking' for item in area_list):
            area_id = area_list[0].id
            parking_only = 'True'

        behavior_space.alongBehavior.leftBound.tags['parking_only'] = \
            behavior_space.againstBehavior.rightBound.tags['parking_only'] = parking_only

        return behavior_space

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

        return

    def derive_segment_speed_limit(self, lanelet, map_ll):
        ll_layer = map_ll.laneletLayer
        a_layer = map_ll.areaLayer
        own_direction = self.find_adjacent(lanelet)
        for ll in own_direction:
            other_direction = ll_layer.findUsages(ll.leftBound.invert())
            neighbor_areas = a_layer.findUsages(ll.leftBound)
            if other_direction:
                if other_direction[0].attributes['type'] not in constants.SEPARATION_TAGS:
                    other_direction = self.find_adjacent(other_direction[0])
                    break
                else:
                    # driving directions are structurally separated
                    break
            elif neighbor_areas:
                if len(neighbor_areas) > 1:
                    logger.warning(f'For lanelet {ll.id}: Multiple adjacent areas have been found.'
                                   f' No distinct derivation of driving directions possible')
                ls_of_area = [ls for ls in neighbor_areas[0].outerBound]
                ls_of_area.remove(ll.leftBound)
                for ls in ls_of_area:
                    lls = ll_layer.findUsages(ls)
                    break

        pass

    def neighbor_next_to_area(self, ll):
        a_layer = self.map_lanelet.areaLayer
        ll_layer = self.map_lanelet.laneletLayer
        neighbor_areas = set.union(set(a_layer.findUsages(ll.leftBound)),
                                   set(a_layer.findUsages(ll.leftBound.invert())))
        if len(neighbor_areas) > 1:
            logger.warning(f'For lanelet {ll.id}: Multiple adjacent areas have been found.'
                           f' No distinct derivation of driving directions possible')
        if neighbor_areas:
            surrounding_lanelets = []
            ls_of_area = [ls for ls in neighbor_areas[0].outerBound]
            ls_of_area.remove(ll.leftBound)
            ls_of_area.remove(ll.leftBound.invert())
            for ls in ls_of_area:
                surrounding_lanelets = surrounding_lanelets + ll_layer.findUsages(ls)
                surrounding_lanelets = surrounding_lanelets + ll_layer.findUsages(ls.invert())
            surrounding_lanelets = [ll for ll in surrounding_lanelets if ll_relevant(ll.attributes)]

            if len(surrounding_lanelets) > 1:
                logger.warning(f'For area {neighbor_areas[0].id}: Multiple adjacent lanelets have been found.'
                               f' No distinct derivation of driving directions possible')

            if len(surrounding_lanelets) == 0:
                return None
            else:
                if surrounding_lanelets[0].rightBound in neighbor_areas[0]:
                    pass

                return surrounding_lanelets
        return []

    def find_adjacent(self, current_ll):
        ll_layer = self.map_lanelet.laneletLayer
        lefts = {ll for ll in ll_layer.findUsages(current_ll.leftBound) if ll_relevant(ll.attributes)}
        rights = {ll for ll in ll_layer.findUsages(current_ll.rightBound) if ll_relevant(ll.attributes)}

        lanelets_for_direction = lefts.union(rights)

        lefts = lefts - lanelets_for_direction
        rights = rights - lanelets_for_direction

        for lanelet in lefts:
            lanelets_for_direction.update(self.find_adjacent(lanelet))

        for lanelet in rights:
            lanelets_for_direction.update(self.find_adjacent(lanelet))

        return lanelets_for_direction