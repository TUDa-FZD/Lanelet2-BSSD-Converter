import lanelet2
from preprocessing import ll_relevant
import constants
import logging
logger = logging.getLogger(__name__)


def derive_behavior(bs, lanelet, map_lanelet, routing_graph):
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)

    a_layer = map_lanelet.areaLayer
    ll_layer = map_lanelet.laneletLayer
    ls_layer = map_lanelet.lineStringLayer

    bs.along.tags['speed_limit'] = str(round(traffic_rules.speedLimit(lanelet).speedLimit))

    bs = derive_behavior_bdr_lat(bs, a_layer, 'left')
    bs = derive_behavior_bdr_lat(bs, a_layer, 'right')

    derive_behavior_bdr_long(bs.along, lanelet, routing_graph, ll_layer, ls_layer)
    derive_behavior_bdr_long(bs.against, lanelet, routing_graph, ll_layer, ls_layer)

    return bs


def derive_behavior_bdr_lat(behavior_space, area_layer, side):
    behavior_space.along.leftBound.tags['crossing'] = \
        behavior_space.against.rightBound.tags['crossing'] = \
        distinguish_lat_boundary(behavior_space.along.leftBound.lineString.attributes, side)

    area_list = area_layer.findUsages(behavior_space.along.leftBound.lineString)
    parking_only = 'False'
    if any(item.attributes['subtype'] == 'parking' for item in area_list):
        area_id = area_list[0].id
        parking_only = 'True'

    behavior_space.along.leftBound.tags['parking_only'] = \
        behavior_space.against.rightBound.tags['parking_only'] = parking_only

    return behavior_space


def derive_behavior_bdr_long(behavior, ll, graph, ll_layer, ls_layer):

    types = ['pedestrian_marking', 'zebra_marking']
    subtypes = ['crosswalk']
    if behavior.longBound and behavior.longBound.ref_line:
        ls = behavior.longBound.ref_line
        ls = ls_layer[ls]
        if ls.attributes['type'] in types:
            set_from_ls = set(ll_layer.findUsages(ls) + ll_layer.findUsages(ls.invert()))
            set_from_conflict = set(graph.conflicting(ll))
            set_common = set_from_ls ^ set_from_conflict

            if any(lanelet.attributes['subtype'] in subtypes for lanelet in set_common):
                behavior.longBound.tags['no_stagnant_traffic'] = 'yes'

    return


def derive_segment_speed_limit(ll, map_ll):
    ll_layer = map_ll.laneletLayer
    own_direction = find_adjacent(ll_layer, ll)
    for ll in own_direction:
        other_direction = ll_layer.findUsages(ll.leftBound)
        if other_direction and other_direction[0].attributes['type'] not in constants.SEPARATION_TAGS:
            other_direction = find_adjacent(ll_layer, other_direction[0])
            break

    pass


def find_adjacent(ll_layer, current_ll):
    lefts = {ll for ll in ll_layer.findUsages(current_ll.leftBound) if ll_relevant(ll.attributes)}
    rights = {ll for ll in ll_layer.findUsages(current_ll.rightBound) if ll_relevant(ll.attributes)}

    lanelets_for_direction = lefts.union(rights)

    lefts = lefts - lanelets_for_direction
    rights = rights - lanelets_for_direction

    for lanelet in lefts:
        lanelets_for_direction.update(find_adjacent(ll_layer, lanelet))

    for lanelet in rights:
        lanelets_for_direction.update(find_adjacent(ll_layer, lanelet))

    return lanelets_for_direction


def are_driving_directions_separated(map_lanelet, ll):

    pass


def distinguish_lat_boundary(att, side):
    if 'type' not in att:
        return ''

    if att['type'] == 'curbstone':
        if 'subtype' not in att:
            return 'not_possible'
        elif att['subtype'] == 'low':
            return 'allowed'
        elif att['subtype'] == 'high':
            return 'not_possible'

    elif att['type'] == 'line_thin' or att['type'] == 'line_thick':
        if 'subtype' not in att:
            return 'prohibited'
        elif att['subtype'] == 'solid':
            return 'prohibited'
        elif att['subtype'] == 'solid_solid':
            return 'prohibited'
        elif att['subtype'] == 'dashed':
            return 'allowed'
        elif att['subtype'] == 'dashed_solid':
            if side == 'left':
                return 'prohibited'
            elif side == 'right':
                return 'allowed'
        elif att['subtype'] == 'solid_dashed':
            if side == 'left':
                return 'allowed'
            elif side == 'right':
                return 'prohibited'
        else:
            return ''

    elif att['type'] == 'virtual':
        return ''

    elif att['type'] == 'road_border' or att['type'] == 'guard_rail' or att['type'] == 'fence':
        return 'not_possible'

    elif att['type'] == 'keepout':
        return 'prohibited'

    elif att['type'] == 'zig-zag':
        return 'allowed'

    elif att['type'] == 'BSSD' and att['subtype'] == 'unmarked':
        return 'allowed'
