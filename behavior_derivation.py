import lanelet2


def derive_behavior(bs, lanelet, map_lanelet):
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)

    a_layer = map_lanelet.areaLayer

    bs.alongBehavior.tags['speed_limit'] = str(round(traffic_rules.speedLimit(lanelet).speedLimit))

    bs = derive_behavior_bdr_lat(bs, a_layer, 'left')
    bs = derive_behavior_bdr_lat(bs, a_layer, 'right')

    return bs


def derive_behavior_bdr_lat(behavior_space, area_layer, side):
    behavior_space.alongBehavior.leftBound.tags['crossing'] = \
        behavior_space.againstBehavior.rightBound.tags['crossing'] = \
        distinguish_lat_boundary(behavior_space.alongBehavior.leftBound.lineString.attributes, side)

    area_list = area_layer.findUsages(behavior_space.alongBehavior.leftBound.lineString)
    parking_only = 'False'
    if any(item.attributes['subtype'] == 'parking' for item in area_list):
        area_id = area_list[0].id
        parking_only = 'True'

    behavior_space.alongBehavior.leftBound.tags['parking_only'] = \
        behavior_space.againstBehavior.rightBound.tags['parking_only'] = parking_only

    return behavior_space


def distinguish_lat_boundary(att, side):
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
