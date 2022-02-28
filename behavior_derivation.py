import lanelet2
from preprocessing import ll_relevant
import constants
import logging
logger = logging.getLogger(__name__)


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
