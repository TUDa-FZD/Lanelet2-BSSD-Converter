import lanelet2
from preprocessing import ll_relevant
import constants
import logging
from bssd.core._types import CrossingType as ct

logger = logging.getLogger('framework.behavior_derivation')

LINE = {'solid': ct.PROHIBITED,
        'solid_solid': ct.PROHIBITED,
        'dashed': ct.PROHIBITED,
        'dashed_solid': {'left': ct.PROHIBITED, 'right': ct.ALLOWED},
        'solid_dashed': {'left': ct.ALLOWED, 'right': ct.PROHIBITED}
        }

lane_mark = {'curbstone': {'low': ct.ALLOWED,
                           'high': ct.NOT_POSSIBLE
                           },
             'line_thick': LINE,
             'line_thin': LINE,
             'virtual': None,
             'unmarked': ct.ALLOWED,
             'road_border': ct.NOT_POSSIBLE,
             'guard_rail': ct.NOT_POSSIBLE,
             'fence': ct.NOT_POSSIBLE,
             'keepout': ct.PROHIBITED,
             'zig-zag': ct.ALLOWED,
             'BSSD': {'boundary': ct.ALLOWED},
             }


def distinguish_lat_boundary(att, side):
    if 'type' in att:
        # op = lane_mark[att['type']]
        op = check_existence(lane_mark, att['type'])
        if isinstance(op, dict):
            if 'subtype' in att:
                # op = op[att['subtype']]
                op = check_existence(op, att['subtype'])
                if isinstance(op, dict):
                    op = op[side]
                    logger.debug(f'For ' + att['type'] + ' ' + att['subtype'] + f' CrossingType {op.value} has been derived.')
                else:
                    # logger.debug(f'For ' + att['type'] + ' ' + att['subtype'] + f' CrossingType {op.value} has been derived.')
                    pass
            else:
                logger.debug(
                    f'Tag \'subtype\' was not found in linestring-attributes. CrossingType derivation not possible.')
                return None
        elif op:
            logger.debug(f'For ' + att['type'] + f' CrossingType {op.value} has been derived.')
        else:
            logger.debug(f'Not tracked/Non-existent linestring key. CrossingType derivation failed.')
    else:
        logger.warning(f'Tag \'type\' was not found in linestring-attributes. CrossingType derivation not possible.')
        return None

    return op

def distinguish_lat_boundary_3(att, side):
    tp = att.get('type')

    if tp:
        op = lane_mark.get(tp)
        if isinstance(op, dict):
            stp = att.get('type')
    else:
        # type not existent
        pass


    if 'type' in att:
        # op = lane_mark[att['type']]
        op = check_existence(lane_mark, att['type'])
        if isinstance(op, dict):
            if 'subtype' in att:
                # op = op[att['subtype']]
                op = check_existence(op, att['subtype'])
                if isinstance(op, dict):
                    op = op[side]
                    logger.debug(f'For ' + att['type'] + ' ' + att['subtype'] + f' CrossingType {op.value} has been derived.')
                else:
                    # logger.debug(f'For ' + att['type'] + ' ' + att['subtype'] + f' CrossingType {op.value} has been derived.')
                    pass
            else:
                logger.debug(
                    f'Tag \'subtype\' was not found in linestring-attributes. CrossingType derivation not possible.')
                return None
        elif op:
            logger.debug(f'For ' + att['type'] + f' CrossingType {op.value} has been derived.')
        else:
            logger.debug(f'Not tracked/Non-existent linestring key. CrossingType derivation failed.')
    else:
        logger.warning(f'Tag \'type\' was not found in linestring-attributes. CrossingType derivation not possible.')
        return None

    return op


def distinguish_lat_boundary_2(att, key):
    if key in att:
        if att[key] in lane_mark:
            op = lane_mark[att[key]]
        #op = check_existence(lane_mark, att[key])
        if isinstance(op, dict):
            if 'subtype' in att:
                # op = op[att['subtype']]
                op = check_existence(op, att['subtype'])


    return op


def check_existence(d, t):
    if t in d:
        return d[t]
    else:
        #print('problem')
        return None



# def distinguish_lat_boundary(att, side):
#     if 'type' not in att:
#         return ''
#
#     if att['type'] == 'curbstone':
#         if 'subtype' not in att:
#             return 'not_possible'
#         elif att['subtype'] == 'low':
#             return 'allowed'
#         elif att['subtype'] == 'high':
#             return 'not_possible'
#
#     elif att['type'] == 'line_thin' or att['type'] == 'line_thick':
#         if 'subtype' not in att:
#             return 'prohibited'
#         elif att['subtype'] == 'solid':
#             return 'prohibited'
#         elif att['subtype'] == 'solid_solid':
#             return 'prohibited'
#         elif att['subtype'] == 'dashed':
#             return 'allowed'
#         elif att['subtype'] == 'dashed_solid':
#             if side == 'left':
#                 return 'prohibited'
#             elif side == 'right':
#                 return 'allowed'
#         elif att['subtype'] == 'solid_dashed':
#             if side == 'left':
#                 return 'allowed'
#             elif side == 'right':
#                 return 'prohibited'
#         else:
#             return ''
#
#     elif att['type'] == 'virtual':
#         return ''
#
#     elif att['type'] == 'road_border' or att['type'] == 'guard_rail' or att['type'] == 'fence':
#         return 'not_possible'
#
#     elif att['type'] == 'keepout':
#         return 'prohibited'
#
#     elif att['type'] == 'zig-zag':
#         return 'allowed'
#
#     elif att['type'] == 'BSSD' and att['subtype'] == 'unmarked':
#         return 'allowed'
