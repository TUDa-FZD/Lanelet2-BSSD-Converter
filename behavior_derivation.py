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
             'virtual': ct.PROHIBITED,
             'unmarked': ct.ALLOWED,
             'road_border': ct.NOT_POSSIBLE,
             'guard_rail': ct.NOT_POSSIBLE,
             'fence': ct.NOT_POSSIBLE,
             'keepout': ct.PROHIBITED,
             'zig-zag': ct.ALLOWED,
             'BSSD': {'boundary': ct.ALLOWED},
             }


def distinguish_lat_boundary(att, side):
    ls_type = get_item(att, 'type')
    ls_subtype = None
    op = lane_mark.get(ls_type)
    if isinstance(op, dict):
        ls_subtype = get_item(att, 'subtype')
        op = op.get(ls_subtype)
        if isinstance(op, dict):
            op = op.get(side)
            logger.debug(f'For {ls_type}: {ls_subtype} and side {side} CrossingType {op.value} has been derived.')
        elif op:
            logger.debug(f'For {ls_type}: {ls_subtype} CrossingType {op.value} has been derived.')
    elif op:
        logger.debug(f'For {ls_type} CrossingType {op.value} has been derived.')
    if not op:
        logger.warning(f'For type: {ls_type} and subtype: {ls_subtype} CrossingType couln\'t be derived.')
    return op


def get_item(d, t):
    if t in d:
        return d[t]
    else:
        #print('problem')
        return None
