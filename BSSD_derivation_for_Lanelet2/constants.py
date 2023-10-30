from bssd.core._types import CrossingType as ct


SUBTYPE_TAGS = ["road",
                 "highway",
                 "play_street",
                 "emergency_lane",
                 "bus_lane"
                 ]

OVERRIDE_TAGS = ["pedestrian",
                 "bicycle"
                 ]

LONG_BDR_TAGS = ['stop_line',
                 'pedestrian_marking',
                 'zebra_marking'
                 ]

LONG_BDR_DICT = {'exact': [None],
                 'protruding': [None, None],
                 'insufficient_half_left': [None, None],
                 'insufficient_half_right': [None, None],
                 'free': [None, None]
                 }

SEPARATION_TAGS = ['fence',
                   'curbstone',
                   'guard_rail',
                   'road_boarder',
                   'wall'
                   ]

RELEVANT_BICYCLE_TAGS = ['line_thin',
                         'line_thick',
                         'virtual',
                         'bike_marking',
                         'keepout',
                         'unmarked'
                         ]

LINE = {'solid': ct.PROHIBITED,
        'solid_solid': ct.PROHIBITED,
        'dashed': ct.ALLOWED,
        'dashed_solid': {'left': ct.PROHIBITED, 'right': ct.ALLOWED},
        'solid_dashed': {'left': ct.ALLOWED, 'right': ct.PROHIBITED}
        }

LANE_MARK = {'curbstone': {'low': ct.PROHIBITED,
                           'high': ct.NOT_POSSIBLE
                           },
             'line_thick': LINE,
             'line_thin': LINE,
             'virtual': ct.PROHIBITED,
             'unmarked': ct.ALLOWED,
             'road_border': ct.NOT_POSSIBLE,
             'guard_rail': ct.NOT_POSSIBLE,
             'fence': ct.NOT_POSSIBLE,
             'wall': ct.NOT_POSSIBLE,
             'keepout': ct.PROHIBITED,
             'zig-zag': ct.ALLOWED,
             'BSSD': {'boundary': ct.ALLOWED},
             'bike_marking': ct.ALLOWED
             }

