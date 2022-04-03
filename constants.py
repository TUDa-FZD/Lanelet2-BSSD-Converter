
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
                 'insufficient_full': [None, None]
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

