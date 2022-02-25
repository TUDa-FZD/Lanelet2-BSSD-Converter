
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

BEHAVIOR_SPACE_TAGS = {"type": "behavior_space"}

BEHAVIOR_TAGS = {"type": "behavior",
                 "speed_max": "",
                 "overtake": "",
                 "speed_time_max": "",
                 "speed_time_interval": "",
                 "speed_wet_interval": ""
                 }

RESERVATION_TAGS = {"type": "reservation",
                    "reservation": "",
                    "pedestrian": "",
                    "bicycle": "",
                    "motor_vehicle": "",
                    "railed_vehicle": "",
                    "traffic_light_active": "",
                    "red_light_condition": "",
                    "turn_arrow_active": ""
                    }

BOUNDARY_LONG_TAGS = {"type": "boundary_long",
                      "crossing": "",
                      "traffic_light_active": "",
                      "red_light_condition": "",
                      "stop": "",
                      "no_stagnant_traffic": "",
                      "no_red_light": "",
                      "residents_only": "",
                      "time_interval_only": ""
                      }

BOUNDARY_LAT_TAGS = {"type": "boundary_lat",
                     "crossing": "",
                     "parking_only": ""
                     }
