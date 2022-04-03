import constants
import logging
logger = logging.getLogger(__name__)


def is_ll_relevant(att):
    # Determine the relevance of a lanelet by first checking its subtype (for instance: shouldn't be "stairs")
    # and second if any overriding participant-tags are being used

    if att['subtype'] in constants.SUBTYPE_TAGS or ('relevant_bicycle_lane' in att and att['relevant_bicycle_lane'] == 'yes'):

        if any('participant' in key.lower() for key, value in att.items()):

            if any(value == 'yes' for key, value in att.items() if 'participant:vehicle' in key.lower()):
                relevant = True
            else:
                relevant = False

        else:
            relevant = True

    else:
        relevant = False

    return relevant


def is_bicycle_ll_relevant(nbrs, bound_att):
    if nbrs and any(nbr for nbr in nbrs if is_ll_relevant(nbr.attributes))\
            and bound_att['type'] in constants.RELEVANT_BICYCLE_TAGS:
            #('line' in att['type'] and att['subtype'] == 'dashed') or att['type'] == 'virtual':
        return True
    else:
        return False
