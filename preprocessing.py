import constants
import logging
logger = logging.getLogger(__name__)


def ll_relevant(att):
    # Determine the relevance of a lanelet by first checking its subtype (for instance: shouldn't be "stairs")
    # and second if any overriding participant-tags are being used

    if att['subtype'] in constants.SUBTYPE_TAGS:

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


def ls_of_pbl(att):
    if ('line' in att['type'] and att['subtype'] == 'dashed') or att['type'] == 'virtual':
        return True
    else:
        return False


def protected_b_lane(nbrs, bound_att):
    if nbrs and \
            ls_of_pbl(bound_att) and\
            ll_relevant(nbrs[0].attributes):
            #('line' in att['type'] and att['subtype'] == 'dashed') or att['type'] == 'virtual':
        return True
    else:
        return False
