import pytest
import framework as fw
from lanelet2.core import AttributeMap

def test_relevance():
    attributes = AttributeMap({'type': 'lanelet', 'subtype': 'road'})
    assert fw.is_ll_relevant(attributes)
    attributes = AttributeMap({'type': 'lanelet',
                               'subtype': 'road',
                               'participant:bicycle': 'yes',
                               'participant:vehicle': 'no',
                               })
    assert not fw.ll_relevant(attributes)
