#!/usr/bin/env python
import core
import composites
import decorators
# For the creating of the behavior tree, we need the module behave to have
# the composites and the default actions
from composites import Selector, Sequence
from actions import Failer, Runner, Succeeder

# Alias so that we can parse files directly generated here: http://editor.behavior3.com/
Priority = Selector
