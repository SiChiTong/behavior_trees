#!/usr/bin/env python
import core
import composites
import decorators

# Here we define the 'default' composites and actions.
# These default classes will be used when parsing a behavior tree.
from actions import Failer, Runner, Succeeder, Wait
from composites import Selector, Sequence
from decorators import Limiter, Repeater


# Aliases for compatibility with the online editor: http://editor.behavior3.com/
Priority = Selector
Error = Failer
