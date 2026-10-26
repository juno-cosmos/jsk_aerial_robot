#!/usr/bin/env python
# -*- coding: utf-8 -*-

from IPython import embed
from orochi_hx.robot_interface import OrochiHxInterface

if __name__ == "__main__":
    ri = OrochiHxInterface(init_node=True)
    embed()
