#!/usr/bin/env python
# coding=utf-8
"""
cmd lib

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

def _sign(data):
    if data > 0:
        return -1
    elif data < 0:
        return 1
    else:
        return 0