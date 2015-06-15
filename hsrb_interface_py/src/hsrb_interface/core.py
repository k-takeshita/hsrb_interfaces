#!/usr/bin/env python
# vim: fileencoding=utf-8


class NoSuchDeviceError(Exception):
    u"""無効なデバイス名が指定された"""
    pass

class Base(object):
    def __init__(self):
        pass
