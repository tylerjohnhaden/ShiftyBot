#!/usr/bin/env python

import os

from shifty_common.Mapper import Mapper

if __name__ == "__main__":
    Mapper(os.path.join(os.path.abspath(__file__), os.pardir, 'site')).run()
