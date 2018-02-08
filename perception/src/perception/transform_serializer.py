#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tft

# Does the transformation described in Lab 29: "transform arithmetic"
# Requires tag1 and wrist locations at time 1, and the tag1 location at
# time 2.
#
# This is reusable, though not named that way. This can be used any time that
# you have two things with a known initial offset, and know where one thing
# adjusts.
def transform_to_t2(B_T_tag1_t1, B_T_tag1_t2,B_T_wrist_t1):
    return np.dot(np.linalg.inv(B_T_tag1_t2),np.dot(B_T_tag1_t1,B_T_wrist_t1))
