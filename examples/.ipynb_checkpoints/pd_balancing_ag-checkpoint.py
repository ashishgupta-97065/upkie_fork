#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria
#
# /// script
# dependencies = ["upkie", "pybullet>=3"]
# ///

"""Try basic balancing in a PyBullet simulation."""

import gymnasium as gym
import numpy as np

import upkie.envs
import math



upkie.envs.register()

if __name__ == "__main__":
    with gym.make("Upkie-PyBullet-Pendulum", frequency=1000, gui=True) as env:
        observation, info = env.reset()
        while True:
            pitch = observation[0]
            ground_position = observation[1]
            ground_velocity = observation[3]
            action = np.clip(
                a=[
                    10.0 * pitch              # "Be upright": rads
                    + 1.0 * ground_position     # "Be at origin": m
                    + 0.1 * ground_velocity     # "Don't oscillate": m/s
                ],
                a_min=-0.99, #limit max and min speeds (capability of motor)
                a_max=0.99,
            )
            observation, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                observation, info = env.reset()

