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
# ========== AG MODIFICATION START ==========
import time
import pybullet
# ========== AG MODIFICATION END ============

import upkie.envs

upkie.envs.register()

if __name__ == "__main__":
    # ========== AG MODIFICATION START ==========
    # Define 12 gain tuning experiments
    experiments = [
        # Baseline
        {"name": "Baseline", "pitch": 10.0, "pos": 1.0, "vel": 0.1},

        # Pitch variations (fix pos=1.0, vel=0.1)
        {"name": "Low Pitch (5.0)", "pitch": 5.0, "pos": 1.0, "vel": 0.1},
        {"name": "High Pitch (15.0)", "pitch": 15.0, "pos": 1.0, "vel": 0.1},
        {"name": "Very High Pitch (20.0)", "pitch": 20.0, "pos": 1.0, "vel": 0.1},

        # Position variations (fix pitch=10.0, vel=0.1)
        {"name": "Low Position (0.5)", "pitch": 10.0, "pos": 0.5, "vel": 0.1},
        {"name": "High Position (2.0)", "pitch": 10.0, "pos": 2.0, "vel": 0.1},
        {"name": "Very High Position (5.0)", "pitch": 10.0, "pos": 5.0, "vel": 0.1},

        # Velocity variations (fix pitch=10.0, pos=1.0)
        {"name": "Low Velocity (0.05)", "pitch": 10.0, "pos": 1.0, "vel": 0.05},
        {"name": "High Velocity (0.2)", "pitch": 10.0, "pos": 1.0, "vel": 0.2},
        {"name": "Very High Velocity (0.5)", "pitch": 10.0, "pos": 1.0, "vel": 0.5},

        # Extreme tests
        {"name": "No Position (0.0)", "pitch": 10.0, "pos": 0.0, "vel": 0.1},
        {"name": "No Velocity (0.0)", "pitch": 10.0, "pos": 1.0, "vel": 0.0},
    ]

    with gym.make("Upkie-PyBullet-Pendulum", frequency=1000, gui=True) as env:
        # Access PyBullet connection from backend
        bullet_client = env.unwrapped.backend._bullet

        for exp_num, exp in enumerate(experiments, start=1):
            # Print to console
            print(f"\n{'='*60}")
            print(f"Experiment {exp_num}/{len(experiments)}: {exp['name']}")
            print(f"Gains: pitch={exp['pitch']:.2f}, pos={exp['pos']:.2f}, vel={exp['vel']:.2f}")
            print(f"Running for 5 seconds...")
            print(f"{'='*60}")

            # Display on PyBullet GUI
            text_id = pybullet.addUserDebugText(
                text=f"Exp {exp_num}/{len(experiments)}: {exp['name']}\n"
                     f"Pitch={exp['pitch']:.1f} | Pos={exp['pos']:.1f} | Vel={exp['vel']:.2f}",
                textPosition=[0, 0, 1.2],
                textColorRGB=[1, 0, 0],
                textSize=2.0,
                physicsClientId=bullet_client
            )

            # Full reset
            observation, info = env.reset()

            # Run for 5 seconds (5000 steps at 1000 Hz)
            for step in range(5000):
                pitch = observation[0]
                ground_position = observation[1]
                ground_velocity = observation[3]

                action = np.clip(
                    a=[
                        exp['pitch'] * pitch
                        + exp['pos'] * ground_position
                        + exp['vel'] * ground_velocity
                    ],
                    a_min=-0.99,
                    a_max=0.99,
                )

                observation, reward, terminated, truncated, info = env.step(action)

                if terminated or truncated:
                    observation, info = env.reset()

            # Remove text before pause
            pybullet.removeUserDebugItem(text_id, physicsClientId=bullet_client)

            # 2 second pause between experiments
            if exp_num < len(experiments):
                print(f"Pausing for 2 seconds...\n")
                time.sleep(2)

        print(f"\n{'='*60}")
        print("All experiments completed!")
        print(f"{'='*60}")
    # ========== AG MODIFICATION END ============
