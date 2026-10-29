#!/usr/bin/env python3
"""Plot validation figures from a ros2 bag of the controller diagnostics.

Usage:
    ros2 bag record -o runs/run1 /mevius_mpc_controller/odom /mevius_mpc_controller/diag_contact \
        /mevius_mpc_controller/diag_force /mevius_mpc_controller/diag_torques \
        /mevius_mpc_controller/diag_watchdog /mevius_mpc_controller/diag_mpc
    python3 plot_gazebo_run.py runs/run1 [out_dir]

Writes fig1_watchdog .. fig6_odom (.png) to out_dir.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

TAU_MAX = 35.0
MAX_TILT_DEG = 60.0
MAX_HEIGHT_ERR = 0.25
MPC_BUDGET_MS = 20.0


def read_bag(path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="sqlite3"), ConverterOptions("", ""))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = {name: [] for name in topics}
    while reader.has_next():
        topic, raw, t = reader.read_next()
        msg = deserialize_message(raw, get_message(topics[topic]))
        data[topic].append((t * 1e-9, msg))
    return data


def find_controller_prefix(data):
    for name in data:
        if name.endswith("/diag_watchdog"):
            return name[: -len("/diag_watchdog")]
    return None


def time_and_values(series):
    if not series:
        return np.array([]), np.zeros((0, 0))
    t = np.array([s[0] for s in series])
    t = t - t[0]
    vals = np.array([list(m.data) for _, m in series])
    return t, vals


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <bag_dir> [out_dir]")
        sys.exit(1)
    bag_dir = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else bag_dir
    os.makedirs(out_dir, exist_ok=True)

    data = read_bag(bag_dir)
    prefix = find_controller_prefix(data)
    if prefix is None:
        print("Aucun topic '.../diag_watchdog' dans ce bag -- topics presents :")
        for name in sorted(data):
            print(" ", name)
        sys.exit(1)
    print(f"prefixe controller detecte : {prefix}")

    def get(suffix):
        return data.get(prefix + suffix, [])

    # fig1: watchdog
    t, vals = time_and_values(get("/diag_watchdog"))
    if len(t):
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
        ax[0].plot(t, np.degrees(vals[:, 0]))
        ax[0].axhline(MAX_TILT_DEG, color="r", ls="--", lw=0.8)
        ax[0].set_ylabel("inclinaison (deg)")
        ax[1].plot(t, vals[:, 1])
        ax[1].axhline(MAX_HEIGHT_ERR, color="r", ls="--", lw=0.8)
        ax[1].set_ylabel("erreur hauteur (m)")
        ax[2].plot(t, vals[:, 2], drawstyle="steps-post")
        ax[2].set_yticks([0, 1, 2])
        ax[2].set_yticklabels(["stabilize", "trot", "fault"])
        ax[2].set_xlabel("t (s)")
        fig.suptitle("Watchdog")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "fig1_watchdog.png"), dpi=120)
        plt.close(fig)

    # fig2: contacts, one subplot per leg
    t, vals = time_and_values(get("/diag_contact"))
    if len(t):
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(8, 7))
        for leg in range(4):
            ax[leg].fill_between(t, vals[:, leg], step="post", alpha=0.3, label="planifie")
            ax[leg].plot(t, vals[:, 4 + leg] - 1.3, drawstyle="steps-post", label="estime")
            ax[leg].plot(t, vals[:, 8 + leg] - 2.6, drawstyle="steps-post", label="bumper brut", lw=0.8)
            ax[leg].set_ylabel(f"leg{leg + 1}")
            ax[leg].set_yticks([])
        ax[0].legend(loc="upper right", fontsize=7, ncol=3)
        ax[-1].set_xlabel("t (s)")
        fig.suptitle("Contact : planifie / estime (force+bumper+hysteresis) / bumper brut")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "fig2_contact.png"), dpi=120)
        plt.close(fig)

    # fig3: vertical forces
    t, vals = time_and_values(get("/diag_force"))
    if len(t):
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(8, 7))
        for leg in range(4):
            ax[leg].plot(t, vals[:, leg], label="commandee (u0)")
            ax[leg].plot(t, vals[:, 4 + leg], label="estimee (couple-gravite)", alpha=0.7)
            ax[leg].set_ylabel(f"leg{leg + 1} Fz (N)")
        ax[0].legend(fontsize=7)
        ax[-1].set_xlabel("t (s)")
        fig.suptitle("Force verticale au pied, par patte")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "fig3_forces.png"), dpi=120)
        plt.close(fig)

    # fig4: joint torques
    t, vals = time_and_values(get("/diag_torques"))
    if len(t):
        fig, ax = plt.subplots(figsize=(8, 4))
        for j in range(vals.shape[1]):
            ax.plot(t, vals[:, j], lw=0.6)
        ax.axhline(TAU_MAX, color="r", ls="--", lw=0.8)
        ax.axhline(-TAU_MAX, color="r", ls="--", lw=0.8)
        ax.set_ylabel("couple (N.m)")
        ax.set_xlabel("t (s)")
        fig.suptitle("Couples articulaires (12), limite TAU_MAX en rouge")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "fig4_torques.png"), dpi=120)
        plt.close(fig)

    # fig5: MPC stats
    t, vals = time_and_values(get("/diag_mpc"))
    if len(t):
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
        ax[0].plot(t, vals[:, 0], drawstyle="steps-post")
        ax[0].set_ylabel("solve ok (0/1)")
        ax[1].plot(t, vals[:, 1], drawstyle="steps-post")
        ax[1].axhline(MPC_BUDGET_MS, color="r", ls="--", lw=0.8)
        ax[1].set_ylabel("t_solve (ms)")
        ax[2].plot(t, vals[:, 2], drawstyle="steps-post")
        ax[2].set_ylabel("iterations OSQP")
        ax[2].set_xlabel("t (s)")
        fig.suptitle("Stats MPC")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "fig5_mpc.png"), dpi=120)
        plt.close(fig)

    # fig6: odometry
    odom = get("/odom")
    if odom:
        t = np.array([s[0] for s in odom])
        t = t - t[0]
        pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                         for _, m in odom])
        vel = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
                         for _, m in odom])
        fig, ax = plt.subplots(2, 1, sharex=True, figsize=(8, 5))
        ax[0].plot(t, pos[:, 0], label="x")
        ax[0].plot(t, pos[:, 1], label="y")
        ax[0].plot(t, pos[:, 2], label="z")
        ax[0].legend()
        ax[0].set_ylabel("position (m)")
        ax[1].plot(t, vel[:, 0], label="vx")
        ax[1].plot(t, vel[:, 1], label="vy")
        ax[1].plot(t, vel[:, 2], label="vz")
        ax[1].legend()
        ax[1].set_ylabel("vitesse corps (m/s)")
        ax[1].set_xlabel("t (s)")
        fig.suptitle("Odometrie ESKF (mevius_mpc_controller/odom)")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "fig6_odom.png"), dpi=120)
        plt.close(fig)

    print(f"figures ecrites dans {out_dir}")


if __name__ == "__main__":
    main()
