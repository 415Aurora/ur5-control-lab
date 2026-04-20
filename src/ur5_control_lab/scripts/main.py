#!/usr/bin/env python3

import sys

import rospy


def main() -> int:
    try:
        from ur5_control_lab.gui.app import UR5LabApp
    except ModuleNotFoundError as exc:
        print(
            "Missing Python dependency while starting UR5 Control Lab: "
            f"{exc.name}. Install the local requirements before launching.",
            file=sys.stderr,
        )
        return 1

    try:
        app = UR5LabApp()
        app.mainloop()
    except rospy.ROSInterruptException:
        pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
