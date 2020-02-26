#! /usr/bin/env python

import roslaunch
import rospkg

class LauncherStarter:
    def __init__(self, pkg_name, launcher_name, *args):
        if not args:
            launcher_path = 'launch/'
        else:
            launcher_path = args[0]

        rospack = rospkg.RosPack()
        path = rospack.get_path(pkg_name)
        path = path + '/'+ launcher_path + launcher_name
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        self.launch.start()

    def stop(self):
        self.launch.shutdown()