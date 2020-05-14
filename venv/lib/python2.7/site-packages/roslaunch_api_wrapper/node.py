#! /usr/bin/env python

import roslaunch

class NodeStarter:
    def __init__(self, pkg_name, node_type):
        node_name = node_type
        node_name = node_name.strip('.py')
        node = roslaunch.core.Node(pkg_name,node_type,node_name,output='screen')
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self.process = launch.launch(node)

    def stop(self):
        self.process.stop()