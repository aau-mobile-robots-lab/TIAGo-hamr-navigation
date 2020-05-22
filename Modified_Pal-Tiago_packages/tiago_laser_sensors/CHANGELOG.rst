^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_laser_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2019-09-23)
------------------

2.0.2 (2019-09-18)
------------------

2.0.1 (2019-09-12)
------------------
* Merge branch 'velodyne' into 'erbium-devel'
  added launch file for velodyne laser
  See merge request robots/tiago_navigation!50
* added launch file for velodyne laser
* Contributors: Sai Kishor Kothakota, Victor Lopez

2.0.0 (2019-06-17)
------------------

1.0.7 (2019-03-22)
------------------
* Merge branch 'fix-tf2-compatibility' into 'erbium-devel'
  tf2 compliance
  See merge request robots/tiago_navigation!47
* tf2 compliance
* Revert "tf2 compliance"
  This reverts commit 5c55400ac1c41562a7bc5b5b6089fdca290987f6.
* tf2 compliance
* Contributors: Procópio Stein, artivis

1.0.6 (2019-03-15)
------------------

1.0.5 (2019-02-13)
------------------
* reduced laser FOV
* removed rplidar
* Contributors: Procópio Stein

1.0.4 (2019-02-05)
------------------

1.0.3 (2019-01-24)
------------------

1.0.2 (2018-12-21)
------------------
* rgbd is started on it's own
* Contributors: Victor Lopez

1.0.1 (2018-12-20)
------------------

1.0.0 (2018-12-19)
------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Specifics refactor
  See merge request robots/tiago_navigation!42
* Add rgbd cloud
* Expand filter parameters for each laser
* Rename laser sensors to the new standard
* Contributors: Victor Lopez

0.12.11 (2018-10-26)
--------------------

0.12.10 (2018-09-28)
--------------------

0.12.9 (2018-09-26)
-------------------
* 0.12.8
* changelog
* Contributors: Procópio Stein

0.12.7 (2018-09-17)
-------------------

0.12.6 (2018-07-23)
-------------------

0.12.5 (2018-04-24)
-------------------

0.12.4 (2018-03-21)
-------------------

0.12.3 (2018-03-08)
-------------------
* Merge branch 'restore-old-hokuyo-node' into 'dubnium-devel'
  Revert "replaced hokuyo_node with urg_node"
  See merge request robots/tiago_navigation!28
* Revert "replaced hokuyo_node with urg_node"
  This reverts commit b6568ab624b817325150434d7fecf7442e8fdfa2.
* Contributors: Procópio Stein, Victor Lopez

0.12.2 (2018-02-15)
-------------------

0.12.1 (2018-02-02)
-------------------

0.12.0 (2018-02-01)
-------------------
* Merge branch 'urg-node-driver' into 'dubnium-devel'
  replaced hokuyo_node with urg_node
  See merge request robots/tiago_navigation!25
* replaced hokuyo_node with urg_node
* Contributors: Procópio Stein

0.11.5 (2018-01-11)
-------------------

0.11.4 (2017-11-27)
-------------------

0.11.3 (2017-11-07)
-------------------

0.11.2 (2017-11-07)
-------------------

0.11.1 (2017-11-02)
-------------------

0.11.0 (2017-10-17)
-------------------

0.10.2 (2017-09-19)
-------------------

0.10.1 (2017-08-09)
-------------------
* fixed typo in robot name
* cosmetic (changed node name to normalize with pmb2)
* added hokuyo scan_raw remap
* filter node in base_laser.launch
* increased fov and activated intensity
* Contributors: Procópio Stein

0.10.0 (2017-05-30)
-------------------

0.9.15 (2017-05-08)
-------------------

0.9.14 (2017-05-05)
-------------------

0.9.13 (2017-05-04)
-------------------
* removed pointcloud_to_laserscan entries and files
  the pointcloud to laserscan files were moved to specific tools
  they will be available only if advanced navigation is active
* added launch and config for rgbd_scan
* reduced max rot vel and adde time offset for all laser configs
* added footprint laser filter
* Allow multiple Tiagos on a single Gazebo
* Contributors: Procópio Stein, davidfernandez

0.9.12 (2016-12-21)
-------------------
* add footprint filter
* Contributors: Jordi Pages

0.9.11 (2016-10-27)
-------------------
* updated dependencies, updated laser filter, normalized config files, updated pc2ls launch and config
* Contributors: Procópio Stein

0.9.10 (2016-10-25)
-------------------

0.9.9 (2016-10-21)
------------------
* fixe RGBD laser-scan frame. Refs #14514
* Contributors: Jordi Pages

0.9.8 (2016-07-28)
------------------

0.9.7 (2016-06-22)
------------------

0.9.6 (2016-06-15)
------------------

0.9.5 (2016-06-10)
------------------
* fix hokuyo port accordingly to new dev rule
* Contributors: Jeremie Deray

0.9.4 (2016-03-30)
------------------
* tiago default laser sick tim 561
* fix lasers launch
* laser launch set laser param
* update maintainer
* new laser launch
* add laser_filter conf
* rm rebujito laser
* add lasers sick 561 571
* Contributors: Jeremie Deray

0.9.3 (2015-04-14)
------------------
* Set hokuyo laser
* Contributors: Enrique Fernandez

0.9.2 (2015-01-20)
------------------

0.9.1 (2015-01-20)
------------------
* renames to tiago (TiaGo)
* Contributors: enriquefernandez
