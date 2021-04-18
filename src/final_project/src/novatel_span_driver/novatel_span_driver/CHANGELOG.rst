^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_span_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2018-04-26)
------------------
* Merge pull request `#27 <https://github.com/ros-drivers/novatel_span_driver/issues/27>`_ from michaelhsmart/ellipsoidal_from_orthometric
  Fix orthometric to ellipsoidal conversion
  Merged based on recommendations from @mikepurvis and @icolwell.
* Deleted extra ')' (`#31 <https://github.com/ros-drivers/novatel_span_driver/issues/31>`_)
* Fix ellipsoidal altitude to use correct conversion from orthometric + undulation.
* Merge pull request `#10 <https://github.com/ros-drivers/novatel_span_driver/issues/10>`_ from prclibo/fix_rpy
  possibly fixed rpy interpretation
* Merge pull request `#20 <https://github.com/ros-drivers/novatel_span_driver/issues/20>`_ from astuff/master
  Adding new maintainers.
* Latch navsat/origin so that it is available to nodes that start after the NovAtel driver (`#12 <https://github.com/ros-drivers/novatel_span_driver/issues/12>`_)
* Fix bad == when publishing IMU message (`#19 <https://github.com/ros-drivers/novatel_span_driver/issues/19>`_)
* Merge pull request `#9 <https://github.com/ros-drivers/novatel_span_driver/issues/9>`_ from lemiant/patch-1
  Update diagnostics.py
  diagnostic_msgs has not been imported, must use DiagnosticStatus directly.
* Contributors: AnkilP, Joshua Whitley, Michael Smart, Mike Purvis, P. J. Reed, lemiant, libo24

1.0.0 (2014-11-25)
------------------
* Add capture from INS PPP, default value for IMU rate, fix test, add docs to example launcher.
* Throttle wheelvelocity commands to 1Hz, as recommended by Novatel.
* Fix velx/y mixup, add diagnostic publisher.
* BESTPOSB -> BESTPOS, add diagnostic constants and shim class.
* Treat altitude as positive-up.
* Update messages to include header, fix header to match documentation.
* Add wheel velocity handler.
* Contributors: Mike Purvis

0.2.0 (2014-10-29)
------------------
* Re-work the publisher class.
* Add a pcap for a SPAN fix.
* Tweaks to topic names.
* Add working rostest, fixes from pepify.
* Add roslaunch file check for example launcher.
* Add roslint test.
* Contributors: Mike Purvis

0.1.0 (2014-10-24)
------------------
* Consolidation and reorganization of novatelcentral driver prepared by NovAtel
  and published here: https://github.com/NovAtel-inc/rosnovatel
* Contributors: Mike Purvis
