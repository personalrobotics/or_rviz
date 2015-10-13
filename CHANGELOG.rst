^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2015-10-12)
------------------
* Added lighting options to the in-process viewer.
* Fixed frame initialization issues with the in-process viewer.
* Added missing dependencies to package.xml.
* Added incude guards as workaround for a Moc bug (`#29 <https://github.com/personalrobotics/or_rviz/issues/29>`_)
* Contributors: Chris Dellin, Michael Koval, Matt Klingsmith 

1.0.1 (2015-06-01)
------------------
* fix frame name /map -> map to work with update
* Contributors: Clint Liddick

1.0.0 (2015-05-01)
------------------
* Changed cylinder radius to match InteractiveMarker spec.
* Added a "Hide IK Controls" option to the ghost arm
* Snap the IK handle when using "Reset DOF Values".
* Updated CHANGELOG.
* Renamed package in CHANGELOG.
* Renamed or_interactivemarker to or_rviz.
* Update the test.py script.
* Added more usage information
* Added some minor doc suggestions.
* Added a note about which methods are in-process only.
* Fleshed out the README.
* Contributors: Michael Koval, Pras, Pras Velagapudi, mklingen

* Renamed or_interactivemarker to or_rviz.
* Added a README (`#14 <https://github.com/personalrobotics/or_interactivemarker/issues/14>`_), thanks @aaronjoh for the feedback
* Contributors: Michael Koval, Pras Velagapudi, Matt Klingensmith, Aaron Johnson

0.1.0 (2015-02-10)
------------------
* line/point size now corresponds to 1mm = 1 pixel
* implemented dummy functions for pure virtuals. fixed a bug in creating manipulator markers
* Re-use existing RViz displays, when possible.
* Default to the fixed frame.
* Fixed environment changing bugs.
* Switched to using kinbody change events.
* moved InteractiveMarkerGraphHandle to a separate file.
* Fixed CreateMarker frame ID.
* Cleaned up Ogre conversion utilities.
* Ported offscreen rendering (still broken).
* Implemented GetCameraImage.
* Implemented drawarrow.
* Implemented SetBkgndColor.
* Implemented viewer image callback.
* Refactored ScopedConnection.
* Switched to the Add/Remove kinbody callbacks.
* Added stub environment change callback.
* Cleaned up EnvironmentDisplay.
* Removed duplicate files from or_rviz import.
* Removed empty Doxygen file.
* Removed Fuerte backwards compatability.
* Added license information.
* Moved ROS conversions into a separate cpp file.
* Renamed plugin file.
* Renamed viewer filenames.
* Implemented or_rviz specific viewer functions.
* Hack to avoid crashing with a NULL IK solver (`#3 <https://github.com/personalrobotics/or_rviz/issues/3>`_)
* Set uninitialized values in JointMarker
* Fixed the joint control snapping problem.
* Added support for changing the parent frame.
* Anonymize or_rviz topic names.
* Got the stripped-down RViz window to open.
* Cleaned up plugin creation.
* Revert "Removed un-used native rendering code."
  This reverts commit ad0ab5fe3642ae999d9846382d5fc15fcb92afea.
* Removed un-used native rendering code.
* or_rviz and or_interactivemarker are working together
* Split RViz headers into a separate directory.
* Refactored in preparation for RViz plugins.
* Merging or_rviz viewer class.
* Imported RViz plugins.
* Added or_rviz dependencies.
* Moved or_conversions into the util namespace.
* Moved markers into a namespace.
* Fixed some missing headers.
* Split core functionality into a library.
* Redraw markers when visibility changes.
* Fixed use of stride in drawtrimesh
* Switched to openrave_catkin.
* Added missing explicit instantiations.
* Correctly apply scale factor to OpenRAVE meshes.
* Fixed bugs in plot3 and drawlinelist.
* Implemented SetEnvironmentSync.
* Implemented the other draw() functions.
* Implemented drawlinelist
* Implemented both plot3() interfaces.
* Templated OR conversion functions on scalar type.
* Partial plot3 support.
* Clean up UserData in RemoveKinBody.
* Re-enabled free joint IK control.
* Fixed joint controls to control position.
* Switch geometry groups for the entire KinBody.
* Switch per-link geometry groups.
* Added geometry visibility settings to kinbody.
* Enabled rendering of both types of geometry.
* Split visual and render geometry flags.
* Enabled switching visual/render geometry.
* Fall back on loading TriMeshes using OpenRAVE.
* Rudimentary menu callbacks.
* Disabled the ROS SigInt handler.
* Enabled naming of custom menu options.
* Partial support for registering custom menu options.
* Fixed rendering of textured models in Hydro.
* Implemented SetActive.
* Simplified joint control logic.
* Fixed toggling of pose controls.
* Added a pose control.
* Fixed primitive geometry.
* Added missing libraries.
* Changing colors to show IK validity.
* Only recompute the IK solution when needed.
* Reverted to (buggy) free joint selection.
* Free joint control works, but is very jerky.
* Split joint controls into two classes.
* Preparing for the KinBodyJointMarker split.
* More JointMarker cleanup.
* Cleaning up joint controls.
* Implemented joint controls.
* Implemented IK toggling correctly.
* Revamped ghost manipulator.
* Retrofitting LinkMarker for ghost manipulators.
* Fixed manipulator child link logic.
* Fixed joint control toggling.
* Detect links that are part of a manipulator.
* Added (broken) joint controls.
* Working on the KinBody-level menus.
* Working on context menus.
* Cleaning up the menu.
* Cleaned up the incremental update logic.
* Revamping the ghost manipulator.
* Render simple joint controls.
* Create the ghost manipulator for IK control.
* Prototype IK controller.
* Added boilderplate for manipulator controls.
* Detect which links are part of a manipulator.
* Added (broken) callback menu.
* Added a test script.
* Switched to one marker per body.
* Render with marker -> link.
* Added the InteractiveMarker server.
* Added some marker conversion code.
* Started mocking up the classes.
* More spacing cleanup.
* Cleaned up more spacing.
* Started updating Property attributes.
* Cleaned up indention.
* Fixed SEGFAULTs in hydro.
* Commented out interactive markers.
* Fixed linking issues in hydro.
* fixing this up for groovy
* Fixed rendering when loading a single DAE file.
* Started Catkinization.
* properly setting up the intrinsics for offscreen rendering
* got a hack working for offscreen rendering. OGRE HATES THREADS
* fixed setvisible thing
* fixed remove deadlock maybe
* Removed a hack we used for the ROCK demo.
* Added a workaround to disable lighting for textures.
* Added <openrave> tag to the manifest.
* added joint controls
* better plotting
* Render the visual geometry group when it is available.
* made orrviz less verbose
* added collision mesh visualization
* trying to get it to work on hal
* no longer segfaults with roscore not running
* Fixed a typo.
* Override collision geometry if the render filename is specified.
* or_rviz is now environment based rather than kinbody based
* trying to get it to work with herbpy. failure
* added plugin description
* or_rviz works again
* renamed superviewer
* weird stuff is happening
* superviewer is now rendering stuff
* superviewer stuff
* more superviewer work. need to load iv files
* added superviewer
* Contributors: Garth Zeglin, Jennifer King, Matt Klingensmith, Michael Koval, Mike Koval, Pras Velagapudi, mklingen
