# or_interactivemarker #

**or_interactivemarker** is an OpenRAVE plugin that provides two ROS-friendly OpenRAVE
viewers that publish an OpenRAVE environment as
[interactive markers](http://wiki.ros.org/interactive_markers)
to be visualized in
[RViz](http://wiki.ros.org/rviz). See below for more information.


## Installation ##

> :warning: **Note:** The version of [interactive_markers](http://wiki.ros.org/interactive_markers)
> shipped with versions of ROS before indigo (e.g. groovy and hydro) have a
> [known issue](https://github.com/ros-visualization/interactive_markers/issues/18)
> that will cause OpenRAVE to SEGFAULT when the viewer is created. This issue is fixed
> [in a pull request](https://github.com/ros-visualization/interactive_markers/pull/19),
> but has not propagated to the official ROS Debian packages. **If you are using
> ROS Groovy or Hydro, you must checkout and build the `indigo-devel` branch of
> `interactive_markers`.**

**or_interactivemarker** is a ROS package that can be installed into any ROS catkin workspace.  It requires the helper package `openrave_catkin` to be automatically loaded into OpenRAVE on startup.

## Usage: Out-of-Process Viewer ##

The **`InteractiveMarker` viewer** publishes the environment to a specified
topic to be visualized by an external RViz process. As a result, this viewer is
extremely lightweight and adds negligibly to the OpenRVE startup time. Since
rendering occurs in a separate process, some of methods exposed by the OpenRAVE
`ViewerBase` interface are not supported.

Once an OpenRAVE environment `env` is created, you can attach the
`InteractiveMarker` publisher to it using the following command:

```python
env.SetViewer('InteractiveMarker')
```

This will publish the OpenRAVE environment `env` as interactive markers on the
`openrave` ROS namespace. You can view these markers in RViz by opening an
external RViz instance (e.g. `rosrun rviz rviz`) and creating an
`InteractiveMarkers` display component that subscribes to the
`/openrave/update` topic. Note that **you must manually create and enable this
display component to view the OpenRAVE environment.**

Note that **a ROS core must be running** for the viewer to function.
Additionally, the following `ViewerBase` methods are not implemented when
running with an out-of-process RViz instance:

- `SetBkgndColor`, `SetSize`, and `Move`
- `GetName` and `SetName`
- `SetCamera`, `GetCameraTransform`, and `GetCameraIntrinsics`
- `GetCameraImage` and `RegisterViewerImageCallback`


## Usage: In-Process Viewer ##
The **`RViz` viewer**  uses librviz and Qt to instantiate RViz in the same
process as OpenRAVE. This RViz instance is automatically configured to display
the OpenRAVE environment and supports the full range of methods exposed by the
`ViewerBase` interface. Additionally, the `RViz` viewer allows you to interact
with the OpenRAVE environment through a custom RViz display plugin; e.g. to
change which OpenRAVE environment is being displayed, and to
load additional OpenRAVE environments and objects.

Once an OpenRAVE environment `env` is created, you can attach an in-process
viewer to it using the following command:

```python
env.SetViewer('RViz')
```

This will open an RViz window and automatically create two display components:

1. An "OpenRAVE Markers" `InteractiveMarker` display that is subscribing to an
   automatically generated topic name.
2. An "OpenRAVE Environment" custom display with a few configuration options:
    - Mapping between the OpenRAVE "world frame" and a TF frame (default: the
      fixed frame in RViz)
    - Current environment being displayed (default: environment with the lowest
      ID when the viewer is created)
    - List of `KinBody`s and `Robot`s in the environment

Note that **a ROS core must be running** for the viewer to function. This is
unfortunate, because both OpenRAVE and the RViz window are running in the same
process. Unfortunately, this is a fundamental limitation inherited from the
design of `librviz`.

The following methods are implemented, but have not yet been fully tested:

- `GetCameraTransform`, and `GetCameraIntrinsics`
- `GetCameraImage` and `RegisterViewerImageCallback`

The following functions are unimplemented:

- `SetCamera` (due to an internal limitation of `librviz`)


## Frequently Asked Questions ##

You may get the following error message when running a standalone RViz instance
after using the in-process viewer:

> [ERROR] PluginlibFactory: The plugin for class
> 'or_interactivemarker::rviz::EnvironmentDisplay' failed to load.  Error:
> Could not find library corresponding to plugin
> or_interactivemarker::rviz::EnvironmentDisplay. Make sure the plugin
> description XML file has the correct name of the library and that the library
> actually exists.

This occurs if save your RViz configuration (i.e. `.vcg` file) from the
in-process `RViz` viewer, then load it in a standalone RViz process. This
configuration contains a custom OpenRAVE `EnvironmentDisplay` component that
can only be constructed when running in the same process as OpenRAVE. You can
prevent this error from printing in the future by deleting the offending
`EnvironmentDisplay` component from RViz and re-saving your configuration.

We are interested in finding a more elegant workaround for this problem. Please
[open an issue](https://github.com/personalrobotics/or_interactivemarker/issues) or
[send us a pull request](https://github.com/personalrobotics/or_interactivemarker/compare)
if you have any suggestions.


## License ##

or_interactivemarker is licensed under a BSD license. See [LICENSE](LICENSE) for more
information.


## Contributors ##

or_interactivemarker is developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the [Robotics
Institute](https://www.ri.cmu.edu) at [Carnegie Mellon
University](http://www.cmu.edu). The out-of-process `InteractiveMarker` viewer
was originally developed by
[Michael Koval](https://github.com/mkoval)
and the in-process `RViz` viewer was originally developed by
[Matt Klingensmith](https://github.com/mklingen).

This is a non-exhaustive list of contributors:
- [Michael Koval](https://github.com/mkoval)
- [Matt Klingensmith](https://github.com/mklingen)
- [Pras Velagapudi](https://github.com/psigen)
- [Jen King](https://github.com/jeking04)

