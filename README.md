or_interactivemarker
====================

This OpenRAVE viewer plugin publishes the environment as InteractiveMarkers
that can be visualized in RViz.

## Usage ##

After an OpenRAVE environment is initialized and a roscore is running, set the 
environment to use `or_interactivemarker` as a viewer using the following command:

```python
env.SetViewer('InteractiveMarker openrave_interactivemarker')
```

The meaning of these arguments is mysterious and undocumented.  If it works correctly,
you will now see several topics being published:

```bash
$ rostopic list
[...]
/openrave/feedback
/openrave/update
/openrave/update_full
[...]
```

In an RViz instance, you should now be able to visualizat an InteractiveMarker topic
called `/openrave`.

*NOTE:*
At the moment, due to a bug in the hydro branch of `interactive_marker`, in order for `or_interactivemarker` to work, you wil need to download the indigo-dev branch of `interactive_marker` and compile it from source before building `or_interactivemarker`. See [this issue](https://github.com/personalrobotics/or_interactivemarker/issues/2).
