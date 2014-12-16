or_rviz
=======

OpenRAVE viewer plugin built using librviz. This package is *both* an openrave viewer plugin, *and* an RViz plugin. The drawing is done inside RViz using Ogre's internal draw tools.

To use, simply say:

    env.SetViewer('or_rviz')
    
in an openrave python script.


The following barebones things are implemented:

* Drawing and updating kinbodies
* Loading kinbodies/environments from a menu
* Switching between active environments
* Plotting lines, points, and arrows
* Setting the background color in RViz
* Rendering the screen from a given viewpoint
* Moving the RViz screen

The following openrave viewer features are **not** implemented:

* Setting the camera position and parameters programatically
* Rendering textured planes
* Setting line/point size
* Resizing the RViz screen
* Setting the title of the RViz Screen
* Object picking/moving (for that, use `or_interactivemarker`)
* Joint rotation
* Deleting graph handles
