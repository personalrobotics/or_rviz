or_rviz
=======

OpenRAVE viewer plugin built using librviz. This package is *both* an openrave viewer plugin, *and* an RViz plugin. The drawing is done inside RViz using Ogre's internal draw tools and interactive markers.

To use, simply say:

    env.SetViewer('or_rviz')
    
in an openrave python script. This will spin up a copy of RViz with the `openrave` plugin enabled. OpenRAVE kinbodies are displayed in a tf frame called `/map`, and can be drawn at the same time as any other RViz display type (such as markers or point clouds).


The following things are implemented:

* Drawing and updating kinbodies
* Loading kinbodies/environments from a menu
* Switching between active environments
* Plotting lines, points, and arrows
* Setting the background color in RViz
* Rendering the screen from a given viewpoint
* Moving the RViz screen
* Object picking/moving (duplicated from `or_interactivemarker`)
* 
The following openrave viewer features are **not** implemented:

* Setting the camera position and parameters programatically
* Rendering textured planes
* Resizing the RViz screen
* Setting the title of the RViz Screen
