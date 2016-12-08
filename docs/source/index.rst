.. VehicleModels.jl documentation master file, created by
   sphinx-quickstart on Wed Dec  7 09:57:33 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

VehicleModels.jl
================




Installation
------------

In Julia, you can install the ``VehicleModels.jl`` package by typing:
::
  Pkg.clone("https://github.com/huckl3b3rry87/VehicleModels.jl")


Examples
--------
The first example demonstrates moving obstacle avoidance for a large-sized high-speed ground vehicle.

.. toctree::
   :maxdepth: 2

   test1

Tutorial
--------

The examples for this package are available by typing:
::
 using IJulia
 notebook(dir = Pkg.dir("VehicleModels")*"/examples")

Then, the ``Three_DOF_ex.ipynb`` can be looked at!


Citation
--------
If you find this package useful, please cite this paper (currently in review):
::
  @article{Febbo2016,
  title = { Moving Obstacle Avoidance for Large, High-Speed Autonomous Ground Vehicles,
  author = {Huckleberry Febbo, Jiechao Liu, Paramsothy Jayakumar, Jeffrey L. Stein, and Tulga Ersal},
  conference = {Dynamic Systems and Control Conference},
  year = {2016},
  }
