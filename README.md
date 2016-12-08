# VehicleModels.jl

[![Latest](https://img.shields.io/badge/docs-latest-blue.svg)](http://vehiclemodelsjl.readthedocs.io/en/latest/)

NOTE: This package is a work in progress!


## Documentation

The full documentation can be found [here](http://vehiclemodelsjl.readthedocs.io/en/latest/).

## Installation

In Julia, you can install the VehicleModels.jl package by typing:
```julia
julia>Pkg.clone("https://github.com/huckl3b3rry87/VehicleModels.jl")
```

## Tutorial

The example for this package are available by typing:
```julia
julia>using IJulia
julia>notebook(dir = Pkg.dir("VehicleModels")*"/examples")
```


## Citation

If you find this package useful, please cite this paper (currently in review):

    @article{Febbo2016,
    title = { Moving Obstacle Avoidance for Large, High-Speed Autonomous Ground Vehicles,
    author = {Huckleberry Febbo, Jiechao Liu, Paramsothy Jayakumar, Jeffrey L. Stein, and Tulga Ersal},
    conference = {Dynamic Systems and Control Conference},
    year = {2016},
    }
