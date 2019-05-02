# Multi-CH-Constructor

A multi-threaded multi-criteria contraction hierarchy graph
creator. The general approach follows [this
paper](http://drops.dagstuhl.de/opus/volltexte/2017/7625/). But a
few improvements have been made to ensure that less shortcuts are
generated. Less shortcuts lead to faster contraction time and higher
speed-up of Dijkstra runs.

# Build
To Build Multi-CH-Constructor first clone it and change into its directory:

``` shell
git clone --recursive https://github.com/lesstat/multi-ch-constructor
cd multi-ch-constructor
```

Then create a build directory for cmake and run cmake and make.
``` shell
cmake -Bbuild
cmake --build build
```

# Usage
The main executable of Multi-CH-Constructor is ``multi-ch``. It has the following CLI options:

``` shell
$ ./build/multi-ch -h
  -h [ --help ]               Prints help message

loading options:
  -t [ --text ] arg           Load graph from text file
  -m [ --multi ] arg          Load graph from multiple files

contraction options:
  -p [ --percent ] arg (=98)  How far the graph should be contracted
  --stats                     Print statistics while contracting
  --threads arg               Maximal number of threads used

saving:
  -w [ --write ] arg          File to save graph to
```

It needs exactly one parameter of the loading category to load a
graph. The text format is described in detail
[here](https://github.com/Lesstat/cyclops/blob/master/README.md#graph-data). The
Dimension of the graph must match the DIMENSION constant in the
graph.hpp file. The application must be recompiled to contract graphs
of other dimensions.

The ``-p`` option specifies how much of the graph will be
contracted. This option can and should be given as decimal value aka
``-p 99.85``.

The ``--stats`` options prints per thread per round information of the contraction.

With the ``--threads`` option the number of threads is specified. If
left out the number of threads is determined by
``std::thread::hardware_concurrency()``

``-w`` specifies where to save the graph. The format is the same as
the text format. 


# Shortcut Reducing Improvements

- Duplicate shortcuts (source target and cost vector) are delete after
  every contraction round.
- Augmented LP:

    The original paper proposes the following LP for finding a
    configuration the certifies the need for a shortcut.
    When checking path p~, for all paths p found form s to t which are not
    p~ introduce a constraint:
    
    ```
    alpha^T * (c(p) - c(p~)) <= 0
    ```
    This does not work when alpha^T \* c(p) == alpha^T \* c(p~). Therefore
    the following LP is used
    ```
    max delta
    alpha^T * (c(p) - c(p~)) + delta  <= 0
    ```
- More than one path with minimal cost:
 
   When delta = 0 after the LP runs and more than one path has the
   same cost as the shortcut a heuristic search is done to find if one
   of the paths has no node that is contracted in this route
   inside. If one such path is found, it certifies that no shortcut is needed.




