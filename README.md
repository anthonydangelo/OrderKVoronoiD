# Order-K Voronoi Diagram (OKVD)

Computes the **k-th order Voronoi diagram** of a set of 2D points and outputs
the resulting arrangement (clipped to a bounding box) as a JSON object on stdout.

## Context

This tool was built as a companion to a 1-Steiner tree project implementing the
Georgakopoulos–Papadimitriou O(n²) 1-Steiner tree algorithm
(Georgakopoulos & Papadimitriou, 1987).
That project includes an interactive
website (using GeoGebra) where the user selects input points and the 1-Steiner
point is computed and displayed. This program was added to the website to
compute and display the k-th order Voronoi diagram of the same input points,
with the aim of analyzing the 1-Steiner point solution in the context of the
Voronoi structure.

See [OneSteinerTreeProblem](https://github.com/anthonydangelo/OneSteinerTreeProblem) for the 1-Steiner tree implementation.

The website is currently offline.

**Reference:** Georgakopoulos, G. K., & Papadimitriou, C. H. (1987). The 1-Steiner Tree Problem. *Journal of Algorithms*, 8(1), 122–130.

### What this project contributes

- A standalone CLI tool (C++11) that wraps the CGAL k-order Delaunay/Voronoi
  algorithm with JSON output suitable for programmatic consumption by a web frontend.   

### Concurrency

Both this program and the 1-Steiner tree program are computationally intensive,
and the website is not expected to handle significant traffic. To prevent the
server from being overwhelmed by concurrent requests (whether legitimate or
not), the program acquires a blocking `flock` on a shared lock file before
doing any work, serializing execution to one request at a time.

The mutex can be disabled with `--no-mutex`, in which case the caller (e.g.,
the website's server code) is responsible for serializing invocations. This is
useful because a server-level lock can detect and handle the case where the
server kills a timed-out HTTP request but the underlying process is still
running — a scenario in which a process-level `flock` would remain held and
block all subsequent requests until the orphaned process eventually exits.   


## What is a k-th order Voronoi diagram?

The standard (1st-order) Voronoi diagram partitions the plane into regions
closest to each site. The **k-th order** Voronoi diagram partitions the plane
into regions where each region is the set of points whose *k nearest* sites
are a fixed k-subset. It is the dual of the k-th order Delaunay triangulation.

## Algorithm

The program computes a weighted point for each k-subset of the input points
(centroid position, weight = −Σd²/k²), builds the regular (power) triangulation
of those weighted points, and outputs its dual (the k-th order Voronoi diagram)
clipped to a bounding box.

Specialized paths exist for k=1, k=2, k=3, and k=n−1; general k uses
brute-force enumeration of all C(n,k) subsets.

**Reference:** The k-th order Delaunay/Voronoi algorithm and its implementation are from the CGAL
`multi_delaunay` ipelet demo (S. Loriot, S. Pion, N. Carrez,
INRIA Sophia-Antipolis, 2005–2009).

## Build

### Dependencies

- g++ (C++11)
- GMP (required by CGAL)
- MPFR (required by CGAL)
- Boost — thread, system (required by CGAL)
- CGAL 4.11.2 — both `include/CGAL/` (headers) and `lib/` (compiled `libCGAL.so`, `libCGAL_Core.so`) must be available. See [CGAL's installation docs](https://doc.cgal.org/4.11.2/Manual/manual/html/Installation.html) for build instructions.

### Compile

```bash
g++ -O2 -frounding-math -std=c++11 -pthread \
    src/OrderKVoronoiD.cpp src/printFormattingUtilFunctions.cpp src/utilityFunctions.cpp \
    -o OrderKVoronoiD \
    -static \
    -L/path/to/CGAL-4.11.2/lib \
    -lCGAL_Core -lCGAL -lmpfr -lgmp -lboost_thread -lboost_system -lpthread
```

For a debug build, replace `-O2` with `-g` and use the debug libraries (`-lCGAL_Core_d -lCGAL_d`).

## Usage

```bash
./OrderKVoronoiD -i "[[1,0],[2,0],[3,0],[1,2],[2,2]]" -o 2
```

| Option | Description |
|--------|-------------|
| `-i`, `--input-list` | Input points as a JSON 2D array: `[[x1,y1],[x2,y2],...]` |
| `-o`, `--order` | Order k (1 ≤ k < n). Default: 1 |
| `--no-mutex` | Skip the flock-based resource mutex |
| `-h`, `--help` | Show help |

### Output format

```json
{
  "order": "2",
  "voronoiDiagram": {
    "voronoiPoints": [
      {"x":"1.5", "y":"0.0"},
      ...
    ],
    "voronoiEdgesPtIndices": [
      {"voronoiEdge": {"firstIndex":"0", "secondIndex":"1"}},
      ...
    ]
  },
  "elapsedTime": {"s":"0", "m":"0", "h":"0"}
}
```

Edges reference points by index into `voronoiPoints`.


## License

This project is distributed under the **GNU Lesser General Public License v3**
(inherited from CGAL).

### Disclaimer

This software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose, and non-infringement. In no event shall the
author be liable for any claim, damages, or other liability arising from, out
of, or in connection with the software or the use or other dealings in the
software.   

### Attribution

The k-th order Delaunay/Voronoi algorithm is derived from the CGAL
`multi_delaunay` ipelet demo by Sébastien Loriot, Sylvain Pion, and
Nicolas Carrez (INRIA Sophia-Antipolis, 2005–2009). See:
https://github.com/CGAL/cgal/blob/v4.11.2/CGAL_ipelets/demo/CGAL_ipelets/multi_delaunay.cpp
The `k_delaunay.h` header (general k-order Delaunay construction) is also from the same demo:
https://github.com/CGAL/cgal/blob/v4.11.2/CGAL_ipelets/demo/CGAL_ipelets/include/CGAL_ipelets/k_delaunay.h   

CGAL is available at https://www.cgal.org
