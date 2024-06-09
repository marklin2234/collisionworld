# Project 2

## Performance Improvements

| Version | mit.in | koch.in |
| --- | --- | --- |
| Original | 45.905592s | 932.316011s |
| quad tree | 9.328597s | 18.491237s | 
| parallelization | 1.546889s | 3.322799s |


## Quad Tree

In my quad tree implementation, I would assign lines to quadrants if
it fully contained the parallelogram of the line and its velocity.
Otherwise, the line would be placed in the root tree. I set a max depth
and line threshold as the base case of my recursive implementation.

### Optimizations

* Jamming  
Similar to loop jamming, I calculate the number of collisions as I create my
node, comparing the current lines with its parent lines, and also comparing
the lines within each node.

* Coarsening  
I set the base case to be larger to reduce function-call overhead and reducing
excessive memory allocation when $n$ was small enough.

* Inlining  
I inlined some functions to reduce function-call overhead.

## Parallelization

I implemented 2 reducers to ensure data consistency.

1. IntersectionEventListReducer  
The `IntersectionEventListReducer` ensures the data consistency of the 
`IntersectionEventList` that is passed into `QuadTree_create` since this
function is called with `cilk_spawn`. The identity function is just
the `head` and `tail` set to `NULL`. The associative binary operation is appending
the `right` to `left`.

2. uintReducer
To ensure the data consistency of `numLineLineCollisions` within `collisionWorld`,
instead of turning `CollisionWorld` into a reducer, I made a simple uint reducer
instead for simplicity, and simply updated `numLineLineCollisions` afterwards.

For parallelization, I call `cilk_for` when checking for line intersections, since
there are no data dependencies when checking for intersections. Similarly, I call
`cilk_spawn` recursively on each child.

Overall, when implementing parallelization using openCilk, cilkscale gives us:  
```
tag,work (seconds),span (seconds),parallelism,burdened_span (seconds),burdened_parallelism
,22.2409,0.792948,28.0483,2.1343,10.4207
```

pretty good!
