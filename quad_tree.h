#ifndef _QUAD_TREE_H_
#define _QUAD_TREE_H_

#include "collision_world.h"
#include "intersection_event_list.h"
#include "line.h"
#include "stdbool.h"
#include "vec.h"

#define LINE_THRESHOLD 50
#define NUM_CHILDREN 4
#define NUM_POINTS 9
#define MAX_DEPTH 75

struct QuadTree {
  struct QuadTree* parent;
  struct QuadTree** children;

  Line** lines;
  unsigned int numOfLines;
  unsigned int lineCapacity;

  Vec tl;
  Vec br;
  unsigned int depth;
};

typedef struct QuadTree QuadTree;

QuadTree *QuadTree_initialize(QuadTree *parent, Vec tl, Vec tr, unsigned int depth);

QuadTree *QuadTree_create(CollisionWorld *collisionWorld,
                          IntersectionEventListReducer *intersectionEventList,
                          Line **lines, unsigned int numOfLines,
                          QuadTree *parent, Vec tl, Vec br,
                          uintReducer *numCollisions);

void QuadTree_destroy(QuadTree *quad_tree);

inline bool isLineInRect(Line *line, Vec tl, Vec br);

Vec **getPoints(Vec tl, Vec br);

void detect_collision(CollisionWorld *collisionWorld,
                      IntersectionEventListReducer *intersectionEventList,
                      Line **lines, unsigned int numOfLines, QuadTree *parent,
                      uintReducer *numCollisions);

void register_collision(CollisionWorld *collisionWorld,
                        IntersectionEventListReducer *intersectionEventList,
                        Line *l1, Line *l2, uintReducer *numCollisions);

void QuadTree_print(QuadTree *quad_tree);

void increase_line_capacity(QuadTree *quad_tree);

void increase_line_capacity2(Line **quad_lines[NUM_CHILDREN],
                            unsigned int m[NUM_CHILDREN],
                            int idx);
#endif
