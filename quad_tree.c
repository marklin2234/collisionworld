#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cilk/cilk.h>

#include "./quad_tree.h"
#include "stdbool.h"

QuadTree *QuadTree_initialize(QuadTree *parent, Vec tl, Vec br, unsigned int depth) {
  QuadTree *quad_tree = (QuadTree *)malloc(sizeof(QuadTree));

  if (quad_tree == NULL)
    return NULL;
  quad_tree->children = NULL;
  quad_tree->parent = parent;
  quad_tree->tl = tl;
  quad_tree->br = br;
  quad_tree->lines = NULL;
  quad_tree->numOfLines = 0;
  quad_tree->lineCapacity = LINE_THRESHOLD;
  quad_tree->depth = depth;
  return quad_tree;
}

QuadTree *QuadTree_create(CollisionWorld *collisionWorld,
                          IntersectionEventListReducer *intersectionEventList,
                          Line **lines, unsigned int numOfLines,
                          QuadTree *parent, Vec tl, Vec br,
                          uintReducer *numCollisions) {

  // Compare this quadtrees lines with its parent.
  if (parent != NULL) {
    // printf("%d, %d\n", numOfLines, parent->numOfLines);
    detect_collision(collisionWorld, intersectionEventList, lines, numOfLines, parent, numCollisions);
  }

  // Then create the new quad_tree
  QuadTree *quad_tree = QuadTree_initialize(parent, tl, br, parent == NULL ? 0 : parent->depth + 1);
  // printf("%d\n", quad_tree->depth);

  if (quad_tree == NULL) {
    return NULL;
  }

  // if too many lines, then we need to create children.
  if (numOfLines > LINE_THRESHOLD || quad_tree->depth == MAX_DEPTH) {
    quad_tree->lines = (Line **)malloc(sizeof(Line *) * LINE_THRESHOLD);
    
    if (quad_tree->lines == NULL) {
      free(quad_tree);
      return NULL;
    }
    // Divide the space into four regions
    quad_tree->children = (QuadTree **)malloc(sizeof(QuadTree *) * NUM_CHILDREN);

    if (quad_tree->children == NULL) {
      free(quad_tree->lines);
      free(quad_tree);
      return NULL;
    }

    // then each child needs to get initialized with their own lines, numOfLines, tl and br
    Vec **vec_array = getPoints(tl, br);
    if (vec_array == NULL) {
      QuadTree_destroy(quad_tree);
      return NULL;
    }

    // for (int i = 0; i < 8; i++) printf("%f, %f\n", vec_array[i]->x, vec_array[i]->y);
    Line **quad_lines[NUM_CHILDREN];
    unsigned int n[NUM_CHILDREN] = { 0 };
    unsigned int m[NUM_CHILDREN] = { LINE_THRESHOLD, LINE_THRESHOLD, LINE_THRESHOLD, LINE_THRESHOLD };

    for (int i = 0; i < NUM_CHILDREN; i++) {
      quad_lines[i] = (Line **)malloc(sizeof(Line *) * LINE_THRESHOLD);
      if (quad_lines[i] == NULL) {
        for (int j = 0; j < i; j++) {
          free(quad_lines[j]);
        }
        QuadTree_destroy(quad_tree);
        return NULL;
      }
    }
    
    // Check if line fits inside a quadrant. If it does, add it.
    // If it doesn't fit into any quadrent, store it in parent.
    for (unsigned int i = 0; i < numOfLines; i++) {
      Line *l = lines[i];
      bool doesFit = false;
      for (int j = 0; j < NUM_CHILDREN; j++) {
        if (isLineInRect(l, *vec_array[j * 2], *vec_array[j * 2 + 1])) {
          *(quad_lines[j] + n[j]++) = l;
          if (n[j] >= m[j]) {
            increase_line_capacity2(quad_lines, m, j);
          }
          doesFit = true;
          break;
        }
      }

      if (!doesFit) {
        quad_tree->lines[quad_tree->numOfLines++] = l;
        if (quad_tree->numOfLines >= quad_tree->lineCapacity) {
          increase_line_capacity(quad_tree);
        }
      }
    }

    cilk_for (int i = 0; i < quad_tree->numOfLines; i++) {
      Line *l1 = quad_tree->lines[i];
      for (int j = i + 1; j < quad_tree->numOfLines; j++) {
        Line *l2 = quad_tree->lines[j];
        register_collision(collisionWorld, intersectionEventList, l1, l2, numCollisions);
      }
    }

    // Create the children
    for (int i = 0; i < NUM_CHILDREN; i++) {
      if (n[i] != 0)
        quad_tree->children[i] =
          cilk_spawn QuadTree_create(collisionWorld, intersectionEventList,
                          quad_lines[i], n[i], quad_tree,
                          *vec_array[i * 2],
                          *vec_array[i * 2 + 1], numCollisions);
    }
    cilk_sync;
  } else {
    quad_tree->lines = lines;
    quad_tree->numOfLines = numOfLines;
    // printf("%d\n", numOfLines);
    cilk_for (int i = 0; i < numOfLines; i++) {
      Line *l1 = lines[i];
      for (int j = i + 1; j < numOfLines; j++) {
        Line *l2 = lines[j];
        register_collision(collisionWorld, intersectionEventList, l1, l2, numCollisions);
      }
    }
  }
  return quad_tree;
}

void QuadTree_destroy(QuadTree *quad_tree) {
  if (quad_tree == NULL) return;
  if (quad_tree->children != NULL) {
    cilk_for (int i = 0; i < NUM_CHILDREN; i++) {
      QuadTree_destroy(quad_tree->children[i]);
    }
  }
  free(quad_tree->lines);
  free(quad_tree->children);
  free(quad_tree);
}

bool isLineInRect(Line *line, Vec tl, Vec br) {
  return ((fmin(line->p1.x, line->p2.x) >= tl.x) &&
          (fmax(line->p1.x, line->p2.x) < br.x) &&
          (fmin(line->p1.y, line->p2.y) >= br.y) &&
          (fmax(line->p1.y, line->p2.y) < tl.y) &&
          (fmin(line->p1.x, line->p2.x) + line->velocity.x * 0.5 >= tl.x) &&
          (fmax(line->p1.x, line->p2.x) + line->velocity.x * 0.5 < br.x) &&
          (fmin(line->p1.y, line->p2.y) + line->velocity.y * 0.5 >= br.y) &&
          (fmax(line->p1.y, line->p2.y) + line->velocity.y * 0.5 < tl.y));
}

Vec **getPoints(Vec tl, Vec br) {
  Vec *vec_array[NUM_POINTS];
  for (int i = 0; i < NUM_POINTS; i++) {
    vec_array[i] = (Vec *)malloc(sizeof(Vec));
    if (vec_array[i] == NULL) {
      for (int j = 0; j < i; j++) {
        free(vec_array[j]);
      }
      return NULL;
    }
  }

  double scalar[3] = { 0, 0.5, 1 };
  double diffx = br.x - tl.x;
  double diffy = tl.y - br.y;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      vec_array[i * 3 + j]->x = tl.x + (diffx * scalar[j]);
      vec_array[i * 3 + j]->y = br.y + (diffy * scalar[i]);
    }
  }
  Vec **ret = (Vec **)malloc(sizeof(Vec *) * 8);
  if (ret == NULL) {
    for (int i = 0; i < NUM_POINTS; i++) {
      free(vec_array[i]);
    }
  }

  int indices[8] = { 6, 4, 7, 5, 3, 1, 4, 2 };
  for (int i = 0; i < 8; i++) {
    ret[i] = vec_array[indices[i]];
  }

  return ret;
}

void detect_collision(CollisionWorld *collisionWorld,
                      IntersectionEventListReducer *intersectionEventList,
                      Line **lines, unsigned int numOfLines, QuadTree *parent,
                      uintReducer *numCollisions) {
  cilk_for (int i = 0; i < numOfLines; i++) {
    Line *l1 = lines[i];

    for (int j = 0; j < parent->numOfLines; j++) {
      Line *l2 = parent->lines[j];
      register_collision(collisionWorld, intersectionEventList, l1, l2, numCollisions);
    }
  }
}

void register_collision(CollisionWorld *collisionWorld,
                        IntersectionEventListReducer *intersectionEventList,
                        Line *l1, Line *l2, uintReducer *numCollisions) {
  // intersect expects compareLines(l1, l2) < 0 to be true.
  // Swap l1 and l2, if necessary.
  if (compareLines(l1, l2) >= 0) {
    Line *temp = l1;
    l1 = l2;
    l2 = temp;
  }

  IntersectionType intersectionType =
      intersect(l1, l2, collisionWorld->timeStep);
  if (intersectionType != NO_INTERSECTION) {
    IntersectionEventList_appendNode(intersectionEventList, l1, l2,
                                      intersectionType);
    *(numCollisions) += 1;
  }
}

void QuadTree_print(QuadTree *quad_tree) {
  if (quad_tree == NULL) return;
  printf("POSITION: (%f, %f), (%f, %f)\n", quad_tree->tl.x, quad_tree->tl.y, quad_tree->br.x, quad_tree->br.y);
  printf("NUM LINES: %d\n", quad_tree->numOfLines);
  if (quad_tree->children != NULL) {
    for (int i = 0; i < NUM_CHILDREN; i++) {
      QuadTree_print(quad_tree->children[i]);
    }
  }
}

void increase_line_capacity(QuadTree *quad_tree) {
  int mul = 2;
  Line **tmp = (Line **)realloc(quad_tree->lines, sizeof(Line *) * quad_tree->lineCapacity * 2);
  if (tmp == NULL) {
    free(tmp);
  } else {
    quad_tree->lines = tmp;
    quad_tree->lineCapacity *= mul;
  }
}

void increase_line_capacity2(Line **quad_lines[NUM_CHILDREN],
                                    unsigned int m[NUM_CHILDREN],
                                    int idx) {
  Line **tmp = (Line **)realloc(quad_lines[idx], sizeof(Line *) * m[idx] * 2);
  if (tmp == NULL) {
    free(tmp);
  } else {
    quad_lines[idx] = tmp;
    m[idx] *= 2;
  }
}
