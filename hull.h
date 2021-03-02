#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

struct vec{
    double x;
    double y;
};

struct vecset {
    struct vec *data;
    size_t size;size_t capacity;
};

double dot(const struct vec *v1, const struct vec *v2);
double cross(const struct vec *p1,const struct vec *p2, const struct vec *p3);
bool is_left_turn(const struct vec *p1,const struct vec *p2, const struct vec *p3);
void vecset_create(struct vecset *self);
void vecset_destroy(struct vecset *self);
void vecset_add(struct vecset *self, struct vec p);
int comp_func_t(const struct vec *p1,const struct vec *p2, const void *ctx);
void vecset_push(struct vecset *self, struct vec p);