#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

// helper function, takes in a pattern and returns the value
static char *xprop(xmlXPathContextPtr ctx, const char *expr) {
  xmlXPathObjectPtr obj = xmlXPathEvalExpression(BAD_CAST expr, ctx);
  if (!obj || !obj->nodesetval || obj->nodesetval->nodeNr == 0) {
    xmlXPathFreeObject(obj);
    return strdup(""); // heap-allocated so caller can free() uniformly
  }

  xmlNodePtr n = obj->nodesetval->nodeTab[0];
  xmlChar *v = xmlNodeGetContent(n);

  char *out = strdup((char *) v);

  xmlFree(v);
  xmlXPathFreeObject(obj);
  return out;
}

#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include <cmink/chain.h>
#include <cmink/geometry.h>
#include <cmink/ops.h>

link_t *lprop(ktree_t *tree, char *lname) {
  for (uint32_t t = 0; t < tree->n_link; ++t) {
    if (strcmp(tree->links[t].name, lname) == 0) {
      return (tree->links + t);
    }
  }return NULL;
}

joint_t *jprop(ktree_t *tree, char *jname) {
  for (uint32_t t = 0; t < tree->n_joint; ++t) {
    if (strcmp(tree->joints[t].name, jname) == 0) {
      return (tree->joints + t);
    }
  }return NULL;
}

joint_t* jpropc(ktree_t *tree, char *child) {
  for (uint32_t t = 0; t < tree->n_joint; ++t) {
    if (strcmp(tree->joints[t].child->name, child) == 0) {
      return (tree->joints + t);
    }
  }return NULL;
}

joint_t* jpropp(ktree_t *tree, char *parent) {
  for (uint32_t t = 0; t < tree->n_joint; ++t) {
    if (strcmp(tree->joints[t].parent->name, parent) == 0) {
      return (tree->joints + t);
    }
  }return NULL;
}

#ifndef MAX_URDF_SIZE
# warning "no compile defition for MAX_URDF_SIZE, falling to default (MAX_URDF_SIZE = 100000)"
# define MAX_URDF_SIZE 100000
#endif

#define RBUFFER_SIZE 1000

enum joint_type jmap(char *type) {
  if (!type) return FIXED;
  if (strcmp(type, "fixed") == 0)      return FIXED;
  if (strcmp(type, "revolute") == 0)   return REVOLUTE;
  if (strcmp(type, "continuous") == 0) return CONTINUOUS;
  if (strcmp(type, "prismatic") == 0)  return PRISMATIC;
  if (strcmp(type, "floating") == 0)   return FLOATING;
  if (strcmp(type, "planar") == 0)     return PLANAR;
  return FIXED;
}

/*
 * as of now this supports only
 * the following joint types
 * fixed
 * revolute
 * continuous
 * prismatic
 * TODO: introduce angle limits for revolute joint
*/
se3_t jointT(double angle, joint_t J) {
  se3_t Tf = J.origin; // fixed transform
  se3_t Tm = se3_I();  // motion transform
 
  if (J.type == REVOLUTE || J.type == CONTINUOUS) {
    Tm.R = axis2rot(angle, J.axis);
  }else if (J.type == PRISMATIC) {
    angle = (angle > J.limit.upper ? J.limit.upper : angle);
    angle = (angle < J.limit.lower ? J.limit.lower : angle);
    Tm.t.x = J.axis.x * angle;
    Tm.t.y = J.axis.y * angle;
    Tm.t.z = J.axis.z * angle;
  }

  return se3_mul(Tf, Tm);
}

// TODO: implement this
bool valid(ktree_t *tree) {
  return false;
}

ktree_t *urdf2chain(char *urdf_string) {
  ktree_t *tree = (ktree_t *) malloc(sizeof(ktree_t));
  if (tree == NULL) {
    fprintf(stderr, "error: malloc failed..\n");  
    exit(1);
  }

  xmlDocPtr doc = xmlReadMemory(
    urdf_string, strlen(urdf_string),
    "blabla.xml", NULL, 0
  );

  if (doc == NULL) {
    fprintf(stderr, "error: failed to parse urdf string\n");
    exit(1);
  }

  xmlNodePtr root = xmlDocGetRootElement(doc);
  xmlXPathContextPtr ctx = xmlXPathNewContext(doc);

  xmlChar *root_name = xmlGetProp(root, (const unsigned char *) "name");
  tree->name = strdup((char *) root_name);
  xmlFree(root_name);

  tree->n_link = 0;
  tree->n_joint = 0;

  // first pass to get the size to allocate in tree
  for (xmlNodePtr node = root->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (strcmp((char *) node->name, "link") == 0) {
      ++(tree->n_link);
    }else if (strcmp((char *) node->name, "joint") == 0) {
      ++(tree->n_joint);
    }
  }

  tree->links = (link_t *) malloc(sizeof(link_t) * tree->n_link);
  if (tree->links == NULL) {
    fprintf(stderr, "error: malloc failed..\n");  
    exit(1);
  }

  tree->n_link = 0;
  for (xmlNodePtr node = root->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (strcmp((char *) node->name, "link") == 0) {
      ctx->node = node;
      link_t *L = (tree->links + tree->n_link);

      L->name = xprop(ctx, "@name");

      // reusage registers
      so3_rpy_t eu; 

      L->vis_mesh = xprop(ctx, "visual/geometry/mesh/@filename");
      char *vxyz = xprop(ctx, "visual/origin/@xyz");
      sscanf(vxyz, "%lf %lf %lf",
        &L->vis_pose.t.x,
        &L->vis_pose.t.y,
        &L->vis_pose.t.z);
      free(vxyz);
     
      char *vrpy = xprop(ctx, "visual/origin/@rpy");
      sscanf(vrpy, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      L->vis_pose.R = rpy2quat(eu);
      free(vrpy);

      L->col_mesh = xprop(ctx, "collision/geometry/mesh/@filename");

      char *cxyz = xprop(ctx, "collision/origin/@xyz");
      sscanf(cxyz, "%lf %lf %lf",
        &L->col_pose.t.x,
        &L->col_pose.t.y,
        &L->col_pose.t.z);
      free(cxyz);
     
      char *crpy = xprop(ctx, "collision/origin/@rpy");
      sscanf(crpy, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      L->col_pose.R = rpy2quat(eu);
      free(crpy);

      char *ixyz = xprop(ctx, "inertial/origin/@xyz");
      sscanf(ixyz, "%lf %lf %lf",
        &L->inertial.origin.t.x,
        &L->inertial.origin.t.y,
        &L->inertial.origin.t.z);
      free(ixyz);

      char *irpy = xprop(ctx, "inertial/origin/@rpy");
      sscanf(irpy, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      free(irpy);

      L->inertial.origin.R    = rpy2quat(eu);
      
      char *m   = xprop(ctx, "inertial/mass/@value");
      char *ixx = xprop(ctx, "inertial/inertia/@ixx");
      char *iyy = xprop(ctx, "inertial/inertia/@iyy");
      char *izz = xprop(ctx, "inertial/inertia/@izz");
      char *ixy = xprop(ctx, "inertial/inertia/@ixy");
      char *iyz = xprop(ctx, "inertial/inertia/@iyz");
      char *izx = xprop(ctx, "inertial/inertia/@izx");

      L->inertial.mass        = atof(m);
      L->inertial.inertia.ixx = atof(ixx);
      L->inertial.inertia.iyy = atof(iyy);
      L->inertial.inertia.izz = atof(izz);
      L->inertial.inertia.ixy = atof(ixy);
      L->inertial.inertia.iyz = atof(iyz);
      L->inertial.inertia.izx = atof(izx);

      free(m);
      free(ixx);
      free(iyy);
      free(izz);
      free(ixy);
      free(iyz);
      free(izx);

      ++(tree->n_link);
    }
  }

  tree->joints = (joint_t *) malloc(sizeof(joint_t) * tree->n_joint);
  if (tree->joints == NULL) {
    fprintf(stderr, "error: malloc failed..\n");  
    exit(1);
  }tree->n_joint = 0;

  // the following algo is O(l*j)
  // TODO: make it O(j)
  for (xmlNodePtr node = root->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (strcmp((char *) node->name, "joint") == 0) {
      ctx->node = node;
      joint_t *J = &tree->joints[tree->n_joint];

      J->name = xprop(ctx, "@name");

      char *type = xprop(ctx, "@type");
      J->type = jmap(type);
      free(type);

      char *xyz = xprop(ctx, "origin/@xyz");
      sscanf(xyz, "%lf %lf %lf",
        &J->origin.t.x,
        &J->origin.t.y,
        &J->origin.t.z);
      free(xyz);

      so3_rpy_t eu;
      char *rpy = xprop(ctx, "origin/@rpy");
      sscanf(rpy, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      J->origin.R = rpy2quat(eu);
      free(rpy);

      // lprop is O(l)
      char *parent = xprop(ctx, "parent/@link");
      char *child  = xprop(ctx, "child/@link");
      J->parent = lprop(tree, parent);
      J->child  = lprop(tree, child);
      free(parent);
      free(child);

      char *axis = xprop(ctx, "axis/@xyz");
      sscanf(axis, "%lf %lf %lf",
        &J->axis.x,
        &J->axis.y,
        &J->axis.z);
      anorm(&J->axis); // axis l2 normalization
      free(axis);

      char *lower    = xprop(ctx, "limit/@lower");
      char *upper    = xprop(ctx, "limit/@upper");
      char *effort   = xprop(ctx, "limit/@effort");
      char *velocity = xprop(ctx, "limit/@velocity");

      J->limit.lower    = atof(lower);
      J->limit.upper    = atof(upper);
      J->limit.effort   = atof(effort);
      J->limit.velocity = atof(velocity);

      free(lower);
      free(upper);
      free(effort);
      free(velocity);
    
      ++(tree->n_joint);
    }
  }

  xmlXPathFreeContext(ctx);
  xmlFreeDoc(doc);
  xmlCleanupParser();

  return tree;
}

ktree_t *furdf2chain(char *urdf_file) {
  int fd = open(urdf_file, O_RDONLY);
  if (fd < 0) {
    fprintf(stderr, "error: no such file: '%s'\n", urdf_file);
    exit(1);
  }

  char urdf_string[MAX_URDF_SIZE];
  char buff[RBUFFER_SIZE];
  uint32_t t = 0;
  uint32_t size;
  while ((size = read(fd, buff, RBUFFER_SIZE)) > 0) {
    if (t + size >= MAX_URDF_SIZE) {
      fprintf(stderr, "error: max urdf size (%d) exceeded\n", MAX_URDF_SIZE);
      fprintf(stderr, "hint : recompile with a higher MAX_URDF_SIZE\n");
      close(fd);
      exit(1);
    }memcpy(urdf_string + t, buff, size);
    t += size;
  }urdf_string[t] = 0;
  close(fd);
  ktree_t *tree = urdf2chain(urdf_string);
  return tree;
}

void free_tree(ktree_t *tree) {
  free(tree->name);
  for (uint32_t t = 0; t < tree->n_link; ++t) {
    free(tree->links[t].name);
    free(tree->links[t].vis_mesh);
    free(tree->links[t].col_mesh);
  }free(tree->links);
  for (uint32_t t = 0; t < tree->n_joint; ++t) {
    free(tree->joints[t].name);
  }free(tree->joints);
  free(tree);
}

chain_t *get_chain(
    ktree_t *tree, char *base, char *tip) {
  chain_t *chain = (chain_t *) malloc(sizeof(chain_t));
  if (chain == NULL) {
    fprintf(stderr, "error: malloc failed...\n");  
    exit(1);
  }chain->joints = (joint_t *) malloc(sizeof(joint_t) * tree->n_joint);
  if (chain->joints == NULL) {
    fprintf(stderr, "error: malloc failed...\n");  
    exit(1);
  }chain->base = lprop(tree, base);
  chain->tip = lprop(tree, tip);
  chain->dof = 0;
  char curr[100]; // assume max joint name is 100
  memset(curr, 0, sizeof(char) * 100);
  strcpy(curr, base);

  while (strcmp(curr, tip) != 0) {
    joint_t *J = jpropp(tree, curr);
    if (J == NULL) {
      fprintf(stderr, "error: link `%s` not found", curr);
      exit(1);
    }
    memcpy(chain->joints + chain->dof, J, sizeof(joint_t));
    strcpy(curr, J->child->name);
    curr[strlen(J->child->name)] = 0;
    ++(chain->dof);
  }

  return chain;
}

void free_chain(chain_t *chain) {
  free(chain->joints);
  free(chain);
}
