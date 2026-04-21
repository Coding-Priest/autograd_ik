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
    return "";
  }

  xmlNodePtr n = obj->nodesetval->nodeTab[0];
  xmlChar *v = xmlNodeGetContent(n);

  char *out = strdup((char*)v);

  xmlFree(v);
  xmlXPathFreeObject(obj);

  return out;
}

#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include <cmink/chain.h>

link_t *lprop(ktree_t *tree, char *lname) {
  for (uint32_t t = 0; t < tree->n_link; ++t) {
    if (strcmp(tree->links[t].name, lname) == 0) {
      return (tree->links + t);
    }
  }return NULL;
}

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

#define RBUFFER_SIZE 1000

bool valid(ktree_t *tree, char *error_message) {
  return false;
}

ktree_t *urdf2chain(char *urdf_string) {
  ktree_t *tree = (ktree_t *) malloc(sizeof(ktree_t));
  xmlDocPtr doc = xmlReadMemory(
    urdf_string, strlen(urdf_string),
    "blabla.xml", NULL, 0
  );

  if (doc == NULL) {
    fprintf(stderr, "failed to parse urdf string\n");
    exit(1);
  }

  xmlNodePtr root = xmlDocGetRootElement(doc);
  xmlXPathContextPtr ctx = xmlXPathNewContext(doc);

  char *robot_name = xmlGetProp(root, "name");
  tree->name = malloc(sizeof(char) * (strlen(robot_name) + 1)); // count the '\0' char
  strcpy(tree->name, robot_name);
  free(robot_name);

  tree->n_link = 0;
  tree->n_joint = 0;

  // first pass to get the size to allocate in tree
  for (xmlNodePtr node = root->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (strcmp(node->name, "link") == 0) {
      ++(tree->n_link);
    }else if (strcmp(node->name, "joint") == 0) {
      ++(tree->n_joint);
    }
  }

  tree->links = (link_t *) malloc(sizeof(link_t) * tree->n_link);
  tree->n_link = 0;
  for (xmlNodePtr node = root->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (strcmp(node->name, "link") == 0) {
      ctx->node = node;
      link_t *L = (tree->links + tree->n_link);

      L->name = xprop(ctx, "@name");

      // reusage registers
      so3_rpy_t eu; 
      char *xyz;
      char *rpy;

      L->vis_mesh = xprop(ctx, "visual/geometry/mesh/@filename");

      xyz = xprop(ctx, "visual/origin/@xyz");
      sscanf(xyz, "%lf %lf %lf",
        &L->vis_pose.t.x,
        &L->vis_pose.t.y,
        &L->vis_pose.t.z);
     
      rpy = xprop(ctx, "visual/origin/@rpy");
      sscanf(rpy, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      L->vis_pose.R = rpy2quat(eu);

      L->col_mesh = xprop(ctx, "collision/geometry/mesh/@filename");

      xyz = xprop(ctx, "collision/origin/@xyz");
      sscanf(xyz, "%lf %lf %lf",
        &L->col_pose.t.x,
        &L->col_pose.t.y,
        &L->col_pose.t.z);
     
      rpy = xprop(ctx, "collision/origin/@rpy");
      sscanf(rpy, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      L->col_pose.R = rpy2quat(eu);

      xyz = xprop(ctx, "inertial/origin/@xyz");
      sscanf(xyz, "%lf %lf %lf",
        &L->inertial.origin.t.x,
        &L->inertial.origin.t.y,
        &L->inertial.origin.t.z);

      rpy = xprop(ctx, "inertial/origin/@rpy");
      sscanf(rpy, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      L->inertial.origin.R    = rpy2quat(eu);
      L->inertial.mass        = atof(xprop(ctx, "inertial/mass/@value"));
      L->inertial.inertia.ixx = atof(xprop(ctx, "inertial/inertia/@ixx"));
      L->inertial.inertia.iyy = atof(xprop(ctx, "inertial/inertia/@iyy"));
      L->inertial.inertia.izz = atof(xprop(ctx, "inertial/inertia/@izz"));
      L->inertial.inertia.ixy = atof(xprop(ctx, "inertial/inertia/@ixy"));
      L->inertial.inertia.iyz = atof(xprop(ctx, "inertial/inertia/@iyz"));
      L->inertial.inertia.izx = atof(xprop(ctx, "inertial/inertia/@izx"));

      ++(tree->n_link);
    }
  }

  tree->joints = (joint_t *) malloc(sizeof(joint_t) * tree->n_joint);
  tree->n_joint = 0;

  // the following algo is O(l*j)
  // TODO: make it O(j)
  for (xmlNodePtr node = root->children; node; node = node->next) {
    if (node->type != XML_ELEMENT_NODE) continue;
    if (strcmp(node->name, "joint") == 0) {
      ctx->node = node;
      joint_t *J = &tree->joints[tree->n_joint];

      J->name = xprop(ctx, "@name");
      J->type = jmap(xprop(ctx, "@type"));

      char *xyz = xprop(ctx, "origin/@xyz");
      sscanf(xyz, "%lf %lf %lf",
        &J->origin.t.x,
        &J->origin.t.y,
        &J->origin.t.z);

      so3_rpy_t eu;
      char *rpy = xprop(ctx, "origin/@rpy");
      sscanf(xyz, "%lf %lf %lf",
        &eu.r,
        &eu.p,
        &eu.y);
      J->origin.R = rpy2quat(eu);

      // lprop is O(l)
      J->parent = lprop(tree, xprop(ctx, "parent/@link"));
      J->child = lprop(tree, xprop(ctx, "child/@link"));
      xyz = xprop(ctx, "axis/@xyz");
      sscanf(xyz, "%lf %lf %lf",
        &J->axis.x,
        &J->axis.y,
        &J->axis.z);
      anorm(&J->axis); // axis l2 normalization 

      J->limit.lower    = atof(xprop(ctx, "limit/@lower"));
      J->limit.upper    = atof(xprop(ctx, "limit/@upper"));
      J->limit.effort   = atof(xprop(ctx, "limit/@effort"));
      J->limit.velocity = atof(xprop(ctx, "limit/@velocity"));
    
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
    fprintf(stderr, "no such file: '%s'\n", urdf_file);
    exit(1);
  }

  char *urdf_string = malloc(sizeof(char) * 100000);
  char buff[RBUFFER_SIZE];
  uint32_t t = 0;
  uint32_t size;
  while ((size = read(fd, buff, RBUFFER_SIZE)) > 0) {
    memcpy(urdf_string + t, buff, size);
    t += size;
  }urdf_string[t] = 0;

  close(fd);

  ktree_t *tree = urdf2chain(urdf_string);
  free(urdf_string);

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
