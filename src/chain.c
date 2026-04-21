#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>

// helper function, takes in a pattern and returns the value
static char* xprop(xmlXPathContextPtr ctx, const char *expr) {
  xmlXPathObjectPtr obj = xmlXPathEvalExpression(BAD_CAST expr, ctx);
  if (!obj || !obj->nodesetval || obj->nodesetval->nodeNr == 0)
    return "";

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

#define RBUFFER_SIZE 1000

bool valid(ktree_t *tree, char *error_message) {
  error_message = (char *) malloc(sizeof(char) * 256);
  memset(error_message, 0, sizeof(char) * 256);
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

  tree->name = malloc(sizeof(char) * strlen(root->name));
  strcpy(tree->name, xmlGetProp(root, "name"));

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
      L->vis_mesh = xprop(ctx, "visual/geometry/mesh/@filename");

      char *xyz = xprop(ctx, "visual/origin/@xyz");
      sscanf(xyz, "%lf %lf %lf",
        L->vis_pose.t.x,
        L->vis_pose.t.y,
        L->vis_pose.t.z);
     
      char *rpy = xprop(ctx, "visual/origin/@rpy");

      ++(tree->n_link);
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

  return urdf2chain(urdf_string);
}
