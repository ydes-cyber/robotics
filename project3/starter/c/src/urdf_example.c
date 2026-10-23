/* Tiny example: reading a URDF with the vendored ezxml parser
 * (third_party/ezxml, compiled in by the Makefile).
 *
 * This only shows the XML plumbing. Deciding what the elements *mean*, which
 * descriptions are valid, and the forward kinematics are your job
 * (project3/spec/ROBOT_DESCRIPTION.md).
 *
 *     make build && ./build/urdf_example some_robot.urdf
 */
#include "ezxml.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: urdf_example FILE.urdf\n");
        return 2;
    }
    FILE *file = fopen(argv[1], "rb");
    if (file == NULL) {
        perror(argv[1]);
        return 1;
    }
    /* A description arrives as one string like this. */
    size_t length = 0, capacity = 4096;
    char *text = malloc(capacity);
    size_t got;
    while (text != NULL && (got = fread(text + length, 1, capacity - length, file)) > 0) {
        length += got;
        if (length == capacity) {
            char *grown = realloc(text, capacity *= 2);
            if (grown == NULL) free(text);
            text = grown;
        }
    }
    fclose(file);
    if (text == NULL) return 1;

    /* ezxml_parse_str() parses the buffer IN PLACE (it writes into it, so
     * pass a writable copy, never a string literal) and keeps pointers into
     * it: free the buffer only after ezxml_free(). It decodes &amp; and
     * character references and skips comments and processing instructions.
     * Be aware that ezxml is much more lenient than XML 1.0: it reports some
     * errors through ezxml_error(), but silently accepts a lot of text that
     * is not well-formed (stray text outside the root, unquoted or duplicate
     * attributes, undeclared entities, ...); a leading UTF-8 byte-order
     * mark is skipped only because it is such stray text. Its
     * <!DOCTYPE ... [ internal subset ]> and UTF-16 code paths have known
     * crash bugs. */
    ezxml_t robot = ezxml_parse_str(text, length);
    if (robot == NULL) {
        fprintf(stderr, "could not parse\n");
        free(text);
        return 1;
    }
    if (ezxml_error(robot)[0] != '\0') {
        fprintf(stderr, "not well-formed XML: %s\n", ezxml_error(robot));
        ezxml_free(robot);
        free(text);
        return 1;
    }
    const char *name = ezxml_attr(robot, "name");
    printf("root element: %s name: %s\n", robot->name, name != NULL ? name : "");
    /* robot->child, then ->ordered, visits the direct children in document
     * order (->next only links siblings with the same tag). */
    for (ezxml_t child = robot->child; child != NULL; child = child->ordered) {
        if (strcmp(child->name, "joint") == 0) {
            ezxml_t origin = ezxml_child(child, "origin"); /* direct children only */
            const char *xyz = origin != NULL ? ezxml_attr(origin, "xyz") : NULL;
            const char *joint_name = ezxml_attr(child, "name");
            const char *type = ezxml_attr(child, "type");
            printf("joint %s %s origin xyz: %s\n", joint_name != NULL ? joint_name : "", type != NULL ? type : "",
                   xyz != NULL ? xyz : "(none)");
        }
    }
    ezxml_free(robot);
    free(text);
    return 0;
}
