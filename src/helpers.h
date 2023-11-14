#include "GLViewKD_Trees.h"
#include "KD_tree.h"
#include "quicksort.h"

using namespace Aftr;

void generate_KD_Tree(GLViewKD_Trees* glview, WO* w, float plane1, float plane2, std::map<WO*, KD_Node*> &PlaneMap, int iteration);