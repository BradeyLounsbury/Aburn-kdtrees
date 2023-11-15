#include "GLViewKD_Trees.h"
#include "WO.h"
#include "WorldList.h"
#include "Model.h"
#include "MGLIndexedGeometry.h"
#include "IndexedGeometryLines.h"
#include "GLSLShaderDefaultIndexedGeometryLinesGL32.h"
#include "KD_tree.h"
#include "quicksort.h"

using namespace Aftr;

KD_Node* generate_KD_Tree(GLViewKD_Trees* glview, Vector pos, std::vector<Vector> verts, Vector min, Vector max, std::map<WO*, KD_Node*> &PlaneMap, int iteration);