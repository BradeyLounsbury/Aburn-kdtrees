#include "GLViewKD_Trees.h"
#include "WO.h"
#include "WORay.h"
#include "WorldList.h"
#include "Model.h"
#include "MGLIndexedGeometry.h"
#include "IndexedGeometryLines.h"
#include "GLSLShaderDefaultIndexedGeometryLinesGL32.h"
#include "KD_tree.h"
#include "quicksort.h"

using namespace Aftr;

KD_Node* generate_KD_Tree(GLViewKD_Trees* glview, Vector pos, std::vector<Vector> verts, Vector min, Vector max, std::map<WO*, KD_Node*> &MapPlanetoTree, std::map<KD_Node*, std::vector<Vector>> &MapNodetoVerts, int iteration, bool isRoot);
bool line_intersects_plane(WO* plane, WORay* ray, Vector& output);