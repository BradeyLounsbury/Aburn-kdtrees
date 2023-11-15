#include "helpers.h"

void generate_KD_Tree(GLViewKD_Trees* glview, Vector pos, std::vector<Vector> verts, Vector min, Vector max, std::map<WO*, KD_Node*>& PlaneMap, int iteration) {
	if (iteration == 0 || verts.empty()) {
		return;
	}

    // I think I should try to just pass in whether the current plane is using the min or max of the bounding box
	if (iteration % 3 == 0) { // x plan
        quickSort(verts, 0, verts.size() - 1, 0);
        //KD_Node* root;
        auto& x_2 = verts[(verts.size() - 1) / 2].x;
        WO* new_plane = WO::New();
        MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(new_plane);
        std::vector<Vector> lines;
        lines.push_back(Vector(x_2, min.y, min.z)); lines.push_back(Vector(x_2, max.y, min.z));
        lines.push_back(Vector(x_2, min.y, min.z)); lines.push_back(Vector(x_2, min.y, max.z));
        lines.push_back(Vector(x_2, max.y, max.z)); lines.push_back(Vector(x_2, max.y, min.z));
        lines.push_back(Vector(x_2, max.y, max.z)); lines.push_back(Vector(x_2, min.y, max.z));
        aftrColor4ub r = aftrColor4ub{ 255,0,0,255 };
        aftrColor4ub g = aftrColor4ub{ 0,255,0,255 };
        aftrColor4ub b = aftrColor4ub{ 0,0,255,255 };
        std::vector< aftrColor4ub > colors = { b,b,b,b,b,b,b,b };
        IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
        geom->setLineWidthInPixels(1.5);
        mgl->setIndexedGeometry(geom);
        GLSLShaderDefaultIndexedGeometryLinesGL32* shdr = GLSLShaderDefaultIndexedGeometryLinesGL32::New();
        mgl->getSkin().setShader(shdr);
        new_plane->setModel(mgl);
        new_plane->setPosition(pos);
        glview->getWorldContainer()->push_back(new_plane);

        std::vector<Vector> v1, v2;
        v1.assign(verts.begin(), verts.begin() + ((verts.size() - 1) / 2));
        v2.assign(verts.begin() + ((verts.size() - 1) / 2), verts.end());

        Vector v1_min = min;
        Vector v1_max = max; v1_max.x = x_2;

        Vector v2_min = min; v2_min.x = x_2;
        Vector v2_max = max; 

        iteration--;
        generate_KD_Tree(glview, pos, v1, v1_min, v1_max, PlaneMap, iteration);
        generate_KD_Tree(glview, pos, v2, v2_min, v2_max, PlaneMap, iteration);

        /*root = init_tree(verts, new_plane);
        PlaneMap.insert(std::pair<WO*, KD_Node*>(new_plane, root));*/
	}
	else if (iteration % 3 == 2) { // y plane
        quickSort(verts, 0, verts.size() - 1, 1);
        //KD_Node* root;
        auto& y_2 = verts[(verts.size() - 1) / 2].y;
        WO* new_plane = WO::New();
        MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(new_plane);
        std::vector<Vector> lines;
        lines.push_back(Vector(min.x, y_2, min.z)); lines.push_back(Vector(max.x, y_2, min.z));
        lines.push_back(Vector(min.x, y_2, min.z)); lines.push_back(Vector(min.x, y_2, max.z));
        lines.push_back(Vector(max.x, y_2, max.z)); lines.push_back(Vector(max.x, y_2, min.z));
        lines.push_back(Vector(max.x, y_2, max.z)); lines.push_back(Vector(min.x, y_2, max.z));
        aftrColor4ub r = aftrColor4ub{ 255,0,0,255 };
        aftrColor4ub g = aftrColor4ub{ 0,255,0,255 };
        aftrColor4ub b = aftrColor4ub{ 0,0,255,255 };
        std::vector< aftrColor4ub > colors = { r,r,r,r,r,r,r,r };
        IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
        geom->setLineWidthInPixels(1.5);
        mgl->setIndexedGeometry(geom);
        GLSLShaderDefaultIndexedGeometryLinesGL32* shdr = GLSLShaderDefaultIndexedGeometryLinesGL32::New();
        mgl->getSkin().setShader(shdr);
        new_plane->setModel(mgl);
        new_plane->setPosition(pos);
        glview->getWorldContainer()->push_back(new_plane);

        std::vector<Vector> v1, v2;
        v1.assign(verts.begin(), verts.begin() + ((verts.size() - 1) / 2));
        v2.assign(verts.begin() + ((verts.size() - 1) / 2), verts.end());

        Vector v1_min = min;
        Vector v1_max = max; v1_max.y = y_2;

        Vector v2_min = min; v2_min.y = y_2;
        Vector v2_max = max;

        iteration--;
        generate_KD_Tree(glview, pos, v1, v1_min, v1_max, PlaneMap, iteration);
        generate_KD_Tree(glview, pos, v2, v2_min, v2_max, PlaneMap, iteration);
    }
	else { // z plane
        quickSort(verts, 0, verts.size() - 1, 2);
        //KD_Node* root;
        auto& z_2 = verts[(verts.size() - 1) / 2].z;
        WO* new_plane = WO::New();
        MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(new_plane);
        std::vector<Vector> lines;
        lines.push_back(Vector(min.x, min.y, z_2)); lines.push_back(Vector(max.x, min.y, z_2));
        lines.push_back(Vector(min.x, min.y, z_2)); lines.push_back(Vector(min.x, max.y, z_2));
        lines.push_back(Vector(max.x, max.y, z_2)); lines.push_back(Vector(max.x, min.y, z_2));
        lines.push_back(Vector(max.x, max.y, z_2)); lines.push_back(Vector(min.x, max.y, z_2));
        aftrColor4ub r = aftrColor4ub{ 255,0,0,255 };
        aftrColor4ub g = aftrColor4ub{ 0,255,0,255 };
        aftrColor4ub b = aftrColor4ub{ 0,0,255,255 };
        std::vector< aftrColor4ub > colors = { g,g,g,g,g,g,g,g };
        IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
        geom->setLineWidthInPixels(1.5);
        mgl->setIndexedGeometry(geom);
        GLSLShaderDefaultIndexedGeometryLinesGL32* shdr = GLSLShaderDefaultIndexedGeometryLinesGL32::New();
        mgl->getSkin().setShader(shdr);
        new_plane->setModel(mgl);
        new_plane->setPosition(pos);
        glview->getWorldContainer()->push_back(new_plane);

        std::vector<Vector> v1, v2;
        v1.assign(verts.begin(), verts.begin() + ((verts.size() - 1) / 2));
        v2.assign(verts.begin() + ((verts.size() - 1) / 2), verts.end());

        Vector v1_min = min;
        Vector v1_max = max; v1_max.z = z_2;

        Vector v2_min = min; v2_min.z = z_2;
        Vector v2_max = max;

        iteration--;
        generate_KD_Tree(glview, pos, v1, v1_min, v1_max, PlaneMap, iteration);
        generate_KD_Tree(glview, pos, v2, v2_min, v2_max, PlaneMap, iteration);
	}
}