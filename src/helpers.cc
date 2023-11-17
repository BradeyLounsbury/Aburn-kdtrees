#include "helpers.h"

KD_Node* generate_KD_Tree(GLViewKD_Trees* glview, Vector pos, std::vector<Vector> verts, Vector min, Vector max, std::map<WO*, KD_Node*>& MapPlanetoTree, int iteration) {
	if (iteration == 0 || verts.empty()) {
		return nullptr;
	}

    WO* new_plane = WO::New();
    KD_Node* new_node = init_node(verts, new_plane);
	if (iteration % 3 == 0) { // yz plane
        quickSort(verts, 0, verts.size() - 1, 0);
        auto& x_2 = verts[(verts.size() - 1) / 2].x;
        
        MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(new_plane);
        std::vector<Vector> lines;
        lines.push_back(Vector(x_2, min.y, min.z)); lines.push_back(Vector(x_2, max.y, min.z));
        lines.push_back(Vector(x_2, min.y, min.z)); lines.push_back(Vector(x_2, min.y, max.z));
        lines.push_back(Vector(x_2, max.y, max.z)); lines.push_back(Vector(x_2, max.y, min.z));
        lines.push_back(Vector(x_2, max.y, max.z)); lines.push_back(Vector(x_2, min.y, max.z));
        
        aftrColor4ub b = aftrColor4ub{ 0,0,255,255 };
        std::vector< aftrColor4ub > colors = { b,b,b,b,b,b,b,b };

        IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
        geom->setLineWidthInPixels(1.5);
        mgl->setIndexedGeometry(geom);
        mgl->getBoundingBox().setBBoxExtrema(lines);
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

        MapPlanetoTree.insert(std::pair<WO*, KD_Node*>(new_plane, new_node));

        iteration--;
        new_node->left = generate_KD_Tree(glview, pos, v1, v1_min, v1_max, MapPlanetoTree, iteration);
        new_node->right = generate_KD_Tree(glview, pos, v2, v2_min, v2_max, MapPlanetoTree, iteration);
	}
	else if (iteration % 3 == 2) { // xz plane
        quickSort(verts, 0, verts.size() - 1, 1);
        auto& y_2 = verts[(verts.size() - 1) / 2].y;

        MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(new_plane);
        std::vector<Vector> lines;
        lines.push_back(Vector(min.x, y_2, min.z)); lines.push_back(Vector(max.x, y_2, min.z));
        lines.push_back(Vector(min.x, y_2, min.z)); lines.push_back(Vector(min.x, y_2, max.z));
        lines.push_back(Vector(max.x, y_2, max.z)); lines.push_back(Vector(max.x, y_2, min.z));
        lines.push_back(Vector(max.x, y_2, max.z)); lines.push_back(Vector(min.x, y_2, max.z));
        
        aftrColor4ub r = aftrColor4ub{ 255,0,0,255 };
        std::vector< aftrColor4ub > colors = { r,r,r,r,r,r,r,r };
        
        IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
        geom->setLineWidthInPixels(1.5);
        mgl->setIndexedGeometry(geom);
        mgl->getBoundingBox().setBBoxExtrema(lines);
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

        MapPlanetoTree.insert(std::pair<WO*, KD_Node*>(new_plane, new_node));

        iteration--;
        new_node->left = generate_KD_Tree(glview, pos, v1, v1_min, v1_max, MapPlanetoTree, iteration);
        new_node->right = generate_KD_Tree(glview, pos, v2, v2_min, v2_max, MapPlanetoTree, iteration);
    }
	else { // xy plane
        quickSort(verts, 0, verts.size() - 1, 2);
        auto& z_2 = verts[(verts.size() - 1) / 2].z;
        
        MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(new_plane);
        std::vector<Vector> lines;
        lines.push_back(Vector(min.x, min.y, z_2)); lines.push_back(Vector(max.x, min.y, z_2));
        lines.push_back(Vector(min.x, min.y, z_2)); lines.push_back(Vector(min.x, max.y, z_2));
        lines.push_back(Vector(max.x, max.y, z_2)); lines.push_back(Vector(max.x, min.y, z_2));
        lines.push_back(Vector(max.x, max.y, z_2)); lines.push_back(Vector(min.x, max.y, z_2));

        aftrColor4ub g = aftrColor4ub{ 0,255,0,255 };
        std::vector< aftrColor4ub > colors = { g,g,g,g,g,g,g,g };
        
        IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
        geom->setLineWidthInPixels(1.5);
        mgl->setIndexedGeometry(geom);
        mgl->getBoundingBox().setBBoxExtrema(lines);
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

        MapPlanetoTree.insert(std::pair<WO*, KD_Node*>(new_plane, new_node));

        iteration--;
        new_node->left = generate_KD_Tree(glview, pos, v1, v1_min, v1_max, MapPlanetoTree, iteration);
        new_node->right = generate_KD_Tree(glview, pos, v2, v2_min, v2_max, MapPlanetoTree, iteration);
	}

    return new_node;
}

bool line_intersects_plane(WO* plane, WORay* ray, Vector& output) {
    Vector min = plane->getModel()->getBoundingBox().getMin() + plane->getPosition();
    Vector max = plane->getModel()->getBoundingBox().getMax() + plane->getPosition();
    Vector head = ray->getRayHead();
    Vector tail = ray->getRayTail();
    Vector dir = (tail - head).normalizeMe();

    if (min.x - max.x == 0) {   // yz plane
        if (head.x <= min.x) {
            if (tail.x >= min.x) {
                float t;
                if (head.x == min.x || dir.x == 0) {
                    t = 0;
                }
                else {
                    t = (min.x - head.x) / dir.x;
                }
                Vector intersect = head + t * dir;

                if (intersect.y >= min.y && intersect.y <= max.y && intersect.z >= min.z && intersect.z <= max.z) {
                    output = intersect;
                    return true;
                }
            }
        }
        else {
            if (tail.x <= min.x) {
                float t;
                if (head.x == min.x || dir.x == 0) {
                    t = 0;
                }
                else {
                    t = (min.x - head.x) / dir.x;
                }
                Vector intersect = head + t * dir;

                if (intersect.y >= min.y && intersect.y <= max.y && intersect.z >= min.z && intersect.z <= max.z) {
                    output = intersect;
                    return true;
                }
            }
        }
    }
    else if (min.y - max.y == 0) {  // xz plane
        if (head.y <= min.y) {
            if (tail.y >= min.y) {
                float t;
                if (head.y == min.y || dir.y == 0) {
                    t = 0;
                }
                else {
                    t = (min.y - head.y) / dir.y;
                }
                Vector intersect = head + t * dir;

                if (intersect.x >= min.x && intersect.x <= max.x && intersect.z >= min.z && intersect.z <= max.z) {
                    output = intersect;
                    return true;
                }
            }
        }
        else {
            if (tail.y <= min.y) {
                float t;
                if (head.y == min.y || dir.y == 0) {
                    t = 0;
                }
                else {
                    t = (min.y - head.y) / dir.y;
                }
                Vector intersect = head + t * dir;

                if (intersect.x >= min.x && intersect.x <= max.x && intersect.z >= min.z && intersect.z <= max.z) {
                    output = intersect;
                    return true;
                }
            }
        }
    }
    else {  // xy plane
        if (head.z <= min.z) {
            if (tail.z >= min.z) {
                float t;
                if (head.z == min.z || dir.z == 0) {
                    t = 0;
                }
                else {
                    t = (min.z - head.z) / dir.z;
                }
                Vector intersect = head + t * dir;

                if (intersect.y >= min.y && intersect.y <= max.y && intersect.x >= min.x && intersect.x <= max.x) {
                    output = intersect;
                    return true;
                }
            }
        }
        else {
            if (tail.z <= min.z) {
                float t;
                if (head.z == min.z || dir.z == 0) {
                    t = 0;
                }
                else {
                    t = (min.z - head.z) / dir.z;
                }
                Vector intersect = head + t * dir;

                if (intersect.y >= min.y && intersect.y <= max.y && intersect.x >= min.x && intersect.x <= max.x) {
                    output = intersect;
                    return true;
                }
            }
        }
    }

    return false;
}