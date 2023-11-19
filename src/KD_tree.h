#pragma once
#include "GLViewKD_Trees.h"

struct KD_Node {
    KD_Node* parent; 
    KD_Node* left; 
    KD_Node* right;
    int height;
    Aftr::WO* plane;
};

KD_Node* init_node(std::map<KD_Node*, std::vector<Aftr::Vector>> &MapNodetoVerts, std::vector<Aftr::Vector> verts, Aftr::WO* wo);
void add_left(KD_Node* root, std::map<KD_Node*, std::vector<Aftr::Vector>> &MapNodetoVerts, std::vector<Aftr::Vector> verts, Aftr::WO* wo);
void add_right(KD_Node* root, std::map<KD_Node*, std::vector<Aftr::Vector>> &MapNodetoVerts, std::vector<Aftr::Vector> verts, Aftr::WO* wo);