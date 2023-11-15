#include "KD_tree.h"

KD_Node* init_node(std::vector<Aftr::Vector> verts, Aftr::WO* wo) {
	KD_Node node;
	node.parent = nullptr;
	node.left = nullptr;
	node.right = nullptr;
	node.height = 0;
	node.verts = verts;
	node.plane = wo;
	return &node;
}

void add_left(KD_Node* root, std::vector<Aftr::Vector> verts, Aftr::WO* wo) {
	KD_Node child;
	child.parent = root;
	child.left = nullptr;
	child.right = nullptr;
	child.height = root->height + 1;
	child.verts = verts;
	child.plane = wo;

	root->left = &child;
}

void add_right(KD_Node* root, std::vector<Aftr::Vector> verts, Aftr::WO* wo) {
	KD_Node child;
	child.parent = root;
	child.left = nullptr;
	child.right = nullptr;
	child.height = root->height + 1;
	child.verts = verts;
	child.plane = wo;

	root->right = &child;
}