#include "node.h"

Node::Node()
{
	this->parent = -1;
}

Node::Node(float x=0, float y=0)
{
    this->x = x;
    this->y = y;
    this->parent = -1;
}

void Node::print()
{
    std::cout << Node::x << ", " << Node::y << std::endl;
}