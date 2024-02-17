#include "shapes/Shape.h"

using namespace Cannon::Shapes;

int Shape::idCounter = 0;

Shape::Shape(ShapeTypes type) : type(type), id(idCounter++) {}

Shape::~Shape() {}
