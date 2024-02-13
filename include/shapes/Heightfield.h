#ifndef Heightfield_h
#define Heightfield_h

#include <vector>
#include "shapes/ConvexPolyhedron.h"
#include "collision/AABB.h"

namespace Cannon::Shapes {

struct HeightfieldCachedPillar {
    ConvexPolyhedron* convex;
    Math::Vec3* offset;
};


class Heightfield {
private:
    std::map<std::string, HeightfieldCachedPillar*> cachedPillars_;

public:
    /**
     * An array of numbers, or height values, that are spread out along the x axis.
     * @property {array} data
     */
    std::vector<float> data;

    /**
     * Max value of the data
     * @property {number} maxValue
     */
    float maxValue;

    /**
     * Max value of the data
     * @property {number} minValue
     */
    float minValue;

    /**
     * The width of each element
     * @property {number} elementSize
     * @todo elementSizeX and Y
     */
    int elementSize;

    bool cacheEnabled = true;

    ConvexPolyhedron* pillarConvex;

    Math::Vec3 pillarOffset;

    /**
     * Heightfield shape class. Height data is given as an array. These data points are spread out evenly with a given distance.
     * @class Heightfield
     * @extends Shape
     * @constructor
     * @param {Array} data An array of Y values that will be used to construct the terrain.
     * @param {object} options
     * @param {Number} [options.minValue] Minimum value of the data points in the data array. Will be computed automatically if not given.
     * @param {Number} [options.maxValue] Maximum value.
     * @param {Number} [options.elementSize=0.1] World spacing between the data points in X direction.
     * @todo Should be possible to use along all axes, not just y
     * @todo should be possible to scale along all axes
     *
     * @example
     *     // Generate some height data (y-values).
     *     var data = [];
     *     for(var i = 0; i < 1000; i++){
     *         var y = 0.5 * Math.cos(0.2 * i);
     *         data.push(y);
     *     }
     *
     *     // Create the heightfield shape
     *     var heightfieldShape = new Heightfield(data, {
     *         elementSize: 1 // Distance between the data points in X and Y directions
     *     });
     *     var heightfieldBody = new Body();
     *     heightfieldBody.addShape(heightfieldShape);
     *     world.addBody(heightfieldBody);
     */
    Heightfield(std::vector<float>* data);

    /**
     * Call whenever you change the data array.
     * @method update
     */
    void update();

    /**
     * Update the .minValue property
     * @method updateMinValue
     */
    void updateMinValue();

    /**
     * Update the .maxValue property
     * @method updateMaxValue
     */
    void updateMaxValue();

    /**
     * Set the height value at an index. Don't forget to update maxValue and minValue after you're done.
     * @method setHeightValueAtIndex
     * @param {integer} xi
     * @param {integer} yi
     * @param {number} value
     */
    void setHeightValueAtIndex(int xi, int yi, float value);

    /**
     * Get max/min in a rectangle in the matrix data
     * @method getRectMinMax
     * @param  {integer} iMinX
     * @param  {integer} iMinY
     * @param  {integer} iMaxX
     * @param  {integer} iMaxY
     * @param  {array} [result] An array to store the results in.
     * @return {array} The result array, if it was passed in. Minimum will be at position 0 and max at 1.
     */
    void getRectMinMax(int iMinX, int iMinY, int iMaxX, int iMaxY, std::array<float, 2>* result);

    /**
     * Get the index of a local position on the heightfield. The indexes indicate the rectangles, so if your terrain is made of N x N height data points, you will have rectangle indexes ranging from 0 to N-1.
     * @method getIndexOfPosition
     * @param  {number} x
     * @param  {number} y
     * @param  {array} result Two-element array
     * @param  {boolean} clamp If the position should be clamped to the heightfield edge.
     * @return {boolean}
     */
    bool getIndexOfPosition(float x, float y, std::array<float, 2>* result, bool clamp);

    bool getTriangleAt(float x, float y, bool edgeClamp, Math::Vec3* a, Math::Vec3* b, Math::Vec3* c);

    void getNormalAt(float x, float y, bool edgeClamp, Math::Vec3* result);

    /**
     * Get an AABB of a square in the heightfield
     * @param  {number} xi
     * @param  {number} yi
     * @param  {AABB} result
     */
    void getAabbAtIndex(int xi, int yi, Collision::AABB* result);

    /**
     * Get the height in the heightfield at a given position
     * @param  {number} x
     * @param  {number} y
     * @param  {boolean} edgeClamp
     * @return {number}
     */
    float getHeightAt(int x, int y, bool edgeClamp);

    std::string getCacheConvexTrianglePillarKey(int xi, int yi, bool getUpperTriangle);

    HeightfieldCachedPillar* getCachedConvexTrianglePillar(int xi, int yi, bool getUpperTriangle);

    void setCachedConvexTrianglePillar(
        int xi,
        int yi,
        bool getUpperTriangle,
        ConvexPolyhedron* convex,
        Math::Vec3* offset);

    void clearCachedConvexTrianglePillar(int xi, int yi, bool getUpperTriangle);

    /**
    * Get a triangle from the heightfield
    * @param  {number} xi
    * @param  {number} yi
    * @param  {boolean} upper
    * @param  {Vec3} a
    * @param  {Vec3} b
    * @param  {Vec3} c
    */
    void getTriangle(int xi, int yi, bool upper, Math::Vec3* a, Math::Vec3* b, Math::Vec3* c);

    /**
    * Get a triangle in the terrain in the form of a triangular convex shape.
    * @method getConvexTrianglePillar
    * @param  {integer} i
    * @param  {integer} j
    * @param  {boolean} getUpperTriangle
    */
    void getConvexTrianglePillar(int xi, int yi, bool getUpperTriangle);

    Math::Vec3* calculateLocalInertia(float mass, Math::Vec3* target);

    double volume();

    void calculateWorldAABB(Math::Vec3 pos, Math::Quaternion quat, Math::Vec3* min, Math::Vec3* max);

    void updateBoundingSphereRadius();

    /**
    * Sets the height values from an image. Currently only supported in browser.
    * @method setHeightsFromImage
    * @param {Image} image
    * @param {Vec3} scale
    */
    // void setHeightsFromImage(int* image, Math::Vec3 scale);
};

}

#endif
