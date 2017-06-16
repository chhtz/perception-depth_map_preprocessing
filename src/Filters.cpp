#include "Filters.hpp"
#include <base/Float.hpp>

using namespace depth_map_preprocessing;

void Filters::filterMinDistance(base::samples::DepthMap& laser_scan, float min_range)
{
    for(unsigned i = 0; i < laser_scan.distances.size(); i++)
    {
        // check min range of measurement
        if(laser_scan.distances[i] < min_range)
            laser_scan.distances[i] = 0.0;
    }
}

void Filters::filterOutliers(base::samples::DepthMap& laser_scan, double max_deviation_angle, unsigned int min_neighbors)
{
    if(laser_scan.horizontal_size == 0 || laser_scan.vertical_size == 0)
        return;

    std::vector<Eigen::Vector3f> points;
    laser_scan.convertDepthMapToPointCloud(points, true, false);
    assert(points.size() == laser_scan.distances.size());

    base::samples::DepthMap::DepthMatrixMap distances = laser_scan.getDistanceMatrixMap();
    unsigned h_max = laser_scan.horizontal_size - 1;
    unsigned v_max = laser_scan.vertical_size - 1;
    unsigned valid_neighbors = 0;
    for(unsigned c = 0; c < laser_scan.vertical_size; c++)
    {
        for(unsigned r = 0; r < laser_scan.horizontal_size; r++)
        {
            const Eigen::Vector3f& origin = points[laser_scan.getIndex(c,r)];
            valid_neighbors = (r > 0 ? checkPoints(origin, points[laser_scan.getIndex(c,r-1)], max_deviation_angle) : 1) +
                              (c > 0 ? checkPoints(origin, points[laser_scan.getIndex(c-1,r)], max_deviation_angle) : 1) +
                              (c < v_max ? checkPoints(origin, points[laser_scan.getIndex(c+1,r)], max_deviation_angle) : 1) +
                              (r < h_max ? checkPoints(origin, points[laser_scan.getIndex(c,r+1)], max_deviation_angle) : 1);

            // invalidate distance measurement
            if(valid_neighbors < min_neighbors)
                distances(c,r) = base::NaN<float>();
        }
    }
}

unsigned Filters::checkPoints(const Eigen::Vector3f& origin, const Eigen::Vector3f& neighbor, double max_angle)
{
    if(!origin.allFinite() || !neighbor.allFinite())
        return 0;
    Eigen::Vector3f from_origin = neighbor - origin;
    if(std::abs(asin(origin.normalized().dot(from_origin) / from_origin.norm())) <= max_angle)
        return 1;
    return 0;
}


// Fit a line through an equidistant set of points.
// Allows to iterate through a moving window, by adding to the right and removing from the left
struct LineFit
{
    LineFit(const Eigen::VectorXf & values)
        : start(0), n(values.size())
        , x_sum(n*(n-1)/2), xx_sum(n*(n-1)*(2*n-1)/6), xy_sum(0.0f)
        , y_sum(values.sum()), yy_sum(values.squaredNorm())
    {
        for(int i=1; i<n; ++i)
            xy_sum += i*values[i];
    }

    void pop_front(float y)
    {
        x_sum -= start; xx_sum -= start*start;
        xy_sum -= start*y;
        y_sum -= y; yy_sum -= y*y;
        ++start; --n;
    }
    void push_back(float y)
    {
        int next = start+n;
        x_sum += next; xx_sum += next * next;
        xy_sum += next * y;
        y_sum += y; yy_sum += y*y;
        ++n;
    }



    int start, n;
    float x_sum, xx_sum, xy_sum, y_sum, yy_sum;
};


void Filters::extractEdges(std::vector<DepthMapEdge>& edges, const base::samples::DepthMap& laser_scan, int kernel_size, float min_gradient)
{
    if(laser_scan.remissions.empty())
    {
        std::cerr << "No remissions in DepthMap, cannot extract edges!\n";
        return;
    }

    typedef base::samples::DepthMap::DepthMatrixMapConst DepthMatrixMapConst;

    DepthMatrixMapConst remissions(laser_scan.remissions.data(), laser_scan.horizontal_size, laser_scan.vertical_size);


    for(int row = 0; row < remissions.rows(); ++row)
    {
        for(int col = kernel_size; col < remissions.cols(); ++col)
        {

        }

    }

}
