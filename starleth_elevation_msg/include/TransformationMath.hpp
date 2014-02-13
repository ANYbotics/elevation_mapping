/*
 * TransformationMath.hpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>

namespace starleth_elevation_msg {

/*!
 * Gets the position of a cell specified by its index in the map frame.
 * @param[out] position the position of the center of the cell in the map frame.
 * @param[in] index of the cell.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution the resolution of the map.
 * @param[in] bufferSize the size of the buffer (optional).
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if index not within range of buffer.
 */
bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const Eigen::Array2d& mapLength,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Gets the index of the cell which contains a position in the map frame.
 * @param[out] index of the cell.
 * @param[in] position the position in the map frame.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution the resolution of the map.
 * @param[in] bufferSize the size of the buffer (optional).
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if position outside of map.
 */
bool getIndexFromPosition(Eigen::Array2i& index,
                          const Eigen::Vector2d& position,
                          const Eigen::Array2d& mapLength,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Checks if position is within the map boundaries.
 * @param[in] position the position which is to be checked.
 * @param[in] mapLength the length of the map.
 * @return true if position is within map, false otherwise.
 */
bool checkIfPositionWithinMap(const Eigen::Vector2d& position,
                              const Eigen::Array2d& mapLength);


/*!
 * Computes how many cells/indeces the map is moved based on a position shift in
 * elevation map frame. Use this function if you are moving the elevation map
 * and want to ensure that the cells match before and after.
 * @param[out] indexShift the corresponding shift of the indices.
 * @param[in] positionShift the desired position shift.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
bool getIndexShiftFromPositionShift(Eigen::Array2i& indexShift,
                                    const Eigen::Vector2d& positionShift,
                                    const double& resolution);

/*!
 * Computes the corresponding position shift from a index shift. Use this function
 * if you are moving the elevation map and want to ensure that the cells match
 * before and after.
 * @param[out] positionShift the corresponding shift in position in the elevation map frame.
 * @param[in] indexShift the desired shift of the indeces.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
bool getPositionShiftFromIndexShift(Eigen::Vector2d& positionShift,
                                    const Eigen::Array2i& indexShift,
                                    const double& resolution);

/*!
 * Checks if index is within range of the buffer.
 * @param[in] index to check.
 * @param[in] bufferSize the size of the buffer.
 * @return true if index is within, and false if index is outside of the buffer.
 */
bool checkIfIndexWithinRange(const Eigen::Array2i& index, const Eigen::Array2i& bufferSize);

/*!
 * Maps an index that runs out of the range of the circular buffer back into allowed the region.
 * This is the 2d version of mapIndexWithinRange(int&, const int&).
 * @param[in/out] index the indeces that will be mapped into the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void mapIndexWithinRange(Eigen::Array2i& index,
                         const Eigen::Array2i& bufferSize);

/*!
 * Maps an index that runs out of the range of the circular buffer back into allowed the region.
 * @param[in/out] index the index that will be mapped into the valid region of the buffer.
 * @param[in] bufferSize the size of the buffer.
 */
void mapIndexWithinRange(int& index, const int& bufferSize);

/*!
 * Limits (cuts off) the position to lie inside the map.
 * @param[in/out] position the position to be limited.
 * @param[in] mapLength the lengths in x and y direction.
 */
void limitPositionToRange(Eigen::Vector2d& position, const Eigen::Array2d& mapLength);

/*!
 * Given a map and a desired submap (defined by position and size), this function computes
 * various information about the submap. The returned submap respects the boundaries of
 * the map and the returned submap might be smaller than the requested size.
 * @param[out] submapTopLeftIndex the top left index of the returned submap.
 * @param[out] submapBufferSize the buffer size of the returned submap.
 * @param[out] submapPosition the position of the submap in the map frame.
 * @param[out] requestedIndexInSubmap the index in the submap that corresponds to the requested
 *             position of the submap.
 * @param[in] requestedSubmapPosition the requested submap position (center) in the map frame.
 * @param[in] requestedSubmapLength the requested submap length.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution resolution the resolution of the map.
 * @param[in] bufferSize the buffer size of the map.
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return
 */
bool getSubmapInformation(Eigen::Array2i& submapTopLeftIndex,
                          Eigen::Array2i& submapBufferSize,
                          Eigen::Vector2d& submapPosition,
                          Eigen::Array2i& requestedIndexInSubmap,
                          const Eigen::Vector2d& requestedSubmapPosition,
                          const Eigen::Vector2d& requestedSubmapLength,
                          const Eigen::Array2d& mapLength,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * Computes the regions in the circular buffer that make up the data for
 * a requested submap.
 * @param[out] submapIndeces the list of indeces (top-left) for the buffer regions.
 * @param[out] submapSizes the sizes of the buffer regions.
 * @param[in] submapIndex the index (top-left) for the requested submap.
 * @param[in] submapSize the size of the requested submap.
 * @param[in] bufferSize the buffer size of the map.
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if requested submap is not fully contained in the map.
 */
bool getBufferRegionsForSubmap(std::vector<Eigen::Array2i>& submapIndeces,
                               std::vector<Eigen::Array2i>& submapSizes,
                               const Eigen::Array2i& submapIndex,
                               const Eigen::Array2i& submapSize,
                               const Eigen::Array2i& bufferSize,
                               const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 * The definition of the buffer regions.
 */
enum class bufferRegion
{
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
};

/*!
 * The definition of the position in the list for the buffer regions.
 */
std::map<bufferRegion, int> bufferRegionIndeces =
{
{ bufferRegion::TopLeft, 0 },
{ bufferRegion::TopRight, 1 },
{ bufferRegion::BottomLeft, 2 },
{ bufferRegion::BottomRight, 3 } };

} // namespace
