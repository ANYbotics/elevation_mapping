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
 * Maps an index that runs out of the range of the circular buffer back into allowed the region.
 * This is the 2d version of mapIndexWithinRange(int&, const int&).
 * @param index the indeces that will be mapped into the valid region of the buffer.
 * @param bufferSize the size of the buffer.
 */
void mapIndexWithinRange(Eigen::Array2i& index,
                         const Eigen::Array2i& bufferSize);

/*!
 * Maps an index that runs out of the range of the circular buffer back into allowed the region.
 * @param index the index that will be mapped into the valid region of the buffer.
 * @param bufferSize the size of the buffer.
 */
void mapIndexWithinRange(int& index, const int& bufferSize);

/*!
 * Given a map and a desired submap (defined by center and size), this function returns the index
 * and size (in cells) of a valid submap. The returned submap indeces respect the boundaries of
 * the map and the returned submap might be smaller than the requested size.
 * @param[out] submapTopLeftIndex the top left index of the returned submap.
 * @param[out] centerIndexInSubmap the index (in the submap) that corresponds to the requested
 *             submap center.
 * @param[out] submapSize the size (in cells) of the returned submap.
 * @param[in] submapCenter the desired submap center in map frame.
 * @param[in] submapLength the desired submap length.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution resolution the resolution of the map.
 * @param[in] bufferSize the size of the buffer.
 * @param[in] bufferStartIndex the index of the starting point of the circular buffer (optional).
 * @return true if successful, false if requested submapCenter lies outside of the map.
 */
bool getSubmapIndexAndSize(Eigen::Array2i& submapTopLeftIndex,
                           Eigen::Array2i& centerIndexInSubmap,
                           Eigen::Array2i& submapSize,
                           const Eigen::Vector2d& submapCenter,
                           const Eigen::Array2d& submapLength,
                           const Eigen::Array2d& mapLength,
                           const double& resolution,
                           const Eigen::Array2i& bufferSize,
                           const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());

/*!
 *
 * @param bufferIndeces
 * @param bufferSizes
 * @param bufferIndex the index of the top left corner of the submap
 * @param size
 * @param bufferSize
 * @param bufferStartIndex  (optional)
 * @return
 */
bool getBufferRegionsForSubmap(std::vector<Eigen::Array2i>& bufferIndeces,
                               std::vector<Eigen::Array2i>& bufferSizes,
                               const Eigen::Array2i& bufferIndex,
                               const Eigen::Array2i& size,
                               const Eigen::Array2i& bufferSize,
                               const Eigen::Array2i& bufferStartIndex = Eigen::Array2i::Zero());


Eigen::Matrix2i getBufferOrderToMapFrameAlignment();

} // namespace
