/* 
 * Copyright (c) 2017, Christos Papachristos, University of Nevada, Reno
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Nevada, Reno nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The provided code implements the functionalities of the Uncertainty-aware Receding
 * Horizon Exploration and Mapping planner. Part of the implemented functionalities
 * are derived from the Receding Horizon Next-Best View planner.
 */

#ifndef TREE_HPP_
#define TREE_HPP_

#include <bsp_planner/tree.h>

template<typename stateVec>
bspPlanning::Node<stateVec>::Node()
{
  parent_ = NULL;
  distance_ = DBL_MAX;
  gain_ = 0.0;
}

template<typename stateVec>
bspPlanning::Node<stateVec>::~Node()
{
  for (typename std::vector<Node<stateVec> *>::iterator it = children_.begin();
      it != children_.end(); it++) {
    delete (*it);
    (*it) = NULL;
  }
}

template<typename stateVec>
bspPlanning::TreeBase<stateVec>::TreeBase()
  : nh_private_(NULL)  //Bsp: Exposes TreeBase to constructing node's parameters
{
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template<typename stateVec>
bspPlanning::TreeBase<stateVec>::TreeBase(volumetric_mapping::OctomapManager * manager, ros::NodeHandle * nh_private)
  : nh_private_(nh_private)  //Bsp: Exposes TreeBase to constructing node's parameters
{
  manager_ = manager;
  bestGain_ = params_.zero_gain_;
  bestNode_ = NULL;
  counter_ = 0;
  rootNode_ = NULL;
}

template<typename stateVec>
bspPlanning::TreeBase<stateVec>::~TreeBase()
{
}

template<typename stateVec>
void bspPlanning::TreeBase<stateVec>::setParams(Params params)
{
  params_ = params;
}

template<typename stateVec>
int bspPlanning::TreeBase<stateVec>::getCounter()
{
  return counter_;
}

template<typename stateVec>
bool bspPlanning::TreeBase<stateVec>::gainFound()
{
  return bestGain_ > params_.zero_gain_;
}

template<typename stateVec>
bool bspPlanning::TreeBase<stateVec>::reGainFound()
{
  return bestReGain_ < initReGain_;
}

template<typename stateVec>
void bspPlanning::TreeBase<stateVec>::insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud, bool localCheck)
{
  if (!localCheck){
    manager_->insertPointcloudWithTf(pointcloud);
  }
  else{
    sensor_msgs::PointCloud2Ptr pointcloudFiltered = boost::make_shared<sensor_msgs::PointCloud2>();
    pointcloudFiltered->header = pointcloud->header;
    //pointcloudFiltered->height = pointcloud->height;
    //pointcloudFiltered->width = pointcloud->width;
    //pointcloudFiltered->is_dense = pointcloud->is_dense;
    pointcloudFiltered->is_bigendian = pointcloud->is_bigendian;
    size_t STEPSIZE = pointcloud->point_step;
    pointcloudFiltered->point_step = STEPSIZE;
    //pointcloudFiltered->row_step = pointcloud->row_step;
    size_t FIELDSIZE =pointcloud->fields.size();
    pointcloudFiltered->fields.resize(FIELDSIZE);
    for (size_t i=0; i<FIELDSIZE; ++i){
      //TODO: verify that fields[0].name=="x" && fields[1].name=="y" && fields[2].name=="z" && fields[0,1,2].datatype==sensor_msgs::PointField::FLOAT32;
      pointcloudFiltered->fields[i].name = pointcloud->fields[i].name;
      pointcloudFiltered->fields[i].offset = pointcloud->fields[i].offset;
      pointcloudFiltered->fields[i].datatype = pointcloud->fields[i].datatype;
      pointcloudFiltered->fields[i].count = pointcloud->fields[i].count;
    }
    size_t PAYLOADSIZE = STEPSIZE - 3 * sizeof(float);  //extra payload apart from x,y,z - combine with TODO above
    size_t DATASIZE = pointcloud->data.size();
    pointcloudFiltered->data.resize(DATASIZE);
    float x_in, y_in, z_in;
    size_t offset_out = 0;
    for (size_t offset_in = 0; offset_in < DATASIZE; offset_in += STEPSIZE)
    {
      memcpy(&x_in, &pointcloud->data[offset_in], sizeof(float)); 
      memcpy(&y_in, &pointcloud->data[offset_in+4], sizeof(float)); 
      memcpy(&z_in, &pointcloud->data[offset_in+8], sizeof(float)); 
      if ( !pcl_isfinite(x_in) || !pcl_isfinite(y_in) || !pcl_isfinite(z_in) ||
           z_in<=0.0f ){
        continue;
      }
      else{
        memcpy(&pointcloudFiltered->data[offset_out], &x_in, sizeof(float)); 
        memcpy(&pointcloudFiltered->data[offset_out+4], &y_in, sizeof(float)); 
        memcpy(&pointcloudFiltered->data[offset_out+8], &z_in, sizeof(float)); 
        memcpy(&pointcloudFiltered->data[offset_out+12], &pointcloud->data[offset_in+12], PAYLOADSIZE); 
        offset_out += STEPSIZE;
      }
    }
    if (offset_out != DATASIZE)
    {
      pointcloudFiltered->data.resize(offset_out);
    }
    pointcloudFiltered->height = 1;
    pointcloudFiltered->width = static_cast<uint32_t>( offset_out / STEPSIZE );
    pointcloudFiltered->row_step = static_cast<uint32_t>( offset_out );
    pointcloudFiltered->is_dense = true;
    manager_->insertPointcloudWithTf(pointcloudFiltered);
  }
}

template<typename stateVec>
void bspPlanning::TreeBase<stateVec>::getRootState(Eigen::Vector3d& rootState_)
{
  rootState_[0] = root_[0];
  rootState_[1] = root_[1];
  rootState_[2] = root_[2]; 
}

template<typename stateVec>
void bspPlanning::TreeBase<stateVec>::clearRootStateBBX(Eigen::Vector3d& boundingBox, const double& minZ)
{
  //Make sure occupancy below minZ isn't cleared (would falsely remove ground voxels) 
  double deltaTopZ = root_[2]+boundingBox[2]/2 - minZ;
  if (deltaTopZ < 0){
    return;
  }
  else if (deltaTopZ >= boundingBox[2]){
    manager_->setFree(Eigen::Vector3d(root_[0],root_[1],root_[2]), boundingBox); //Creates required non-existing nodes
  }
  else{
    boundingBox[2] = deltaTopZ;
    manager_->setFree(Eigen::Vector3d(root_[0],root_[1],minZ+deltaTopZ/2), boundingBox); //Creates required non-existing nodes
  }
}

#endif // TREE_HPP_
