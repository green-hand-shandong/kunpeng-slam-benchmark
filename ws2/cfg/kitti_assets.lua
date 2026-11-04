-- KITTI → assets_writer 配置（从 pbstream 渲染 PCD）
VOXEL_SIZE = 5e-2
include "transform.lua"

options = {
  tracking_frame = "imu_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 100.,
    },
    {
      action = "dump_num_points",
    },
    {
      action = "write_pcd",
      filename = "kitti_0001_map.pcd",
    },
  },
}

options.transform = {
  translation = { 0., 0., 0. },
  rotation = { 0., 0., 0., 1. },
}
return options
